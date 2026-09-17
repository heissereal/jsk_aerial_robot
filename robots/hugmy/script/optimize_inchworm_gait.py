#!/usr/bin/env python3
"""Bayesian optimization of Hugmy's four-arm MuJoCo inchworm gait.

The outer process never imports rospy.  Every evaluation starts a fresh ROS
master and MuJoCo process, runs this file once more in ``--episode-json`` mode,
collects one result, and shuts the simulation down.  This is slower than an
in-process reset but also resets controller integrators, ROS time, pneumatic
state, and all MuJoCo state exactly between trials.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np
import yaml
from scipy.linalg import solve_triangular
from scipy.stats import norm


RESULT_PREFIX = "HUGMY_EPISODE_RESULT="
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_PATH = SCRIPT_PATH.parents[1]
DEFAULT_CONFIG = PACKAGE_PATH / "config" / "inchworm_optimization.yaml"


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    config = document["optimization"]
    if len(config["direction_xy"]) != 2:
        raise ValueError("direction_xy must contain exactly two values")
    if not config.get("required_phases"):
        config["required_phases"] = [8, 9]
    if not config["parameters"]:
        raise ValueError("at least one gait parameter is required")
    aggregation = str(config.get("reward_aggregation", "mean"))
    if aggregation not in ("mean", "worst"):
        raise ValueError("reward_aggregation must be either 'mean' or 'worst'")
    minimum_progress = float(config.get("minimum_completed_progress_m", 0.0))
    if minimum_progress < 0.0:
        raise ValueError("minimum_completed_progress_m must not be negative")
    for group in ("parameters", "domain_randomization"):
        for name, bounds in config[group].items():
            if float(bounds["min"]) >= float(bounds["max"]):
                raise ValueError(f"invalid bounds for {group}/{name}")
    seed_candidate = config.get("seed_candidate")
    if seed_candidate is not None:
        expected = set(config["parameters"])
        if set(seed_candidate) != expected:
            raise ValueError("seed_candidate must define every gait parameter exactly once")
        for name, value in seed_candidate.items():
            bounds = config["parameters"][name]
            if not float(bounds["min"]) <= float(value) <= float(bounds["max"]):
                raise ValueError(f"seed_candidate/{name} is outside its bounds")
    return config


def parameter_layout(config: Mapping) -> Tuple[List[str], np.ndarray, np.ndarray]:
    names = list(config["parameters"].keys())
    lower = np.asarray([config["parameters"][name]["min"] for name in names], dtype=float)
    upper = np.asarray([config["parameters"][name]["max"] for name in names], dtype=float)
    return names, lower, upper


def latin_hypercube(count: int, dimension: int, rng: np.random.Generator) -> np.ndarray:
    points = np.empty((count, dimension), dtype=float)
    for axis in range(dimension):
        points[:, axis] = (rng.permutation(count) + rng.random(count)) / count
    return points


def matern52(lhs: np.ndarray, rhs: np.ndarray, length_scale: float = 0.32) -> np.ndarray:
    delta = lhs[:, None, :] - rhs[None, :, :]
    distance = np.sqrt(np.sum(delta * delta, axis=2)) / length_scale
    root5_distance = math.sqrt(5.0) * distance
    return (1.0 + root5_distance + 5.0 * distance * distance / 3.0) * np.exp(-root5_distance)


def expected_improvement_suggestion(
    observations: np.ndarray,
    rewards: np.ndarray,
    candidates: np.ndarray,
) -> np.ndarray:
    """Return the candidate with maximum GP expected improvement."""
    reward_mean = float(np.mean(rewards))
    reward_scale = max(float(np.std(rewards)), 1.0e-6)
    normalized_reward = (rewards - reward_mean) / reward_scale
    kernel = matern52(observations, observations)
    # Episodes have contact discontinuities; retain a small observation noise.
    kernel.flat[:: kernel.shape[0] + 1] += 2.5e-3
    try:
        cholesky = np.linalg.cholesky(kernel)
    except np.linalg.LinAlgError:
        kernel.flat[:: kernel.shape[0] + 1] += 2.5e-2
        cholesky = np.linalg.cholesky(kernel)
    alpha = solve_triangular(
        cholesky.T,
        solve_triangular(cholesky, normalized_reward, lower=True),
        lower=False,
    )
    cross_kernel = matern52(observations, candidates)
    mean = cross_kernel.T @ alpha
    projected = solve_triangular(cholesky, cross_kernel, lower=True)
    variance = np.maximum(1.0e-10, 1.0 - np.sum(projected * projected, axis=0))
    sigma = np.sqrt(variance)
    improvement = mean - float(np.max(normalized_reward)) - 0.01
    z_value = improvement / sigma
    expected_improvement = improvement * norm.cdf(z_value) + sigma * norm.pdf(z_value)
    return candidates[int(np.argmax(expected_improvement))]


class CandidateGenerator:
    def __init__(self, config: Mapping):
        self.names, self.lower, self.upper = parameter_layout(config)
        self.rng = np.random.default_rng(int(config["seed"]))
        self.initial_count = int(config["initial_random_trials"])
        self.initial = latin_hypercube(self.initial_count, len(self.names), self.rng)
        if config.get("seed_candidate") is not None:
            seed = np.asarray(
                [config["seed_candidate"][name] for name in self.names], dtype=float
            )
            self.initial[0] = (seed - self.lower) / (self.upper - self.lower)
        self.pool_size = int(config["candidate_pool_size"])

    def suggest(self, index: int, x_history: Sequence[np.ndarray], y_history: Sequence[float]) -> dict:
        if index < self.initial_count:
            normalized = self.initial[index]
        elif len(x_history) < 2:
            normalized = self.rng.random(len(self.names))
        else:
            pool = self.rng.random((self.pool_size, len(self.names)))
            normalized = expected_improvement_suggestion(
                np.asarray(x_history), np.asarray(y_history), pool
            )
        physical = self.lower + normalized * (self.upper - self.lower)
        return {name: float(value) for name, value in zip(self.names, physical)}

    def normalize(self, parameters: Mapping[str, float]) -> np.ndarray:
        physical = np.asarray([parameters[name] for name in self.names], dtype=float)
        return (physical - self.lower) / (self.upper - self.lower)


def sample_domain(config: Mapping, index: int) -> dict:
    # Common random numbers: every gait sees the same set of physical domains.
    rng = np.random.default_rng(int(config["seed"]) + 100003 * (index + 1))
    result = {}
    for name, bounds in config["domain_randomization"].items():
        result[name] = float(rng.uniform(float(bounds["min"]), float(bounds["max"])))
    radius = result.pop("arm_radius_m")
    result["human_arm_radius"] = radius
    return result


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_for_master(environment: Mapping[str, str], process: subprocess.Popen, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and process.poll() is None:
        try:
            probe = subprocess.run(
                ["rosnode", "list"], env=environment,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3.0,
            )
        except subprocess.TimeoutExpired:
            time.sleep(0.1)
            continue
        if probe.returncode == 0:
            return True
        time.sleep(0.1)
    return False


def stop_process_group(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    for sig, wait_seconds in ((signal.SIGINT, 4.0), (signal.SIGTERM, 2.0)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait_seconds)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def launch_arguments(parameters: Mapping[str, float], domain: Mapping[str, float], config: Mapping) -> List[str]:
    arguments = []
    for name, value in {**parameters, **domain}.items():
        arguments.append(f"{name}:={value:.10g}")
    arguments.append(f"stroke_distance_m:={float(config['command_distance_m']):.10g}")
    return arguments


def evaluate_domain(
    parameters: Mapping[str, float],
    domain: Mapping[str, float],
    config: Mapping,
    trial_directory: Path,
    gui: bool = False,
) -> dict:
    trial_directory.mkdir(parents=True, exist_ok=True)
    input_path = trial_directory / "episode_input.json"
    with input_path.open("w", encoding="utf-8") as stream:
        json.dump({"parameters": parameters, "domain": domain, "config": config}, stream, indent=2)

    port = free_tcp_port()
    environment = os.environ.copy()
    environment["ROS_MASTER_URI"] = f"http://127.0.0.1:{port}"
    environment["ROS_HOSTNAME"] = "127.0.0.1"
    ros_log = trial_directory / "roslog"
    ros_log.mkdir(exist_ok=True)
    environment["ROS_LOG_DIR"] = str(ros_log)

    launch_command = [
        "roslaunch", "-p", str(port), "hugmy",
        str(config.get("trial_launch", "inchworm_optimization_trial.launch")),
        f"gui:={'true' if gui else 'false'}",
        *launch_arguments(parameters, domain, config),
    ]
    launch_log_path = trial_directory / "roslaunch.log"
    with launch_log_path.open("w", encoding="utf-8") as launch_log:
        simulation = subprocess.Popen(
            launch_command, env=environment, stdout=launch_log,
            stderr=subprocess.STDOUT, start_new_session=True, text=True,
        )
        try:
            startup_timeout = float(config["startup_timeout_wall_sec"])
            if not wait_for_master(environment, simulation, startup_timeout):
                return {"reward": -100.0, "failed": True, "reason": "ROS/MuJoCo startup failed"}
            episode_command = [
                sys.executable, str(SCRIPT_PATH), "--episode-json", str(input_path),
            ]
            episode = subprocess.run(
                episode_command, env=environment, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                # The workspace normally uses a Debug build and currently
                # advances MuJoCo at roughly 0.2--0.4x wall time.
                timeout=startup_timeout + max(
                    60.0, 8.0 * float(config["episode_timeout_sim_sec"])
                ),
            )
            (trial_directory / "episode.log").write_text(episode.stdout, encoding="utf-8")
            for line in reversed(episode.stdout.splitlines()):
                if line.startswith(RESULT_PREFIX):
                    return json.loads(line[len(RESULT_PREFIX):])
            return {
                "reward": -100.0, "failed": True,
                "reason": f"episode runner failed with code {episode.returncode}",
            }
        except subprocess.TimeoutExpired:
            return {"reward": -100.0, "failed": True, "reason": "episode wall-time timeout"}
        finally:
            stop_process_group(simulation)


def aggregate_results(results: Sequence[Mapping], method: str = "mean") -> dict:
    rewards = [float(result["reward"]) for result in results]
    if method == "worst":
        reward = min(rewards)
    else:
        reward = float(np.mean(rewards))
    aggregate = {
        "reward": reward,
        "reward_aggregation": method,
        "domain_rewards": rewards,
    }
    metric_names = set().union(*(result.get("metrics", {}).keys() for result in results))
    aggregate["metrics"] = {
        name: float(np.mean([result.get("metrics", {}).get(name, 0.0) for result in results]))
        for name in sorted(metric_names)
    }
    # A robust candidate must complete in every randomized physical domain.
    aggregate["failed"] = any(bool(result.get("failed", False)) for result in results)
    return aggregate


def append_csv(path: Path, trial: int, parameters: Mapping, result: Mapping) -> None:
    fields = ["trial", *parameters.keys(), "reward", "domain_rewards", "metrics", "failed"]
    new_file = not path.exists()
    with path.open("a", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if new_file:
            writer.writeheader()
        row = {"trial": trial, **parameters, "reward": result["reward"]}
        row["domain_rewards"] = json.dumps(result["domain_rewards"], separators=(",", ":"))
        row["metrics"] = json.dumps(result.get("metrics", {}), separators=(",", ":"))
        row["failed"] = int(bool(result.get("failed", False)))
        writer.writerow(row)


def load_history(path: Path, generator: CandidateGenerator) -> Tuple[List[np.ndarray], List[float], List[dict]]:
    x_history: List[np.ndarray] = []
    y_history: List[float] = []
    rows: List[dict] = []
    if not path.exists():
        return x_history, y_history, rows
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            parameters = {name: float(row[name]) for name in generator.names}
            reward = float(row["reward"])
            x_history.append(generator.normalize(parameters))
            y_history.append(reward)
            rows.append({"parameters": parameters, "reward": reward})
    return x_history, y_history, rows


def synthetic_reward(parameters: Mapping[str, float], config: Mapping) -> float:
    names, lower, upper = parameter_layout(config)
    point = (np.asarray([parameters[name] for name in names]) - lower) / (upper - lower)
    target = np.linspace(0.25, 0.75, len(names))
    return float(1.0 - np.sum((point - target) ** 2))


def optimize(args: argparse.Namespace) -> int:
    config = load_config(Path(args.config))
    if args.trials is not None:
        config["trials"] = args.trials
    if args.domain_samples is not None:
        config["domain_samples"] = args.domain_samples
    output = Path(args.output or config["result_directory"]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    generator = CandidateGenerator(config)
    csv_path = output / "trials.csv"
    if csv_path.exists() and not args.resume:
        raise RuntimeError(f"{csv_path} already exists; use --resume or select another --output")
    x_history, y_history, rows = load_history(csv_path, generator) if args.resume else ([], [], [])
    # Reproduce the random candidate-pool position used before an interrupted
    # run; otherwise the first resumed BO step could revisit an old pool.
    for _ in range(max(0, len(rows) - generator.initial_count)):
        generator.rng.random((generator.pool_size, len(generator.names)))

    trial_count = int(config["trials"])
    for trial in range(len(rows), trial_count):
        parameters = generator.suggest(trial, x_history, y_history)
        print(f"trial {trial + 1}/{trial_count}: {json.dumps(parameters, sort_keys=True)}", flush=True)
        if args.dry_run:
            result = {
                "reward": synthetic_reward(parameters, config),
                "domain_rewards": [], "metrics": {}, "failed": False,
            }
        else:
            domain_results = []
            for domain_index in range(int(config["domain_samples"])):
                domain = sample_domain(config, domain_index)
                directory = output / f"trial_{trial:04d}" / f"domain_{domain_index:02d}"
                episode_result = evaluate_domain(parameters, domain, config, directory)
                domain_results.append(episode_result)
                print(
                    f"  domain {domain_index + 1}: reward={episode_result['reward']:.4f} "
                    f"{episode_result.get('reason', '')}", flush=True,
                )
            result = aggregate_results(
                domain_results, str(config.get("reward_aggregation", "mean"))
            )
        append_csv(csv_path, trial, parameters, result)
        x_history.append(generator.normalize(parameters))
        y_history.append(float(result["reward"]))
        rows.append({"parameters": parameters, "reward": float(result["reward"]), "result": result})
        best = max(rows, key=lambda item: item["reward"])
        best_document = {
            "reward": best["reward"], "parameters": best["parameters"],
            "result": best.get("result", {}), "config": str(Path(args.config).resolve()),
        }
        (output / "best_gait.json").write_text(
            json.dumps(best_document, indent=2, sort_keys=True), encoding="utf-8"
        )
        label = str(config.get("reward_aggregation", "mean"))
        print(f"  {label} reward={result['reward']:.4f}; best={best['reward']:.4f}", flush=True)
    print(f"results: {output}")
    return 0


def replay(args: argparse.Namespace) -> int:
    config = load_config(Path(args.config))
    with Path(args.replay).open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    parameters = document.get("parameters", document)
    domain = sample_domain(config, int(args.replay_domain))
    output = Path(args.output or config["result_directory"]) / "replay"
    result = evaluate_domain(parameters, domain, config, output, gui=args.gui)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not result.get("failed", False) else 2


def quaternion_roll_pitch(x: float, y: float, z: float, w: float) -> Tuple[float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


def run_episode(input_path: Path) -> int:
    # ROS imports live only in the short-lived process. This avoids carrying a
    # rospy instance across ROS-master and simulated-time resets.
    import rospy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from spinal.msg import PneumaticCommand, Thrust
    from std_msgs.msg import Float32MultiArray, UInt8
    from std_srvs.srv import SetBool

    with input_path.open("r", encoding="utf-8") as stream:
        specification = json.load(stream)
    config = specification["config"]
    weights = config["reward"]
    base_pressure = float(specification["parameters"]["base_pressure_kpa"])
    direction = np.asarray(config["direction_xy"], dtype=float)
    direction /= max(1.0e-12, float(np.linalg.norm(direction)))

    state = {
        "odom": None, "pressure": None, "grip": None, "phase": None,
        "visited_phases": set(),
        "started": False, "start_position": None, "last_position": None,
        "start_stamp": None, "stamp": 0.0, "max_tilt": 0.0,
        "grip_samples": 0, "detached_samples": 0, "slip_sum": 0.0,
        "pump_effort": 0.0, "last_pump_stamp": None,
    }

    def odom_callback(message: Odometry) -> None:
        position = np.asarray([
            message.pose.pose.position.x, message.pose.pose.position.y,
            message.pose.pose.position.z,
        ])
        state["odom"] = message
        state["last_position"] = position
        state["stamp"] = message.header.stamp.to_sec()
        if state["started"]:
            orientation = message.pose.pose.orientation
            roll, pitch = quaternion_roll_pitch(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            state["max_tilt"] = max(state["max_tilt"], abs(roll), abs(pitch))

    def pressure_callback(message: Float32MultiArray) -> None:
        if len(message.data) == 4:
            state["pressure"] = np.asarray(message.data, dtype=float)

    def grip_callback(message: Float32MultiArray) -> None:
        if len(message.data) < 12:
            return
        state["grip"] = np.asarray(message.data, dtype=float)
        if state["started"]:
            active_count = int(np.count_nonzero(state["grip"][:4] > 0.5))
            state["grip_samples"] += 1
            if active_count < 2:
                state["detached_samples"] += 1
            state["slip_sum"] += float(np.sum(np.maximum(0.0, state["grip"][4:8] - 1.0)))

    def phase_callback(message: UInt8) -> None:
        state["phase"] = int(message.data)
        if state["started"]:
            state["visited_phases"].add(state["phase"])

    def pneumatic_callback(message: PneumaticCommand) -> None:
        stamp = state["stamp"]
        previous = state["last_pump_stamp"]
        if state["started"] and previous is not None and stamp >= previous:
            dt = min(0.1, stamp - previous)
            duty = float(message.pump_pwm) * (
                sum(float(value) for value in message.supply_pwm) +
                float(message.bottom_supply_pwm)
            )
            state["pump_effort"] += duty * dt
        state["last_pump_stamp"] = stamp

    rospy.init_node("hugmy_inchworm_episode", anonymous=True, disable_signals=True)
    rospy.Subscriber("/quadrotor/ground_truth", Odometry, odom_callback, queue_size=20)
    rospy.Subscriber(
        "/quadrotor/independent_arm_pressure_controller/pressure",
        Float32MultiArray, pressure_callback, queue_size=20,
    )
    rospy.Subscriber("/quadrotor/pneumatic/grip_state", Float32MultiArray, grip_callback, queue_size=20)
    rospy.Subscriber("/quadrotor/attitude_pressure_controller/state", UInt8, phase_callback, queue_size=20)
    rospy.Subscriber("/quadrotor/pneumatic/command", PneumaticCommand, pneumatic_callback, queue_size=20)
    command_publisher = rospy.Publisher(
        "/quadrotor/attitude_pressure_controller/inchworm_command",
        Vector3Stamped, queue_size=1, latch=True,
    )
    enable_name = "/quadrotor/attitude_pressure_controller/enable"
    startup_deadline = time.monotonic() + float(config["startup_timeout_wall_sec"])
    try:
        rospy.wait_for_service(enable_name, timeout=float(config["startup_timeout_wall_sec"]))
    except rospy.ROSException:
        result = {"reward": -100.0, "failed": True, "reason": "enable service not available"}
        print(RESULT_PREFIX + json.dumps(result, separators=(",", ":")))
        return 2
    enable = rospy.ServiceProxy(enable_name, SetBool)
    while time.monotonic() < startup_deadline and not rospy.is_shutdown():
        if state["odom"] is not None and state["pressure"] is not None and state["grip"] is not None:
            try:
                response = enable(True)
                if response.success:
                    break
            except rospy.ServiceException:
                pass
        time.sleep(0.02)
    else:
        result = {"reward": -100.0, "failed": True, "reason": "sensors/controller not ready"}
        print(RESULT_PREFIX + json.dumps(result, separators=(",", ":")))
        return 2

    # Allow all four chambers to establish the common grasp before measuring.
    grasp_start = state["stamp"]
    while not rospy.is_shutdown() and state["stamp"] - grasp_start < 15.0:
        pressure_ready = state["pressure"] is not None and bool(
            np.all(np.abs(state["pressure"] - base_pressure) <= 2.5)
        )
        if pressure_ready:
            break
        time.sleep(0.005)

    state["started"] = True
    state["start_position"] = np.copy(state["last_position"])
    state["start_stamp"] = float(state["stamp"])
    command = Vector3Stamped()
    command.header.stamp = rospy.Time.now()
    command.header.frame_id = "main_body"
    command.vector.x = float(direction[0])
    command.vector.y = float(direction[1])
    command.vector.z = float(config["command_distance_m"])
    connection_deadline = time.monotonic() + 3.0
    while command_publisher.get_num_connections() == 0 and time.monotonic() < connection_deadline:
        time.sleep(0.01)
    command_publisher.publish(command)

    seen_motion_state = False
    returned_to_grasp_at = None
    timeout = float(config["episode_timeout_sim_sec"])
    while not rospy.is_shutdown() and state["stamp"] - state["start_stamp"] < timeout:
        if state["phase"] is not None and state["phase"] != 0:
            seen_motion_state = True
            returned_to_grasp_at = None
        elif seen_motion_state and state["phase"] == 0:
            if returned_to_grasp_at is None:
                returned_to_grasp_at = state["stamp"]
            elif state["stamp"] - returned_to_grasp_at >= 0.5:
                break
        time.sleep(0.005)

    # A return to GRASP can also be an abort path. Each gait config lists the
    # state-machine phases that constitute a complete locomotion cycle.
    required_phases = {int(value) for value in config["required_phases"]}
    completed_sequence = (
        seen_motion_state and returned_to_grasp_at is not None
        and required_phases.issubset(state["visited_phases"])
    )
    start_position = state["start_position"]
    final_position = state["last_position"]
    displacement = final_position - start_position
    progress = float(np.dot(displacement[:2], direction))
    minimum_progress = float(config.get("minimum_completed_progress_m", 0.0))
    completed = completed_sequence and progress >= minimum_progress
    lateral_direction = np.asarray([-direction[1], direction[0]])
    lateral = abs(float(np.dot(displacement[:2], lateral_direction)))
    vertical = abs(float(displacement[2]))
    grip_samples = max(1, int(state["grip_samples"]))
    detached_fraction = float(state["detached_samples"]) / grip_samples
    slip_excess = float(state["slip_sum"]) / grip_samples
    metrics = {
        "progress_m": progress,
        "lateral_m": lateral,
        "vertical_m": vertical,
        "max_tilt_rad": float(state["max_tilt"]),
        "detached_fraction": detached_fraction,
        "slip_excess": slip_excess,
        "pump_effort": float(state["pump_effort"]),
        "duration_sim_sec": float(state["stamp"] - state["start_stamp"]),
        "cycle_completed": 1.0 if completed_sequence else 0.0,
        "completed": 1.0 if completed else 0.0,
    }
    reward = (
        float(weights["progress"]) * progress
        - float(weights["lateral"]) * lateral
        - float(weights["vertical"]) * vertical
        - float(weights["max_tilt"]) * state["max_tilt"]
        - float(weights["detached_fraction"]) * detached_fraction
        - float(weights["slip_excess"]) * slip_excess
        - float(weights["pump_effort"]) * state["pump_effort"]
        - (0.0 if completed else float(weights["timeout"]))
    )
    try:
        enable(False)
    except rospy.ServiceException:
        pass
    if completed:
        failure_reason = ""
    elif completed_sequence:
        failure_reason = (
            f"retained forward progress {progress:.6f} m is below required "
            f"{minimum_progress:.6f} m"
        )
    else:
        failure_reason = "configured locomotion cycle was not completed"
    result = {
        "reward": float(reward), "metrics": metrics,
        "visited_phases": sorted(int(value) for value in state["visited_phases"]),
        "failed": not completed,
        "reason": failure_reason,
    }
    print(RESULT_PREFIX + json.dumps(result, separators=(",", ":")))
    return 0 if completed else 2


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--trials", type=int)
    parser.add_argument("--domain-samples", type=int)
    parser.add_argument("--output")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="test BO logic using a synthetic reward")
    parser.add_argument("--replay", help="best_gait.json or a parameter-only JSON file")
    parser.add_argument("--replay-domain", type=int, default=0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--episode-json", help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.episode_json:
        return run_episode(Path(args.episode_json))
    if args.replay:
        return replay(args)
    return optimize(args)


if __name__ == "__main__":
    raise SystemExit(main())
