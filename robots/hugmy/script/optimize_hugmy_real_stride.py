#!/usr/bin/env python3
"""Operator-confirmed Bayesian tuning of one real Hugmy stride.

Only six bounded trajectory scales are optimized.  Each physical trial needs
an explicit ``RUN`` confirmation unless ``--yes`` is supplied together with
``--execute``.  The runner itself still requires one-shot arming and enforces
its independent safety envelope.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np
import rospkg
import yaml
from scipy.linalg import solve_triangular
from scipy.stats import norm


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    optimization = document["optimization"]
    parameters = optimization["parameters"]
    if not parameters:
        raise ValueError("optimization.parameters must not be empty")
    for name, bounds in parameters.items():
        if float(bounds["min"]) >= float(bounds["max"]):
            raise ValueError(f"invalid bounds for {name}")
    if set(optimization["seed_candidate"]) != set(parameters):
        raise ValueError("seed_candidate must specify every scale")
    return document


def layout(config: Mapping) -> Tuple[List[str], np.ndarray, np.ndarray]:
    parameters = config["optimization"]["parameters"]
    names = list(parameters)
    lower = np.asarray([parameters[name]["min"] for name in names], dtype=float)
    upper = np.asarray([parameters[name]["max"] for name in names], dtype=float)
    return names, lower, upper


def latin_hypercube(count: int, dimension: int,
                    rng: np.random.Generator) -> np.ndarray:
    points = np.empty((count, dimension), dtype=float)
    for axis in range(dimension):
        points[:, axis] = (rng.permutation(count) + rng.random(count)) / count
    return points


def matern52(left: np.ndarray, right: np.ndarray,
             length_scale: float = 0.35) -> np.ndarray:
    delta = left[:, None, :] - right[None, :, :]
    distance = np.sqrt(np.sum(delta * delta, axis=2)) / length_scale
    root5 = math.sqrt(5.0) * distance
    return (1.0 + root5 + 5.0 * distance * distance / 3.0) * np.exp(-root5)


class CandidateGenerator:
    def __init__(self, config: Mapping):
        optimization = config["optimization"]
        self.names, self.lower, self.upper = layout(config)
        self.rng = np.random.default_rng(int(optimization["seed"]))
        self.initial_count = int(optimization["initial_random_trials"])
        self.pool_size = int(optimization["candidate_pool_size"])
        self.initial = latin_hypercube(
            self.initial_count, len(self.names), self.rng)
        seed = np.asarray([
            optimization["seed_candidate"][name] for name in self.names],
            dtype=float)
        self.initial[0] = (seed - self.lower) / (self.upper - self.lower)

    def normalize(self, candidate: Mapping[str, float]) -> np.ndarray:
        values = np.asarray([candidate[name] for name in self.names])
        return (values - self.lower) / (self.upper - self.lower)

    def suggest(self, index: int, history_x: Sequence[np.ndarray],
                history_y: Sequence[float]) -> Dict[str, float]:
        if index < self.initial_count:
            normalized = self.initial[index]
        elif len(history_x) < 2:
            normalized = self.rng.random(len(self.names))
        else:
            observations = np.asarray(history_x)
            rewards = np.asarray(history_y)
            reward_scale = max(float(np.std(rewards)), 1.0e-6)
            normalized_rewards = (rewards - np.mean(rewards)) / reward_scale
            kernel = matern52(observations, observations)
            kernel.flat[::kernel.shape[0] + 1] += 1.0e-2
            cholesky = np.linalg.cholesky(kernel)
            alpha = solve_triangular(
                cholesky.T,
                solve_triangular(cholesky, normalized_rewards, lower=True),
                lower=False)
            pool = self.rng.random((self.pool_size, len(self.names)))
            cross = matern52(observations, pool)
            mean = cross.T @ alpha
            projected = solve_triangular(cholesky, cross, lower=True)
            sigma = np.sqrt(np.maximum(
                1.0e-10, 1.0 - np.sum(projected * projected, axis=0)))
            improvement = mean - float(np.max(normalized_rewards)) - 0.01
            z_value = improvement / sigma
            acquisition = improvement * norm.cdf(z_value) + sigma * norm.pdf(z_value)
            normalized = pool[int(np.argmax(acquisition))]
        values = self.lower + normalized * (self.upper - self.lower)
        return dict(zip(self.names, (float(value) for value in values)))


def score(result: Mapping, config: Mapping) -> float:
    weights = config["optimization"]["reward"]
    metrics = result.get("metrics", {})
    progress = float(metrics.get("progress_m", 0.0))
    minimum_progress = float(weights["minimum_progress_m"])
    progress_cap = float(weights["progress_cap_m"])
    reward = float(weights["progress_per_m"]) * min(progress, progress_cap)
    reward -= float(weights["shortfall_per_m"]) * max(
        0.0, minimum_progress - progress)
    reward -= float(weights["overshoot_per_m"]) * max(0.0, progress - progress_cap)
    reward -= float(weights["lateral_per_m"]) * float(metrics.get("lateral_m", 0.0))
    reward -= float(weights["vertical_per_m"]) * float(metrics.get("vertical_m", 0.0))
    reward -= float(weights["tilt_per_rad"]) * float(
        metrics.get("maximum_relative_tilt_rad", 0.0))
    reward -= float(weights["thrust_integral"]) * float(
        metrics.get("thrust_integral_n_s", 0.0))
    reward -= float(weights["thrust_change"]) * float(
        metrics.get("thrust_change_n", 0.0))
    if bool(result.get("aborted", False)):
        reward -= float(weights["abort"])
    return reward


def append_result(path: Path, trial: int, candidate: Mapping[str, float],
                  reward: float, result: Mapping) -> None:
    fields = ["trial", *candidate.keys(), "reward", "aborted", "result_json"]
    new_file = not path.exists()
    with path.open("a", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        if new_file:
            writer.writeheader()
        writer.writerow({
            "trial": trial, **candidate, "reward": reward,
            "aborted": int(bool(result.get("aborted", False))),
            "result_json": json.dumps(result, separators=(",", ":")),
        })


def load_history(path: Path, generator: CandidateGenerator
                 ) -> Tuple[List[np.ndarray], List[float], List[dict]]:
    history_x: List[np.ndarray] = []
    history_y: List[float] = []
    rows: List[dict] = []
    if not path.exists():
        return history_x, history_y, rows
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            candidate = {name: float(row[name]) for name in generator.names}
            reward = float(row["reward"])
            result = json.loads(row["result_json"])
            history_x.append(generator.normalize(candidate))
            history_y.append(reward)
            rows.append({"candidate": candidate, "reward": reward, "result": result})
    return history_x, history_y, rows


def parse_arguments() -> argparse.Namespace:
    package = Path(rospkg.RosPack().get_path("hugmy"))
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(package / "config" / "hugmy_real_stride.yaml"))
    parser.add_argument("--output", default="/tmp/hugmy_real_stride_optimization")
    parser.add_argument("--runner", default="/quadrotor/hugmy_real_stride_runner")
    parser.add_argument("--trials", type=int)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--execute", action="store_true",
                        help="Permit calls to the real one-stride runner")
    parser.add_argument("--yes", action="store_true",
                        help="Skip per-trial RUN prompt (requires --execute)")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    config = load_config(Path(args.config))
    generator = CandidateGenerator(config)
    trial_count = int(args.trials or config["optimization"]["trials"])
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "trials.csv"
    if csv_path.exists() and not args.resume:
        raise SystemExit(f"{csv_path} exists; use --resume or another --output")
    history_x, history_y, rows = load_history(csv_path, generator) if args.resume else ([], [], [])
    for _ in range(max(0, len(rows) - generator.initial_count)):
        generator.rng.random((generator.pool_size, len(generator.names)))

    start_trial = len(rows)
    first = generator.suggest(start_trial, history_x, history_y)
    if args.yes and not args.execute:
        raise SystemExit("--yes requires --execute")
    if not args.execute:
        print("Configuration validated. No actuator command was sent.")
        print("First candidate:")
        print(yaml.safe_dump(first, sort_keys=False).strip())
        print("Add --execute to perform operator-confirmed trials.")
        return
    import rospy
    from std_srvs.srv import SetBool, Trigger

    rospy.init_node("hugmy_real_stride_optimizer", anonymous=True)
    arm_name = args.runner.rstrip("/") + "/arm"
    run_name = args.runner.rstrip("/") + "/run_once"
    rospy.wait_for_service(arm_name, timeout=10.0)
    rospy.wait_for_service(run_name, timeout=10.0)
    arm = rospy.ServiceProxy(arm_name, SetBool)
    run_once = rospy.ServiceProxy(run_name, Trigger)

    for trial in range(start_trial, trial_count):
        candidate = first if trial == start_trial else generator.suggest(
            trial, history_x, history_y)
        print(f"\ntrial {trial + 1}/{trial_count}")
        print(yaml.safe_dump(candidate, sort_keys=False).strip())
        if not args.yes:
            confirmation = input(
                "Secure the dummy cylinder, tether the robot, clear the area, then type RUN: ")
            if confirmation.strip() != "RUN":
                print("Stopped without arming.")
                break
        for name, value in candidate.items():
            rospy.set_param(args.runner.rstrip("/") + f"/scales/{name}", value)
        arm_response = arm(True)
        if not arm_response.success:
            raise RuntimeError("runner refused to arm: " + arm_response.message)
        response = run_once()
        try:
            result = json.loads(response.message)
        except json.JSONDecodeError as exc:
            raise RuntimeError("runner returned a non-JSON result: " + response.message) from exc
        reward = score(result, config)
        append_result(csv_path, trial, candidate, reward, result)
        history_x.append(generator.normalize(candidate))
        history_y.append(reward)
        rows.append({"candidate": candidate, "reward": reward, "result": result})
        best = max(rows, key=lambda item: item["reward"])
        with (output / "best_scales.yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump({
                "reward": best["reward"], "scales": best["candidate"],
                "result": best["result"]}, stream, sort_keys=False)
        metrics = result.get("metrics", {})
        print(
            f"reward={reward:.3f}, progress={1000.0 * float(metrics.get('progress_m', 0.0)):.1f} mm, "
            f"aborted={result.get('aborted', False)}, best={best['reward']:.3f}")
        if result.get("aborted", False):
            print("Safety abort:", result.get("reason", "unknown"))
    print(f"results: {output}")


if __name__ == "__main__":
    main()
