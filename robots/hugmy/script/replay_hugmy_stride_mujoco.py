#!/usr/bin/env python3
"""Replay an exported physical Hugmy stride in the MuJoCo Gym plant."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Mapping

import numpy as np
import rospkg
import yaml


FRONT_ARMS = (0, 3)
REAR_ARMS = (1, 2)


def environment_arguments(configuration: Dict[str, Any], stage: int,
                          direction: str) -> Dict[str, Any]:
    arguments = dict(configuration.get("environment", {}))
    arguments.update(arguments.pop("reward", {}))
    arguments["target_direction_mode"] = direction
    arguments["curriculum"] = configuration.get("curriculum", {})
    arguments["curriculum_stage"] = stage
    return arguments


def physical_to_normalized(sample: Mapping[str, Any], environment: Any) -> np.ndarray:
    pressure = np.asarray(sample["pressure_kpa"], dtype=np.float64)
    thrust = np.asarray(sample["thrust_n"], dtype=np.float64)
    front_pressure = float(np.mean(pressure[list(FRONT_ARMS)]))
    rear_pressure = float(np.mean(pressure[list(REAR_ARMS)]))
    front_thrust = float(np.mean(thrust[list(FRONT_ARMS)]))
    rear_thrust = float(np.mean(thrust[list(REAR_ARMS)]))

    def normalized_thrust(value: float) -> float:
        limit = (environment.maximum_forward_thrust_n if value >= 0.0
                 else environment.maximum_reverse_thrust_n)
        return value / limit

    return np.clip(np.asarray([
        2.0 * front_pressure / environment.maximum_pressure_kpa - 1.0,
        2.0 * rear_pressure / environment.maximum_pressure_kpa - 1.0,
        normalized_thrust(front_thrust), normalized_thrust(rear_thrust),
        2.0 * float(sample["bottom_pressure_kpa"])
        / environment.maximum_bottom_pressure_kpa - 1.0,
    ], dtype=np.float32), -1.0, 1.0)


def parse_arguments() -> argparse.Namespace:
    package = Path(rospkg.RosPack().get_path("hugmy"))
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--config", default=str(package / "config" / "hugmy_gym.yaml"))
    parser.add_argument("--stage", type=int, default=0)
    parser.add_argument("--trace-csv")
    parser.add_argument("--minimum-progress-m", type=float, default=0.010)
    parser.add_argument("--maximum-progress-m", type=float, default=0.100)
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    package = Path(rospkg.RosPack().get_path("hugmy"))
    script_directory = str(package / "script")
    if script_directory in sys.path:
        sys.path.remove(script_directory)
    sys.path.insert(0, script_directory)
    from hugmy_gym_env import HugmyMujocoEnv
    from run_hugmy_real_stride import load_trajectory

    trajectory = load_trajectory(args.trajectory)
    direction = "forward" if float(trajectory.get("target_direction", 1.0)) > 0 else "backward"
    with open(args.config, "r", encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    environment = HugmyMujocoEnv(**environment_arguments(
        configuration, args.stage, direction))
    trace = []
    try:
        environment.reset(seed=1)
        start = environment._snapshot()["position"].copy()  # noqa: SLF001
        final_info: Dict[str, Any] = {}
        squared_bend_error = 0.0
        bend_error_count = 0
        for index, sample in enumerate(trajectory["samples"]):
            action = physical_to_normalized(sample, environment)
            _, _, terminated, truncated, info = environment.step(action)
            state = environment._snapshot()  # noqa: SLF001
            measured_bend = np.asarray(state["grip"][24:28], dtype=np.float64)
            expected_bend = sample.get("expected_arm_bend_rad")
            if expected_bend is not None:
                error = measured_bend - np.asarray(expected_bend, dtype=np.float64)
                squared_bend_error += float(np.sum(error * error))
                bend_error_count += error.size
            trace.append({
                "step": index,
                "time_sec": float(sample["time_sec"]),
                "progress_m": float(info.get("total_progress_m", 0.0)),
                "stride_progress_m": float(info.get("stride_progress_m", 0.0)),
                "contact_count": int(info.get("contact_count", 0)),
                **{f"bend_{arm + 1}_rad": float(measured_bend[arm])
                   for arm in range(4)},
            })
            final_info = info
            if terminated or truncated:
                break
        state = environment._snapshot()  # noqa: SLF001
        direction_axis = np.asarray(trajectory["direction_body_xyz"], dtype=float)
        direction_axis *= float(trajectory.get("target_direction", 1.0))
        progress = float(np.dot(state["position"] - start, direction_axis))
        result = {
            "progress_m": progress,
            "expected_progress_m": float(trajectory.get("completed_stride_m", 0.0)),
            "completed_stride_count": int(final_info.get("completed_stride_count", 0)),
            "stride_completed": bool(final_info.get("stride_completed", False)),
            "contact_count": int(final_info.get("contact_count", 0)),
            "fallen": bool(final_info.get("fallen", False)),
            "detached": bool(final_info.get("detached", False)),
            "runaway": bool(final_info.get("stride_runaway", False)),
            "arm_bend_rmse_rad": (
                float(np.sqrt(squared_bend_error / bend_error_count))
                if bend_error_count else None),
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        if args.trace_csv:
            output = Path(args.trace_csv).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
                writer.writeheader()
                writer.writerows(trace)
            print(f"trace: {output}")
        successful = (
            args.minimum_progress_m <= progress <= args.maximum_progress_m
            and not result["fallen"] and not result["detached"]
            and not result["runaway"])
        if not successful:
            raise SystemExit(2)
    finally:
        environment.close()


if __name__ == "__main__":
    main()
