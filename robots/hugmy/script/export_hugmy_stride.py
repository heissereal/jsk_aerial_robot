#!/usr/bin/env python3
"""Export one successful stride from a trained Hugmy SAC policy.

The exported YAML contains physical pressure/thrust commands rather than
normalized SAC actions.  It is therefore suitable for guarded, open-loop
playback on a secured test article without reproducing MuJoCo-only contact
force observations on the real robot.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import sys
from typing import Any, Dict, List

import numpy as np
import rospkg
import yaml


def parse_arguments() -> argparse.Namespace:
    package = Path(rospkg.RosPack().get_path("hugmy"))
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--vec-normalize", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config", default=str(package / "config" / "hugmy_gym.yaml"))
    parser.add_argument("--stage", type=int, default=0)
    parser.add_argument("--direction", choices=("forward", "backward"), default="forward")
    parser.add_argument("--maximum-steps", type=int, default=600)
    parser.add_argument("--stochastic", action="store_true")
    return parser.parse_args()


def environment_arguments(configuration: Dict[str, Any], stage: int,
                          direction: str) -> Dict[str, Any]:
    arguments = dict(configuration.get("environment", {}))
    arguments.update(arguments.pop("reward", {}))
    arguments["target_direction_mode"] = direction
    arguments["curriculum"] = configuration.get("curriculum", {})
    arguments["curriculum_stage"] = stage
    return arguments


def main() -> None:
    args = parse_arguments()
    package = Path(rospkg.RosPack().get_path("hugmy"))
    script_directory = str(package / "script")
    if script_directory in sys.path:
        sys.path.remove(script_directory)
    sys.path.insert(0, script_directory)

    try:
        from stable_baselines3 import SAC
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        from hugmy_gym_env import HugmyMujocoEnv
    except ImportError as exc:
        raise SystemExit(
            "Gym dependencies are missing. Install robots/hugmy/requirements-rl.txt.\n"
            + str(exc))

    with open(args.config, "r", encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    raw_env = HugmyMujocoEnv(**environment_arguments(
        configuration, args.stage, args.direction))
    vec_env = DummyVecEnv([lambda: raw_env])
    vec_env = VecNormalize.load(args.vec_normalize, vec_env)
    vec_env.training = False
    vec_env.norm_reward = False
    model = SAC.load(args.model, env=vec_env)

    records: List[Dict[str, Any]] = []
    observation = vec_env.reset()
    previous_stride_count = 0
    try:
        for _ in range(args.maximum_steps):
            action, _ = model.predict(
                observation, deterministic=not args.stochastic)
            normalized_action = np.asarray(action[0], dtype=np.float64)
            pressures, _, bottom_pressure = raw_env._physical_action(  # noqa: SLF001
                normalized_action)
            observation, _, dones, infos = vec_env.step(action)
            info = infos[0]
            state = raw_env._snapshot()  # noqa: SLF001
            records.append({
                "time_sec": len(records) * raw_env.control_dt,
                "pressure_kpa": [float(value) for value in pressures],
                # Store the rate-limited command that actually reached MuJoCo.
                "thrust_n": [float(value) for value in raw_env._commanded_thrust],  # noqa: SLF001
                "bottom_pressure_kpa": float(bottom_pressure),
                "expected_progress_m": float(info.get("stride_progress_m", 0.0)),
                "expected_pitch_rad": float(state["pitch"]),
                "expected_roll_rad": float(state["roll"]),
                # MuJoCo grip telemetry [24:28] is the sum of the articulated
                # arm joint coordinates: straight=0, bending is positive.
                "expected_arm_bend_rad": [
                    float(value) for value in state["grip"][24:28]],
                "expected_contact_count": int(info.get("contact_count", 0)),
                "recovery_phase": bool(info.get("stride_recovery_phase", False)),
            })
            stride_count = int(info.get("completed_stride_count", 0))
            if stride_count > previous_stride_count:
                completed_distance = float(info.get("completed_stride_m", 0.0))
                document = {
                    "format": "hugmy_stride_trajectory",
                    "version": 1,
                    "source": {
                        "model": os.path.abspath(args.model),
                        "vec_normalize": os.path.abspath(args.vec_normalize),
                        "config": os.path.abspath(args.config),
                        "curriculum_stage": args.stage,
                        "deterministic": not args.stochastic,
                    },
                    "control_dt": raw_env.control_dt,
                    "direction_body_xyz": [float(value) for value in raw_env.direction_xyz],
                    "target_direction": float(info.get("target_direction", 1.0)),
                    "completed_stride_m": completed_distance,
                    "samples": records,
                }
                output = Path(args.output).expanduser().resolve()
                output.parent.mkdir(parents=True, exist_ok=True)
                with output.open("w", encoding="utf-8") as stream:
                    yaml.safe_dump(document, stream, sort_keys=False)
                print(
                    f"exported {len(records)} samples ({records[-1]['time_sec'] + raw_env.control_dt:.2f} s), "
                    f"completed stride {1000.0 * completed_distance:.1f} mm to {output}")
                return
            previous_stride_count = stride_count
            if bool(dones[0]):
                records = []
                # DummyVecEnv already returns the automatically reset
                # observation when done is true.
                previous_stride_count = 0
    finally:
        vec_env.close()
    raise SystemExit(
        f"No completed stride was observed in {args.maximum_steps} policy steps")


if __name__ == "__main__":
    main()
