#!/usr/bin/env python3
"""Evaluate a trained Hugmy SAC policy in the MuJoCo Gym environment."""

import argparse
import os
import sys

import rospkg
import yaml


def main() -> None:
    default_config = os.path.join(
        rospkg.RosPack().get_path("hugmy"), "config", "hugmy_gym.yaml")
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--vec-normalize", required=True)
    parser.add_argument("--config", default=default_config)
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--stage", type=int, default=0)
    parser.add_argument(
        "--direction", choices=("random", "forward", "backward"),
        default=None, help="Override the evaluation target direction")
    parser.add_argument("--stochastic", action="store_true")
    args = parser.parse_args()

    # Prefer the source module over catkin's same-named executable relay.
    script_directory = os.path.join(
        rospkg.RosPack().get_path("hugmy"), "script")
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
    environment = dict(configuration.get("environment", {}))
    environment.update(environment.pop("reward", {}))
    if args.direction is not None:
        environment["target_direction_mode"] = args.direction
    environment["curriculum"] = configuration.get("curriculum", {})
    environment["curriculum_stage"] = args.stage
    raw_env = HugmyMujocoEnv(**environment)
    vec_env = DummyVecEnv([lambda: raw_env])
    vec_env = VecNormalize.load(args.vec_normalize, vec_env)
    vec_env.training = False
    vec_env.norm_reward = False
    model = SAC.load(args.model, env=vec_env)

    observation = vec_env.reset()
    completed = 0
    episode_reward = 0.0
    try:
        while completed < args.episodes:
            action, _ = model.predict(
                observation, deterministic=not args.stochastic)
            observation, rewards, dones, infos = vec_env.step(action)
            episode_reward += float(rewards[0])
            if dones[0]:
                info = infos[0]
                completed += 1
                print(
                    f"episode {completed}: reward={episode_reward:.3f}, "
                    f"progress={info.get('total_progress_m', float('nan')):.4f} m, "
                    f"strides={info.get('completed_stride_count', 0)}, "
                    f"mean_stride={1000.0 * info.get('mean_completed_stride_m', 0.0):.1f} mm, "
                    f"partial={1000.0 * info.get('stride_progress_m', float('nan')):.1f} mm, "
                    f"success={info.get('stride_success', False)}, "
                    f"runaway={info.get('stride_runaway', False)}, "
                    f"direction={info.get('target_direction', float('nan')):+.0f}, "
                    f"fallen={info.get('fallen', False)}, "
                    f"detached={info.get('detached', False)}")
                episode_reward = 0.0
    finally:
        vec_env.close()


if __name__ == "__main__":
    main()
