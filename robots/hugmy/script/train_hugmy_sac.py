#!/usr/bin/env python3
"""Train SAC on the ROS-connected Hugmy MuJoCo Gymnasium environment."""

import argparse
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Any, Dict

import rospkg
import yaml


def save_atomically(save_function: Any, destination: Path) -> None:
    """Keep an existing artifact intact when serialization fails midway."""
    temporary = destination.with_name(
        f".{destination.stem}.tmp{destination.suffix}")
    try:
        save_function(str(temporary))
        os.replace(str(temporary), str(destination))
    finally:
        if temporary.exists():
            temporary.unlink()


def load_configuration(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    if not isinstance(configuration, dict):
        raise ValueError("Gym configuration must be a YAML mapping")
    return configuration


def environment_arguments(configuration: Dict[str, Any], stage: int) -> Dict[str, Any]:
    arguments = dict(configuration.get("environment", {}))
    arguments.update(arguments.pop("reward", {}))
    arguments["curriculum"] = configuration.get("curriculum", {})
    arguments["curriculum_stage"] = stage
    return arguments


def parse_arguments() -> argparse.Namespace:
    default_config = os.path.join(
        rospkg.RosPack().get_path("hugmy"), "config", "hugmy_gym.yaml")
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=default_config)
    parser.add_argument("--output", default="/tmp/hugmy_sac")
    parser.add_argument("--total-timesteps", type=int, default=None,
                        help="Total budget across selected stages")
    parser.add_argument("--start-stage", type=int, default=0)
    parser.add_argument("--stop-stage", type=int, default=None)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--device", choices=("auto", "cpu", "cuda"), default=None,
        help="SB3/PyTorch device; overrides training.device in the YAML")
    parser.add_argument("--resume", help="Existing SAC .zip file")
    parser.add_argument("--replay-buffer",
                        help="Existing SAC replay-buffer .pkl file")
    parser.add_argument("--vec-normalize", help="Existing VecNormalize .pkl")
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    # catkin_install_python creates relay scripts in devel/lib/hugmy.  Because
    # the environment module has the same filename there, a normal import can
    # resolve to that relay and recursively import an empty module.  Prefer the
    # package's actual script directory explicitly.
    script_directory = os.path.join(
        rospkg.RosPack().get_path("hugmy"), "script")
    if script_directory in sys.path:
        sys.path.remove(script_directory)
    sys.path.insert(0, script_directory)

    # SB3 imports torch.utils.tensorboard even when no TensorBoard log directory
    # is requested.  Ubuntu 20.04's apt pyOpenSSL can be incompatible with a
    # newer user-installed cryptography package pulled in through TensorBoard;
    # that optional integration then raises AttributeError instead of the
    # ImportError that SB3 handles.  Probe it first and mask only TensorBoard
    # when it is broken, allowing SAC itself to run normally.
    tensorboard_available = True
    tensorboard_error = ""
    try:
        from torch.utils.tensorboard import SummaryWriter as _SummaryWriter  # noqa: F401
    except Exception as exc:  # Optional dependency may fail below its import root.
        tensorboard_available = False
        tensorboard_error = f"{type(exc).__name__}: {exc}"
        for module_name in list(sys.modules):
            if (module_name == "tensorboard" or
                    module_name.startswith("tensorboard.") or
                    module_name == "torch.utils.tensorboard" or
                    module_name.startswith("torch.utils.tensorboard.")):
                del sys.modules[module_name]
        sys.modules["tensorboard"] = None
    try:
        from gymnasium.utils.env_checker import check_env
        from stable_baselines3 import SAC
        from stable_baselines3.common.callbacks import CheckpointCallback
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        from hugmy_gym_env import HugmyMujocoEnv
    except ImportError as exc:
        raise SystemExit(
            "Gym dependencies are missing. Install robots/hugmy/requirements-rl.txt "
            "in a virtualenv created with --system-site-packages.\n" + str(exc))

    configuration = load_configuration(args.config)
    stages = configuration.get("curriculum", {}).get("stages", [])
    if not stages:
        stages = [{"name": "single", "timesteps": 100000, "parameters": {}}]
    stop_stage = len(stages) - 1 if args.stop_stage is None else args.stop_stage
    if not 0 <= args.start_stage <= stop_stage < len(stages):
        raise SystemExit("Invalid curriculum stage range")

    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "resolved_config.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(configuration, stream, sort_keys=False)

    initial_stage = args.start_stage
    raw_env = HugmyMujocoEnv(**environment_arguments(configuration, initial_stage))
    if args.check_only:
        # ROS callback timing and MuJoCo sensor noise are intentionally not
        # bitwise deterministic across two physical resets, even with the
        # policy/domain RNG seeded. Tell Gymnasium to check API/space validity
        # without imposing that inappropriate deterministic-step assertion.
        raw_env.spec = SimpleNamespace(nondeterministic=True)
        check_env(
            raw_env, warn=True, skip_render_check=True,
            skip_close_check=True)
        raw_env.close()
        print("Gymnasium environment check passed")
        return

    vec_env = DummyVecEnv([
        lambda: Monitor(
            raw_env, filename=str(output / "monitor.csv"),
            info_keywords=(
                "target_direction", "total_progress_m", "stride_progress_m",
                "completed_stride_count", "mean_completed_stride_m",
                "stride_success", "stride_overshoot", "stride_runaway",
                "fallen", "detached"))
    ])
    training = dict(configuration.get("training", {}))
    normalize_observation = bool(training.pop("normalize_observation", True))
    normalize_reward = bool(training.pop("normalize_reward", True))
    checkpoint_frequency = int(training.pop("checkpoint_frequency", 25000))
    configured_device = training.pop("device", "auto")
    device = args.device if args.device is not None else configured_device
    verbose = int(training.pop("verbose", 1))
    policy = training.pop("policy", "MlpPolicy")
    tensorboard_log = None
    if tensorboard_available:
        tensorboard_log = str(output / "tensorboard")
    else:
        print(
            "WARNING: TensorBoard is unavailable; training will continue "
            f"without TensorBoard logging ({tensorboard_error}).")

    if args.vec_normalize:
        vec_env = VecNormalize.load(args.vec_normalize, vec_env)
        vec_env.training = True
        vec_env.norm_reward = normalize_reward
    else:
        vec_env = VecNormalize(
            vec_env, norm_obs=normalize_observation,
            norm_reward=normalize_reward, clip_obs=10.0)

    if args.resume:
        model = SAC.load(args.resume, env=vec_env, device=device)
        if args.replay_buffer:
            model.load_replay_buffer(args.replay_buffer)
    else:
        model = SAC(
            policy, vec_env, seed=args.seed, verbose=verbose, device=device,
            tensorboard_log=tensorboard_log, **training)

    checkpoint = CheckpointCallback(
        save_freq=max(1, checkpoint_frequency), save_path=str(output / "checkpoints"),
        name_prefix="hugmy_sac", save_replay_buffer=True,
        save_vecnormalize=True)
    remaining = args.total_timesteps
    try:
        for stage_index in range(args.start_stage, stop_stage + 1):
            requested = int(stages[stage_index].get("timesteps", 100000))
            stage_steps = requested if remaining is None else min(requested, remaining)
            if stage_steps <= 0:
                break
            raw_env.set_curriculum_stage(stage_index)
            stage_name = stages[stage_index].get("name", f"stage_{stage_index + 1}")
            print(f"curriculum {stage_index + 1}/{len(stages)} ({stage_name}): "
                  f"{stage_steps} steps")
            model.learn(
                total_timesteps=stage_steps, callback=checkpoint,
                reset_num_timesteps=False, tb_log_name="SAC")
            model.save(str(output / f"model_stage_{stage_index + 1}"))
            vec_env.save(str(output / f"vecnormalize_stage_{stage_index + 1}.pkl"))
            if remaining is not None:
                remaining -= stage_steps
    finally:
        # A CUDA launch failure poisons the CUDA context, so even copying model
        # tensors to CPU for serialization can fail.  Save each artifact
        # independently and atomically: a partial save must never replace the
        # last valid checkpoint or masquerade as a usable model_final.zip.
        training_failed = sys.exc_info()[0] is not None
        save_errors = []
        final_artifacts = (
            (model.save, output / "model_final.zip"),
            (model.save_replay_buffer, output / "replay_buffer_final.pkl"),
            (vec_env.save, output / "vecnormalize_final.pkl"),
        )
        for save_function, destination in final_artifacts:
            try:
                save_atomically(save_function, destination)
            except Exception as exc:
                save_errors.append((destination, exc))
                print(
                    f"WARNING: could not save {destination}: "
                    f"{type(exc).__name__}: {exc}", file=sys.stderr)
        vec_env.close()
        if save_errors and not training_failed:
            raise save_errors[0][1]
    print(f"saved trained policy to {output / 'model_final.zip'}")


if __name__ == "__main__":
    main()
