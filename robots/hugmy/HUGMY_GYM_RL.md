# Hugmy Gymnasium / Stable-Baselines3 environment

This environment keeps the existing ROS controllers and Hugmy MuJoCo hardware
plugin in the loop:

```
observation -> SAC policy -> pressure / thrust / bottom command -> MuJoCo
```

## Install the Python learning dependencies

ROS Noetic's `rospy` must remain visible in the virtual environment:

```bash
python3 -m venv --system-site-packages ~/venvs/hugmy_rl
source ~/venvs/hugmy_rl/bin/activate
pip install -r ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hugmy/requirements-rl.txt
```

TensorBoard is optional and is deliberately not installed by the requirements
file. The trainer uses it when its import is healthy and otherwise continues
with Monitor CSV and checkpoint logging only. This avoids mixing Ubuntu 20.04's
apt pyOpenSSL packages with incompatible user-installed cryptography versions.

Build and source the workspace after changing the plugin:

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build hugmy
source devel/setup.bash
```

## Start the plant and check the environment

Terminal 1:

```bash
roslaunch hugmy hugmy_gym_mujoco.launch gui:=false
```

Terminal 2 (with the virtual environment and workspace sourced):

```bash
rosrun hugmy train_hugmy_sac.py --check-only
```

The Gym launch enables a simulation gate. Headless MuJoCo therefore advances
one 0.1 s policy interval only after `env.step(action)`, instead of running far
ahead of the Python process. Do not run two Gym environments against the same
ROS namespace; the current implementation intentionally uses one MuJoCo plant
and `DummyVecEnv`.

For training only, the launch uses a 2 ms physics timestep instead of the
normal 1 ms timestep. This roughly halves the ROS/controller updates per policy
step while retaining two samples across the arm-stop model's 4 ms time
constant. Values above 2 ms are capped for contact and joint-limit stability.

## Action and observation

The normalized five-dimensional action is
`[P_front, P_rear, T_front, T_rear, u_bottom]` in `[-1, 1]`.

- Arm pressure maps linearly from 0 to 50 kPa.
- Positive thrust maps to 0 to 8 N; negative thrust maps to 0 to -3 N.
- `abs(u_bottom)` sets 0 to 30 kPa and its sign selects the rocking direction.
- Front is arms 1/4 and rear is arms 2/3, matching the +x travel direction.

The 31 observations, in order, are progress velocity, pitch and pitch rate,
roll and roll rate, four exact joint-sum arm bend angles, four measured
pressures, four commanded thrusts, four contact flags, four normal forces,
four tangential forces, bottom pressure, and the target direction sign. A bend
angle of zero means straight; a larger positive value means more flexion.

## Reward and termination

The first-stage reward is deliberately small:

```
r = 1000 * delta_forward
    - 2 * max(0, abs(roll)  - 15 deg)^2
    - 2 * max(0, abs(pitch) - 30 deg)^2
    - 20 * I[failure]
    - 0.02 * abs(u_bottom)
    - 0.10 * I[bottom direction switched]
    - 0.02 * sum_i(abs(T_i))
    - 0.05 * sum_i(abs(T_i - T_i_previous))
```

Thus 1 mm of whole-body CoM progress gives +1. Pitch inside 30 degrees is not
penalized, so rocking with the bottom inflatable remains available. Failure is
all four contacts being lost for 0.2 s, excessive attitude, or a large vertical
departure. Merely moving a released front arm does not earn progress reward.
Thrust usage and thrust changes are accumulated per rotor at every control
step; there is no instantaneous combined-thrust cap. For example, holding two
rotors at 6 N costs 0.24 per step, while changing both from 0 to 6 N costs an
additional 0.60 once. Full bottom actuation costs only 0.02 per step.

## Train and evaluate

The default YAML runs five curriculum stages: easy fixed cylinder, friction,
radius, cylinder angle, then actuator/sensor randomization.

```bash
rosrun hugmy train_hugmy_sac.py \
  --output /tmp/hugmy_sac_run \
  --total-timesteps 700000
```

For a short pipeline test, use for example `--total-timesteps 1000`. Continue
from saved state with both the model and observation statistics:

```bash
rosrun hugmy train_hugmy_sac.py \
  --output /tmp/hugmy_sac_resume \
  --resume /tmp/hugmy_sac_run/model_final.zip \
  --replay-buffer /tmp/hugmy_sac_run/checkpoints/hugmy_sac_replay_buffer_75000_steps.pkl \
  --vec-normalize /tmp/hugmy_sac_run/vecnormalize_final.pkl
```

Evaluate deterministically (launch with `gui:=true` if visual inspection is
needed):

```bash
rosrun hugmy evaluate_hugmy_sac.py \
  --model /tmp/hugmy_sac_run/model_final.zip \
  --vec-normalize /tmp/hugmy_sac_run/vecnormalize_final.pkl \
  --episodes 10
```

Summarize an interrupted or completed run and write a reward/episode-length
plot without TensorBoard:

```bash
rosrun hugmy analyze_hugmy_sac.py \
  --output /tmp/hugmy_sac_run --plot
```

The action limits, reward weights, SAC parameters, and curriculum ranges are
all in `config/hugmy_gym.yaml`. Slip, excessive contact force, and energy terms
are intentionally not active yet; add those only after forward gait discovery
is reproducible.
