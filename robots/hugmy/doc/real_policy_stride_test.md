# Learned-policy stride test on the real Hugmy

This procedure does **not** train SAC on hardware.  It freezes the learned
policy, exports one successful MuJoCo stride as physical commands, and tunes
six bounded sim-to-real scale factors on a secured dummy cylinder.

## 1. Export one deterministic stride

Start the synchronous MuJoCo plant:

```bash
roslaunch hugmy hugmy_gym_mujoco.launch gui:=false
```

In a second terminal (use the same SB3 version as training):

```bash
rosrun hugmy export_hugmy_stride.py \
  --model /path/to/model_stage_1.zip \
  --vec-normalize /path/to/vecnormalize_stage_1.pkl \
  --direction forward \
  --output /tmp/hugmy_forward_stride.yaml
```

The YAML records four arm pressures, four signed thrusts, bottom pressure, and
expected MuJoCo state (including all four arm bend angles) for the first
completed stride.

Replay the exported open-loop trajectory in MuJoCo before hardware testing:

```bash
rosrun hugmy replay_hugmy_stride_mujoco.py \
  --trajectory /tmp/hugmy_forward_stride.yaml \
  --trace-csv /tmp/hugmy_forward_stride_replay.csv
```

## 2. Verify shadow mode on the real ROS graph

Start the normal real-machine bridge/bringup and pressure controller.  Stop
any other node publishing `/pwm_test`, either pressure-target topic, or the
bottom-target topic; the runner rejects competing command publishers. Then run:

```bash
roslaunch hugmy hugmy_real_stride.launch \
  trajectory:=/tmp/hugmy_forward_stride.yaml
```

This is shadow mode: it publishes intended physical commands only on
`/quadrotor/hugmy_real_stride_runner/command`.  It never writes pressure or
PWM outputs.  Confirm fresh `/mocap/pose`, `/imu`, `/neuron/adc_states`,
`/neuron/imu_states`, and `/pressure/bottom`, and inspect the debug command
with rqt_plot or `rostopic echo`.

Arm and execute exactly one shadow stride:

```bash
rosservice call /quadrotor/hugmy_real_stride_runner/arm true
rosservice call /quadrotor/hugmy_real_stride_runner/run_once
```

## 3. One secured physical stride

Use a rigid dummy cylinder, mechanical tether, rotor guard, hardware E-stop,
and a clear area.  Physical output requires both explicit launch flags:

```bash
roslaunch hugmy hugmy_real_stride.launch \
  trajectory:=/tmp/hugmy_forward_stride.yaml \
  shadow:=false allow_actuation:=true
```

Each `arm true` is consumed by one `run_once`.  The stop service is:

```bash
rosservice call /quadrotor/hugmy_real_stride_runner/stop
```

The runner neutralizes thrust on completion/abort, sends an empty `PwmTest` to
leave direct test mode, and keeps the configured 30 kPa grasp target.  It
aborts for stale feedback, more than 100 mm progress, excessive lateral or
vertical motion, excessive relative tilt, excessive measured pressure, or a
60 N s total absolute-thrust integral.

The real runner does not replay samples on a fixed wall clock.  It linearly
interpolates the learned pressure and thrust path on one virtual clock, then
slows that clock when either the configured representative arm pressures
(``pressure_feedback/arm_indices``, currently zero-based joints 0 and 1) or the
measured bottom pressure lag their targets.  It also compares the four Neuron
IMU bend measurements with ``expected_arm_bend_rad`` in the exported MuJoCo
trajectory.  The learned pressure/thrust samples are feed-forward; bounded
pressure and thrust trims correct the residual bend error.  A per-stride
initial bias cancels fixed Neuron mounting-frame offsets, so only the learned
bend change is followed.  All four arms remain covered by the pressure safety
abort.  A nonzero minimum playback rate prevents an unreachable target from
holding thrust forever.  A smooth initial pressure lead-in is performed at
zero thrust before learned playback starts.
Inspect the adaptation with:

```bash
rostopic echo /quadrotor/hugmy_real_stride_runner/playback_feedback
```

The eight array entries are wall time, virtual trajectory time, playback rate,
maximum reference-arm pressure error, bottom-pressure error, maximum all-arm
pressure error, maximum reference-arm bend error, and maximum all-arm bend
error.  Angles are in radians.  The separate ``~angle_feedback`` array contains
four expected angles, four measured angles, four errors, four pressure trims,
and four thrust trims.  The final result records the same maxima and the
per-stride angle bias.

## 4. Tune only the sim-to-real scales

First validate the optimizer without commands:

```bash
rosrun hugmy optimize_hugmy_real_stride.py \
  --output /tmp/hugmy_real_tuning
```

Then perform operator-confirmed trials:

```bash
rosrun hugmy optimize_hugmy_real_stride.py \
  --output /tmp/hugmy_real_tuning --execute
```

Before every trial the program prints the six candidate scales and requires
the exact text `RUN`.  It optimizes front/rear pressure, positive/negative
thrust, bottom pressure, and playback time.  Progress comes from mocap; thrust
integral and thrust changes are penalized more heavily than bottom use.  Use
`--resume` after interruption.  The current best is written to
`best_scales.yaml`.
