# Four-arm inchworm gait optimization in MuJoCo

This experiment optimizes the existing four-arm Action-A locomotion sequence:

1. establish a four-arm grasp;
2. unload, extend, and re-anchor each of the two leading arms;
3. unload the two trailing arms and pull the body toward the leading anchors;
4. restore the four-arm grasp.

The default travel direction is `+X`, along the long axis of the artificial
forearm. One candidate is evaluated in multiple randomized arm/contact and
pneumatic domains. Each episode starts a fresh ROS master and MuJoCo process,
so model state, simulated time, pressures, and controller integrators are reset
between candidates.

## Build and smoke test

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build aerial_robot_simulation hugmy
source devel/setup.bash

# One real MuJoCo episode, one physical domain
rosrun hugmy optimize_inchworm_gait.py \
  --trials 1 --domain-samples 1 \
  --output /tmp/hugmy_inchworm_smoke
```

## Optimize

```bash
rosrun hugmy optimize_inchworm_gait.py \
  --output /tmp/hugmy_inchworm_run
```

The optimizer starts from the configured cycle-completing reference, fills the
remaining initial design with Latin-hypercube samples, then uses a Matérn-5/2
Gaussian process with Expected Improvement. It depends only on the NumPy and
SciPy already available in the workspace; Ax/PyTorch is not required.

Results are written incrementally:

- `trials.csv`: every parameter set, reward, domain rewards, and metrics;
- `best_gait.json`: best parameters so far;
- `trial_N/domain_M/roslaunch.log`: simulator/controller log;
- `trial_N/domain_M/episode.log`: evaluator log.

Interrupted optimization can continue with `--resume`.

```bash
rosrun hugmy optimize_inchworm_gait.py \
  --output /tmp/hugmy_inchworm_run --resume
```

Replay the best candidate with the MuJoCo GUI:

```bash
rosrun hugmy optimize_inchworm_gait.py \
  --replay /tmp/hugmy_inchworm_run/best_gait.json --gui
```

## Objective and signals

Positive progress along the commanded arm direction is rewarded. Lateral and
vertical motion, maximum roll/pitch, fewer than two active anchors, slip
above the friction limit, pump effort, and episode timeout are penalized. The
weights, search bounds, direction, distance, number of domains, and randomized
physical ranges are in `config/inchworm_optimization.yaml`.

The reach target is expressed as straightening relative to the grasp pose,
rather than an absolute arm angle. This makes the same candidate meaningful
when forearm radius and the initial contact geometry are randomized. The normal
controller keeps its existing absolute-angle behavior unless this simulation
parameter is explicitly positive.

The workspace currently uses a Debug build, so MuJoCo can advance slower than
wall time and the default robust run can take hours. Use the one-episode smoke
test above before starting the full run.

The MuJoCo pneumatic plugin publishes
`/quadrotor/pneumatic/grip_state` as a 12-element `Float32MultiArray`:

```text
[active arm1..4, slip_ratio arm1..4, grip_force_N arm1..4]
```

A slip ratio above 1 means the requested tangential grip force exceeded the
friction limit.

## Safety boundary

This optimizer is intended for simulation and a rigid dummy forearm. Do not
apply an optimized trajectory directly to a person. Validate the best
candidates on a rigid instrumented cylinder with conservative pressure/thrust
limits before any human-subject procedure.
