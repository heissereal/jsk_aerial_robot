# Whole-body rocking gait optimization in MuJoCo

This experiment implements alternating-pivot locomotion on a rigid cylindrical
forearm. It is separate from the arm-reaching Action A/B controllers: the
inflatable contact and the whole vehicle move, rather than commanding an arm
trajectory through free space.

One cycle is:

1. keep all four arms pressurized and inflate bottom at the front-side
   application point until the signed rearward/nose-up preload is reached;
2. with bottom held, pressurize the front pair to a measured 120-deg wind-up;
3. begin front SWING while keeping bottom inflated. SWING is explicitly split
   into `RELEASE` (exhaust the front pair with zero thrust until both contacts
   release), `EXTEND` (5--8 N forward thrust to a measured near-straight
   angle), `PLACE` (start bottom exhaust, slowly reduce thrust, and add front
   pressure), and `REGRIP`;
4. keep the front pair anchored and exhaust the rear pair, with bottom kept
   exhausted;
5. apply reverse thrust only to the rear rotor pair to change the rear contact
   point; finish this phase when either the intended rocking angle or a directly
   measured 3 mm forward body/rear-tip recovery is obtained;
6. pressurize the rear pair, return all rotors to neutral, and settle in the
   normal four-arm grasp with bottom still exhausted.

States 19 through 24 on
`/quadrotor/attitude_pressure_controller/state` correspond to those six
phases. Every timeout or the 30-degree attitude safety limit cancels thrust,
exhausts the bottom section, and returns to the normal four-arm grasp.

## Build and one-episode test

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build aerial_robot_simulation hugmy
source devel/setup.bash

rosrun hugmy optimize_rocking_gait.py \
  --trials 1 --domain-samples 1 \
  --output /tmp/hugmy_rocking_smoke
```

The artificial forearm is a finite cylinder (local x axis), not a box. The
50 kPa calibration maps to 15 deg and the preload transition is set to 90% of
that pressure-derived angle so a small compliant settling error cannot turn an
achievable posture into a timeout. This is a model/state-sequence check, not an
optimized gait result.

To inspect a trial with the MuJoCo GUI:

```bash
rosrun hugmy optimize_rocking_gait.py \
  --replay /tmp/hugmy_rocking_smoke/best_gait.json --gui
```

## Optimize and resume

```bash
rosrun hugmy optimize_rocking_gait.py \
  --output /tmp/hugmy_rocking_run

rosrun hugmy optimize_rocking_gait.py \
  --output /tmp/hugmy_rocking_run --resume
```

Results produced by an earlier configuration must not be resumed: the motion
parameters, success criterion, domain count, and reward aggregation have
changed. Keep the old directory for comparison and start this version in a
new output directory.

The default run starts with Latin-hypercube samples and then uses a Matérn-5/2
Gaussian process with Expected Improvement. Each candidate runs in fresh ROS
and MuJoCo processes and is evaluated in three randomized physical domains.
The candidate score is the reward of its worst domain, not the mean. A cycle
is successful only if all gait phases finish and at least 0.5 mm of forward
displacement remains after regripping. Lateral and vertical displacement are
allowed and receive no penalty; loss of anchors, slip, pump use, backward
motion, and an incomplete/non-forward cycle are penalized. The 30-degree state
machine limit still provides attitude safety.

Gait bounds, the seed, reward weights, and physical randomization are in
`config/rocking_optimization.yaml`. The front wind-up is made by chamber
pressure; locomotion thrust comes from the front extension trajectory and rear
reverse rotor thrust. Bottom
pressure is only a posture-bias variable:
its tilt transition is derived automatically from the calibrated pressure-angle
relation and is no longer independently optimized. Other variables include the
rear release pressure, wind-up pressure, extension thrust/angle,
placement bend/pressure, front lowering rate, recovery angle, timeout, and
thrust ramp. Results are written to:

- `trials.csv`: candidate parameters, rewards, and metrics;
- `best_gait.json`: best candidate so far;
- `trial_N/domain_M/episode.log`: phase and objective result;
- `trial_N/domain_M/roslaunch.log`: complete ROS/MuJoCo log.

The workspace uses Python 3.8, so this implementation uses the available
NumPy/SciPy GP instead of Ax 1.3, which requires Python 3.11 or newer.

## Bottom inflatable model and I/O

The MuJoCo pneumatic plugin integrates bottom pressure from the existing
`bottom_supply_pwm` and `bottom_exhaust_pwm` fields and publishes
`/quadrotor/pneumatic/bottom_pressure`. Bottom supply/exhaust rates are separate
from the arm chambers (defaults: 100/80 kPa/s versus 25/18 kPa/s). In addition
to the pressure-proportional contact force, a virtual compliant attitude servo
maps 50 kPa to 15 deg of rocking. This removes the former 2--3 deg saturation
caused by contact/friction variation without directly translating the vehicle.
The translucent `hugmy_bottom_inflatable` geom shows its active position and
inflation.

`/quadrotor/pneumatic/bottom_state` contains pressure, nominal contact force,
application point xyz, virtual target angle, measured relative angle, and
virtual torque in indices 0 through 7.

`/quadrotor/pneumatic/bottom_force_direction` selects the application side
relative to travel. For a +body-x command, this gait selects the front-side
application point so positive bottom pressure produces negative pitch, i.e.
the requested rearward/nose-up preload. The same side selection is retained
while pressure is exhausted; rear-contact recovery is produced by the rear
rotor pair.

The current `spinal/PneumaticCommand` has only one bottom supply/exhaust
channel. Therefore MuJoCo models one controlled section whose effective side
is selected by the gait direction. If the real vehicle has independent front/rear
plumbing, add separate command and pressure fields before deploying this gait
to hardware.

## Cylinder contact telemetry

`/quadrotor/pneumatic/grip_state` preserves its original first 12 values and
adds cylinder-surface coordinates for trajectory design:

- indices 0--3: grip active flag for arms 1--4;
- indices 4--7: requested tangential friction / Coulomb limit (slip ratio);
- indices 8--11: applied grip-force magnitude [N];
- indices 12--15: contact coordinate along the cylinder axis [m];
- indices 16--19: contact azimuth around the cylinder [rad];
- indices 20--23: nonnegative distance to the cylindrical side/rim [m].
- indices 24--27: MuJoCo articulated joint-sum bend angle [rad].

The controller uses these values with the deliberate
wind-up/extension/re-flexion SWING trajectory to place the front pair farther
forward, rather than treating an accidental thrust response as the trajectory.

The controller now uses those values during `PLACE`: both front contacts must
be active and their mean cylinder-axis coordinate must be at least 3 mm ahead
of the pre-release contact. A time-based thrust return alone is not accepted as
a landing. On hardware, where this MuJoCo telemetry is absent, the configured
bend-angle and pressure target is used as a conservative fallback.

## Safety boundary

This is a simulation optimizer for a rigid dummy cylinder. Do not copy its
pressure or reverse-thrust result directly to a person. First identify the real
bottom force/pressure curve and valve layout, then validate a conservatively
limited trajectory on an instrumented rigid cylinder.
