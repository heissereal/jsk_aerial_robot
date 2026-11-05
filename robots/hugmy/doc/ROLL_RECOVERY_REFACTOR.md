# Hugmy roll recovery: staged model refactor

This change introduces an independently testable **roll-recovery planner**
(`include/hugmy/control/roll_recovery_planner.h`) and a diagnostic ROS adapter
(`src/control/roll_recovery_planner_adapter.cpp`).  It **does not replace** the
existing action-C recovery actuator/state machine.  No motor or valve behavior
is changed by default.

## Why the old candidate signs are hard to trust

1. `recoveryContactPivot()` ignores which pair/mode is being evaluated and
   uses the one event-locked pivot for every case.
2. The circle centre is fitted from body-frame forward kinematics.  Its body
   coordinates must move as Hugmy rolls, even for a world-fixed cylinder.
3. The old released candidate is evaluated at `bend=0` before any pressure
   change.  In reality rotor pose is `r_i(alpha_i)` and the calibrated
   equilibrium bend is `alpha_i=f_i(P_i,T_i)`.
4. The legacy emergency extension latch and grasp-first priority can override
   the result of the signed-moment comparison.

It is **not** valid to assume that a circle centre or the arithmetic mean of
several contact points is a reaction-free pivot under general frictional
multi-contact.  The new support estimate is a *testable hypothesis*: the
opposite pair stays in contact while one pair is driven.  Compare the model's
torque sign with actual roll acceleration before using it for real actuation.

## Lightweight dynamics and candidate scoring

For candidate `k` (pair 0+1 or 2+3, grasp or release) with support estimate
`C_k` and rotor unit directions `u_i(alpha_i)`:

```
b_k = sum_i ((r_i(alpha_i) - C_k) x u_i(alpha_i))_x
tau_k = b_k * T                   # common per-rotor thrust
```

This is the **one-degree-of-freedom roll Jacobian transpose**.  Grasped
candidates use measured angles.  For released candidates the planner samples
one thrust `T` and evaluates `alpha_i=f_i(P_release,T)` from the existing
pressure/thrust hysteresis table.  It chooses the smallest `T` satisfying the
signed restoring moment and thrust limit, avoiding an artificial sum of
"extension thrust" and "roll thrust" from the *same* rotors.

The comparison also exposes a soft release penalty, estimated transition
time, and selection hysteresis.  These are **not** contact-force estimates.

Emergency risk is separately evaluated from outward roll rate and the time
remaining to an unsafe angle.  In particular, a large angle with an inward
rate is not by itself an emergency trigger.  The existing safety latch remains
unchanged until the new model is calibrated and deliberately enabled.

## Diagnostic use

In `config/attitude_pressure_controller.yaml`:

```yaml
recovery_unified_shadow_enabled: true
recovery_unsafe_roll_rad: 0.7853981633974483 # 45 deg, illustrative only
recovery_extension_response_sec: 0.5       # calibrate from real inflation/vent logs
```

Search ROS logs for `Unified recovery SHADOW`.  The four indices are:

| Index | Candidate |
| --- | --- |
| 0 | 0+1 grasped |
| 1 | 0+1 released |
| 2 | 2+3 grasped |
| 3 | 2+3 released |

For each trial, compare the four **signed** levers and required thrusts with
the legacy selected pair, measured roll/error rate, pressure, and the eventual
roll acceleration after pneumatic transients.  Include both a ~80-mm
forearm-like cylinder and a larger cylinder.  Do **not** infer correct sign
from a static no-motion probe; stiction may prevent measurable acceleration.

For a static cylinder the adapter locks its centre in the mocap world frame
and transforms it into the changing body frame when a fresh mocap pose is
available.  The pose and FK frames must be calibrated consistently.  If pose
is missing, a diagnostic warning identifies the body-frozen fallback.
This approximation does not track a moving human forearm.

## Tests (no ROS installation required)

```bash
python3 robots/hugmy/test/test_roll_recovery_planner.py
python3 robots/hugmy/test/test_roll_recovery_adapter.py
python3 robots/hugmy/test/test_recovery_geometry.py
python3 robots/hugmy/test/test_action_c_resume.py
```

The standalone tests cover sign, frame rotation, 4-way mode selection,
thrust/arm-shape coupling, hysteresis and emergency risk; existing controller
regression tests cover the unchanged actuator path.

## Required before driving hardware with the new selector

1. Confirm mocap quaternion vs robot-body FK orientation, including any
   marker-to-body fixed offset; calibrate or reject ambiguous frames.
2. Check the assumed supporting pair and instant centre of rotation against
   mocap roll/body-velocity data.  For a nonzero roll rate about an x-axis:
   `C_y = G_y - v_G,z / omega_x`, `C_z = G_z + v_G,y / omega_x`.
3. Verify released-arm **transient** bend/pressure dynamics.  The current
   lookup is a *steady-state* prediction, not a 0.5-s motion trajectory.
4. Integrate the experimentally validated planner output as an optional
   actuator selector, retaining the established safety and measured-response
   monitors.  Do not silently delete the hard-stop or pressure-handoff paths.
5. Extract the remaining recovery state machine into its own controller with
   `start/update/stop/status` API.  Both standalone recovery and locomotion's
   recovery handoff should call that API; only the owner may command actuators
   at a time.

Those last two items are *intentionally not activated by this patch*.
