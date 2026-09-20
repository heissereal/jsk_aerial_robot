# Hugmy in-flight Quad/Bi controller

`aerial_robot_control/hugmy_quad_bi_controller` derives from
`UnderActuatedLQIController`. The parent controller continues flight control,
and its wrench allocation and inertia compensation use the current articulated
model. The derived layer manages the shape transition and pneumatic targets.
The current articulated model updates wrench allocation and inertia
compensation continuously. The takeoff LQI gains are held during deformation,
because the existing CARE implementation is not numerically robust for every
intermediate compliant-arm geometry.

The Bi shape follows `FOLDED_ARMS = (2, 4)` from
`optimize_hugmy_fixed_tilt_gravity_hanging_v2.py`: arms 2 and 4 bend to the
configured angle while arms 1 and 3 remain straight. All four rotors remain
active in both modes.

## Start

For MuJoCo simulation:

```bash
roslaunch hugmy bringup.launch \
  simulation:=true real_machine:=false mujoco:=true \
  fixed_arms:=false quad_bi_controller:=true headless:=false
```

`quad_bi_controller:=true` loads `config/QuadBiControl.yaml` and starts the
independent arm pressure controller. `headless:=false` starts both the MuJoCo
GUI and RViz; the `bringup.launch` default is `headless:=true`.

## Switch mode

The request is accepted only while hovering, with fresh bend feedback, small
roll/pitch, and a small body angular rate.

```bash
# Quad -> Bi
rosservice call /quadrotor/controller/set_bi_mode "data: true"

# Bi -> Quad
rosservice call /quadrotor/controller/set_bi_mode "data: false"
```

State is published on `/quadrotor/controller/quad_bi/state`:

- `0`: `QUAD`
- `1`: `TO_BI`
- `2`: `BI`
- `3`: `TO_QUAD`
- `4`: `ERROR`

The target bend, measured bend, and pressure target are available under
`/quadrotor/controller/quad_bi/`. During a transition, the bend target is
slew-limited and converted to pressure by inverting the measured
pressure/thrust/bend table. The hover PID state is retained across the mode
request, and each rotor's geometry-aware thrust allocation is slew-limited by
`transition_thrust_rate_n_s`. The Quad hover trim is blended toward the robust
fixed-tilt optimizer's Bi trim using `bi_straight_arm_thrust_scale` for arms 1
and 3 and `bi_bent_arm_thrust_scale` for arms 2 and 4, while changes from the
altitude PID remain active. During Quad-to-Bi the bending pair is temporarily
unloaded farther so follower force cannot hold the compliant arms straight;
its optimized Bi thrust is restored in proportion to measured bend feedback.
Thus arms 1 and 3 are loaded while arms 2 and 4 are unloaded without first
reducing all four rotor commands. If bend feedback or the entry flight
condition remains unsafe, Quad-to-Bi is automatically reversed. In `TO_BI`
and `BI`, excessive target-height error, roll/pitch, or angular rate sustained
for `recovery_trigger_duration_s` also starts `TO_QUAD`. Quad recovery
continues even while the aircraft is tilted or bend feedback is temporarily
stale, and a manual `data: false` request remains available under those same
conditions. `ERROR` is reserved for a Quad recovery that itself times out.

Normal fixed-rotor flight clamps the final mixed thrust of every rotor to
0 N or above. This clamp is after roll/pitch/yaw mixing, so an attitude
correction cannot select reverse thrust. Direct `pwm_test` bypasses the flight
conversion and retains both directions; in MuJoCo it temporarily selects the
PWM motor model even though normal flight uses ideal force input.

Before a real flight, check `measured_bend_deg` in the straight pose and set
`neuron_bend_offset_deg` in `QuadBiControl.yaml` if required. The nominal
runtime model distributes the measured total bend equally over joints 1--3;
the offline optimization should continue to evaluate uncertainty in this
distribution.
