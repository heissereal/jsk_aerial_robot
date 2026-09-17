# Hugmy MuJoCo pneumatic simulation

Build the generated articulated MJCF and the Hugmy hardware plugin:

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build aerial_robot_simulation hugmy
source devel/setup.bash
```

Start the standard simulation with pneumatic dynamics enabled:

```bash
roslaunch hugmy bringup.launch \
  simulation:=true real_machine:=false mujoco:=true fixed_arms:=false
```

The plugin consumes the same ROS interfaces as the real pneumatic path:

- `/quadrotor/pneumatic/command` (`spinal/PneumaticCommand`)
- `/quadrotor/target_thrust` (`spinal/Thrust`)

It publishes:

- `/quadrotor/neuron/adc_states` (`spinal/NeuronAdcStates`)
- `/quadrotor/neuron/imu_states` (`spinal/NeuronImuStates`)
- `/quadrotor/joint_states` (`sensor_msgs/JointState`)
- `/quadrotor/pneumatic/grip_force_markers` (`visualization_msgs/MarkerArray`)
- `/quadrotor/pneumatic/grip_state` (`std_msgs/Float32MultiArray`)
- `/quadrotor/pneumatic/bottom_pressure` (`std_msgs/Float32`)
- `/quadrotor/pneumatic/bottom_state` (`std_msgs/Float32MultiArray`)
- `/quadrotor/pneumatic/reset_count` (`std_msgs/UInt32`, latched)
- `/quadrotor/pneumatic/step_count` (`std_msgs/UInt32`, latched)

It provides `/quadrotor/pneumatic/reset_simulation` for an episode reset. With
`simulation/pneumatic/gym_step_mode:=true`,
`/quadrotor/pneumatic/step_simulation` advances one configured policy interval;
normal launches leave this gate disabled. See `HUGMY_GYM_RL.md`.

For the whole-body gait it also consumes
`/quadrotor/pneumatic/bottom_force_direction`. The bottom pressure produces a
surface-normal force at an offset point on the body plus a virtual compliant
rocking torque. The default calibration is 15 deg at 50 kPa; it supplies a
posture bias, while the arm rotors remain the gait's locomotion actuators.
For the forward gait, inflation first creates a signed rearward/nose-up preload.
That pressure is held while the front pair releases and extends. The bottom
command switches to exhaust when both front arms reach their measured
near-straight target and `PLACE` begins, then remains exhausted for the rest of
the cycle.
Bottom inflation and exhaust have independent 100/80 kPa/s defaults, so they
do not inherit the slower arm-chamber rates. See
`ROCKING_GAIT_OPTIMIZATION.md`.

The first five values of `/quadrotor/pneumatic/bottom_state` retain their old
meaning (pressure, nominal force, and local application point xyz). Values 5,
6, and 7 are the virtual tilt target [rad], measured relative tilt [rad], and
applied virtual torque [N m].

The fixed target is a finite cylinder aligned with its local x axis. Grip and
bottom contact are computed against its cylindrical side (with a finite-length
rim), rather than an enclosing box. `/quadrotor/pneumatic/grip_state` keeps the legacy
active/slip/force values in indices 0--11 and adds, per arm, cylinder-axis
coordinate in 12--15, azimuth in 16--19, and surface gap in 20--23.
The MuJoCo joint-sum bend angle [rad] is appended in 24--27; the rocking
controller uses it in simulation and keeps Neuron IMU angle as the hardware
fallback. Per-arm normal forces [N] are in 28--31 and tangential forces [N] in
32--35.

Use the convenience launch file to enable the pressure controller and the
fixed human-arm target contained in the MJCF:

```bash
roslaunch hugmy pneumatic_perching_sim.launch
```

The pressure-dependent equilibrium bend angle is interpolated from the
measured pressure/thrust table. MuJoCo native passive stiffness and damping are
used for numerical stability of the light cable-carrier links.  The zero-angle
link-to-link mechanical stops are represented by stiff unilateral MuJoCo joint
limits, so rotor thrust cannot fold the carrier links through the straight
pose.

The spring model reads the actual bidirectional PWM rotor force, including
`PwmTest`, so the simulated pressure/thrust equilibrium follows the measured
bend table during gait pulses. Residual pressure below 1.5 kPa is treated as
an open grip; otherwise the pressure controller's approximately 1 kPa
zero-target tolerance would leave a permanent virtual attachment.

The embedded flight controller compiles its `PwmTest` callback out in
simulation builds. The MuJoCo controller therefore supplies the equivalent
per-motor overlay itself and holds all unselected motors at the configured
bidirectional neutral PWM (0.75 for Hugmy). This prevents the old stopped value
0.5 from becoming unintended maximum reverse thrust.
