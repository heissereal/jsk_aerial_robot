# Human-arm locomotion with a hand mocap reference

Rigid body ID 1 is the robot and rigid body ID 2 is a position-only marker on
the hand. The gait score and the 50-mm goal use the change in robot-to-hand
XY distance. Translating or rotating the arm and robot together therefore does
not count as robot locomotion. ID 2 height and orientation are never used.

Start the real-machine bridge and both mocap rigid bodies:

```bash
roslaunch hugmy bringup.launch \
  simulation:=false real_machine:=true mujoco:=false fixed_arms:=false \
  estimate_mode:=1 track_hand_mocap:=true \
  mocap_robot_rigid_body_id:=1 mocap_hand_rigid_body_id:=2
```

Start Action C and the joystick interface (this does not enable actuation):

```bash
roslaunch hugmy human_arm_locomotion.launch
```

Start the PPG serial node in another terminal:

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=115200
rostopic echo /ppg/data
```

`/ppg/data.data[0]` is the pulse waveform and `data[1]` is the heart-rate
estimate. After every completed gait segment and every posture recovery, the
human-arm launch keeps arm grasp pressure, commands all rotor thrust to zero,
commands the bottom bladder to 0 kPa, and waits for 10 continuous seconds at a
stable deflated endpoint before continuing. The wait is a quiet measurement
window; it intentionally does not require a finite heart-rate estimate, so a
temporary PPG `nan` or a disconnected serial node cannot deadlock locomotion.
Override it with `ppg_settle_sec:=SECONDS`; ordinary and MuJoCo launches keep
the feature disabled by default (`0.0`).

Before pressing the enable button, verify both streams and the relative
position:

```bash
rostopic hz /quadrotor/mocap/pose
rostopic hz /quadrotor/mocap/hand_pose
rostopic echo /quadrotor/attitude_pressure_controller/hand_relative_position
```

Default buttons are:

- button 4: enable the attitude/locomotion controller and its pressure loop
- button 5: disable both controllers
- button 3: command 50 mm toward the hand
- button 1: stop locomotion while leaving pressure enabled

The button indices and distance are launch arguments. The target attitude is
latched at `[0, 0, 0]`; human-arm mode rejects nonzero attitude targets. Loss
of either mocap stream during a gait triggers the normal gait stop path.
