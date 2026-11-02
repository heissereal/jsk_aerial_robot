# Robot-only locomotion and optional hand mocap reference

The default mode uses only robot rigid body ID 1.  Progress and the travel goal
are measured from the robot mocap position, so rigid body ID 2 is not required.

Start the real-machine bridge and robot mocap:

```bash
roslaunch hugmy bringup.launch \
  simulation:=false real_machine:=true mujoco:=false fixed_arms:=false \
  estimate_mode:=1 track_hand_mocap:=false \
  mocap_robot_rigid_body_id:=1
```

Start Action C and the joystick interface (this does not enable actuation).
Bounded online trajectory optimization is enabled by default: the first
measured stroke supplies the contact-specific baseline, and subsequent
comparable strokes explore unloading depth, pull magnitude/mix, and rear
reverse assist.  Use `hybrid_locomotion_outer_enabled:=false` only when a
fixed baseline trajectory is deliberately required:

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

Before pressing the enable button, verify the robot stream:

```bash
rostopic hz /quadrotor/mocap/pose
```

Default buttons are:

- button 4: enable the attitude/locomotion controller and its pressure loop
- button 5: disable both controllers
- button 3: command 100 mm in the robot-forward direction and run the online search from its
  first measured baseline
- button 1: stop locomotion while leaving pressure enabled

The button indices and distance are launch arguments. The target attitude is
latched at `[0, 0, 0]`. Loss of robot mocap during a gait triggers the normal
gait stop path.

To compensate for motion of a human arm, start rigid body ID 2 and explicitly
enable the optional relative reference:

```bash
roslaunch hugmy bringup.launch \
  simulation:=false real_machine:=true mujoco:=false fixed_arms:=false \
  estimate_mode:=1 track_hand_mocap:=true \
  mocap_robot_rigid_body_id:=1 mocap_hand_rigid_body_id:=2

roslaunch hugmy human_arm_locomotion.launch \
  hand_relative_mocap_enabled:=true \
  hand_target_selects_direction:=true
```

In this opt-in mode, ID 2 is a position-only marker. The gait score and goal
use robot-to-hand XY distance; ID 2 height and orientation are never used.
