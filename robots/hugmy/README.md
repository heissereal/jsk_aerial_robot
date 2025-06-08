# Hugmy - Aerial Robot That Can Perch on a Human

This repository contains the ROS packages for the Hugmy aerial robot, as introduced in the paper:
**"Flexible Morphing Aerial Robot with Inflatable Structure for Perching-based Human-Robot Interaction"**

---
## Features

- Inflatable morphing arms for safe human perching
- Autonomous approach to a human using vision and depth sensing
- Air pressure control for joint and bottom inflatable actuators
- Full perching sequence: Approach → Ready → Perch → Deperch

## Flight Control Overview
### 'aerial_robot_control' Package
**🔗 Related to: Section 4 and Appendix H and I**
- Position Control: `aerial_robot_control/src/control/base/pose_linear_controller.cpp`
**🔗 Related to: Appendix I.1**
- Attitude Control:`aerial_robot_control/src/control/under_actuated_lqi_controller.cpp`
**🔗 Related to: Appendix I.2**
## Source Overview

### `src/Navigation` – Perching-Aware Flight Control  
**🔗 Related to: Section 4.2, 5.3**
- 'hugmy_navigation.cpp': 
    Extends BaseNavigator by adding logic to handle perching-specific state transitions and to modify the takeoff behavior(Deperch) after a perching event.
    - Subscribes to `/perching_state` and sets an internal flag when the robot enters the perching state. 
    - In `motorArming()`, if this flag is set, the robot performs "deperch" by setting the target hovering height to 0.8 m to the perching height.

---
## `script/` – Core Behavior Scripts

### `approaching_human_with_depth.py`  
**🔗 Related to: Section 5.4.2, Appendix E and J**
Enables the robot to autonomously approach a person safely, while providing feedback on its status using depth data and object detection.

- Subscribes to bounding boxes of detected humans and selects the largest box as the target (assumed closest).

- Processes the center region of the selected bounding box in the depth image to estimate distance, with local sampling for noise reduction.

- Employs a PD controller to compute forward velocity based on distance error and adjusts yaw to align the robot’s heading with the human target.

- Publishes a perching state to signal readiness for landing when the robot reaches an appropriate proximity or other predefined conditions are met.

- Includes a fail-safe mechanism that commands the robot to land if valid depth data is not received for a specified number of frames.

### `air_pressure_control_2valve_new.py`  
**🔗 Related to: Section 3.2, 3.3, and Section 5**
Controls an air pump and two solenoid valves using PWM signals to regulate internal pressure in the inflatable actuators.

- Subscribes to multiple air pressure sensors to monitor both joint and bottom inflatable actuators' pressures.

- Implements a multi-phase perching control sequence:
    - Approach Phase: Inflates the bottom inflatable actuator to a predefined standby pressure.
    
    - Ready Phase: Increases joint pressure to a ready level before initiating perching by the bottom inflatable actuator.

    - Perch Phase 1(Free-fall): Maximizes pressure to joint inflatable actuator during landing.

    - Perch Phase 2(Keep grasping): Maintains pressure to keep a firm grasp after landing.

    - Deperch Phase: Releases air and re-arms motors to take off from the perching state.

- Manages flight and perching states through subscriptions to `/quadrotor/flight_state` and `/perching_state`, responding to joystick input or internal flags to advance perching phases.

- Includes safety mechanisms such as pressure threshold checks, automatic stop conditions, and fallback behavior in case of abnormal sensor readings.

- Operates autonomously with optional manual overrides, executing all logic in a loop that evaluates current sensor states and control flags to transition between perching stages safely and reliably.

### `merge.py`
- Synchronizes object detection results (rectangles) and classification results (labels), extracts only the rectangles classified as "person".
- Publishes filtered bounding boxes to `approaching_human_with_depth.py`

---
## Launch Files
### `bringup.launch`  
Starts the complete system.

### `camera_and_sensor.launch`  
Initializes the camera, object detection, and pressure sensors.

### `approach_and_pressure.launch`  
**🔗 Related to: Section 5 Combined Evaluation and Appendix E and J**
Launches:
- `approaching_human_with_depth.py`  
- `air_pressure_control_2valve_new.py`  
→ *Used to demonstrate the integrated perching mechanism with human interaction.*

---

## How to Run

```bash
roslaunch hugmy camera_and_sensor.launch
roslaunch hugmy approach_and_pressure.launch
roslaunch hugmy bringup.launch