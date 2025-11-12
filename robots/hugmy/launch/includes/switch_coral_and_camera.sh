#!/bin/bash

if [ "$1" == "realsense" ]; then
    echo "Switching to Realsense environment..."
    source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
    shift
    roslaunch realsense2_camera rs_camera.launch "$@"
elif [ "$1" == "coral" ]; then
    source ~/coral_ws/devel/setup.bash
    shift
    roslaunch coral_usb edgetpu_object_detector.launch "$@"
else
    echo "Usage: $0 [realsense|coral]"
    source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
    exit 1
fi
