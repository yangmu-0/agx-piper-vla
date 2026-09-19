#!/usr/bin/env bash
set -e

unset ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES PYTHONPATH
source /opt/ros/foxy/setup.bash
source /home/agilex/piper_ros/install/setup.bash

cd /home/agilex/piper_ros
bash can_muti_activate.sh --ignore
ros2 launch piper start_two_piper.launch.py can_left_port:=can_arm1 can_right_port:=can_arm2 auto_enable:=true girpper_exist:=true
