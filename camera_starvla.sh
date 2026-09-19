#!/usr/bin/env bash
set -e

unset ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/foxy/setup.bash

start_camera() {
    local cam_name=$1
    local serial=$2

    echo "Starting ${cam_name} color camera (serial: ${serial})..."
    ros2 launch realsense2_camera rs_launch.py \
        camera_namespace:=camera \
        camera_name:=${cam_name} \
        serial_no:="'${serial}'" \
        initial_reset:=false \
        wait_for_device_timeout:=10.0 \
        reconnect_timeout:=10.0 \
        enable_depth:=false \
        enable_color:=true \
        rgb_camera.profile:=640x480x30 \
        enable_infra1:=false \
        enable_infra2:=false \
        enable_gyro:=false \
        enable_accel:=false \
        enable_sync:=false &
}

echo "Starting StarVLA RGB-only RealSense cameras..."
start_camera "top" '335222073051'
sleep 18
start_camera "left" '239122070290'
sleep 18
start_camera "right" '327122076728'

echo "StarVLA camera launch commands sent."
wait
