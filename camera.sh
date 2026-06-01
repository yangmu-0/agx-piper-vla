#!/bin/bash
unset ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/foxy/setup.bash

# 定义一个函数来启动相机
start_camera() {
    local cam_name=$1
    local serial=$2
    
    echo "正在启动 $cam_name (序列号: $serial)..."

    ros2 launch realsense2_camera rs_launch.py \
        camera_namespace:=camera \
        camera_name:=${cam_name} \
        serial_no:="'$serial'" \
        initial_reset:=true \
        wait_for_device_timeout:=10.0 \
        reconnect_timeout:=10.0 \
        enable_depth:=true \
        enable_color:=true \
        depth_module.profile:=640x480x30 \
        rgb_camera.profile:=640x480x30 \
        enable_infra1:=false \
        enable_infra2:=false \
        enable_gyro:=false \
        enable_accel:=false \
        enable_sync:=false &
}

# --- 主程序 ---

echo "开始启动所有 RealSense 相机..."

# 启动顶部相机
start_camera "top" '335222073051'
sleep 20 # 给 USB 复位、设备枚举和深度流启动留出时间

# 启动左侧相机
start_camera "left" '239122070290'
sleep 20

# 启动右侧相机
start_camera "right" '327122076728'

echo "所有相机启动命令已发送。"
echo "使用 'ros2 topic list' 查看话题。"
echo "使用 'ros2 node list' 查看节点。"

# 保持脚本运行，或者按 Ctrl+C 退出（相机进程仍在后台）
wait
