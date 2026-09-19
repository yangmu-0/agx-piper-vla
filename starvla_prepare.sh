#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source /home/agilex/piper_ros/install/setup.bash

check_hz() {
    local topic=$1
    local log
    local attempt
    for attempt in 1 2 3; do
        log=$(mktemp)
        if timeout 15s ros2 topic hz "$topic" > "$log" 2>&1; then
            true
        fi
        if grep -q "average rate:" "$log"; then
            local rate
            rate=$(grep "average rate:" "$log" | tail -n 1 | awk '{print $3}')
            echo "OK $topic average_rate=$rate"
            rm -f "$log"
            return 0
        fi
        echo "WAIT $topic no hz yet (attempt $attempt/3)"
        cat "$log"
        rm -f "$log"
        sleep 3
    done
    echo "FAIL $topic did not publish after retries"
    return 1
}

check_image() {
    local topic=$1
    local log
    local attempt
    for attempt in 1 2 3; do
        log=$(mktemp)
        if timeout 10s ros2 topic echo --qos-profile sensor_data --no-arr --truncate-length 16 "$topic" sensor_msgs/msg/Image > "$log" 2>&1; then
            true
        fi
        if grep -q "height:" "$log"; then
            echo "OK $topic image messages received"
            rm -f "$log"
            return 0
        fi
        echo "WAIT $topic no image yet (attempt $attempt/3)"
        cat "$log"
        rm -f "$log"
        sleep 3
    done
    echo "FAIL $topic image messages not received after retries"
    return 1
}

echo "[1/5] sudo cache"
sudo -v

echo "[2/5] cleanup old StarVLA/Piper/RealSense processes"
pkill -f examples.aloha_real.starvla_main || true
ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: false}" || true
pkill -TERM -f '/home/agilex/piper_ros/install/piper/lib/piper/piper_single_ctrl' || true
pkill -TERM -f 'ros2 launch piper start_two_piper.launch.py' || true
pkill -TERM -f 'bash eval.sh' || true
pkill -TERM -f 'bash eval_starvla.sh' || true
pkill -TERM -f 'bash start_robot.sh' || true
pkill -TERM -f 'camera.sh' || true
pkill -TERM -f 'camera_starvla.sh' || true
pkill -TERM -f 'realsense2_camera' || true
pkill -TERM -f 'rs_launch.py' || true
sleep 8

ros2 daemon stop || true
sleep 2
ros2 daemon start || true
sleep 3

echo "[3/5] start RGB-only cameras"
cd /home/agilex/piper_ros
nohup bash camera_starvla.sh > /tmp/starvla_camera_rgb.log 2>&1 &
sleep 45

echo "[4/5] start Piper controllers"
nohup bash eval_starvla.sh > /tmp/starvla_eval_starvla.log 2>&1 &
sleep 25

echo "[5/5] verify ROS topics"
pgrep -af 'piper_single_ctrl|start_two_piper|eval_starvla.sh|camera_starvla.sh|realsense2_camera|rs_launch.py'
ros2 topic info -v /pos_cmd_left | grep -E 'Publisher count|Subscription count|Node name'
ros2 topic info -v /pos_cmd_right | grep -E 'Publisher count|Subscription count|Node name'

/usr/bin/python3.8 /home/agilex/piper_ros/starvla_check_topics.py 45

echo "StarVLA ROS preparation completed."
