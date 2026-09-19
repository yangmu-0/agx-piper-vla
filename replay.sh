#!/bin/bash
# bash replay.sh eef
# bash replay.sh joint
unset ROS_DISTRO ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI ROS_ETC_DIR ROSLISP_PACKAGE_DIRECTORIES PYTHONPATH
source /opt/ros/foxy/setup.bash
source install/setup.bash
set -euo pipefail

DATASET_DIR="/home/agilex/data/yrh"
TASK_NAME="put_bowl_into_plate_0909"

EPISODE_IDX=5


FRAME_RATE=30
POS_CMD_MODE1=0
POS_CMD_MODE2=0
PRINT_DATA_INFO=1
PRINT_EVERY_N=1
REPLAY_MODE="${1:-eef}"  # eef: 末端数据回放, joint: 关节数据回放

POS_CMD_LEFT_TOPIC="/pos_cmd_left"
POS_CMD_RIGHT_TOPIC="/pos_cmd_right"
JOINT_CTRL_CMD_LEFT_TOPIC="/joint_ctrl_cmd_left"
JOINT_CTRL_CMD_RIGHT_TOPIC="/joint_ctrl_cmd_right"

case "$REPLAY_MODE" in
    eef|end|cartesian|末端)
        # 末端数据回放: 仅让控制节点接收 /pos_cmd_*，关节控制命令发布到无订阅话题
        JOINT_CTRL_CMD_LEFT_TOPIC="/unused_joint_ctrl_cmd_left"
        JOINT_CTRL_CMD_RIGHT_TOPIC="/unused_joint_ctrl_cmd_right"
        ;;
    joint|joints|关节)
        # 关节数据回放: 仅让控制节点接收 /joint_ctrl_cmd_*，末端控制命令发布到无订阅话题
        POS_CMD_LEFT_TOPIC="/unused_pos_cmd_left"
        POS_CMD_RIGHT_TOPIC="/unused_pos_cmd_right"
        ;;
    *)
        echo "用法: $0 [eef|joint]"
        echo "  eef   : 使用末端数据回放 (默认)"
        echo "  joint : 使用关节数据回放"
        exit 1
        ;;
esac

echo "Replay mode: $REPLAY_MODE"
echo "Dataset: $DATASET_DIR/$TASK_NAME/episode_$EPISODE_IDX.hdf5"

while true; do
    read -r -p "请输入每多少步暂停一次（正整数）: " PAUSE_EVERY_N
    if [[ "$PAUSE_EVERY_N" =~ ^[1-9][0-9]*$ ]]; then
        break
    fi
    echo "输入无效，请输入大于 0 的整数，例如 30。"
done

while true; do
    read -r -p "请输入每执行 ${PAUSE_EVERY_N} 步后的额外延迟（秒，可输入小数，0 表示不延迟）: " PAUSE_SECONDS
    if [[ "$PAUSE_SECONDS" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
        break
    fi
    echo "输入无效，请输入大于等于 0 的数字，例如 2 或 0.5。"
done

VIDEO_ARGS=()
while true; do
    read -r -p "是否录制机械臂动作回放期间三路相机的实时高清画面？[y/n]: " RECORD_LIVE_VIDEOS
    case "${RECORD_LIVE_VIDEOS,,}" in
        y|yes)
            VIDEO_OUTPUT_DIR="$DATASET_DIR/$TASK_NAME/replay_videos/episode_${EPISODE_IDX}_live_$(date +%Y%m%d_%H%M%S)"
            VIDEO_ARGS+=(
                --record_live_videos
                --video_output_dir "$VIDEO_OUTPUT_DIR"
                --live_video_fps 30
                --live_camera_timeout 15
            )
            echo "三路实时无损视频将保存到: $VIDEO_OUTPUT_DIR"
            echo "开始动作回放前会等待三路实体相机画面，缺少任一路将安全退出。"
            break
            ;;
        n|no)
            echo "本次动作回放不录制实时相机视频。"
            break
            ;;
        *)
            echo "输入无效，请输入 y 或 n。"
            ;;
    esac
done

# source ~/miniconda3/bin/activate
# conda activate aloha
cd collect_data

PRINT_ARGS=()
if [ "$PRINT_DATA_INFO" = "1" ]; then
    PRINT_ARGS+=(--print_data_info)
fi

python3 replay_data.py \
    --dataset_dir "$DATASET_DIR" \
    --task_name "$TASK_NAME" \
    --episode_idx "$EPISODE_IDX" \
    --frame_rate "$FRAME_RATE" \
    --pause_every_n "$PAUSE_EVERY_N" \
    --pause_seconds "$PAUSE_SECONDS" \
    --pos_cmd_mode1 "$POS_CMD_MODE1" \
    --pos_cmd_mode2 "$POS_CMD_MODE2" \
    --print_every_n "$PRINT_EVERY_N" \
    "${PRINT_ARGS[@]}" \
    "${VIDEO_ARGS[@]}" \
    --joint_states_left_topic /joint_states_left \
    --joint_states_right_topic /joint_states_right \
    --joint_left_topic /joint_left \
    --joint_right_topic /joint_right \
    --joint_ctrl_cmd_left_topic "$JOINT_CTRL_CMD_LEFT_TOPIC" \
    --joint_ctrl_cmd_right_topic "$JOINT_CTRL_CMD_RIGHT_TOPIC" \
    --end_pose_left_topic /end_pose_left \
    --end_pose_right_topic /end_pose_right \
    --pos_cmd_left_topic "$POS_CMD_LEFT_TOPIC" \
    --pos_cmd_right_topic "$POS_CMD_RIGHT_TOPIC" \
    --img_left_topic /left/color/image_raw \
    --img_right_topic /right/color/image_raw \
    --img_top_topic /top/color/image_raw \
    --img_left_depth_topic /left/depth/image_rect_raw \
    --img_right_depth_topic /right/depth/image_rect_raw \
    --img_top_depth_topic /top/depth/image_rect_raw
