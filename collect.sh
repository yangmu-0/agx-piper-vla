#!/bin/bash
source install/setup.bash
set -euo pipefail

DATASET_DIR="/home/agilex/data/fold_towel"
TASK_NAME="fold_towel_0601"

MAX_STEPS=100000
FRAME_RATE=30
LANGUAGE_RAW="Fold the towel."
PRINT_DATA_INFO=1
PRINT_EVERY_N=1

cd collect_data

mkdir -p "$DATASET_DIR/$TASK_NAME"

PRINT_ARGS=()
if [ "$PRINT_DATA_INFO" = "1" ]; then
    PRINT_ARGS+=(--print_data_info)
fi

next_episode_idx() {
    local max_num
    max_num=$(find "$DATASET_DIR/$TASK_NAME" -type f -name "episode_*.hdf5" |
        sed -E 's/.*episode_([0-9]+)\.hdf5/\1/' |
        sort -n |
        tail -1)

    if [ -z "$max_num" ]; then
        max_num=-1
    fi

    echo $((max_num + 1))
}

run_collect() {
    local next_num="$1"

    python3 collect_data_ros2.py \
        --dataset_dir "$DATASET_DIR" \
        --task_name "$TASK_NAME" \
        --max_timesteps "$MAX_STEPS" \
        --frame_rate "$FRAME_RATE" \
        --episode_idx "$next_num" \
        --language_raw "$LANGUAGE_RAW" \
        --print_every_n "$PRINT_EVERY_N" \
        "${PRINT_ARGS[@]}" \
        --joint_states_left_topic /joint_states_left \
        --joint_states_right_topic /joint_states_right \
        --end_pose_left_topic /end_pose_left \
        --end_pose_right_topic /end_pose_right \
        --img_left_topic /left/color/image_raw \
        --img_right_topic /right/color/image_raw \
        --img_top_topic /top/color/image_raw \
        --img_left_depth_topic /left/depth/image_rect_raw \
        --img_right_depth_topic /right/depth/image_rect_raw \
        --img_top_depth_topic /top/depth/image_rect_raw
        # --use_depth_image False \
}

while true; do
    next_num=$(next_episode_idx)
    echo "Next episode: $next_num"
    echo "Press s to start recording, Enter to save, q to discard this episode, Ctrl+C to exit."
    if run_collect "$next_num"; then
        continue
    else
        status=$?
        if [ "$status" -eq 130 ]; then
            exit 130
        fi
        exit "$status"
    fi
done
