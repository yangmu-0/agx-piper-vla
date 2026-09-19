#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"
set -u

DATASET_DIR="/home/agilex/data/yrh"
TASK_NAME="clean_table_0914"
MAX_STEPS=100000
FRAME_RATE=30
LANGUAGE_RAW="Sort objects on the table."
PRINT_DATA_INFO=1
PRINT_EVERY_N=1
CAMERA_NAMES=(cam_high cam_left_wrist cam_right_wrist)

cd "$SCRIPT_DIR/collect_data"

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

validate_depth_episode() {
    local episode_file="$DATASET_DIR/$TASK_NAME/episode_$1.hdf5"

    if [ ! -f "$episode_file" ]; then
        echo "No episode file written; depth validation skipped."
        return 0
    fi

    python3 - "$episode_file" "${CAMERA_NAMES[@]}" <<'PY'
import sys
import h5py

path = sys.argv[1]
expected = tuple(sys.argv[2:])

with h5py.File(path, "r") as root:
    missing = []
    if "observations/images_depth" not in root:
        missing = [f"observations/images_depth/{name}" for name in expected]
    else:
        group = root["observations/images_depth"]
        for name in expected:
            if name not in group:
                missing.append(f"observations/images_depth/{name}")
                continue
            if group[name].shape[0] == 0:
                missing.append(f"observations/images_depth/{name} (0 frames)")

    if missing:
        print(f"ERROR: saved episode has no required depth data: {path}")
        for item in missing:
            print(f"  missing: {item}")
        sys.exit(1)

    shapes = ", ".join(
        f"{name}={tuple(root[f'observations/images_depth/{name}'].shape)}"
        for name in expected
    )
    print(f"Depth data verified: {path} ({shapes})")
PY
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
        --camera_names "${CAMERA_NAMES[@]}" \
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
        --img_top_depth_topic /top/depth/image_rect_raw \
        --use_depth_image true
    local collect_status=$?
    if [ "$collect_status" -ne 0 ]; then
        return "$collect_status"
    fi

    validate_depth_episode "$next_num"
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
