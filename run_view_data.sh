#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="$SCRIPT_DIR/collect_data/view_data.py"
DATASET_DIR="/home/agilex/data/fold_towel"
TASK_NAME="fold_towel_0602"
EPISODE_IDX="10"
FILE_PATH=""
FPS=30
START=0
SHOW_DEPTH=1
SHOW_VELOCITY=0
SHOW_EFFORT=0
LOOP=0
TILE_WIDTH=430
TILE_HEIGHT=360
COLUMNS=2
PANEL_WIDTH=920

usage() {
  cat <<EOF
Usage:
  $(basename "$0") --task <task_name> [options]
  $(basename "$0") --file <episode.hdf5> [options]

Options:
  --task <name>         Task folder under dataset_dir
  --episode <idx>       Episode index (default: latest)
  --latest              Open the latest episode in the task folder
  --dataset-dir <dir>   Dataset root (default: /home/agilex/data)
  --file <path>         Direct path to hdf5 file
  --fps <num>           Playback fps (default: 20)
  --start <idx>         Start frame (default: 0)
  --depth               Show depth images (default)
  --no-depth            Only show RGB images
  --vel                 Show velocity text
  --effort              Show effort text
  --loop                Replay from the first frame after reaching the end
  --tile-width <num>    Image tile width (default: 430)
  --tile-height <num>   Image tile height (default: 360)
  --columns <num>       Image grid columns (default: 2)
  --panel-width <num>   Right info panel width (default: 920)
  -h, --help            Show help

Examples:
  $(basename "$0") --task fold_towel_0601
  $(basename "$0") --task fold_towel_0601 --episode 20
  $(basename "$0") --file /home/agilex/data/fold_towel/fold_towel_0601/episode_20.hdf5 --vel
  $(basename "$0") --file /home/agilex/data/fold_towel/fold_towel_0601/episode_19.hdf5 --no-depth
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task)
      TASK_NAME="$2"
      shift 2
      ;;
    --episode)
      EPISODE_IDX="$2"
      shift 2
      ;;
    --latest)
      EPISODE_IDX=""
      shift
      ;;
    --dataset-dir)
      DATASET_DIR="$2"
      shift 2
      ;;
    --file)
      FILE_PATH="$2"
      shift 2
      ;;
    --fps)
      FPS="$2"
      shift 2
      ;;
    --start)
      START="$2"
      shift 2
      ;;
    --depth)
      SHOW_DEPTH=1
      shift
      ;;
    --no-depth)
      SHOW_DEPTH=0
      shift
      ;;
    --vel)
      SHOW_VELOCITY=1
      shift
      ;;
    --effort)
      SHOW_EFFORT=1
      shift
      ;;
    --loop)
      LOOP=1
      shift
      ;;
    --tile-width)
      TILE_WIDTH="$2"
      shift 2
      ;;
    --tile-height)
      TILE_HEIGHT="$2"
      shift 2
      ;;
    --columns)
      COLUMNS="$2"
      shift 2
      ;;
    --panel-width)
      PANEL_WIDTH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "$PY_SCRIPT" ]]; then
  echo "Cannot find viewer script: $PY_SCRIPT" >&2
  exit 1
fi

latest_episode_idx() {
  local task_dir="$DATASET_DIR/$TASK_NAME"
  local max_num

  max_num=$(find "$task_dir" -maxdepth 1 -type f -name "episode_*.hdf5" 2>/dev/null |
    sed -E 's/.*episode_([0-9]+)\.hdf5/\1/' |
    sort -n |
    tail -1)

  if [[ -z "$max_num" ]]; then
    echo "Cannot find episode_*.hdf5 in $task_dir" >&2
    exit 1
  fi

  echo "$max_num"
}

cmd=(
  python3 "$PY_SCRIPT"
  --fps "$FPS"
  --start "$START"
  --tile_width "$TILE_WIDTH"
  --tile_height "$TILE_HEIGHT"
  --columns "$COLUMNS"
  --panel_width "$PANEL_WIDTH"
)

if [[ -n "$FILE_PATH" ]]; then
  cmd+=(--file "$FILE_PATH")
else
  if [[ -z "$TASK_NAME" ]]; then
    echo "Need --task or --file" >&2
    usage
    exit 1
  fi
  if [[ -z "$EPISODE_IDX" ]]; then
    EPISODE_IDX="$(latest_episode_idx)"
  fi
  cmd+=(--dataset_dir "$DATASET_DIR" --task_name "$TASK_NAME" --episode_idx "$EPISODE_IDX")
fi

[[ "$SHOW_DEPTH" -eq 1 ]] && cmd+=(--show_depth)
[[ "$SHOW_VELOCITY" -eq 1 ]] && cmd+=(--show_velocity)
[[ "$SHOW_EFFORT" -eq 1 ]] && cmd+=(--show_effort)
[[ "$LOOP" -eq 1 ]] && cmd+=(--loop)

exec "${cmd[@]}"
