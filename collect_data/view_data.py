#!/usr/bin/env python3
# coding=utf-8
"""Interactive HDF5 episode viewer for camera images and arm joint data."""
'''
  python3 /home/agilex/piper_ros/collect_data/view_data.py \
    --dataset_dir /home/agilex/data \
    --task_name fold_towel/fold_towel_0610 \
    --episode_idx 30 --show_depth --show_velocity --show_effort

'''
import argparse
import math
import os
from typing import Dict, List, Optional, Tuple

import cv2
import h5py
import numpy as np


class EpisodeViewerData:
    def __init__(self, hdf5_path: str, show_depth: bool = False):
        self.hdf5_path = hdf5_path
        self.show_depth = show_depth
        self.root = h5py.File(hdf5_path, "r")

        self.compress_len = self.root["/compress_len"][()] if "/compress_len" in self.root else None

        self.rgb_cams = self._list_group_keys("/observations/images")
        self.depth_cams = self._list_group_keys("/observations/images_depth") if show_depth else []

        self.image_streams: List[Tuple[str, h5py.Dataset, bool, Optional[int]]] = []
        cam_idx = 0
        for cam_name in self.rgb_cams:
            ds = self.root[f"/observations/images/{cam_name}"]
            self.image_streams.append((cam_name, ds, self._is_compressed_dataset(ds), cam_idx))
            cam_idx += 1

        for cam_name in self.depth_cams:
            ds = self.root[f"/observations/images_depth/{cam_name}"]
            self.image_streams.append((f"depth/{cam_name}", ds, self._is_compressed_dataset(ds), cam_idx))
            cam_idx += 1

        self.joints = self._load_joint_data(self.root)
        self.num_frames = self._infer_num_frames()

    def _list_group_keys(self, path: str) -> List[str]:
        if path not in self.root:
            return []
        return list(self.root[path].keys())

    @staticmethod
    def _is_compressed_dataset(ds: h5py.Dataset) -> bool:
        return ds.ndim == 2 and ds.dtype == np.uint8

    def _infer_num_frames(self) -> int:
        candidates = []

        for _, ds, _, _ in self.image_streams:
            candidates.append(int(ds.shape[0]))

        for key in [
            "qpos_left",
            "qpos_right",
            "cmd_left",
            "cmd_right",
            "qvel_left",
            "qvel_right",
            "effort_left",
            "effort_right",
            "end_pose_left",
            "end_pose_right",
            "base_action",
            "subtask",
        ]:
            arr = self.joints.get(key)
            if arr is not None:
                candidates.append(int(arr.shape[0]))

        if not candidates:
            raise RuntimeError("No image or arm data found in this hdf5 file.")
        return min(candidates)

    @staticmethod
    def _safe_get(root: h5py.File, path: str) -> Optional[np.ndarray]:
        return root[path][()] if path in root else None

    @classmethod
    def _load_joint_data(cls, root: h5py.File) -> Dict[str, Optional[np.ndarray]]:
        required = [
            "/arm/joint_states_left/position",
            "/arm/joint_states_right/position",
        ]
        missing = [p for p in required if p not in root]
        if missing:
            raise RuntimeError(
                "Only new-format arm data is supported. Missing required datasets: "
                + ", ".join(missing)
            )

        out: Dict[str, Optional[np.ndarray]] = {
            "qpos_left": None,
            "qpos_right": None,
            "qvel_left": None,
            "qvel_right": None,
            "effort_left": None,
            "effort_right": None,
            "cmd_left": None,
            "cmd_right": None,
            "end_pose_left": None,
            "end_pose_right": None,
            "base_action": None,
            "subtask": None,
        }

        out["qpos_left"] = root["/arm/joint_states_left/position"][()]
        out["qpos_right"] = root["/arm/joint_states_right/position"][()]
        out["qvel_left"] = cls._safe_get(root, "/arm/joint_states_left/velocity")
        out["qvel_right"] = cls._safe_get(root, "/arm/joint_states_right/velocity")
        out["effort_left"] = cls._safe_get(root, "/arm/joint_states_left/effort")
        out["effort_right"] = cls._safe_get(root, "/arm/joint_states_right/effort")
        out["end_pose_left"] = cls._safe_get(root, "/arm/end_pose_left")
        out["end_pose_right"] = cls._safe_get(root, "/arm/end_pose_right")
        out["base_action"] = cls._safe_get(root, "/base_action")
        out["subtask"] = cls._safe_get(root, "/subtask")
        # In this dataset format, joint command equals joint state.
        out["cmd_left"] = out["qpos_left"]
        out["cmd_right"] = out["qpos_right"]

        return out

    def get_language(self) -> str:
        if "/language_raw" not in self.root:
            return "N/A"
        raw = self.root["/language_raw"][()]
        if isinstance(raw, np.ndarray) and raw.size > 0:
            raw = raw[0]
        if isinstance(raw, bytes):
            return raw.decode("utf-8", errors="replace")
        return str(raw)

    def get_summary_lines(self) -> List[str]:
        lines = [
            f"file: {os.path.basename(self.hdf5_path)}",
            f"language: {self.get_language()}",
            f"frames: {self.num_frames}",
            f"rgb: {', '.join(self.rgb_cams) if self.rgb_cams else 'N/A'}",
        ]
        if self.depth_cams:
            lines.append(f"depth: {', '.join(self.depth_cams)}")
        attrs = [f"{k}={self.root.attrs[k]}" for k in self.root.attrs.keys()]
        if attrs:
            lines.append("attrs: " + ", ".join(attrs))
        return lines

    @staticmethod
    def _ensure_bgr(img: np.ndarray) -> np.ndarray:
        if img.ndim == 2:
            if img.dtype != np.uint8:
                img = EpisodeViewerData._normalize_to_u8(img)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        if img.ndim == 3 and img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        if img.dtype != np.uint8:
            img = EpisodeViewerData._normalize_to_u8(img)
        return img

    @staticmethod
    def _normalize_to_u8(img: np.ndarray) -> np.ndarray:
        arr = img.astype(np.float32)
        if arr.size == 0:
            return np.zeros_like(arr, dtype=np.uint8)
        vmin = float(np.percentile(arr, 1.0))
        vmax = float(np.percentile(arr, 99.0))
        if vmax <= vmin:
            vmax = vmin + 1.0
        arr = np.clip((arr - vmin) / (vmax - vmin), 0.0, 1.0)
        return (arr * 255.0).astype(np.uint8)

    def _decode_compressed(self, row: np.ndarray, cam_idx: Optional[int], frame_idx: int) -> np.ndarray:
        if self.compress_len is not None and cam_idx is not None:
            if cam_idx < self.compress_len.shape[0] and frame_idx < self.compress_len.shape[1]:
                n = int(self.compress_len[cam_idx, frame_idx])
                n = max(0, min(n, row.shape[0]))
                payload = row[:n]
            else:
                payload = row
        else:
            # Fallback: trim trailing zeros used as padding in compressed buffers.
            end = row.shape[0]
            while end > 0 and row[end - 1] == 0:
                end -= 1
            payload = row[:end] if end > 0 else row

        decoded = cv2.imdecode(payload, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            raise RuntimeError("Failed to decode compressed image frame.")
        return decoded

    def get_frame_images(self, frame_idx: int) -> Dict[str, np.ndarray]:
        out: Dict[str, np.ndarray] = {}
        for stream_name, ds, is_compressed, cam_idx in self.image_streams:
            frame = ds[frame_idx]
            if is_compressed:
                frame = self._decode_compressed(frame, cam_idx, frame_idx)
            out[stream_name] = self._ensure_bgr(frame)
        return out

    def close(self):
        self.root.close()


def _format_number(value: float, decimals: int) -> str:
    if not np.isfinite(value):
        return str(value)
    if decimals <= 0:
        return str(int(round(value)))
    return f"{value:+.{decimals}f}"


def format_vec(vec: Optional[np.ndarray], decimals: int = 3) -> str:
    if vec is None:
        return "N/A"
    arr = np.asarray(vec).reshape(-1)
    if arr.size == 0:
        return "[]"
    return "[" + ", ".join(_format_number(float(v), decimals) for v in arr) + "]"


def vector_lines(
    label: str,
    vec: Optional[np.ndarray],
    color: Tuple[int, int, int],
    decimals: int = 3,
    chunk_size: int = 4,
    scale: float = 0.58,
    thickness: int = 1,
) -> List[Tuple[str, Tuple[int, int, int], float, int]]:
    if vec is None:
        return [(f"{label}: N/A", color, scale, thickness)]

    arr = np.asarray(vec).reshape(-1)
    if arr.size == 0:
        return [(f"{label}: []", color, scale, thickness)]

    lines = []
    for start in range(0, arr.size, chunk_size):
        end = min(start + chunk_size, arr.size)
        chunk = format_vec(arr[start:end], decimals)
        if arr.size <= chunk_size:
            prefix = f"{label}:"
        else:
            prefix = f"{label}[{start}:{end}]:"
        lines.append((f"{prefix} {chunk}", color, scale, thickness))
    return lines


def safe_frame_value(arr: Optional[np.ndarray], frame_idx: int) -> Optional[np.ndarray]:
    if arr is None or frame_idx >= arr.shape[0]:
        return None
    return arr[frame_idx]


def resize_fit(img: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    h, w = img.shape[:2]
    if h <= 0 or w <= 0:
        return np.zeros((max_h, max_w, 3), dtype=np.uint8)
    scale = min(max_w / float(w), max_h / float(h))
    target_w = max(1, int(round(w * scale)))
    target_h = max(1, int(round(h * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(img, (target_w, target_h), interpolation=interpolation)


def draw_label(img: np.ndarray, label: str) -> None:
    cv2.rectangle(img, (0, 0), (img.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(img, label, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.76, (80, 255, 120), 2, cv2.LINE_AA)


def make_tile(name: str, img: np.ndarray, tile_w: int, tile_h: int) -> np.ndarray:
    tile = np.full((tile_h, tile_w, 3), 12, dtype=np.uint8)
    vis = resize_fit(img, tile_w, tile_h)
    y0 = max(0, (tile_h - vis.shape[0]) // 2)
    x0 = max(0, (tile_w - vis.shape[1]) // 2)
    tile[y0 : y0 + vis.shape[0], x0 : x0 + vis.shape[1]] = vis
    draw_label(tile, f"{name}  {img.shape[1]}x{img.shape[0]} {img.dtype}")
    return tile


def make_mosaic(images: Dict[str, np.ndarray], tile_w: int = 430, tile_h: int = 300, columns: int = 2) -> np.ndarray:
    if not images:
        return np.zeros((tile_h, tile_w, 3), dtype=np.uint8)

    columns = max(1, min(columns, len(images)))
    rows = int(math.ceil(len(images) / float(columns)))
    tiles = [make_tile(name, img, tile_w, tile_h) for name, img in images.items()]
    blank = np.full((tile_h, tile_w, 3), 12, dtype=np.uint8)
    while len(tiles) < rows * columns:
        tiles.append(blank.copy())

    row_imgs = []
    for row in range(rows):
        start = row * columns
        row_imgs.append(cv2.hconcat(tiles[start : start + columns]))
    return cv2.vconcat(row_imgs)


def trim_text(text: str, max_chars: int) -> str:
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 3)] + "..."


def put_lines(
    panel: np.ndarray,
    lines: List[Tuple[str, Tuple[int, int, int], float, int]],
    x: int,
    y: int,
    line_h: int,
    bottom_margin: int = 16,
) -> int:
    for text, color, scale, thickness in lines:
        if y > panel.shape[0] - bottom_margin:
            break
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
        y += line_h
    return y


def arm_lines(title: str, qpos, qvel, effort, end_pose, show_velocity: bool, show_effort: bool):
    heading = (115, 230, 255)
    fg = (245, 245, 245)
    muted = (205, 215, 220)
    lines = [
        (title, heading, 0.78, 2),
    ]
    lines.extend(vector_lines("qpos", qpos, fg, decimals=3, chunk_size=4, scale=0.6, thickness=1))
    if qpos is not None and len(qpos) > 0:
        lines.append((f"gripper: {float(qpos[-1]):+.4f}", muted, 0.6, 1))
    if show_velocity:
        lines.extend(vector_lines("qvel", qvel, fg, decimals=3, chunk_size=4, scale=0.56, thickness=1))
    if show_effort:
        lines.extend(vector_lines("eff", effort, fg, decimals=3, chunk_size=4, scale=0.56, thickness=1))
    lines.extend(vector_lines("eef", end_pose, fg, decimals=3, chunk_size=4, scale=0.6, thickness=1))
    return lines


def draw_joint_panel(
    h: int,
    frame_idx: int,
    total: int,
    fps: float,
    playing: bool,
    loop: bool,
    joints: Dict[str, Optional[np.ndarray]],
    summary_lines: List[str],
    show_velocity: bool,
    show_effort: bool,
    panel_w: int = 920,
) -> np.ndarray:
    panel = np.full((h, panel_w, 3), 18, dtype=np.uint8)
    fg = (245, 245, 245)
    em = (115, 230, 255)
    muted = (205, 215, 220)
    ok = (120, 245, 145)
    section_bg = (34, 38, 41)
    header_bg = (42, 48, 52)
    border = (70, 78, 82)
    margin = 20

    def section_box(top: int, bottom: int, fill=section_bg) -> None:
        bottom = max(top + 1, min(bottom, h - 1))
        cv2.rectangle(panel, (12, top), (panel_w - 12, bottom), fill, -1)
        cv2.rectangle(panel, (12, top), (panel_w - 12, bottom), border, 1)

    y = 36
    state = "PLAY" if playing else "PAUSE"
    pct = 100.0 * frame_idx / max(1, total - 1)
    header = f"{state}   frame {frame_idx + 1}/{total}   {pct:5.1f}%   fps {fps:g}   loop {'on' if loop else 'off'}"
    section_box(10, 82, header_bg)
    put_lines(panel, [(header, em, 0.78, 2)], margin, y, 30)

    bar_x, bar_y, bar_w, bar_h = margin, 56, panel_w - 40, 12
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (82, 90, 94), 1)
    fill_w = int(round(bar_w * (frame_idx + 1) / max(1, total)))
    cv2.rectangle(panel, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), ok, -1)
    y = 112

    meta = [(trim_text(line, 104), muted, 0.56, 1) for line in summary_lines[:5]]
    meta_height = 24 + 22 * len(meta)
    section_box(y - 26, y - 26 + meta_height)
    y = put_lines(panel, [("Episode", em, 0.68, 2)], margin, y, 24)
    y = put_lines(panel, meta, margin, y, 22)
    y += 16

    ql = joints.get("qpos_left")
    qr = joints.get("qpos_right")
    vl = joints.get("qvel_left")
    vr = joints.get("qvel_right")
    el = joints.get("effort_left")
    er = joints.get("effort_right")
    eef_l = joints.get("end_pose_left")
    eef_r = joints.get("end_pose_right")

    left_q = safe_frame_value(ql, frame_idx)
    right_q = safe_frame_value(qr, frame_idx)
    left_v = safe_frame_value(vl, frame_idx)
    right_v = safe_frame_value(vr, frame_idx)
    left_e = safe_frame_value(el, frame_idx)
    right_e = safe_frame_value(er, frame_idx)
    left_eef = safe_frame_value(eef_l, frame_idx)
    right_eef = safe_frame_value(eef_r, frame_idx)
    base = safe_frame_value(joints.get("base_action"), frame_idx)
    subtask = safe_frame_value(joints.get("subtask"), frame_idx)

    global_lines = [
        ("Frame Data", em, 0.72, 2),
        (f"subtask: {format_vec(subtask, 0)}", fg, 0.62, 1),
        (f"base [v,w]: {format_vec(base)}", fg, 0.62, 1),
    ]
    section_box(y - 24, y + 74)
    y = put_lines(panel, global_lines, margin, y, 25)
    y += 14

    left_lines = arm_lines("Left Arm", left_q, left_v, left_e, left_eef, show_velocity, show_effort)
    left_box_h = 20 + 24 * len(left_lines)
    section_box(y - 24, y - 24 + left_box_h)
    y = put_lines(panel, left_lines, margin, y, 24)
    y += 14

    right_lines = arm_lines("Right Arm", right_q, right_v, right_e, right_eef, show_velocity, show_effort)
    right_box_h = 20 + 24 * len(right_lines)
    section_box(y - 24, y - 24 + right_box_h)
    y = put_lines(panel, right_lines, margin, y, 24)

    controls = [
        ("space play/pause   r restart   l loop", fg, 0.54, 1),
        ("a/d or arrows step 1   A/D step 10   Home/End jump", fg, 0.54, 1),
        ("v velocity   e effort   q/Esc quit", fg, 0.54, 1),
    ]
    control_y = max(y + 16, h - 72)
    if control_y < h - 12:
        section_box(control_y - 20, h - 12, header_bg)
        put_lines(panel, controls, margin, control_y, 22)

    return panel


def resolve_hdf5_path(args) -> str:
    if args.file:
        return os.path.abspath(os.path.expanduser(args.file))

    if not args.dataset_dir or not args.task_name:
        raise ValueError("Need --file, or --dataset_dir + --task_name + --episode_idx")

    path = os.path.join(
        os.path.abspath(os.path.expanduser(args.dataset_dir)),
        args.task_name,
        f"episode_{args.episode_idx}.hdf5",
    )
    return path


def main():
    parser = argparse.ArgumentParser(description="View episode images + arm joint data from hdf5.")
    parser.add_argument("--file", type=str, default=None, help="Direct path to *.hdf5")
    parser.add_argument("--dataset_dir", type=str, default=None, help="Dataset root, e.g. ~/data")
    parser.add_argument("--task_name", type=str, default=None, help="Task folder under dataset_dir")
    parser.add_argument("--episode_idx", type=int, default=0, help="Episode index")
    parser.add_argument("--show_depth", action="store_true", help="Also show /observations/images_depth")
    parser.add_argument("--fps", type=float, default=15, help="Playback fps")
    parser.add_argument("--start", type=int, default=0, help="Start frame")
    parser.add_argument("--show_velocity", action="store_true", help="Show qvel text")
    parser.add_argument("--show_effort", action="store_true", help="Show effort text")
    parser.add_argument("--loop", action="store_true", help="Loop back to frame 0 after the last frame")
    parser.add_argument("--tile_width", type=int, default=430, help="Image tile width")
    parser.add_argument("--tile_height", type=int, default=360, help="Image tile height")
    parser.add_argument("--columns", type=int, default=2, help="Image grid columns")
    parser.add_argument("--panel_width", type=int, default=920, help="Right info panel width")
    args = parser.parse_args()

    hdf5_path = resolve_hdf5_path(args)
    if not os.path.isfile(hdf5_path):
        raise FileNotFoundError(f"File not found: {hdf5_path}")

    data = EpisodeViewerData(hdf5_path, show_depth=args.show_depth)
    print(f"Opened: {hdf5_path}")
    print(f"Frames: {data.num_frames}")
    print(f"RGB cameras: {data.rgb_cams}")
    if args.show_depth:
        print(f"Depth cameras: {data.depth_cams}")

    frame_idx = max(0, min(args.start, data.num_frames - 1))
    playing = True
    loop = bool(args.loop)
    show_velocity = bool(args.show_velocity)
    show_effort = bool(args.show_effort)
    summary_lines = data.get_summary_lines()

    wait_ms = max(1, int(1000.0 / max(1e-3, args.fps)))

    win_name = "HDF5 Data Viewer"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    while True:
        images = data.get_frame_images(frame_idx)
        mosaic = make_mosaic(
            images,
            tile_w=max(160, args.tile_width),
            tile_h=max(120, args.tile_height),
            columns=max(1, args.columns),
        )
        panel = draw_joint_panel(
            h=mosaic.shape[0],
            frame_idx=frame_idx,
            total=data.num_frames,
            fps=args.fps,
            playing=playing,
            loop=loop,
            joints=data.joints,
            summary_lines=summary_lines,
            show_velocity=show_velocity,
            show_effort=show_effort,
            panel_w=max(560, args.panel_width),
        )

        view = cv2.hconcat([mosaic, panel])
        if frame_idx == max(0, min(args.start, data.num_frames - 1)):
            cv2.resizeWindow(win_name, min(view.shape[1], 1920), min(view.shape[0], 1080))
        cv2.imshow(win_name, view)

        key = cv2.waitKeyEx(wait_ms if playing else 0)
        key_low = key & 0xFF
        if key_low in [ord("q"), 27]:
            break
        if key_low == ord(" "):
            playing = not playing
        elif key_low == ord("r"):
            frame_idx = 0
            playing = True
        elif key_low == ord("l"):
            loop = not loop
        elif key_low == ord("v"):
            show_velocity = not show_velocity
        elif key_low == ord("e"):
            show_effort = not show_effort
        elif key_low == ord("a") or key in [2424832, 65361]:
            frame_idx = max(0, frame_idx - 1)
            playing = False
        elif key_low == ord("d") or key in [2555904, 65363]:
            frame_idx = min(data.num_frames - 1, frame_idx + 1)
            playing = False
        elif key_low == ord("A"):
            frame_idx = max(0, frame_idx - 10)
            playing = False
        elif key_low == ord("D"):
            frame_idx = min(data.num_frames - 1, frame_idx + 10)
            playing = False
        elif key in [2359296, 65360]:
            frame_idx = 0
            playing = False
        elif key in [2293760, 65367]:
            frame_idx = data.num_frames - 1
            playing = False

        if playing:
            frame_idx += 1
            if frame_idx >= data.num_frames:
                if loop:
                    frame_idx = 0
                else:
                    frame_idx = data.num_frames - 1
                    playing = False

    data.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
