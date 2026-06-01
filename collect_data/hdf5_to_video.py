#!/usr/bin/env python3
# coding=utf-8
"""Convert an HDF5 episode to an mp4 preview video.

Example:
  python3 hdf5_to_video.py \
    --file /home/agilex/data/clean_desktop/zcl/episode_0.hdf5 \
    --output episode_0.mp4 \
    --fps 20 \
    --show_velocity --show_effort
"""

import argparse
import os
from typing import Dict, Iterable, Optional

import numpy as np

try:
    import cv2

    from view_data import EpisodeViewerData, draw_joint_panel, make_mosaic
except ModuleNotFoundError as exc:
    raise SystemExit(
        f"Missing python dependency: {exc.name}\n"
        "Install dependencies first, for example:\n"
        "  python3 -m pip install -r requirements.txt"
    ) from exc


DEFAULT_HDF5 = "/home/agilex/data/clean_desktop/zcl/episode_0.hdf5"


def parse_args():
    parser = argparse.ArgumentParser(description="Convert HDF5 episode images and joint data to a video.")
    parser.add_argument("--file", type=str, default=DEFAULT_HDF5, help="Path to episode_*.hdf5")
    parser.add_argument("--output", type=str, default=None, help="Output video path. Default: ./<episode>.mp4")
    parser.add_argument("--fps", type=float, default=20.0, help="Output video fps")
    parser.add_argument("--start", type=int, default=0, help="Start frame index")
    parser.add_argument("--end", type=int, default=None, help="End frame index, exclusive")
    parser.add_argument("--max_frames", type=int, default=None, help="Maximum number of frames to write")
    parser.add_argument("--stride", type=int, default=1, help="Write every Nth frame")
    parser.add_argument("--tile_height", type=int, default=380, help="Height of each camera tile")
    parser.add_argument("--codec", type=str, default="mp4v", help="OpenCV fourcc codec, e.g. mp4v or XVID")
    parser.add_argument("--cameras", nargs="+", default=None, help="Camera names to include, default uses all RGB cameras")
    parser.add_argument("--show_depth", action="store_true", help="Also include /observations/images_depth streams")
    parser.add_argument("--show_velocity", action="store_true", help="Show joint velocity in the side panel")
    parser.add_argument("--show_effort", action="store_true", help="Show joint effort in the side panel")
    parser.add_argument("--no_panel", action="store_true", help="Only write camera images, no joint text panel")
    parser.add_argument("--preview", action="store_true", help="Show frames while writing")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output video if it already exists")
    parser.add_argument("--progress_every", type=int, default=50, help="Print progress every N written frames")
    return parser.parse_args()


def default_output_path(hdf5_path: str) -> str:
    stem = os.path.splitext(os.path.basename(hdf5_path))[0]
    return os.path.abspath(f"{stem}.mp4")


def selected_images(images: Dict[str, np.ndarray], camera_names: Optional[Iterable[str]]) -> Dict[str, np.ndarray]:
    if not camera_names:
        return images

    selected = {}
    requested = set(camera_names)
    for name, image in images.items():
        base_name = name.split("/", 1)[-1]
        if name in requested or base_name in requested:
            selected[name] = image
    return selected


def pad_to_even(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    pad_bottom = h % 2
    pad_right = w % 2
    if not pad_bottom and not pad_right:
        return frame
    return cv2.copyMakeBorder(
        frame,
        0,
        pad_bottom,
        0,
        pad_right,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )


def render_frame(data: EpisodeViewerData, frame_idx: int, args) -> np.ndarray:
    images = data.get_frame_images(frame_idx)
    images = selected_images(images, args.cameras)
    if not images:
        raise RuntimeError("No camera images selected. Check --cameras.")

    mosaic = make_mosaic(images, target_h=args.tile_height)
    if args.no_panel:
        return pad_to_even(mosaic)

    panel = draw_joint_panel(
        h=mosaic.shape[0],
        frame_idx=frame_idx,
        total=data.num_frames,
        joints=data.joints,
        show_velocity=args.show_velocity,
        show_effort=args.show_effort,
    )
    return pad_to_even(cv2.hconcat([mosaic, panel]))


def build_frame_indices(total_frames: int, args) -> range:
    start = max(0, min(args.start, total_frames - 1))
    end = total_frames if args.end is None else max(start, min(args.end, total_frames))
    stride = max(1, args.stride)

    if args.max_frames is not None:
        max_end = start + max(0, args.max_frames) * stride
        end = min(end, max_end)

    if end <= start:
        raise ValueError(f"Invalid frame range: start={start}, end={end}")
    return range(start, end, stride)


def main():
    args = parse_args()

    hdf5_path = os.path.abspath(os.path.expanduser(args.file))
    if not os.path.isfile(hdf5_path):
        raise FileNotFoundError(f"File not found: {hdf5_path}")

    output_path = os.path.abspath(os.path.expanduser(args.output)) if args.output else default_output_path(hdf5_path)
    if os.path.exists(output_path) and not args.overwrite:
        raise FileExistsError(f"Output already exists: {output_path}. Use --overwrite to replace it.")

    parent = os.path.dirname(output_path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    data = EpisodeViewerData(hdf5_path, show_depth=args.show_depth)
    writer = None
    try:
        frame_indices = build_frame_indices(data.num_frames, args)
        first_idx = frame_indices.start
        first_frame = render_frame(data, first_idx, args)

        fourcc = cv2.VideoWriter_fourcc(*args.codec[:4])
        writer = cv2.VideoWriter(output_path, fourcc, float(args.fps), (first_frame.shape[1], first_frame.shape[0]))
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {output_path}")

        print(f"Input : {hdf5_path}")
        print(f"Output: {output_path}")
        print(f"Frames: {len(frame_indices)} / {data.num_frames}, fps={args.fps}, size={first_frame.shape[1]}x{first_frame.shape[0]}")
        print(f"RGB cameras: {data.rgb_cams}")
        if args.show_depth:
            print(f"Depth cameras: {data.depth_cams}")

        for written, frame_idx in enumerate(frame_indices, start=1):
            frame = first_frame if frame_idx == first_idx else render_frame(data, frame_idx, args)
            writer.write(frame)

            if args.preview:
                cv2.imshow("HDF5 to Video", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in [ord("q"), 27]:
                    print("Stopped by user.")
                    break

            if args.progress_every > 0 and (written == 1 or written % args.progress_every == 0):
                print(f"Wrote {written}/{len(frame_indices)} frames, current frame={frame_idx}")

        print("Done.")
    finally:
        if writer is not None:
            writer.release()
        data.close()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
