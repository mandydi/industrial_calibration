#!/usr/bin/env python3
"""Undistort calibration images and write them as PNG files."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="directory containing raw images")
    parser.add_argument("output", type=Path, help="output images directory")
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument(
        "--distortion",
        type=float,
        nargs="+",
        required=True,
        metavar="D",
        help="OpenCV distortion coefficients in camera calibration order",
    )
    parser.add_argument("--new-fx", type=float)
    parser.add_argument("--new-fy", type=float)
    parser.add_argument("--new-cx", type=float)
    parser.add_argument("--new-cy", type=float)
    parser.add_argument(
        "--strip-suffix",
        default="",
        help="remove this suffix from each input stem, for example _Color",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.input.is_dir():
        raise NotADirectoryError(args.input)

    camera_matrix = np.array(
        [[args.fx, 0.0, args.cx], [0.0, args.fy, args.cy], [0.0, 0.0, 1.0]], dtype=float
    )
    new_camera_matrix = np.array(
        [
            [args.new_fx or args.fx, 0.0, args.new_cx if args.new_cx is not None else args.cx],
            [0.0, args.new_fy or args.fy, args.new_cy if args.new_cy is not None else args.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    distortion = np.asarray(args.distortion, dtype=float)
    args.output.mkdir(parents=True, exist_ok=True)

    extensions = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}
    files = sorted(path for path in args.input.iterdir() if path.suffix.lower() in extensions)
    if not files:
        raise FileNotFoundError(f"No images found in {args.input}")

    for source in files:
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Failed to read {source}")
        undistorted = cv2.undistort(image, camera_matrix, distortion, None, new_camera_matrix)
        stem = source.stem
        if args.strip_suffix and stem.endswith(args.strip_suffix):
            stem = stem[: -len(args.strip_suffix)]
        destination = args.output / f"{stem}.png"
        if not cv2.imwrite(str(destination), undistorted):
            raise RuntimeError(f"Failed to write {destination}")

    print(f"Wrote {len(files)} undistorted images to {args.output}")
    print("Use these undistorted intrinsics in cal_data.yaml:")
    print(f"  fx: {new_camera_matrix[0, 0]:.12g}")
    print(f"  fy: {new_camera_matrix[1, 1]:.12g}")
    print(f"  cx: {new_camera_matrix[0, 2]:.12g}")
    print(f"  cy: {new_camera_matrix[1, 2]:.12g}")


if __name__ == "__main__":
    main()
