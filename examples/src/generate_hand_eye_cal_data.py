#!/usr/bin/env python3
"""Generate cal_data.yaml from matching images/ and poses/ files."""

import argparse
import csv
from pathlib import Path

import yaml


#ROOT = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[2] #加载模板
DEFAULT_TEMPLATE = ROOT / "examples/data/vision_align_left_eye_in_hand/cal_data.yaml"


def natural_key(path):
    return (0, int(path.stem)) if path.stem.isdigit() else (1, path.stem)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, help="directory containing images/ and poses/")
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--previous-result", type=Path, help="previous saved hand-eye result used as initial guesses")
    parser.add_argument(
        "--pose-csv",
        type=Path,
        help="optional samples CSV with image_id and tcp_x_m..tcp_qw columns; writes poses/*.yaml",
    )
    parser.add_argument("--output", type=Path, help="default: DATASET/cal_data.yaml")
    return parser.parse_args()


def main():
    args = parse_args()
    image_dir = args.dataset / "images"
    pose_dir = args.dataset / "poses"
    if not image_dir.is_dir():
        raise NotADirectoryError(f"Missing images directory: {image_dir}")

    if args.pose_csv:
        pose_dir.mkdir(parents=True, exist_ok=True)
        with args.pose_csv.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                pose = {
                    "x": float(row["tcp_x_m"]),
                    "y": float(row["tcp_y_m"]),
                    "z": float(row["tcp_z_m"]),
                    "qx": float(row["tcp_qx"]),
                    "qy": float(row["tcp_qy"]),
                    "qz": float(row["tcp_qz"]),
                    "qw": float(row["tcp_qw"]),
                }
                pose_file = pose_dir / f"{int(row['image_id'])}.yaml"
                pose_file.write_text(yaml.safe_dump(pose, sort_keys=False), encoding="utf-8")
    elif not pose_dir.is_dir():
        raise NotADirectoryError(f"Missing poses directory: {pose_dir}")

    image_extensions = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}
    images = sorted((p for p in image_dir.iterdir() if p.suffix.lower() in image_extensions), key=natural_key)
    if not images:
        raise FileNotFoundError(f"No images found in {image_dir}")

    data = []
    for image in images:
        pose = pose_dir / f"{image.stem}.yaml"
        if not pose.is_file():
            alternate = pose.with_suffix(".yml")
            if not alternate.is_file():
                raise FileNotFoundError(f"Missing pose for {image.name}: expected {pose.name}")
            pose = alternate
        data.append({"image": f"images/{image.name}", "pose": f"poses/{pose.name}"})

    config = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    config["data"] = data

    if args.previous_result:
        result = yaml.safe_load(args.previous_result.read_text(encoding="utf-8"))
        config["camera_mount_to_camera_guess"] = result["camera_mount_to_camera"]
        config["target_mount_to_target_guess"] = result["target_mount_to_target"]

    output = args.output or args.dataset / "cal_data.yaml"
    output.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    print(f"Wrote {len(data)} observations to {output}")
    print("Now check intrinsics and target_finder.spacing in that file.")


if __name__ == "__main__":
    main()
