#!/usr/bin/env python3
"""Convert the vision_align eye-in-hand session into GUI-compatible inputs."""

import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
VISION = Path("/mnt/w/Github/vision_align")
SESSION = VISION / "output/left_eye_in_hand_calib_20260805_160101"
OUTPUT = ROOT / "examples/data/vision_align_left_eye_in_hand"
RESULT = ROOT / "global_hand_eye_result.json"

K = np.array([[616.0441, 0.0, 639.7722], [0.0, 616.1274, 410.804], [0.0, 0.0, 1.0]])
D = np.array([-0.787511, 0.063795, 0.00044492, -0.000076, 0.114368, -0.766721, 0.034261, 0.125889])


def pose_yaml(translation, quaternion):
    values = [*translation, *quaternion]
    keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
    return "\n".join(f"{key}: {float(value):.12g}" for key, value in zip(keys, values)) + "\n"


def matrix_pose(matrix):
    return matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()


def main():
    result = json.loads(RESULT.read_text(encoding="utf-8"))
    sample_ids = result["sample_ids_used"]
    new_k = np.asarray(result["new_camera_matrix"], dtype=float)
    tcp_cam = np.asarray(result["T_tcp_cam_initial"], dtype=float)
    base_board = np.asarray(result["T_base_board_initial"], dtype=float)

    rows = {}
    with (SESSION / "samples_left_arm.csv").open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            rows[int(row["image_id"])] = row

    image_dir = OUTPUT / "images"
    pose_dir = OUTPUT / "poses"
    for directory in (image_dir, pose_dir):
        directory.mkdir(parents=True, exist_ok=True)

    observations = []
    for sample_id in sample_ids:
        raw = cv2.imread(str(SESSION / "left1" / f"{sample_id}_Color.png"))
        if raw is None:
            raise FileNotFoundError(f"Missing images {sample_id}")
        image = cv2.undistort(raw, K, D, None, new_k)
        cv2.imwrite(str(image_dir / f"{sample_id}.png"), image)

        row = rows[sample_id]
        translation = [row["tcp_x_m"], row["tcp_y_m"], row["tcp_z_m"]]
        quaternion = [row["tcp_qx"], row["tcp_qy"], row["tcp_qz"], row["tcp_qw"]]
        (pose_dir / f"{sample_id}.yaml").write_text(pose_yaml(translation, quaternion), encoding="utf-8")
        observations.append(f"  - image: images/{sample_id}.png\n    pose: poses/{sample_id}.yaml")

    tcp_t, tcp_q = matrix_pose(tcp_cam)
    board_t, board_q = matrix_pose(base_board)
    intr = new_k
    config = f"""# Derived from vision_align left eye-in-hand session.
# Images are full camera frames after lens undistortion.
data:
{chr(10).join(observations)}

target_finder:
  type: ModifiedCircleGridTargetFinder
  asymmetric: true
  rows: 5
  cols: 4
  spacing: 0.1
  circle_detector_params:
    minThreshold: 20
    maxThreshold: 240
    nThresholds: 20
    minRepeatability: 2
    circleInclusionRadius: 5.0
    maxRadiusDiff: 5.0
    maxAverageEllipseError: 0.1
    filterByColor: true
    circleColor: 255
    filterByArea: true
    minArea: 300.0
    maxArea: 15000.0
    filterByCircularity: true
    minCircularity: 0.6
    maxCircularity: 99999999.0
    filterByInertia: false
    minInertiaRatio: 0.1
    maxInertiaRatio: 99999999.0
    filterByConvexity: false
    minConvexity: 0.1
    maxConvexity: 99999999.0

homography_threshold: 2.0
intrinsics:
  fx: {intr[0, 0]:.12g}
  fy: {intr[1, 1]:.12g}
  cx: {intr[0, 2]:.12g}
  cy: {intr[1, 2]:.12g}

camera_mount_to_camera_guess:
{chr(10).join('  ' + line for line in pose_yaml(tcp_t, tcp_q).splitlines())}
target_mount_to_target_guess:
{chr(10).join('  ' + line for line in pose_yaml(board_t, board_q).splitlines())}
static_camera: false
"""
    (OUTPUT / "cal_data.yaml").write_text(config, encoding="utf-8")
    print(f"Wrote {len(sample_ids)} observations to {OUTPUT}")


if __name__ == "__main__":
    main()
