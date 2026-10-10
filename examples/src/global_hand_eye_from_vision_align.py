#!/usr/bin/env python3
"""Global 2D-3D eye-in-hand refinement for the vision_align data layout.

The optimized transforms use the convention

    p_tcp = T_tcp_cam @ p_cam
    p_base = T_base_board @ p_board

and minimize all rectified images-point residuals jointly:

    project(K_new, inv(T_tcp_cam) @ inv(T_base_tcp_i)
                   @ T_base_board @ p_board_j) - pixel_ij
"""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


IMAGE_SIZE = (1280, 800)
CAMERA_MATRIX = np.array(
    [[616.0441, 0.0, 639.7722], [0.0, 616.1274, 410.804], [0.0, 0.0, 1.0]], dtype=np.float64
)
DIST_COEFFS = np.array(
    [-0.787511, 0.063795, 0.00044492, -0.000076, 0.114368, -0.766721, 0.034261, 0.125889],
    dtype=np.float64,
)
T_TCP_CAM_INITIAL = np.array(
    [
        [0.025678, 0.999344, -0.025525, 0.094347],
        [-0.999669, 0.025702, 0.000618, 0.011310],
        [0.001274, 0.025500, 0.999674, -0.126398],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def make_transform(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def pose_to_parameters(transform):
    rotation_vector = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    return np.concatenate((rotation_vector, transform[:3, 3]))


def parameters_to_pose(parameters):
    return make_transform(Rotation.from_rotvec(parameters[:3]).as_matrix(), parameters[3:6])


def asymmetric_board_points(rows, cols, horizontal_spacing_m):
    """Match vision_align.get_board_3d_points exactly."""
    unit = horizontal_spacing_m / 2.0
    return np.array(
        [[(2 * col + row % 2) * unit, row * unit, 0.0] for row in range(rows) for col in range(cols)],
        dtype=np.float64,
    )


def read_robot_poses(csv_path):
    poses = {}
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            sample_id = int(row["image_id"])
            quaternion = np.array(
                [float(row["tcp_qx"]), float(row["tcp_qy"]), float(row["tcp_qz"]), float(row["tcp_qw"])],
                dtype=np.float64,
            )
            quaternion /= np.linalg.norm(quaternion)
            translation = np.array(
                [float(row["tcp_x_m"]), float(row["tcp_y_m"]), float(row["tcp_z_m"])], dtype=np.float64
            )
            poses[sample_id] = make_transform(Rotation.from_quat(quaternion).as_matrix(), translation)
    return poses


def read_image_points(points_dir):
    points = {}
    for path in points_dir.glob("*_Color.txt"):
        sample_id = int(path.name.split("_", 1)[0])
        value = np.loadtxt(path, dtype=np.float64).reshape(-1, 2)
        points[sample_id] = value
    return points


def mean_pose(transforms):
    rotations = Rotation.from_matrix(np.stack([transform[:3, :3] for transform in transforms]))
    translation = np.mean(np.stack([transform[:3, 3] for transform in transforms]), axis=0)
    return make_transform(rotations.mean().as_matrix(), translation)


def project_points(points_camera, intrinsics):
    normalized = points_camera[:, :2] / points_camera[:, 2:3]
    pixels = np.empty_like(normalized)
    pixels[:, 0] = intrinsics[0, 0] * normalized[:, 0] + intrinsics[0, 2]
    pixels[:, 1] = intrinsics[1, 1] * normalized[:, 1] + intrinsics[1, 2]
    return pixels


def residual_vector(parameters, robot_poses, board_points, image_points, intrinsics):
    tcp_to_camera = parameters_to_pose(parameters[:6])
    base_to_board = parameters_to_pose(parameters[6:12])
    camera_to_tcp = np.linalg.inv(tcp_to_camera)

    residuals = []
    board_h = np.column_stack((board_points, np.ones(len(board_points))))
    points_base = (base_to_board @ board_h.T).T
    for robot_pose, observed in zip(robot_poses, image_points):
        points_tcp = (np.linalg.inv(robot_pose) @ points_base.T).T
        points_camera = (camera_to_tcp @ points_tcp.T).T[:, :3]
        predicted = project_points(points_camera, intrinsics)
        residuals.append((predicted - observed).reshape(-1))
    return np.concatenate(residuals)


def error_statistics(residuals, sample_ids, points_per_sample):
    vectors = residuals.reshape(len(sample_ids), points_per_sample, 2)
    norms = np.linalg.norm(vectors, axis=2)
    return {
        "rms_px": float(np.sqrt(np.mean(np.square(norms)))),
        "mean_px": float(np.mean(norms)),
        "median_px": float(np.median(norms)),
        "max_px": float(np.max(norms)),
        "per_sample_mean_px": {str(sample_id): float(value) for sample_id, value in zip(sample_ids, norms.mean(axis=1))},
    }


def covariance_diagnostics(result, labels):
    jacobian = result.jac
    degrees_of_freedom = max(1, jacobian.shape[0] - jacobian.shape[1])
    residual_variance = 2.0 * result.cost / degrees_of_freedom
    covariance = residual_variance * np.linalg.pinv(jacobian.T @ jacobian)
    deviations = np.sqrt(np.maximum(np.diag(covariance), 0.0))
    denominator = np.outer(deviations, deviations)
    correlation = np.divide(covariance, denominator, out=np.zeros_like(covariance), where=denominator > 0.0)
    high = []
    for row in range(len(labels)):
        for col in range(row + 1, len(labels)):
            if abs(correlation[row, col]) >= 0.8:
                high.append({"a": labels[row], "b": labels[col], "correlation": float(correlation[row, col])})
    return covariance, correlation, deviations, sorted(high, key=lambda item: abs(item["correlation"]), reverse=True)


def matrix_as_list(matrix):
    return [[float(value) for value in row] for row in matrix]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--points-dir", type=Path, required=True)
    parser.add_argument("--poses-csv", type=Path, required=True)
    parser.add_argument("--horizontal-spacing-m", type=float, default=0.1)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--cols", type=int, default=4)
    parser.add_argument("--max-pnp-error-px", type=float, default=0.2)
    parser.add_argument("--output", type=Path, default=Path("global_hand_eye_result.json"))
    args = parser.parse_args()

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        CAMERA_MATRIX, DIST_COEFFS, IMAGE_SIZE, 0.0, IMAGE_SIZE, centerPrincipalPoint=False
    )
    board_points = asymmetric_board_points(args.rows, args.cols, args.horizontal_spacing_m)
    robot_by_id = read_robot_poses(args.poses_csv)
    raw_points_by_id = read_image_points(args.points_dir)
    sample_ids = sorted(set(robot_by_id) & set(raw_points_by_id))
    if set(robot_by_id) != set(raw_points_by_id):
        raise ValueError(
            f"Pose/point ID mismatch: pose-only={sorted(set(robot_by_id) - set(raw_points_by_id))}, "
            f"point-only={sorted(set(raw_points_by_id) - set(robot_by_id))}"
        )
    if not sample_ids:
        raise ValueError("No matching samples found")

    tcp_to_camera_initial = T_TCP_CAM_INITIAL.copy()
    tcp_to_camera_initial[:3, :3] = Rotation.from_matrix(tcp_to_camera_initial[:3, :3]).as_matrix()
    candidates = []
    kept = []
    pnp_errors = {}
    rectified_by_id = {}
    for sample_id in sample_ids:
        raw_points = raw_points_by_id[sample_id]
        if len(raw_points) != len(board_points):
            raise ValueError(f"Sample {sample_id}: expected {len(board_points)} points, got {len(raw_points)}")
        rectified = cv2.undistortPoints(
            raw_points.reshape(-1, 1, 2), CAMERA_MATRIX, DIST_COEFFS, P=new_camera_matrix
        ).reshape(-1, 2)
        rectified_by_id[sample_id] = rectified
        success, rotation_vector, translation = cv2.solvePnP(
            board_points, rectified, new_camera_matrix, None, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            pnp_errors[sample_id] = float("inf")
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        camera_to_board = make_transform(rotation, translation.reshape(3))
        reprojected, _ = cv2.projectPoints(board_points, rotation_vector, translation, new_camera_matrix, None)
        error = float(np.mean(np.linalg.norm(reprojected.reshape(-1, 2) - rectified, axis=1)))
        pnp_errors[sample_id] = error
        if error <= args.max_pnp_error_px:
            kept.append(sample_id)
            candidates.append(robot_by_id[sample_id] @ tcp_to_camera_initial @ camera_to_board)

    if len(kept) < 8:
        raise ValueError(f"Only {len(kept)} samples pass the PnP threshold: {pnp_errors}")

    base_to_board_initial = mean_pose(candidates)
    robot_poses = [robot_by_id[sample_id] for sample_id in kept]
    image_points = [rectified_by_id[sample_id] for sample_id in kept]
    initial_parameters = np.concatenate(
        (pose_to_parameters(tcp_to_camera_initial), pose_to_parameters(base_to_board_initial))
    )
    initial_residuals = residual_vector(initial_parameters, robot_poses, board_points, image_points, new_camera_matrix)
    result = least_squares(
        residual_vector,
        initial_parameters,
        args=(robot_poses, board_points, image_points, new_camera_matrix),
        method="trf",
        loss="linear",
        x_scale="jac",
        max_nfev=1000,
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
        verbose=1,
    )
    final_residuals = residual_vector(result.x, robot_poses, board_points, image_points, new_camera_matrix)
    tcp_to_camera_final = parameters_to_pose(result.x[:6])
    base_to_board_final = parameters_to_pose(result.x[6:12])
    labels = [
        "tcp_cam_rx", "tcp_cam_ry", "tcp_cam_rz", "tcp_cam_x", "tcp_cam_y", "tcp_cam_z",
        "base_board_rx", "base_board_ry", "base_board_rz", "base_board_x", "base_board_y", "base_board_z",
    ]
    covariance, correlation, deviations, high_correlations = covariance_diagnostics(result, labels)

    output = {
        "success": bool(result.success),
        "message": result.message,
        "sample_ids_used": kept,
        "sample_ids_excluded": [sample_id for sample_id in sample_ids if sample_id not in kept],
        "pnp_mean_error_px": {str(key): value for key, value in pnp_errors.items()},
        "new_camera_matrix": matrix_as_list(new_camera_matrix),
        "valid_roi": [int(value) for value in roi],
        "initial_error": error_statistics(initial_residuals, kept, len(board_points)),
        "final_error": error_statistics(final_residuals, kept, len(board_points)),
        "T_tcp_cam_initial": matrix_as_list(tcp_to_camera_initial),
        "T_tcp_cam_optimized": matrix_as_list(tcp_to_camera_final),
        "T_base_board_initial": matrix_as_list(base_to_board_initial),
        "T_base_board_optimized": matrix_as_list(base_to_board_final),
        "parameter_stddev_local": {label: float(value) for label, value in zip(labels, deviations)},
        "high_correlations_abs_ge_0_8": high_correlations,
        "covariance": matrix_as_list(covariance),
        "correlation": matrix_as_list(correlation),
    }
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False), encoding="utf-8")

    print("Used samples:", kept)
    print("Excluded samples:", output["sample_ids_excluded"])
    print("PnP mean errors (px):", pnp_errors)
    print("Initial error:", output["initial_error"])
    print("Final error:", output["final_error"])
    print("T_tcp_cam optimized:\n", np.array(output["T_tcp_cam_optimized"]))
    print("T_base_board optimized:\n", np.array(output["T_base_board_optimized"]))
    print("High correlations:", high_correlations)
    print("Wrote:", args.output.resolve())


if __name__ == "__main__":
    main()
