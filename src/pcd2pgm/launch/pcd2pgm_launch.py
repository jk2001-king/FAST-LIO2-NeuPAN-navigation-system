#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import yaml


PCD_PATH = Path("pcd/cloud_registered_accum1.pcd")
PGM_OUTPUT = Path("src/nav2/2dmap/fastlio_map_2d_1.pgm")
YAML_OUTPUT = Path("src/nav2/2dmap/fastlio_map_2d_1.yaml")

RESOLUTION = 0.05
Z_MIN = 0.30
Z_MAX = 1.60
MIN_POINTS_PER_CELL = 3
OBSTACLE_DILATION_PX = 1
CLOSING_KERNEL_PX = 3
XY_TRIM_PERCENTILE = 0.0
MIN_COMPONENT_AREA_PX = 25


def robust_crop(points: np.ndarray) -> np.ndarray:
    if points.shape[0] < 1000 or XY_TRIM_PERCENTILE <= 0.0:
        return points

    cropped = points
    for axis in (0, 1):
        lo = np.percentile(cropped[:, axis], XY_TRIM_PERCENTILE)
        hi = np.percentile(cropped[:, axis], 100.0 - XY_TRIM_PERCENTILE)
        cropped = cropped[(cropped[:, axis] >= lo) & (cropped[:, axis] <= hi)]
        if cropped.size == 0:
            return points
    return cropped


def main():
    if not PCD_PATH.exists():
        raise FileNotFoundError(f"Input PCD not found: {PCD_PATH}")

    pcd = o3d.io.read_point_cloud(str(PCD_PATH))
    points = np.asarray(pcd.points)
    if points.size == 0:
        raise RuntimeError(f"Input PCD is empty: {PCD_PATH}")

    points = points[(points[:, 2] > Z_MIN) & (points[:, 2] < Z_MAX)]
    if points.size == 0:
        raise RuntimeError("No points remain after Z filtering.")

    points = robust_crop(points)

    x_points = points[:, 0]
    y_points = points[:, 1]

    min_x, max_x = float(np.min(x_points)), float(np.max(x_points))
    min_y, max_y = float(np.min(y_points)), float(np.max(y_points))

    width = int(np.ceil((max_x - min_x) / RESOLUTION)) + 1
    height = int(np.ceil((max_y - min_y) / RESOLUTION)) + 1

    grid = np.full((height, width), 255, dtype=np.uint8)
    px = np.clip(((x_points - min_x) / RESOLUTION).astype(np.int32), 0, width - 1)
    py = np.clip(((y_points - min_y) / RESOLUTION).astype(np.int32), 0, height - 1)
    counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(counts, (height - 1 - py, px), 1)
    grid[counts >= MIN_POINTS_PER_CELL] = 0

    if OBSTACLE_DILATION_PX > 0:
        occ = (grid == 0).astype(np.uint8) * 255
        kernel = np.ones((2 * OBSTACLE_DILATION_PX + 1, 2 * OBSTACLE_DILATION_PX + 1), np.uint8)
        occ = cv2.dilate(occ, kernel, iterations=1)
        if CLOSING_KERNEL_PX > 1:
            closing_kernel = np.ones((CLOSING_KERNEL_PX, CLOSING_KERNEL_PX), np.uint8)
            occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, closing_kernel)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
        filtered_occ = np.zeros_like(occ)
        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area >= MIN_COMPONENT_AREA_PX:
                filtered_occ[labels == label] = 255
        occ = filtered_occ
        grid = np.where(occ > 0, 0, 255).astype(np.uint8)

    PGM_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(PGM_OUTPUT), grid)

    yaml_data = {
        "image": PGM_OUTPUT.name,
        "mode": "trinary",
        "resolution": RESOLUTION,
        "origin": [min_x, min_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }

    with open(YAML_OUTPUT, "w", encoding="utf-8") as file_obj:
        yaml.dump(yaml_data, file_obj, default_flow_style=False, sort_keys=False)

    print(
        f"완료! Origin 좌표는 [{min_x:.6f}, {min_y:.6f}, 0.0] 입니다. "
        f"size=({width * RESOLUTION:.2f}m x {height * RESOLUTION:.2f}m), "
        f"points={points.shape[0]}"
    )


if __name__ == "__main__":
    main()
