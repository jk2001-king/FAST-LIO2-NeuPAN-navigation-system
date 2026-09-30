#!/usr/bin/env python3
from pathlib import Path

import numpy as np
import open3d as o3d


INPUT_PCD = Path("pcd/cloud_registered_accum1.pcd")
OUTPUT_PCD = Path("pcd/map_lite1.pcd")
VOXEL_SIZE = 0.20
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 1.5
XY_TRIM_PERCENTILE = 0.5
Z_MIN = 0.45
Z_MAX = 1.40


def summarize(points: np.ndarray) -> str:
    if points.size == 0:
        return "empty"
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = maxs - mins
    return f"span=({span[0]:.2f}, {span[1]:.2f}, {span[2]:.2f})"


def robust_trim(points: np.ndarray) -> np.ndarray:
    if points.shape[0] < 1000 or XY_TRIM_PERCENTILE <= 0.0:
        return points

    trimmed = points
    for axis in (0, 1):
        lo = np.percentile(trimmed[:, axis], XY_TRIM_PERCENTILE)
        hi = np.percentile(trimmed[:, axis], 100.0 - XY_TRIM_PERCENTILE)
        trimmed = trimmed[(trimmed[:, axis] >= lo) & (trimmed[:, axis] <= hi)]
        if trimmed.size == 0:
            return points
    return trimmed


def main():
    if not INPUT_PCD.exists():
        raise FileNotFoundError(f"Input PCD not found: {INPUT_PCD}")

    pcd = o3d.io.read_point_cloud(str(INPUT_PCD))
    if pcd.is_empty():
        raise RuntimeError(f"Input PCD is empty: {INPUT_PCD}")

    raw_points = np.asarray(pcd.points)
    z_filtered_points = raw_points[(raw_points[:, 2] >= Z_MIN) & (raw_points[:, 2] <= Z_MAX)]
    if z_filtered_points.size == 0:
        raise RuntimeError(
            f"No points remain after lite-map Z filtering: [{Z_MIN:.2f}, {Z_MAX:.2f}]"
        )

    trimmed_points = robust_trim(z_filtered_points)
    trimmed_pcd = o3d.geometry.PointCloud()
    trimmed_pcd.points = o3d.utility.Vector3dVector(trimmed_points.astype(np.float64, copy=False))

    down_pcd = trimmed_pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    if len(down_pcd.points) >= OUTLIER_NB_NEIGHBORS:
        down_pcd, _ = down_pcd.remove_statistical_outlier(
            nb_neighbors=OUTLIER_NB_NEIGHBORS,
            std_ratio=OUTLIER_STD_RATIO,
        )

    OUTPUT_PCD.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(OUTPUT_PCD), down_pcd, write_ascii=False, compressed=False)

    lite_points = np.asarray(down_pcd.points)
    print(
        "✅ map_lite 생성 완료! "
        f"포인트: {len(raw_points)} -> {len(z_filtered_points)} -> {len(trimmed_points)} -> {len(lite_points)}, "
        f"raw_{summarize(raw_points)}, zf_{summarize(z_filtered_points)}, "
        f"trimmed_{summarize(trimmed_points)}, lite_{summarize(lite_points)}"
    )


if __name__ == "__main__":
    main()
