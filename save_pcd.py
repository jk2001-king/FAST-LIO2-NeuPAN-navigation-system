#!/usr/bin/env python3
import os
import struct

import numpy as np
import open3d as o3d
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def pc2_to_xyz(points_msg: PointCloud2) -> np.ndarray:
    step = points_msg.point_step
    data = points_msg.data
    n = len(data) // step

    offsets = {field.name: field.offset for field in points_msg.fields}
    ox, oy, oz = offsets["x"], offsets["y"], offsets["z"]

    pts = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        pts[i, 0] = struct.unpack_from("<f", data, base + ox)[0]
        pts[i, 1] = struct.unpack_from("<f", data, base + oy)[0]
        pts[i, 2] = struct.unpack_from("<f", data, base + oz)[0]
    return pts


def write_pcd_open3d(path: str, pts: np.ndarray) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts.astype(np.float64, copy=False))
    o3d.io.write_point_cloud(path, cloud, write_ascii=False, compressed=False)


class SaveCloud(Node):
    def __init__(self):
        super().__init__("save_cloud_registered_pcd")
        self.declare_parameter("topic", "/cloud_registered")
        self.declare_parameter("out", "pcd/cloud_registered_accum1.pcd")
        self.declare_parameter("raw_out", "pcd/cloud_registered_accum_raw1.pcd")
        self.declare_parameter("save_raw_copy", True)
        self.declare_parameter("warmup_sec", 3.0)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("remove_outliers", True)
        self.declare_parameter("outlier_nb_neighbors", 20)
        self.declare_parameter("outlier_std_ratio", 2.0)
        self.declare_parameter("trim_percentile", 0.5)
        self.declare_parameter("trim_axes", ["x", "y"])
        self.declare_parameter("z_filter_min", 0.30)
        self.declare_parameter("z_filter_max", 1.60)
        self.declare_parameter("remove_far_floor", True)
        self.declare_parameter("max_sensor_range_m", 5.0)

        self.topic = self.get_parameter("topic").value
        self.out = self.get_parameter("out").value
        self.raw_out = self.get_parameter("raw_out").value
        self.save_raw_copy = bool(self.get_parameter("save_raw_copy").value)
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.remove_outliers = bool(self.get_parameter("remove_outliers").value)
        self.outlier_nb_neighbors = int(self.get_parameter("outlier_nb_neighbors").value)
        self.outlier_std_ratio = float(self.get_parameter("outlier_std_ratio").value)
        self.trim_percentile = float(self.get_parameter("trim_percentile").value)
        self.trim_axes = list(self.get_parameter("trim_axes").value)
        self.z_filter_min = float(self.get_parameter("z_filter_min").value)
        self.z_filter_max = float(self.get_parameter("z_filter_max").value)
        self.remove_far_floor = bool(self.get_parameter("remove_far_floor").value)
        self.max_sensor_range_m = float(self.get_parameter("max_sensor_range_m").value)

        self.accum = []
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.warmup_logged = False
        self.latest_position = None

        self.sub = self.create_subscription(PointCloud2, self.topic, self.cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/Odometry", self.odom_cb, 10)
        self.get_logger().info(f"Subscribing: {self.topic}")
        self.get_logger().info(f"Warmup skip: {self.warmup_sec:.1f}s")
        self.get_logger().info(f"Will save filtered map on exit to: {self.out}")
        self.get_logger().info(
            f"Filtered map Z range: [{self.z_filter_min:.2f}, {self.z_filter_max:.2f}]"
        )
        self.get_logger().info(f"Per-scan max sensor range: {self.max_sensor_range_m:.1f} m")
        if self.save_raw_copy:
            self.get_logger().info(f"Will also save raw accumulation to: {self.raw_out}")

    def in_warmup(self) -> bool:
        elapsed = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        return elapsed < self.warmup_sec

    def cb(self, msg: PointCloud2):
        try:
            if self.in_warmup():
                if not self.warmup_logged:
                    self.get_logger().info("Skipping early scans during warmup window.")
                    self.warmup_logged = True
                return

            pts = pc2_to_xyz(msg)
            pts = pts[np.isfinite(pts).all(axis=1)]
            if pts.size == 0:
                return

            pts = self.filter_scan_by_range(pts)
            if pts.size == 0:
                return

            self.accum.append(pts)
            if len(self.accum) % 20 == 0:
                total = sum(arr.shape[0] for arr in self.accum)
                self.get_logger().info(f"Accumulated chunks={len(self.accum)}, points≈{total}")
        except Exception as exc:
            self.get_logger().error(str(exc))

    def odom_cb(self, msg: Odometry):
        pose = msg.pose.pose.position
        self.latest_position = np.array([pose.x, pose.y, pose.z], dtype=np.float32)

    def robust_trim(self, pts: np.ndarray) -> np.ndarray:
        if pts.shape[0] < 1000 or self.trim_percentile <= 0.0:
            return pts

        trimmed = pts
        axis_map = {"x": 0, "y": 1, "z": 2}
        for axis_name in self.trim_axes:
            axis = axis_map.get(axis_name)
            if axis is None:
                continue
            lo = np.percentile(trimmed[:, axis], self.trim_percentile)
            hi = np.percentile(trimmed[:, axis], 100.0 - self.trim_percentile)
            trimmed = trimmed[(trimmed[:, axis] >= lo) & (trimmed[:, axis] <= hi)]
            if trimmed.size == 0:
                return pts
        return trimmed

    def filter_scan_by_range(self, pts: np.ndarray) -> np.ndarray:
        if self.max_sensor_range_m <= 0.0 or self.latest_position is None or pts.size == 0:
            return pts

        delta = pts - self.latest_position
        dist_xy = np.linalg.norm(delta[:, :2], axis=1)
        filtered = pts[dist_xy <= self.max_sensor_range_m]
        return filtered if filtered.size else pts

    def filter_map_points(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return pts

        filtered = pts[(pts[:, 2] >= self.z_filter_min) & (pts[:, 2] <= self.z_filter_max)]
        if filtered.size == 0:
            return pts

        if self.remove_far_floor:
            radius = np.linalg.norm(filtered[:, :2], axis=1)
            keep = np.ones(filtered.shape[0], dtype=bool)
            rules = (
                (20.0, 0.12),
                (40.0, 0.30),
                (80.0, 0.60),
            )
            for min_radius, min_z in rules:
                keep &= ~((radius >= min_radius) & (filtered[:, 2] < min_z))
            if keep.any():
                filtered = filtered[keep]

        return filtered

    def save(self):
        if not self.accum:
            self.get_logger().warn("No data accumulated. Nothing to save.")
            return

        pts = np.vstack(self.accum)
        raw_points = pts.shape[0]

        if self.save_raw_copy:
            write_pcd_open3d(self.raw_out, pts)
            self.get_logger().info(f"Saved raw PCD: {self.raw_out} (points={raw_points})")

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts.astype(np.float64, copy=False))

        if self.voxel_size > 0.0:
            cloud = cloud.voxel_down_sample(self.voxel_size)

        if self.remove_outliers and len(cloud.points) >= self.outlier_nb_neighbors:
            cloud, _ = cloud.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio,
            )

        filtered_pts = np.asarray(cloud.points, dtype=np.float32)
        z_filtered_pts = self.filter_map_points(filtered_pts)
        trimmed_pts = self.robust_trim(z_filtered_pts)
        write_pcd_open3d(self.out, trimmed_pts)

        self.get_logger().info(
            f"Saved filtered PCD: {self.out} "
            f"(raw_points={raw_points}, filtered_points={filtered_pts.shape[0]}, "
            f"z_filtered_points={z_filtered_pts.shape[0]}, "
            f"trimmed_points={trimmed_pts.shape[0]}, voxel={self.voxel_size})"
        )


def main():
    rclpy.init()
    node = SaveCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
