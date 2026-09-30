#!/usr/bin/env python3

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file_path', 'pcd/cloud_registered_accum.pcd')
        self.declare_parameter('interval', 5)
        self.declare_parameter('display_voxel_size', 0.15)
        self.declare_parameter('display_min_z', 0.45)
        self.declare_parameter('display_max_z', 1.40)
        path = self.get_parameter('map_file_path').value
        interval = self.get_parameter('interval').value
        display_voxel_size = float(self.get_parameter('display_voxel_size').value)
        display_min_z = float(self.get_parameter('display_min_z').value)
        display_max_z = float(self.get_parameter('display_max_z').value)
        
        self.global_map = None
        if path:
            try:
                self.global_map = o3d.io.read_point_cloud(path)
                if not self.global_map.is_empty():
                    points = np.asarray(self.global_map.points)
                    points = points[(points[:, 2] >= display_min_z) & (points[:, 2] <= display_max_z)]
                    filtered_map = o3d.geometry.PointCloud()
                    filtered_map.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
                    self.global_map = filtered_map
                if display_voxel_size > 0.0 and not self.global_map.is_empty():
                    self.global_map = self.global_map.voxel_down_sample(display_voxel_size)
                self.get_logger().info(f'Loaded map from: {path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load PCD: {e}')
        else:
            self.get_logger().warn('No map_file_path provided; map not loaded')

        self.pub_map = self.create_publisher(PointCloud2, '/global_map', 1)
        self.create_timer(interval, self.publish_map)
        self.get_logger().info(f'Interval for publishing map: {interval} seconds')
        self.get_logger().info('Map Publisher Node Initialized')

    def publish_map(self):
        if self.global_map is None:
            self.get_logger().warn('Global map is not loaded; skipping publish')
            return
        points = np.asarray(self.global_map.points)
        if points.size == 0:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        try:
            cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        except AttributeError:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud = point_cloud2.create_cloud(header, fields, points.tolist())
        self.pub_map.publish(cloud)
        # self.get_logger().info('Published global map')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()        
