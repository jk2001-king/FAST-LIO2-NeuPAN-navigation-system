#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
import tf_transformations

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, Float32, Bool


class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('map_voxel_size', 0.1)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('freq_localization', 0.5)      # Hz
        self.declare_parameter('localization_th', 0.9)
        self.declare_parameter('fov', 2 * np.pi)
        self.declare_parameter('fov_far', 100.0)
        self.declare_parameter('stationary_linear_th', 0.03)
        self.declare_parameter('stationary_angular_th', 0.08)
        self.declare_parameter('accept_initial_guess_fallback', True)

        self.map_voxel_size    = self.get_parameter('map_voxel_size').value
        self.scan_voxel_size   = self.get_parameter('scan_voxel_size').value
        self.freq_localization = self.get_parameter('freq_localization').value
        self.localization_th   = self.get_parameter('localization_th').value
        self.FOV               = self.get_parameter('fov').value
        self.FOV_FAR           = self.get_parameter('fov_far').value
        self.stationary_linear_th = self.get_parameter('stationary_linear_th').value
        self.stationary_angular_th = self.get_parameter('stationary_angular_th').value
        self.accept_initial_guess_fallback = bool(
            self.get_parameter('accept_initial_guess_fallback').value
        )

        # ─── State Variables ─────────────────────────────────────
        self.global_map    = None
        self.initialized   = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom      = None
        self.cur_scan      = None
        self.pending_initial_pose = None
        self.pending_initial_pose_needs_attempt = False
        self.localization_timer = None
        self.last_icp_fitness = None
        self.last_residual_proxy = None
        self.last_localization_ok = False
        self.last_motion_pose = None
        self.last_motion_stamp = None
        self.estimated_linear_speed = 0.0
        self.estimated_angular_speed = 0.0

        # ─── Publishers ──────────────────────────────────────────
        self.pub_pc_in_map   = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap      = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry,     '/map_to_odom', 1)
        self.pub_icp_fitness = self.create_publisher(Float32, '/fastlio/icp_fitness', 10)
        self.pub_slam_res    = self.create_publisher(Float32, '/slam_res_mean', 10)
        self.pub_loc_ok      = self.create_publisher(Bool, '/fastlio/localization_ok', 10)

        # ─── Subscriptions ───────────────────────────────────────
        self.create_subscription(PointCloud2,                  '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry,                    '/Odometry',         self.cb_save_cur_odom,  1)
        self._map_sub  = self.create_subscription(PointCloud2, '/global_map',               self.cb_init_map,      1)
        self._init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.cb_init_pose,
            1
        )
        self.status_timer = self.create_timer(0.2, self.publish_status_snapshot)

        self.get_logger().info('GlobalLocalizationNode initialized.')

    def pc2_to_array(self, pc_msg: PointCloud2) -> np.ndarray:
        """PointCloud2 → (N×3) NumPy array"""
        pts = []
        for x, y, z in pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=True):
            pts.append((x, y, z))
        return np.array(pts, dtype=np.float32)

    def cb_init_map(self, msg: PointCloud2):
        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.global_map = self.voxel_down_sample(pcd, self.map_voxel_size)
        self.get_logger().info('Global map received and downsampled.')
        self.destroy_subscription(self._map_sub)
        self.try_initial_localization()

    def cb_init_pose(self, msg: PoseWithCovarianceStamped):
        self.pending_initial_pose = copy.deepcopy(msg)
        self.pending_initial_pose_needs_attempt = True
        self.initialized = False
        self.get_logger().info('Received /initialpose request.')
        self.try_initial_localization()

    def cb_save_cur_odom(self, msg: Odometry):
        self.cur_odom = msg
        self.update_motion_estimate(msg)
        self.try_initial_localization()

    def cb_save_cur_scan(self, msg: PointCloud2):
        msg.header.frame_id = 'camera_init'
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(msg)

        pts = self.pc2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        self.cur_scan = pcd
        self.try_initial_localization()

    def timer_callback(self):
        if self.cur_odom is None:
            return
        if self.is_stationary(self.cur_odom):
            return
        self.global_localization(self.T_map_to_odom)

    def is_stationary(self, odom_msg: Odometry) -> bool:
        twist = odom_msg.twist.twist
        linear_speed = np.linalg.norm([twist.linear.x, twist.linear.y, twist.linear.z])
        angular_speed = np.linalg.norm([twist.angular.x, twist.angular.y, twist.angular.z])

        # FAST-LIO /Odometry twist can stay near zero in sim even while pose changes.
        # Fall back to pose-delta based speed estimation in that case.
        linear_speed = max(linear_speed, self.estimated_linear_speed)
        angular_speed = max(angular_speed, self.estimated_angular_speed)
        return (
            linear_speed < self.stationary_linear_th
            and angular_speed < self.stationary_angular_th
        )

    def update_motion_estimate(self, odom_msg: Odometry):
        stamp = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        pos = odom_msg.pose.pose.position
        quat = odom_msg.pose.pose.orientation
        yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

        cur_pose = np.array([pos.x, pos.y, pos.z, yaw], dtype=np.float64)

        if self.last_motion_pose is None or self.last_motion_stamp is None:
            self.last_motion_pose = cur_pose
            self.last_motion_stamp = stamp
            return

        dt = stamp - self.last_motion_stamp
        if dt <= 1e-3:
            return

        dpos = cur_pose[:3] - self.last_motion_pose[:3]
        dyaw = cur_pose[3] - self.last_motion_pose[3]
        dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi

        self.estimated_linear_speed = float(np.linalg.norm(dpos) / dt)
        self.estimated_angular_speed = float(abs(dyaw) / dt)
        self.last_motion_pose = cur_pose
        self.last_motion_stamp = stamp

    def try_initial_localization(self):
        if not self.pending_initial_pose_needs_attempt or self.pending_initial_pose is None:
            return

        if self.global_map is None:
            self.get_logger().warn('Waiting for global map before initial localization.')
            return
        if self.cur_scan is None:
            self.get_logger().warn('Waiting for first scan before initial localization.')
            return
        if self.cur_odom is None:
            self.get_logger().warn('Waiting for first odometry before initial localization.')
            return

        # /initialpose is a map->base estimate. Convert it into a map->odom guess
        # because ICP refines the transform that maps FAST-LIO odom-frame scans into map.
        T_map_to_base_guess = self.pose_to_mat(self.pending_initial_pose)
        T_odom_to_base = self.pose_to_mat(self.cur_odom)
        initial_guess = T_map_to_base_guess @ np.linalg.inv(T_odom_to_base)

        success = self.global_localization(initial_guess)
        self.pending_initial_pose_needs_attempt = False
        if success:
            self.initialized = True
            self.pending_initial_pose = None
            if self.localization_timer is None:
                period = 1.0 / self.freq_localization
                self.localization_timer = self.create_timer(period, self.timer_callback)
            self.get_logger().info('Initial global localization succeeded.')
        else:
            if self.accept_initial_guess_fallback:
                self.T_map_to_odom = initial_guess
                self.publish_map_to_odom(initial_guess, self.cur_odom.header.stamp)
                self.publish_localization_ok(True)
                self.initialized = True
                self.pending_initial_pose = None
                if self.localization_timer is None:
                    period = 1.0 / self.freq_localization
                    self.localization_timer = self.create_timer(period, self.timer_callback)
                self.get_logger().warn(
                    'Initial ICP did not pass threshold; accepting /initialpose guess as fallback.'
                )
            else:
                self.get_logger().warn('Initial global localization failed. Please resend /initialpose.')

    def global_localization(self, pose_est):
        self.get_logger().info('Performing global localization via ICP...')
        scan_copy = copy.deepcopy(self.cur_scan)

        submap = self.crop_global_map_in_FOV(scan_copy, pose_est, self.cur_odom)

        T, _       = self.registration_at_scale(scan_copy, submap, initial=pose_est, scale=5)
        T, fitness = self.registration_at_scale(scan_copy, submap, initial=T,         scale=1)
        self.get_logger().info(f'ICP fitness: {fitness:.3f}')
        self.publish_localization_metrics(fitness)

        if fitness > self.localization_th:
            self.T_map_to_odom = T
            self.publish_map_to_odom(T, self.cur_odom.header.stamp)
            self.publish_localization_ok(True)
            return True

        self.get_logger().warn('Global localization failed (fitness below threshold).')
        self.publish_localization_ok(False)
        return False

    def publish_localization_metrics(self, fitness: float):
        self.last_icp_fitness = float(fitness)
        self.last_residual_proxy = float(max(0.0, 1.0 - fitness) * 0.15)

        fitness_msg = Float32()
        fitness_msg.data = self.last_icp_fitness
        self.pub_icp_fitness.publish(fitness_msg)

        # ICP fitness는 [0, 1] 범위라서 그대로 쓰면 res_norm(0.15)와 스케일이 맞지 않는다.
        # 토글용 proxy residual은 기존 목표 스케일(약 0.15)을 넘지 않도록 축소한다.
        res_msg = Float32()
        res_msg.data = self.last_residual_proxy
        self.pub_slam_res.publish(res_msg)

    def publish_localization_ok(self, ok: bool):
        self.last_localization_ok = bool(ok)
        msg = Bool()
        msg.data = self.last_localization_ok
        self.pub_loc_ok.publish(msg)

    def publish_status_snapshot(self):
        if self.last_icp_fitness is not None:
            msg = Float32()
            msg.data = float(self.last_icp_fitness)
            self.pub_icp_fitness.publish(msg)

        if self.last_residual_proxy is not None:
            msg = Float32()
            msg.data = float(self.last_residual_proxy)
            self.pub_slam_res.publish(msg)

        ok_msg = Bool()
        ok_msg.data = bool(self.last_localization_ok)
        self.pub_loc_ok.publish(ok_msg)

    def publish_map_to_odom(self, transform: np.ndarray, stamp):
        odom = Odometry()
        xyz = tf_transformations.translation_from_matrix(transform)
        quat = tf_transformations.quaternion_from_matrix(transform)
        odom.pose.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        self.pub_map_to_odom.publish(odom)

    def crop_global_map_in_FOV(self, scan, pose_est, odom):
        T_scan     = self.pose_to_mat(odom)
        T_map2scan = np.linalg.inv(pose_est @ T_scan)

        pts = np.asarray(self.global_map.points)
        hom = np.hstack([pts, np.ones((pts.shape[0],1))])
        pts_scan = (T_map2scan @ hom.T).T

        if self.FOV >= 2*np.pi:
            mask = (pts_scan[:,0] < self.FOV_FAR)
        else:
            ang  = np.arctan2(pts_scan[:,1], pts_scan[:,0])
            mask = (pts_scan[:,0]>0)&(pts_scan[:,0]<self.FOV_FAR)&(np.abs(ang)<self.FOV/2)

        subpts = pts[mask]
        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(subpts)

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = pc2.create_cloud_xyz32(header, subpts[::10].tolist())
        self.pub_submap.publish(cloud)

        return submap

    def registration_at_scale(self, scan, submap, initial, scale):
        def down(p): return p.voxel_down_sample(self.scan_voxel_size * scale)
        reg = o3d.pipelines.registration.registration_icp(
            down(scan), down(submap),
            max_correspondence_distance=1.0*scale,
            init=initial,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return reg.transformation, reg.fitness

    @staticmethod
    def pose_to_mat(pose_stamped):
        t = pose_stamped.pose.pose.position
        q = pose_stamped.pose.pose.orientation
        return tf_transformations.translation_matrix([t.x,t.y,t.z]) \
             @ tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])

    @staticmethod
    def voxel_down_sample(pcd, vs):
        try:
            return pcd.voxel_down_sample(vs)
        except:
            return o3d.geometry.voxel_down_sample(pcd, vs)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
