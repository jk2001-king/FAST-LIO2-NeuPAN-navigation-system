#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import rclpy
from rclpy.node import Node
import numpy as np

# NumPy 1.24+ 대응을 위한 패치
if not hasattr(np, 'float'):
    np.float = float

import tf2_ros
try:
    import tf_transformations
except ImportError:
    from tf2_ros import transformations as tf_transformations

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.declare_parameter('freq_pub_localization', 50.0)
        self.FREQ_PUB_LOCALIZATION = float(
            self.get_parameter('freq_pub_localization').value
        )

        self.lock = threading.Lock()
        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        # 구독자
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)

        # 퍼블리셔, 브로드캐스터
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        self.tf_broadcaster    = tf2_ros.TransformBroadcaster(self)

        # 주기 타이머
        period = 1.0 / self.FREQ_PUB_LOCALIZATION
        self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f'Transform Fusion Node Initialized @ {self.FREQ_PUB_LOCALIZATION:.1f} Hz'
        )

    def cb_save_cur_odom(self, msg: Odometry):
        with self.lock:
            self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg: Odometry):
        with self.lock:
            self.cur_map_to_odom = msg

    def pose_to_mat(self, odom_msg: Odometry) -> np.ndarray:
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
        rot   = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        return trans @ rot

    def timer_callback(self):
        with self.lock:
            odom    = copy.deepcopy(self.cur_odom_to_baselink)
            map2odom = copy.deepcopy(self.cur_map_to_odom)
            
        if odom is None:
            return

        # Nav2/controller는 "현재 시각" 기준 TF를 요구한다.
        # FAST-LIO /Odometry stamp가 약간 늦으면 map->odom이 stale 하다고 판단해
        # "Transform data too old when converting from odom to map"가 발생할 수 있다.
        # 따라서 TF와 fused localization은 현재 ROS 시간으로 stamp 한다.
        current_time = self.get_clock().now().to_msg()

        # map -> odom 계산
        T_map_to_odom = self.pose_to_mat(map2odom) if map2odom is not None else np.eye(4)
        trans = tf_transformations.translation_from_matrix(T_map_to_odom)
        quat  = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        # tf 브로드캐스트 (map -> odom)
        t_msg = TransformStamped()
        t_msg.header.stamp = current_time # 수정됨
        t_msg.header.frame_id    = 'map'
        t_msg.child_frame_id     = 'odom'
        t_msg.transform.translation.x = float(trans[0])
        t_msg.transform.translation.y = float(trans[1])
        t_msg.transform.translation.z = float(trans[2])
        t_msg.transform.rotation.x    = float(quat[0])
        t_msg.transform.rotation.y    = float(quat[1])
        t_msg.transform.rotation.z    = float(quat[2])
        t_msg.transform.rotation.w    = float(quat[3])
        self.tf_broadcaster.sendTransform(t_msg)

        # fused localization (map -> base_link)
        T_odom_to_base = self.pose_to_mat(odom)
        T_map_to_base  = T_map_to_odom @ T_odom_to_base
        xyz   = tf_transformations.translation_from_matrix(T_map_to_base)
        quat2 = tf_transformations.quaternion_from_matrix(T_map_to_base)

        loc_msg = Odometry()
        loc_msg.header.stamp          = current_time
        loc_msg.header.frame_id       = 'map'
        loc_msg.child_frame_id        = 'base_link'
        loc_msg.pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        loc_msg.pose.pose.orientation = Quaternion(
            x=quat2[0], y=quat2[1], z=quat2[2], w=quat2[3]
        )
        loc_msg.twist = odom.twist
        self.pub_localization.publish(loc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
