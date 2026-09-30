#!/usr/bin/env python

import os
import threading
import traceback
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2

try:
    from neupan import neupan
    from neupan.util import get_transform
except ImportError as e:
    raise ImportError(f"Failed to import 'neupan' package: {e}. Please install NeuPAN first.") from e

from neupan_ros2.visualization_manager import VisualizationManager
from neupan_ros2.utils import yaw_to_quat, quat_to_yaw

class NeupanCore(Node):
    def __init__(self) -> None:
        super().__init__("neupan_node")
        self._state_lock = threading.Lock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()
        self.pkg_dir = get_package_share_directory("neupan_ros2")

        self.declare_parameter("robot_type", "")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_config_dir", "")
        self.declare_parameter("planner_config_file", "planner.yaml")
        self.declare_parameter("dune_checkpoint_file", "models/dune_model_5000.pth")
        self.declare_parameter("neupan_config_file", "NOT SET")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "laser_link")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("marker_z", 1.0)
        self.declare_parameter("scan_angle_max", 3.14)
        self.declare_parameter("scan_angle_min", -3.14)
        self.declare_parameter("scan_downsample", 1)
        self.declare_parameter("scan_range_min", 0.1)
        self.declare_parameter("scan_range_max", 5.0)
        self.declare_parameter("refresh_initial_path", False)
        self.declare_parameter("flip_angle", False)
        self.declare_parameter("include_initial_path_direction", False)
        self.declare_parameter("control_frequency", 50.0)

        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("enable_dune_markers", True)
        self.declare_parameter("enable_nrmp_markers", True)
        self.declare_parameter("enable_robot_marker", True)

        self.declare_parameter("cmd_vel_topic", "/neupan_cmd_vel")
        self.declare_parameter("plan_output_topic", "/neupan_plan")
        self.declare_parameter("ref_state_topic", "/neupan_ref_state")
        self.declare_parameter("initial_path_topic", "/neupan_initial_path")
        self.declare_parameter("dune_markers_topic", "/dune_point_markers")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("nrmp_markers_topic", "/nrmp_point_markers")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_input_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        # TF 축 정의가 반대로 들어온 경우에만 사용 (기본은 끔)
        self.declare_parameter("yaw_flip_180", False)
        # 로봇이 뒤로만 가는 등 Twist 축이 반대일 때만 사용 (기본은 끔)
        self.declare_parameter("invert_cmd_linear_x", False)
        self.declare_parameter("invert_cmd_angular_z", False)
        self.declare_parameter("ignore_goal_orientation", False)
        # 출력 Twist 안정화 (과도한 회전으로 제자리 빙빙 방지)
        self.declare_parameter("max_abs_angular_z", 0.6)
        self.declare_parameter("max_linear_x", 0.5)
        self.declare_parameter("min_linear_x", 0.05)
        self.declare_parameter("turn_slowdown_gain", 1.5)

        robot_config_dir = self.get_parameter("robot_config_dir").get_parameter_value().string_value
        planner_config_file = self.get_parameter("planner_config_file").get_parameter_value().string_value
        self.planner_config_file = os.path.join(robot_config_dir, planner_config_file)
        dune_checkpoint_file = self.get_parameter("dune_checkpoint_file").get_parameter_value().string_value
        self.dune_checkpoint = os.path.join(robot_config_dir, dune_checkpoint_file)

        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value
        self.scan_range = np.array([
            self.get_parameter("scan_range_min").get_parameter_value().double_value,
            self.get_parameter("scan_range_max").get_parameter_value().double_value
        ])
        self.scan_downsample = self.get_parameter("scan_downsample").get_parameter_value().integer_value
        self.refresh_initial_path = self.get_parameter("refresh_initial_path").get_parameter_value().bool_value
        self.include_initial_path_direction = self.get_parameter("include_initial_path_direction").get_parameter_value().bool_value

        self.yaw_flip_180 = self.get_parameter("yaw_flip_180").get_parameter_value().bool_value
        self.invert_cmd_linear_x = self.get_parameter("invert_cmd_linear_x").get_parameter_value().bool_value
        self.invert_cmd_angular_z = self.get_parameter("invert_cmd_angular_z").get_parameter_value().bool_value
        self.ignore_goal_orientation = self.get_parameter("ignore_goal_orientation").get_parameter_value().bool_value
        self.max_abs_angular_z = float(self.get_parameter("max_abs_angular_z").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.min_linear_x = float(self.get_parameter("min_linear_x").value)
        self.turn_slowdown_gain = float(self.get_parameter("turn_slowdown_gain").value)

        self.enable_visualization = self.get_parameter("enable_visualization").get_parameter_value().bool_value
        self.enable_dune_markers = self.get_parameter("enable_dune_markers").get_parameter_value().bool_value
        self.enable_nrmp_markers = self.get_parameter("enable_nrmp_markers").get_parameter_value().bool_value
        self.enable_robot_marker = self.get_parameter("enable_robot_marker").get_parameter_value().bool_value

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)
        self.get_logger().info("✅ NeuPAN planner initialized successfully")

        self.obstacle_points: Optional[npt.NDArray] = None
        self.robot_state: Optional[npt.NDArray] = None
        self.stop: bool = False
        self.arrive: bool = False
        self.goal: Optional[npt.NDArray] = None

        self.vel_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").get_parameter_value().string_value, 10)
        self.plan_pub = self.create_publisher(Path, self.get_parameter("plan_output_topic").get_parameter_value().string_value, 10)
        self.ref_state_pub = self.create_publisher(Path, self.get_parameter("ref_state_topic").get_parameter_value().string_value, 10)
        self.ref_path_pub = self.create_publisher(Path, self.get_parameter("initial_path_topic").get_parameter_value().string_value, 10)
        self.arrive_pub = self.create_publisher(Bool, '/neupan/arrived', 10)

        viz_config = {
            'enable_visualization': self.enable_visualization,
            'enable_dune_markers': self.enable_dune_markers,
            'enable_nrmp_markers': self.enable_nrmp_markers,
            'enable_robot_marker': self.enable_robot_marker,
            'map_frame': self.map_frame,
            'marker_size': self.marker_size,
            'marker_z': self.marker_z,
            'dune_markers_topic': self.get_parameter("dune_markers_topic").get_parameter_value().string_value,
            'nrmp_markers_topic': self.get_parameter("nrmp_markers_topic").get_parameter_value().string_value,
            'robot_marker_topic': self.get_parameter("robot_marker_topic").get_parameter_value().string_value,
            'state_lock': self._state_lock
        }
        self.viz_manager = VisualizationManager(self, viz_config)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        scan_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, self.get_parameter("scan_topic").get_parameter_value().string_value, self.scan_callback, scan_qos_profile, callback_group=self.callback_group)
        self.create_subscription(Path, self.get_parameter("plan_input_topic").get_parameter_value().string_value, self.path_callback, 10, callback_group=self.callback_group)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").get_parameter_value().string_value, self.goal_callback, 10, callback_group=self.callback_group)

        self.control_frequency = self.get_parameter("control_frequency").get_parameter_value().double_value
        time_period = 1.0 / self.control_frequency
        self.create_timer(time_period, self.run, callback_group=self.control_group)

    def _get_robot_transform(self) -> bool:
        try:
            # 맵(camera_init)에서 로봇(body)까지의 변환을 가져옵니다.
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # [수정] float()으로 확실히 변환하고 np.pi를 사용하여 에러 방지
            yaw = float(quat_to_yaw(trans.transform.rotation))

            # TF 축 정의가 반대로 들어오는 환경에서만 180도 뒤집기 옵션을 사용합니다.
            if self.yaw_flip_180:
                yaw += np.pi
                if yaw > np.pi:
                    yaw -= 2 * np.pi
                elif yaw < -np.pi:
                    yaw += 2 * np.pi

            new_state = np.array([x, y, yaw]).reshape(3, 1)
            with self._state_lock:
                self.robot_state = new_state
            
            # 주행 시작 시 딱 한 번만 로그를 찍어 확인합니다.
            self.get_logger().info(f"✅ Robot state initialized! Yaw: {yaw:.2f}rad", once=True)
            return True
        except Exception as e:
            # 에러가 나면 왜 나는지 로그를 찍어줍니다.
            # self.get_logger().warn(f"TF Wait: {e}", throttle_duration_sec=2.0)
            return False

    def _validate_planning_prerequisites(self) -> bool:
        with self._state_lock:
            if self.robot_state is None: 
                return False
            if len(self.neupan_planner.waypoints) >= 1 and self.neupan_planner.initial_path is None:
                self.neupan_planner.set_initial_path_from_state(self.robot_state)
            if self.neupan_planner.initial_path is None: 
                self.get_logger().warn("⚠️ 경로(Path)가 생성되지 않았습니다. Goal을 기다립니다...", throttle_duration_sec=2.0)
                return False
        return True

    def _execute_planning(self) -> Tuple[Optional[npt.NDArray], Dict[str, Any]]:
        with self._state_lock:
            robot_state_copy = self.robot_state.copy() if self.robot_state is not None else None
            obstacle_points_copy = self.obstacle_points.copy() if self.obstacle_points is not None else None
        
        action, info = self.neupan_planner(robot_state_copy, obstacle_points_copy)

        with self._state_lock:
            self.stop = info["stop"]
            self.arrive = info["arrive"]

        # [디버깅 로그] 실제로 어떤 속도 명령이 생성되는지 확인
        if action is not None:
            v, w = action[0, 0], action[1, 0]
            if info["stop"]:
                self.get_logger().warn(f"🛑 충돌 위험으로 정지 중 (V:{v:.2f}, W:{w:.2f})", throttle_duration_sec=2.0)
            elif not info["arrive"]:
                self.get_logger().info(f"🚀 주행 중 - 속도: {v:.2f}m/s, 회전: {w:.2f}rad/s", throttle_duration_sec=2.0)

        return action, info

    def _publish_planning_results(self, action: Optional[npt.NDArray], info: Dict[str, Any]) -> None:
        if self.neupan_planner.initial_path is not None:
            self.ref_path_pub.publish(self.generate_path_msg(self.neupan_planner.initial_path))
        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
        arrive_msg = Bool()
        arrive_msg.data = bool(info["arrive"])
        self.arrive_pub.publish(arrive_msg)
        vel_msg = self.generate_twist_msg(action, info["stop"], info["arrive"])
        self.vel_pub.publish(vel_msg)
        self.viz_manager.publish_visualization(self.neupan_planner, self.robot_state)

    def run(self) -> None:
        if not self._get_robot_transform(): return
        if not self._validate_planning_prerequisites(): return
        action, info = self._execute_planning()
        self._publish_planning_results(action, info)

    def scan_callback(self, scan_msg: PointCloud2) -> Optional[npt.NDArray]:
        # 현재 로봇 위치가 없으면 계산 안 함
        with self._state_lock:
            if self.robot_state is None: return None
            curr_state = self.robot_state.copy()

        try:
            gen = pc2.read_points(scan_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(gen)
        except Exception: return None
        if not points_list: return None

        x_raw = np.array([p[0] for p in points_list])
        y_raw = np.array([p[1] for p in points_list])
        z_raw = np.array([p[2] for p in points_list])

        # [필터 1] 바닥을 과하게 자르지 않도록 낮은 장애물까지 포함
        z_mask = (z_raw >= 0.05) & (z_raw <= 1.2)
        
        # [필터 2] 로봇 몸체는 피하되, 가까운 장애물은 늦지 않게 포함
        dist = np.hypot(x_raw, y_raw)
        range_mask = (dist >= 0.25) & (dist <= self.scan_range[1])

        valid_mask = z_mask & range_mask
        if not np.any(valid_mask):
            with self._state_lock: self.obstacle_points = None
            return None

        point_array = np.vstack([x_raw[valid_mask], y_raw[valid_mask]])

        # 점 개수 압축 (최대 1200개)
        if point_array.shape[1] > 1200:
            idx = np.linspace(0, point_array.shape[1]-1, 1200, dtype=int)
            point_array = point_array[:, idx]

        # [필터 3] 로봇의 현재 뇌(robot_state)와 장애물의 위치를 강제로 동기화
        try:
            # 로봇의 현재 상태 [x, y, yaw]
            rx, ry, ryaw = curr_state[0, 0], curr_state[1, 0], curr_state[2, 0]
            
            # 로봇 위치를 기준으로 점들을 맵 좌표계로 변환
            trans_m, rot_m = get_transform(np.c_[rx, ry, ryaw].reshape(3, 1))
            transformed_points = rot_m @ point_array + trans_m

            with self._state_lock:
                self.obstacle_points = transformed_points
            
            self.get_logger().info(f"📊 3D->2D 투영 완료 (점: {transformed_points.shape[1]}개)", throttle_duration_sec=2.0)
            return transformed_points
        except Exception:
            return None

    def path_callback(self, path: Path) -> None:
        n_poses = len(path.poses)
        if n_poses == 0: return
        
        if self.include_initial_path_direction:
            data = [(p.pose.position.x, p.pose.position.y, quat_to_yaw(p.pose.orientation)) for p in path.poses]
            xs, ys, thetas = np.array(data).T
        else:
            coords = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
            xs, ys = np.array(coords).T
            dx = np.diff(xs, append=xs[-1])
            dy = np.diff(ys, append=ys[-1])
            thetas = np.arctan2(dy, dx)
            if n_poses > 1: thetas[-1] = thetas[-2]

        ones = np.ones(n_poses)
        initial_point_array = np.vstack([xs, ys, thetas, ones])
        initial_point_list = [initial_point_array[:, i:i + 1] for i in range(n_poses)]

        with self._state_lock:
            if self.neupan_planner.initial_path is None or self.refresh_initial_path:
                self.neupan_planner.set_initial_path(initial_point_list)

    def goal_callback(self, goal: PoseStamped) -> None:
        x, y = goal.pose.position.x, goal.pose.position.y
        theta = quat_to_yaw(goal.pose.orientation)
        if self.ignore_goal_orientation and self.robot_state is not None:
            theta = float(self.robot_state[2, 0])
        new_goal = np.array([[x], [y], [theta]])

        # [중요 로그 복구] 목표를 받았을 때 상태를 알려줍니다.
        self.get_logger().info(f"🎯 RViz Goal 접수 완료! [x:{x:.2f}, y:{y:.2f}]")

        if self.robot_state is None:
            self.get_logger().error("🛑 에러: 로봇 위치(TF)를 몰라서 Goal을 무시합니다. (TF 에러 로그를 확인하세요)")
            self.goal = new_goal
            return

        with self._state_lock:
            self.goal = new_goal
            self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)
            self.neupan_planner.reset()
            if self.neupan_planner.initial_path is not None:
                self.ref_path_pub.publish(self.generate_path_msg(self.neupan_planner.initial_path))
            self.get_logger().info("✅ Goal을 향해 주행 계산을 시작합니다!")

    def generate_path_msg(self, path_list: List[npt.NDArray]) -> Path:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if len(path_list) == 0: return path

        normalized_points = []
        for point in path_list:
            point_arr = np.array(point)
            if point_arr.ndim == 1: point_arr = point_arr.reshape(-1, 1)
            point_arr = point_arr[:3, :]
            normalized_points.append(point_arr)

        points_matrix = np.hstack(normalized_points)
        for x, y, yaw in zip(points_matrix[0, :], points_matrix[1, :], points_matrix[2, :]):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = yaw_to_quat(float(yaw))
            path.poses.append(ps)
        return path

    def generate_twist_msg(self, vel: Optional[npt.NDArray], stop: bool, arrive: bool) -> Twist:
        if vel is None or stop or arrive: return Twist()
        action = Twist()
        vx = float(vel[0, 0])
        wz = float(vel[1, 0])
        if self.invert_cmd_linear_x:
            vx = -vx
        if self.invert_cmd_angular_z:
            wz = -wz

        # 1) 회전 속도 포화 (기본 0.6rad/s)
        if self.max_abs_angular_z > 0.0:
            wz = float(np.clip(wz, -self.max_abs_angular_z, self.max_abs_angular_z))

        # 2) 회전이 크면 전진 속도 자동 감쇠 (빙빙/요동 방지)
        # v := v / (1 + k*|w|)
        if self.turn_slowdown_gain > 0.0:
            vx = vx / (1.0 + self.turn_slowdown_gain * abs(wz))

        # 3) 전진 속도 상/하한
        if self.max_linear_x > 0.0:
            vx = float(np.clip(vx, -self.max_linear_x, self.max_linear_x))
        if abs(vx) < self.min_linear_x:
            vx = float(np.sign(vx) * self.min_linear_x) if vx != 0.0 else 0.0

        action.linear.x = vx
        action.angular.z = wz
        return action

def main(args=None):
    rclpy.init(args=args)
    neupan_node, executor = None, None
    try:
        neupan_node = NeupanCore()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(neupan_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor: executor.shutdown()
        if neupan_node: neupan_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
