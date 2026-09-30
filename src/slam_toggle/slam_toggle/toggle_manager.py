import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import math
import tf2_ros
from rclpy.duration import Duration

try:
    from tf2_geometry_msgs import do_transform_pose_stamped
except ImportError:
    do_transform_pose_stamped = None

class ToggleManager(Node):
    def __init__(self):
        super().__init__('toggle_manager')
        self.declare_parameter('arrive_distance_threshold', 0.9)
        self.declare_parameter('arrive_confirm_sec', 1.0)
        self.declare_parameter('publish_recovery_initialpose', False)
        self.declare_parameter('recovery_realign_threshold', 0.45)
        self.declare_parameter('recovery_goal_resend_delay_sec', 0.8)
        self.create_subscription(PoseStamped, '/goal_pose', self.global_goal_cb, 10)
        self.create_subscription(Bool, '/slam_degradation_flag', self.flag_cb, 10)
        self.create_subscription(Bool, '/fastlio/localization_ok', self.loc_ok_cb, 10)
        self.create_subscription(Bool, '/neupan/arrived', self.neupan_arrived_cb, 10)
        self.create_subscription(Odometry, '/neupan/current_odom', self.odom_cb, 10)
        self.create_subscription(Path, '/plan', self.global_plan_cb, 10)
        self.create_subscription(Path, '/neupan_plan', self.neupan_plan_cb, 10)
        self.create_subscription(Path, '/neupan_initial_path', self.neupan_initial_path_cb, 10)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/neupan/final_goal', 10)
        self.neupan_plan_input_pub = self.create_publisher(Path, '/neupan_plan_input_odom', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.override_pub = self.create_publisher(Twist, '/cmd_vel_override', 10)
        self.goal_cached_pub = self.create_publisher(Bool, '/hybrid/goal_cached', latched_qos)
        self.neupan_plan_viz_pub = self.create_publisher(Path, '/neupan_plan_viz_map', 10)
        self.neupan_initial_path_viz_pub = self.create_publisher(Path, '/neupan_initial_path_viz_map', 10)
        self.neupan_goal_viz_pub = self.create_publisher(Path, '/neupan_goal_viz_map', 10)
        self.neupan_goal_dist_pub = self.create_publisher(Float32, '/plot/neupan_goal_dist', 10)
        self.recovery_align_error_pub = self.create_publisher(Float32, '/plot/recovery_align_error', 10)
        self.nav2_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.is_degraded = False
        self.localization_ok = False
        self.arrived = False
        self.global_goal = None
        self.latest_global_plan = None
        self.goal_for_neupan = None
        self.goal_for_neupan_cached = None
        self.frozen_map_to_odom_tf = None
        self.frozen_odom_to_map_tf = None
        self.healthy_map_to_odom_tf = None
        self.healthy_odom_to_map_tf = None
        self.current_pose = None
        self.last_log_time = self.get_clock().now()
        self.last_near_check_time = None
        self.nav2_goal_handle = None
        self.nav2_goal_pending = False
        self.pending_nav2_goal = None
        self.arrive_distance_threshold = float(
            self.get_parameter('arrive_distance_threshold').value
        )
        self.arrive_confirm_sec = float(
            self.get_parameter('arrive_confirm_sec').value
        )
        self.publish_recovery_initialpose = bool(
            self.get_parameter('publish_recovery_initialpose').value
        )
        self.recovery_realign_threshold = float(
            self.get_parameter('recovery_realign_threshold').value
        )
        self.recovery_goal_resend_delay_sec = float(
            self.get_parameter('recovery_goal_resend_delay_sec').value
        )
        self.near_goal_duration = 0.0
        self.last_odom_time = None
        self.recovery_goal_timer = None
        self.recovery_hold_until = None

        self.brake_timer = self.create_timer(0.05, self.brake_loop)
        self.healthy_tf_timer = self.create_timer(0.2, self._update_healthy_transforms)
        self.publish_goal_cached(False)

    def _goal_for_neupan(self, msg: PoseStamped) -> PoseStamped:
        if msg.header.frame_id == 'odom' or do_transform_pose_stamped is None:
            return msg
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return do_transform_pose_stamped(msg, tf)
        except Exception as exc:
            self.get_logger().warn(f'Goal transform to odom failed, using raw goal: {exc}')
            return msg

    def _goal_for_neupan_frozen(self, msg: PoseStamped) -> PoseStamped:
        if msg.header.frame_id == 'odom' or do_transform_pose_stamped is None:
            return msg
        if msg.header.frame_id == 'map' and self.frozen_map_to_odom_tf is not None:
            try:
                return do_transform_pose_stamped(msg, self.frozen_map_to_odom_tf)
            except Exception as exc:
                self.get_logger().warn(f'Frozen goal transform to odom failed: {exc}')
        return self._goal_for_neupan(msg)

    def _viz_tf_odom_to_map(self):
        if self.is_degraded and self.frozen_odom_to_map_tf is not None:
            return self.frozen_odom_to_map_tf
        try:
            return self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None

    def _publish_viz_path(self, src_path: Path, pub):
        if not self.is_degraded:
            return

        if do_transform_pose_stamped is None or src_path is None:
            return

        tf = self._viz_tf_odom_to_map()
        if tf is None:
            return

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'

        for pose in src_path.poses:
            try:
                out.poses.append(do_transform_pose_stamped(pose, tf))
            except Exception:
                continue

        pub.publish(out)

    def _transform_path(self, src_path: Path, tf, frame_id: str) -> Path | None:
        if do_transform_pose_stamped is None or src_path is None or tf is None:
            return None

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = frame_id
        for pose in src_path.poses:
            try:
                out.poses.append(do_transform_pose_stamped(pose, tf))
            except Exception:
                continue
        return out

    def _publish_neupan_plan_input(self):
        if not self.is_degraded or self.latest_global_plan is None:
            return None
        if self.frozen_map_to_odom_tf is None:
            return None
        transformed = self._transform_path(
            self.latest_global_plan, self.frozen_map_to_odom_tf, 'odom'
        )
        if transformed is None or len(transformed.poses) == 0:
            return None
        self.neupan_plan_input_pub.publish(transformed)
        return transformed

    def _goal_from_path_endpoint(self, path: Path):
        if path is None or len(path.poses) == 0:
            return None
        endpoint = PoseStamped()
        endpoint.header = path.poses[-1].header
        endpoint.pose = path.poses[-1].pose
        return endpoint

    def _clear_viz_paths(self):
        empty = Path()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = 'map'
        self.neupan_plan_viz_pub.publish(empty)
        self.neupan_initial_path_viz_pub.publish(empty)
        self.neupan_goal_viz_pub.publish(empty)

    def _update_healthy_transforms(self):
        if self.is_degraded or not self.localization_ok:
            return
        try:
            self.healthy_map_to_odom_tf = self.tf_buffer.lookup_transform(
                'odom',
                'map',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            self.healthy_odom_to_map_tf = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return

    def _publish_goal_viz(self):
        if not self.is_degraded:
            return
        if self.goal_for_neupan is None or do_transform_pose_stamped is None:
            return
        tf = self._viz_tf_odom_to_map()
        if tf is None:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        try:
            out.poses.append(do_transform_pose_stamped(self.goal_for_neupan, tf))
        except Exception:
            return
        self.neupan_goal_viz_pub.publish(out)

    def _publish_recovery_initialpose(self):
        if self.current_pose is None or self.frozen_odom_to_map_tf is None or do_transform_pose_stamped is None:
            return False
        pose_odom = PoseStamped()
        pose_odom.header.stamp = self.get_clock().now().to_msg()
        pose_odom.header.frame_id = 'odom'
        pose_odom.pose = self.current_pose
        try:
            pose_map = do_transform_pose_stamped(pose_odom, self.frozen_odom_to_map_tf)
        except Exception as exc:
            self.get_logger().warn(f'Failed to build recovery initialpose: {exc}')
            return False

        msg = PoseWithCovarianceStamped()
        msg.header = pose_map.header
        msg.pose.pose = pose_map.pose
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.3,
        ]
        self.initialpose_pub.publish(msg)
        self.get_logger().info('Published recovery /initialpose from frozen odom->map handoff.')
        return True

    def _estimate_recovery_realign_error(self):
        if self.current_pose is None or self.frozen_odom_to_map_tf is None or do_transform_pose_stamped is None:
            return None
        pose_odom = PoseStamped()
        pose_odom.header.stamp = self.get_clock().now().to_msg()
        pose_odom.header.frame_id = 'odom'
        pose_odom.pose = self.current_pose
        try:
            frozen_pose_map = do_transform_pose_stamped(pose_odom, self.frozen_odom_to_map_tf)
            live_tf = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            live_pose_map = do_transform_pose_stamped(pose_odom, live_tf)
        except Exception:
            return None
        dx = frozen_pose_map.pose.position.x - live_pose_map.pose.position.x
        dy = frozen_pose_map.pose.position.y - live_pose_map.pose.position.y
        return math.hypot(dx, dy)

    def _schedule_recovery_goal_resend(self, delay_sec=None):
        if self.global_goal is None:
            return
        if delay_sec is None:
            delay_sec = self.recovery_goal_resend_delay_sec
        if self.recovery_goal_timer is not None:
            try:
                self.recovery_goal_timer.cancel()
            except Exception:
                pass
            self.recovery_goal_timer = None

        def _cb():
            if self.recovery_goal_timer is not None:
                try:
                    self.recovery_goal_timer.cancel()
                except Exception:
                    pass
                self.recovery_goal_timer = None
            if not self.is_degraded and self.global_goal is not None:
                self._send_nav2_goal(self.global_goal, 'recovery resend')

        self.recovery_goal_timer = self.create_timer(float(delay_sec), _cb)

    def _send_nav2_goal(self, goal_msg: PoseStamped, reason: str):
        if goal_msg is None:
            return
        if not self.nav2_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Nav2 action server not available; cannot send goal.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        self.nav2_goal_pending = True
        self.get_logger().info(f'Sending Nav2 goal ({reason}).')
        future = self.nav2_action_client.send_goal_async(goal)
        future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future):
        self.nav2_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.nav2_goal_handle = None
            self.get_logger().warn(f'Nav2 goal response failed: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self.nav2_goal_handle = None
            self.get_logger().warn('Nav2 goal rejected.')
            return
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Nav2 result callback failed: {exc}')
            self.nav2_goal_handle = None
            return

        self.nav2_goal_handle = None
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 goal succeeded (toggle_manager).')
            self.global_goal = None
            self.pending_nav2_goal = None
            self.goal_for_neupan = None
            self.goal_for_neupan_cached = None
            self.publish_goal_cached(False)
            self.arrived = True
            self.near_goal_duration = 0.0
            self.last_near_check_time = None
            self._clear_viz_paths()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Nav2 goal canceled (toggle_manager).')
        else:
            self.get_logger().warn(f'Nav2 goal ended with status {status}.')

    def _cancel_nav2_goal(self, reason: str):
        if self.nav2_goal_handle is None:
            return
        self.get_logger().warn(f'Canceling Nav2 goal ({reason}).')
        cancel_future = self.nav2_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _f: None)

    def loc_ok_cb(self, msg: Bool):
        prev = self.localization_ok
        self.localization_ok = bool(msg.data)
        if (
            self.localization_ok
            and not prev
            and not self.is_degraded
            and self.pending_nav2_goal is not None
        ):
            self.get_logger().info('Localization recovered; sending pending Nav2 goal.')
            pending_goal = self.pending_nav2_goal
            self._send_nav2_goal(pending_goal, 'pending resend after localization_ok')
            self.goal_for_neupan = self._goal_for_neupan(pending_goal)
            self.goal_for_neupan_cached = self.goal_for_neupan
            self.goal_pub.publish(self.goal_for_neupan)
            self.publish_goal_cached(True)
            self.pending_nav2_goal = None

    def neupan_arrived_cb(self, msg: Bool):
        if self.is_degraded and bool(msg.data) and not self.arrived:
            self.get_logger().warn('Arrived (NeuPAN)')
            self.arrived = True

    def publish_goal_cached(self, cached: bool):
        msg = Bool()
        msg.data = bool(cached)
        self.goal_cached_pub.publish(msg)

    def brake_loop(self):
        if self.recovery_hold_until is not None:
            if self.get_clock().now() < self.recovery_hold_until:
                stop_twist = Twist()
                self.override_pub.publish(stop_twist)
                return
            self.recovery_hold_until = None
        if self.arrived:
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.override_pub.publish(stop_twist)

    def global_goal_cb(self, msg):
        self.global_goal = msg
        self.pending_nav2_goal = None
        self.arrived = False
        self.near_goal_duration = 0.0
        self.last_near_check_time = None
        if not self.is_degraded and self.localization_ok:
            self._send_nav2_goal(msg, 'new RViz goal')
            self.goal_for_neupan = self._goal_for_neupan(msg)
            self.goal_for_neupan_cached = self.goal_for_neupan
            self.goal_pub.publish(self.goal_for_neupan)
            self.publish_goal_cached(True)
            self.get_logger().info('Received Global goal and cached stable odom goal for NeuPAN')
            return

        if not self.is_degraded and not self.localization_ok:
            self.pending_nav2_goal = msg
            # Even if raw localization_ok is temporarily false, keep a provisional
            # odom-frame fallback goal so auto-toggle can arm and NeuPAN has a
            # goal to take over with if degradation happens before recovery.
            self.goal_for_neupan = self._goal_for_neupan(msg)
            self.goal_for_neupan_cached = self.goal_for_neupan
            self.goal_pub.publish(self.goal_for_neupan)
            self.publish_goal_cached(True)
            self.get_logger().warn(
                'Received goal while localization_ok=false; holding Nav2 goal until localization recovers, '
                'but caching provisional odom fallback goal for hybrid takeover.'
            )
            return

        if self.is_degraded and self.frozen_map_to_odom_tf is not None:
            transformed_plan = self._publish_neupan_plan_input()
            self.goal_for_neupan = self._goal_from_path_endpoint(transformed_plan)
            if self.goal_for_neupan is None:
                self.goal_for_neupan = self._goal_for_neupan_frozen(msg)
            self.goal_for_neupan_cached = self.goal_for_neupan
            self.goal_pub.publish(self.goal_for_neupan)
            self._publish_goal_viz()
            self.publish_goal_cached(True)
            self.get_logger().warn(
                'Received goal while degraded; using frozen map->odom transform for NeuPAN goal.'
            )
            return

        if self.goal_for_neupan_cached is not None:
            self.goal_for_neupan = self.goal_for_neupan_cached
            self.goal_pub.publish(self.goal_for_neupan)
            self._publish_goal_viz()
            self.publish_goal_cached(True)
            self.get_logger().warn('Received goal while degraded; reusing last stable odom goal for NeuPAN')
        else:
            self.goal_for_neupan = None
            self.publish_goal_cached(False)
            self.get_logger().warn(
                'Received goal while degraded without cached stable odom goal; '
                'holding NeuPAN goal until localization becomes healthy.'
            )
            return
        self.get_logger().info('Received Global goal')

    def neupan_plan_cb(self, msg: Path):
        self._publish_viz_path(msg, self.neupan_plan_viz_pub)

    def neupan_initial_path_cb(self, msg: Path):
        self._publish_viz_path(msg, self.neupan_initial_path_viz_pub)

    def global_plan_cb(self, msg: Path):
        self.latest_global_plan = msg
        if self.is_degraded:
            self._publish_neupan_plan_input()

    def flag_cb(self, msg):
        if self.is_degraded == msg.data: return
        self.is_degraded = msg.data
        if self.is_degraded:
            self.get_logger().warn('Emergency')
            self._cancel_nav2_goal('switch to NeuPAN')
            self.pending_nav2_goal = None
            if self.healthy_map_to_odom_tf is not None and self.healthy_odom_to_map_tf is not None:
                self.frozen_map_to_odom_tf = self.healthy_map_to_odom_tf
                self.frozen_odom_to_map_tf = self.healthy_odom_to_map_tf
                self.get_logger().warn('Using last healthy map->odom transform for degraded-mode goals.')
            else:
                try:
                    self.frozen_map_to_odom_tf = self.tf_buffer.lookup_transform(
                        'odom',
                        'map',
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.2),
                    )
                    self.frozen_odom_to_map_tf = self.tf_buffer.lookup_transform(
                        'map',
                        'odom',
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.2),
                    )
                    self.get_logger().warn('Frozen current map->odom transform for degraded-mode goals.')
                except Exception as exc:
                    self.frozen_map_to_odom_tf = None
                    self.frozen_odom_to_map_tf = None
                    self.get_logger().warn(f'Failed to freeze map->odom transform: {exc}')
            if self.global_goal is not None and self.frozen_map_to_odom_tf is not None:
                transformed_plan = self._publish_neupan_plan_input()
                self.goal_for_neupan = self._goal_from_path_endpoint(transformed_plan)
                if self.goal_for_neupan is None:
                    self.goal_for_neupan = self._goal_for_neupan_frozen(self.global_goal)
                self.goal_for_neupan_cached = self.goal_for_neupan
                self.goal_pub.publish(self.goal_for_neupan)
                self._publish_goal_viz()
                self.publish_goal_cached(True)
                self.get_logger().warn('Recomputed stable odom goal from transformed global path endpoint for NeuPAN.')
            elif self.goal_for_neupan_cached is not None:
                self.goal_for_neupan = self.goal_for_neupan_cached
                self.goal_pub.publish(self.goal_for_neupan)
                self._publish_neupan_plan_input()
                self._publish_goal_viz()
                self.publish_goal_cached(True)
                self.get_logger().warn('Republished cached stable odom goal to NeuPAN')
            elif self.goal_for_neupan is not None:
                self.goal_pub.publish(self.goal_for_neupan)
                self._publish_neupan_plan_input()
                self._publish_goal_viz()
            else:
                self.publish_goal_cached(False)
        else:
            self.get_logger().info('SLAM repair')
            realign_error = self._estimate_recovery_realign_error()
            if (
                self.publish_recovery_initialpose
                and realign_error is not None
                and realign_error > self.recovery_realign_threshold
            ):
                if self._publish_recovery_initialpose():
                    self.recovery_hold_until = (
                        self.get_clock().now() + Duration(seconds=self.recovery_goal_resend_delay_sec)
                    )
                    self.get_logger().warn(
                        f'Recovery realign error {realign_error:.2f}m -> holding and re-anchoring before Nav2 resend.'
                    )
            self.frozen_map_to_odom_tf = None
            self.frozen_odom_to_map_tf = None
            self.arrived = False
            self.near_goal_duration = 0.0
            self.last_near_check_time = None
            self._clear_viz_paths()
            if self.global_goal is not None:
                self._schedule_recovery_goal_resend()

    def odom_cb(self, msg):
        if not self.is_degraded:
            return

        self.current_pose = msg.pose.pose
        align_error = self._estimate_recovery_realign_error()
        if align_error is not None:
            align_msg = Float32()
            align_msg.data = float(align_error)
            self.recovery_align_error_pub.publish(align_msg)
        if self.goal_for_neupan and not self.arrived:
            dx = self.goal_for_neupan.pose.position.x - self.current_pose.position.x
            dy = self.goal_for_neupan.pose.position.y - self.current_pose.position.y
            dist = math.hypot(dx, dy)
            dist_msg = Float32()
            dist_msg.data = float(dist)
            self.neupan_goal_dist_pub.publish(dist_msg)

            current_time = self.get_clock().now()
            if self.last_near_check_time is None:
                dt = 0.0
            else:
                dt = max(0.0, (current_time - self.last_near_check_time).nanoseconds / 1e9)
            self.last_near_check_time = current_time
            if (current_time - self.last_log_time).nanoseconds > 1e9:
                self.get_logger().info(f'Arrive distance: {dist:.2f}m')
                self.last_log_time = current_time
            # Do not force degraded-mode arrival purely from odom distance.
            # In the fallback phase, map<->odom mismatch can remain non-trivial,
            # and a loose distance threshold can stop the robot early before it
            # actually clears the last obstacle or reaches the final goal
            # region. The authoritative degraded arrival signal should come
            # from NeuPAN itself via /neupan/arrived. Keep the distance here
            # only as a debug/analysis signal for plotting and monitoring.

def main(args=None):
    rclpy.init(args=args)
    node = ToggleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
