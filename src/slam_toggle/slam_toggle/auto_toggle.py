import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
import math

class AutoToggler(Node):
    def __init__(self):
        super().__init__('auto_toggle')
        self.declare_parameter('enable_auto', False)
        self.declare_parameter('w_res', 0.4)
        self.declare_parameter('w_vel', 0.4)
        self.declare_parameter('w_z', 0.2)
        self.declare_parameter('s_norm', 0.15)
        self.declare_parameter('v_norm', 0.5)
        self.declare_parameter('z_norm', 0.25)
        self.declare_parameter('degrade_threshold', 1.0)
        self.declare_parameter('recover_threshold', 0.7)
        self.declare_parameter('bad_count_threshold', 10)
        self.declare_parameter('good_count_threshold', 20)
        self.declare_parameter('localization_fail_penalty', 0.6)
        self.declare_parameter('localization_false_streak_threshold', 3)
        self.declare_parameter('localization_true_streak_recover_threshold', 3)
        self.declare_parameter('c_degrad_ema_alpha', 0.25)
        self.declare_parameter('z_baseline_alpha', 0.05)
        self.declare_parameter('enable_fitness_gate', True)
        self.declare_parameter('fitness_degrade_threshold', 0.35)
        self.declare_parameter('fitness_recover_threshold', 0.50)
        self.declare_parameter('degrade_confirm_sec', 1.0)
        self.declare_parameter('recover_confirm_sec', 3.0)
        self.declare_parameter('arming_delay_sec', 5.0)
        self.declare_parameter('require_fitness_for_loc_bad', True)
        self.declare_parameter('require_localization_ok_for_recovery', False)
        self.declare_parameter('require_alignment_for_recovery', True)
        self.declare_parameter('recovery_align_threshold', 0.30)
        self.enable_auto = bool(self.get_parameter('enable_auto').value)
        self.w_res = float(self.get_parameter('w_res').value)
        self.w_vel = float(self.get_parameter('w_vel').value)
        self.w_z = float(self.get_parameter('w_z').value)
        self.s_norm = float(self.get_parameter('s_norm').value)
        self.v_norm = float(self.get_parameter('v_norm').value)
        self.z_norm = float(self.get_parameter('z_norm').value)
        self.degrade_threshold = float(self.get_parameter('degrade_threshold').value)
        self.recover_threshold = float(self.get_parameter('recover_threshold').value)
        self.bad_count_threshold = int(self.get_parameter('bad_count_threshold').value)
        self.good_count_threshold = int(self.get_parameter('good_count_threshold').value)
        self.localization_fail_penalty = float(self.get_parameter('localization_fail_penalty').value)
        self.localization_false_streak_threshold = int(
            self.get_parameter('localization_false_streak_threshold').value
        )
        self.localization_true_streak_recover_threshold = int(
            self.get_parameter('localization_true_streak_recover_threshold').value
        )
        self.c_degrad_ema_alpha = float(self.get_parameter('c_degrad_ema_alpha').value)
        self.z_baseline_alpha = float(self.get_parameter('z_baseline_alpha').value)
        self.enable_fitness_gate = bool(self.get_parameter('enable_fitness_gate').value)
        self.fitness_degrade_threshold = float(self.get_parameter('fitness_degrade_threshold').value)
        self.fitness_recover_threshold = float(self.get_parameter('fitness_recover_threshold').value)
        self.degrade_confirm_sec = float(self.get_parameter('degrade_confirm_sec').value)
        self.recover_confirm_sec = float(self.get_parameter('recover_confirm_sec').value)
        self.arming_delay_sec = float(self.get_parameter('arming_delay_sec').value)
        self.require_fitness_for_loc_bad = bool(self.get_parameter('require_fitness_for_loc_bad').value)
        self.require_localization_ok_for_recovery = bool(
            self.get_parameter('require_localization_ok_for_recovery').value
        )
        self.require_alignment_for_recovery = bool(
            self.get_parameter('require_alignment_for_recovery').value
        )
        self.recovery_align_threshold = float(
            self.get_parameter('recovery_align_threshold').value
        )
        
        self.create_subscription(Odometry, '/Odometry', self.slam_cb, 10)
        self.create_subscription(Odometry, '/odom', self.wheel_cb, 10)
        self.create_subscription(Float32, '/slam_res_mean', self.res_cb, 10)
        self.create_subscription(Float32, '/fastlio/icp_fitness', self.fitness_cb, 10)
        self.create_subscription(Bool, '/fastlio/localization_ok', self.loc_ok_cb, 10)
        self.create_subscription(Bool, '/hybrid/goal_cached', self.goal_cached_cb, 10)
        self.create_subscription(Odometry, '/neupan/current_odom', self.current_pos_cb, 10) 
        self.create_subscription(Float32, '/plot/recovery_align_error', self.recovery_align_cb, 10)
        self.create_subscription(Bool, '/slam_degradation_manual', self.manual_cb, 10)
        
        self.flag_pub = self.create_publisher(Bool, '/slam_degradation_flag', 10)
        self.c_degrad_pub = self.create_publisher(Float32, '/plot/c_degrad', 10)
        self.c_res_pub = self.create_publisher(Float32, '/plot/c_res', 10)
        self.c_vel_pub = self.create_publisher(Float32, '/plot/c_vel', 10)
        self.c_z_pub = self.create_publisher(Float32, '/plot/c_z', 10)
        self.c_raw_pub = self.create_publisher(Float32, '/plot/c_degrad_raw', 10)
        self.auto_armed_pub = self.create_publisher(Bool, '/plot/auto_armed', 10)
        self.goal_cached_pub_dbg = self.create_publisher(Bool, '/plot/goal_cached', 10)
        self.localization_ok_pub_dbg = self.create_publisher(Bool, '/plot/localization_ok_dbg', 10)
        self.fitness_bad_pub = self.create_publisher(Bool, '/plot/fitness_bad', 10)
        self.c_bad_pub = self.create_publisher(Bool, '/plot/c_bad', 10)
        self.loc_bad_pub = self.create_publisher(Bool, '/plot/loc_bad', 10)
        self.recovery_align_good_pub = self.create_publisher(Bool, '/plot/recovery_align_good', 10)
        
        self.slam_v = 0.0
        self.slam_z = 0.0
        self.wheel_v = 0.0
        self.is_degraded = False
        self.slam_score = 0.0
        self.current_fused_pose = None

        self.bad_count = 0
        self.good_count = 0
        self.localization_ok = True
        self.saw_localization_ok_once = False
        self.goal_cached = False
        self.localization_false_streak = 0
        self.localization_true_streak = 0
        self.c_degrad_ema = None
        self.slam_z_baseline = None
        self.latest_fitness = None
        self.latest_recovery_align_error = None
        self.bad_duration = 0.0
        self.good_duration = 0.0
        self.last_check_time = self.get_clock().now()
        self.arm_start_time = None

        self.timer = self.create_timer(0.1, self.check_health)
        self.last_slam_time = self.get_clock().now()
        
        self.prev_slam_x = None
        self.prev_slam_y = None
        self.prev_slam_t = None
        self.manual_override = None

    def manual_cb(self, msg):
        self.manual_override = bool(msg.data)
        self.is_degraded = self.manual_override
        self.bad_duration = 0.0
        self.good_duration = 0.0
        self.publish_flag()
        self.get_logger().warn(f'Manual toggle -> degraded={self.is_degraded}')

    def current_pos_cb(self, msg):
        self.current_fused_pose = msg

    def slam_cb(self, msg):
        self.last_slam_time = self.get_clock().now()
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        self.slam_z = msg.pose.pose.position.z
        curr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_slam_x is not None and self.prev_slam_t is not None:
            dt = curr_t - self.prev_slam_t
            if dt > 0.0:
                dx = curr_x - self.prev_slam_x
                dy = curr_y - self.prev_slam_y
                self.slam_v = math.hypot(dx, dy) / dt

        self.prev_slam_x, self.prev_slam_y, self.prev_slam_t = curr_x, curr_y, curr_t

        # z absolute value itself is not degradation; drift from a healthy baseline is.
        if self.localization_ok and self.goal_cached and not self.is_degraded:
            if self.slam_z_baseline is None:
                self.slam_z_baseline = self.slam_z
            else:
                a = min(max(self.z_baseline_alpha, 0.0), 1.0)
                self.slam_z_baseline = a * self.slam_z + (1.0 - a) * self.slam_z_baseline

    def wheel_cb(self, msg):
        self.wheel_v = abs(msg.twist.twist.linear.x)

    def res_cb(self, msg):
        self.slam_score = msg.data

    def fitness_cb(self, msg):
        self.latest_fitness = float(msg.data)

    def loc_ok_cb(self, msg):
        self.localization_ok = bool(msg.data)
        if self.localization_ok:
            self.saw_localization_ok_once = True
            self.localization_true_streak += 1
            self.localization_false_streak = 0
        else:
            self.localization_false_streak += 1
            self.localization_true_streak = 0

    def goal_cached_cb(self, msg):
        self.goal_cached = bool(msg.data)

    def recovery_align_cb(self, msg):
        self.latest_recovery_align_error = float(msg.data)

    def check_health(self):
        if self.manual_override is not None:
            self.publish_flag()
            return

        if not self.enable_auto:
            self.publish_flag()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_slam_time).nanoseconds / 1e9
        dt_check = (current_time - self.last_check_time).nanoseconds / 1e9
        self.last_check_time = current_time
        if dt_check <= 0.0:
            dt_check = 0.1

        # Raw localization_ok can remain false even when the robot is already
        # navigating with a usable last map->odom estimate. Using it as an
        # arming gate prevents degradation detection from ever starting in the
        # exact corridor case we want to study. Arm once a goal is cached and
        # fitness is being published.
        auto_ready = self.goal_cached and (self.latest_fitness is not None)
        if auto_ready:
            if self.arm_start_time is None:
                self.arm_start_time = current_time
            armed_elapsed = (current_time - self.arm_start_time).nanoseconds / 1e9
            auto_armed = armed_elapsed >= self.arming_delay_sec
        else:
            self.arm_start_time = None
            auto_armed = False

        if dt > 0.5 and auto_armed:
            self.trigger_degraded("Break SLAM")
            self.publish_flag()
            return

        v_diff = abs(abs(self.slam_v) - abs(self.wheel_v))
        z_ref = self.slam_z_baseline if self.slam_z_baseline is not None else self.slam_z
        z_drift = abs(self.slam_z - z_ref)
        res_mean = self.slam_score

        c_res = self.w_res * (res_mean / max(self.s_norm, 1e-6))
        c_vel = self.w_vel * (v_diff / max(self.v_norm, 1e-6))
        c_z = self.w_z * (z_drift / max(self.z_norm, 1e-6))
        raw_c_degrad = c_res + c_vel + c_z

        if self.localization_false_streak >= self.localization_false_streak_threshold:
            raw_c_degrad += self.localization_fail_penalty

        if self.c_degrad_ema is None:
            self.c_degrad_ema = raw_c_degrad
        else:
            a = min(max(self.c_degrad_ema_alpha, 0.0), 1.0)
            self.c_degrad_ema = a * raw_c_degrad + (1.0 - a) * self.c_degrad_ema

        C_degrad = float(self.c_degrad_ema)

        msg_raw = Float32(); msg_raw.data = float(raw_c_degrad); self.c_raw_pub.publish(msg_raw)
        msg_res = Float32(); msg_res.data = float(c_res); self.c_res_pub.publish(msg_res)
        msg_vel = Float32(); msg_vel.data = float(c_vel); self.c_vel_pub.publish(msg_vel)
        msg_z = Float32(); msg_z.data = float(c_z); self.c_z_pub.publish(msg_z)
        msg_c = Float32(); msg_c.data = float(C_degrad); self.c_degrad_pub.publish(msg_c)
        arm_msg = Bool(); arm_msg.data = bool(auto_armed); self.auto_armed_pub.publish(arm_msg)
        goal_cached_msg = Bool(); goal_cached_msg.data = bool(self.goal_cached); self.goal_cached_pub_dbg.publish(goal_cached_msg)
        loc_ok_msg = Bool(); loc_ok_msg.data = bool(self.localization_ok); self.localization_ok_pub_dbg.publish(loc_ok_msg)

        fitness_bad = (
            self.enable_fitness_gate
            and self.latest_fitness is not None
            and self.latest_fitness < self.fitness_degrade_threshold
        )
        c_bad = C_degrad > self.degrade_threshold
        loc_bad = self.localization_false_streak >= self.localization_false_streak_threshold

        fit_bad_msg = Bool(); fit_bad_msg.data = bool(fitness_bad); self.fitness_bad_pub.publish(fit_bad_msg)
        c_bad_msg = Bool(); c_bad_msg.data = bool(c_bad); self.c_bad_pub.publish(c_bad_msg)
        loc_bad_msg = Bool(); loc_bad_msg.data = bool(loc_bad); self.loc_bad_pub.publish(loc_bad_msg)
        align_good = (
            self.latest_recovery_align_error is None
            or self.latest_recovery_align_error <= self.recovery_align_threshold
        )
        align_good_msg = Bool(); align_good_msg.data = bool(align_good); self.recovery_align_good_pub.publish(align_good_msg)

        # 연구용 관측값은 항상 publish하되, 자동 토글 판정은
        # 초기 localization/startup transient가 지난 뒤부터만 시작한다.
        if not auto_armed:
            self.is_degraded = False
            self.bad_duration = 0.0
            self.good_duration = 0.0
            self.publish_flag()
            return

        fitness_good = (
            self.enable_fitness_gate
            and self.latest_fitness is not None
            and self.latest_fitness > self.fitness_recover_threshold
        )
        c_good = C_degrad < self.recover_threshold
        loc_good = self.localization_true_streak >= self.localization_true_streak_recover_threshold

        # 현재 실험 환경에서는 ICP fitness가 degraded corridor 구간을 가장 잘 분리한다.
        # 반면 C_degrad는 분석용으론 유용하지만 실시간 스위치 조건으로는 충분히 민감하지 않았다.
        # 따라서 실제 토글은 fitness/loc failure를 1차 기준으로 사용하고,
        # C_degrad는 논문용 분석/시각화 지표로 유지한다.
        if self.require_fitness_for_loc_bad:
            bad_condition = fitness_bad and (loc_bad or not loc_good)
        else:
            bad_condition = (fitness_bad and (loc_bad or not loc_good)) or c_bad
        if self.require_localization_ok_for_recovery:
            good_condition = self.goal_cached and loc_good and fitness_good
        else:
            # Raw localization_ok is noisy in this setup and can stay false even
            # while the map alignment has effectively recovered. Recovery should
            # therefore primarily follow sustained ICP fitness improvement.
            good_condition = self.goal_cached and fitness_good

        if self.require_alignment_for_recovery:
            good_condition = good_condition and align_good

        if bad_condition:
            self.bad_duration += dt_check
        else:
            self.bad_duration = 0.0

        if good_condition:
            self.good_duration += dt_check
        else:
            self.good_duration = 0.0

        if not self.is_degraded and self.bad_duration >= self.degrade_confirm_sec:
            if fitness_bad and self.latest_fitness is not None:
                self.trigger_degraded(f"ICP fitness:{self.latest_fitness:.2f}")
            elif loc_bad:
                self.trigger_degraded("Localization false streak")
            else:
                self.trigger_degraded(f"C_degrad:{C_degrad:.2f}")
        elif self.is_degraded and self.good_duration >= self.recover_confirm_sec:
            self.trigger_recovery()

        self.publish_flag()

    def trigger_degraded(self, reason):
        if not self.is_degraded:
            self.get_logger().warn(f'Emergency: {reason}')
        self.is_degraded = True
        self.bad_duration = 0.0
        self.good_duration = 0.0

    def trigger_recovery(self):
        self.is_degraded = False
        self.get_logger().info('SLAM repair')
        self.bad_duration = 0.0
        self.good_duration = 0.0

    def publish_flag(self):
        msg = Bool(); msg.data = self.is_degraded
        self.flag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoToggler()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
