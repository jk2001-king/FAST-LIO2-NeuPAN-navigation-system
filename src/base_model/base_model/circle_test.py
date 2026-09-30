#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class CircleSimple(Node):

    def __init__(self):
        super().__init__('circle_simple')

        # 1. 퍼블리셔 및 서브스크라이버 설정
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # 20Hz 주기로 타이머 실행 (0.05초)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 2. 상태 변수 초기화
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.total_rotation = 0.0
        self.prev_yaw = None
        
        # SLAM 안정화를 위한 대기 로직
        self.start_time = None
        self.wait_duration = 3.0  # 첫 위치 수신 후 3초 대기
        self.is_running = False

        self.get_logger().info("=== Circle Node for SLAM Evaluation Started ===")
        self.get_logger().info("Target: Radius 3.0m, Linear 0.4m/s")

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        self.current_yaw = yaw

        # 첫 위치 기록
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = yaw
            self.prev_yaw = yaw
            self.start_time = time.time()
            self.get_logger().info(f"Start Pose Recorded: x={self.start_x:.2f}, y={self.start_y:.2f}")
            return

        # 회전량 누적 계산 (abs를 제거하여 노이즈에 의한 누적 오류 방지)
        delta = yaw - self.prev_yaw
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi

        # 우리가 시계 반대방향(Angular Z > 0)으로 돌기 때문에 delta를 그대로 누적
        self.total_rotation += delta
        self.prev_yaw = yaw

    def timer_callback(self):
        # 1. 오도메트리 수신 전이면 대기
        if self.start_x is None:
            return

        # 2. SLAM 안정화를 위한 초기 3초 대기
        elapsed_since_start = time.time() - self.start_time
        if elapsed_since_start < self.wait_duration:
            if int(elapsed_since_start * 2) % 2 == 0: # 1초마다 로그
                self.get_logger().info(f"Waiting for SLAM stabilization... {self.wait_duration - elapsed_since_start:.1f}s", once=False)
            return
        
        if not self.is_running:
            self.get_logger().info("Starting movement!")
            self.is_running = True

        # 3. 주행 거리(시작점으로부터의 거리) 계산
        dist_from_start = math.sqrt(
            (self.current_x - self.start_x)**2 +
            (self.current_y - self.start_y)**2
        )

        # 4. 종료 조건 체크: 한 바퀴(2*pi) 이상 회전했고, 시작점에 0.3m 이내로 접근했을 때
        # 정밀도를 위해 dist 범위를 0.5 -> 0.3으로 좁혔습니다.
        if abs(self.total_rotation) > (2 * math.pi * 0.95) and dist_from_start < 0.3:
            self.get_logger().info(f"Circle completed! Total Rotation: {self.total_rotation:.2f} rad")
            self.get_logger().info(f"Final Distance from Start: {dist_from_start:.4f}m")
            
            # 정지 명령
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            
            # 잠시 후 노드 종료
            time.sleep(1.0)
            self.get_logger().info("Evaluation session ended. Please save your rosbag.")
            rclpy.shutdown()
            return

        # 5. 속도 명령 발행 (R=3m, v=0.4, w=0.4/3)
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.15
        self.cmd_pub.publish(twist)

        # 진행 상황 출력 (5초에 한 번씩)
        if int(self.total_rotation * 10) % 31 == 0: # 대략 0.5파이마다 출력
            self.get_logger().info(f"Progress: Rot={self.total_rotation:.2f}/6.28, Dist={dist_from_start:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = CircleSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()