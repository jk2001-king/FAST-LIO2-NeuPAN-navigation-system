import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class LinearEvaluator(Node):
    def __init__(self):
        super().__init__('linear_simple')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- [실험 파라미터 설정] ---
        self.target_distance = 10.0  # 목표 거리 10m 고정
        self.linear_speed = 0.8      # 실험 속도 (0.2, 0.5, 0.8로 변경하며 실험)
        # --------------------------

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_pose = None
        self.current_distance = 0.0
        self.finished = False

    def odom_callback(self, msg):
        curr_pos = msg.pose.pose.position
        if self.start_pose is None:
            self.start_pose = curr_pos
            return

        # 시작 지점으로부터 현재 지점까지의 직선 거리 계산 (Euclidean Distance)
        self.current_distance = math.sqrt(
            (curr_pos.x - self.start_pose.x)**2 + 
            (curr_pos.y - self.start_pose.y)**2
        )

    def timer_callback(self):
        twist = Twist()
        if self.current_distance < self.target_distance:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0  # 직선 주행이므로 회전은 0
            self.publisher_.publish(twist)
            # 1m 간격으로 로그 출력
            if int(self.current_distance * 10) % 10 == 0:
                self.get_logger().info(f'Moving... Distance: {self.current_distance:.2f} m')
        else:
            if not self.finished:
                self.get_logger().info(f'Target reached! Speed: {self.linear_speed} m/s')
                self.get_logger().info(f'Final Distance: {self.current_distance:.2f} m')
                self.stop_robot()
                self.finished = True

    def stop_robot(self):
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info('Evaluation session ended. Save your bag file.')

def main():
    rclpy.init()
    node = LinearEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()