import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuFix(Node):
    def __init__(self):
        super().__init__('imu_fix')

        self.sub = self.create_subscription(
            Imu,
            '/imu/data_filtered',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Imu,
            '/imu/data_fixed',
            10
        )

    def callback(self, msg):
        # ⭐ (1) 시간 동기화 - 핵심
        msg.header.stamp = self.get_clock().now().to_msg()

        # ⭐ (2) frame 유지 (절대 base_link로 바꾸지 마세요)
        # msg.header.frame_id 그대로 imu_link 유지

        # ⭐ (3) covariance만 보정
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFix()
    rclpy.spin(node)


if __name__ == '__main__':
    main()