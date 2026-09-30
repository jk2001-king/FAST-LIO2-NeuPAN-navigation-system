import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from geometry_msgs.msg import Quaternion

def get_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def get_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x, q.y, q.z = 0.0, 0.0, math.sin(yaw / 2.0)
    return q

class OdomSwitcher(Node):
    def __init__(self):
        super().__init__('odom_switcher')
        self.declare_parameter('publish_tf', False)
        self.slam_sub = self.create_subscription(Odometry, '/Odometry', self.slam_cb, 10)
        self.wheel_sub = self.create_subscription(Odometry, '/odom', self.wheel_cb, 10)
        self.flag_sub = self.create_subscription(Bool, '/slam_degradation_flag', self.flag_cb, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/neupan/current_odom', 10)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.is_degraded = False

        self.s_x, self.s_y, self.s_yaw = 0.0, 0.0, 0.0
        self.w_x, self.w_y, self.w_yaw = 0.0, 0.0, 0.0
        
        self.out_x, self.out_y, self.out_yaw = 0.0, 0.0, 0.0
        self.out_z = 0.0

        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0
        
        self.first_slam = True

    def flag_cb(self, msg):
        if self.is_degraded == msg.data: 
            return
        
        self.is_degraded = msg.data

        if self.is_degraded:
            self.get_logger().warn('Wheel odometry')
            self.offset_yaw = self.out_yaw - self.w_yaw
            self.offset_x = self.out_x - (self.w_x * math.cos(self.offset_yaw) - self.w_y * math.sin(self.offset_yaw))
            self.offset_y = self.out_y - (self.w_x * math.sin(self.offset_yaw) + self.w_y * math.cos(self.offset_yaw))
        else:
            self.get_logger().info('SLAM odometry')
            self.offset_yaw = self.out_yaw - self.s_yaw
            self.offset_x = self.out_x - (self.s_x * math.cos(self.offset_yaw) - self.s_y * math.sin(self.offset_yaw))
            self.offset_y = self.out_y - (self.s_x * math.sin(self.offset_yaw) + self.s_y * math.cos(self.offset_yaw))

    def slam_cb(self, msg):
        self.s_x = msg.pose.pose.position.x
        self.s_y = msg.pose.pose.position.y
        self.s_yaw = get_yaw(msg.pose.pose.orientation)

        if self.first_slam:
            self.out_x, self.out_y, self.out_yaw = self.s_x, self.s_y, self.s_yaw
            self.first_slam = False

        if not self.is_degraded:
            self.out_yaw = self.s_yaw + self.offset_yaw
            self.out_x = self.offset_x + self.s_x * math.cos(self.offset_yaw) - self.s_y * math.sin(self.offset_yaw)
            self.out_y = self.offset_y + self.s_x * math.sin(self.offset_yaw) + self.s_y * math.cos(self.offset_yaw)
            self.out_z = msg.pose.pose.position.z

            self.publish_all(msg.header.stamp, msg.twist.twist)

    def wheel_cb(self, msg):
        self.w_x = msg.pose.pose.position.x
        self.w_y = msg.pose.pose.position.y
        self.w_yaw = get_yaw(msg.pose.pose.orientation)

        if self.is_degraded:
            self.out_yaw = self.w_yaw + self.offset_yaw
            self.out_x = self.offset_x + self.w_x * math.cos(self.offset_yaw) - self.w_y * math.sin(self.offset_yaw)
            self.out_y = self.offset_y + self.w_x * math.sin(self.offset_yaw) + self.w_y * math.cos(self.offset_yaw)

            self.publish_all(msg.header.stamp, msg.twist.twist)

    def publish_all(self, stamp, twist):
        calc_quat = get_quat(self.out_yaw)

        out_msg = Odometry()
        out_msg.header.stamp = stamp
        out_msg.header.frame_id = 'odom'
        out_msg.child_frame_id = 'base_link'
        out_msg.pose.pose.position.x = self.out_x
        out_msg.pose.pose.position.y = self.out_y
        out_msg.pose.pose.position.z = self.out_z
        out_msg.pose.pose.orientation = calc_quat
        out_msg.twist.twist = twist
        self.odom_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
