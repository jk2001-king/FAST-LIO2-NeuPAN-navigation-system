import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('nav2_topic', '/cmd_vel_nav2')
        self.declare_parameter('neupan_topic', '/cmd_vel_neupan')
        self.declare_parameter('override_topic', '/cmd_vel_override')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('override_timeout_sec', 0.25)
        self.declare_parameter('source_timeout_sec', 0.25)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.nav2_topic = str(self.get_parameter('nav2_topic').value)
        self.neupan_topic = str(self.get_parameter('neupan_topic').value)
        self.override_topic = str(self.get_parameter('override_topic').value)
        self.out_topic = str(self.get_parameter('out_topic').value)

        self.override_timeout_sec = float(self.get_parameter('override_timeout_sec').value)
        self.source_timeout_sec = float(self.get_parameter('source_timeout_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.is_degraded = False
        self.last_nav2 = Twist()
        self.last_neupan = Twist()
        self.last_nav2_time = None
        self.last_neupan_time = None
        self.last_override = None
        self.last_override_time = None
        self.last_selected_source = ''

        self.create_subscription(Bool, '/slam_degradation_flag', self.flag_cb, 10)
        self.create_subscription(Twist, self.nav2_topic, self.nav2_cb, 10)
        self.create_subscription(Twist, self.neupan_topic, self.neupan_cb, 10)
        self.create_subscription(Twist, self.override_topic, self.override_cb, 10)

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.source_pub = self.create_publisher(String, '/cmd_vel_mux/source', 10)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_cb)

        self.get_logger().info(
            f"cmd_vel mux online. degraded=false→{self.nav2_topic}, degraded=true→{self.neupan_topic}, "
            f"override({self.override_timeout_sec:.2f}s)→{self.override_topic} => {self.out_topic}"
        )

    def flag_cb(self, msg: Bool):
        self.is_degraded = bool(msg.data)

    def nav2_cb(self, msg: Twist):
        self.last_nav2 = msg
        self.last_nav2_time = self.get_clock().now()

    def neupan_cb(self, msg: Twist):
        self.last_neupan = msg
        self.last_neupan_time = self.get_clock().now()

    def override_cb(self, msg: Twist):
        self.last_override = msg
        self.last_override_time = self.get_clock().now()

    def publish_cb(self):
        now = self.get_clock().now()
        selected_source = 'neupan' if self.is_degraded else 'nav2'
        selected_twist = self.last_neupan if self.is_degraded else self.last_nav2
        selected_time = self.last_neupan_time if self.is_degraded else self.last_nav2_time

        if self.last_override is not None and self.last_override_time is not None:
            age = (now - self.last_override_time).nanoseconds / 1e9
            if age <= self.override_timeout_sec:
                selected_source = 'override'
                selected_twist = self.last_override
                self._publish_source(selected_source)
                self.pub.publish(selected_twist)
                return

        # Do not replay an old non-zero command forever after the planner stops
        # publishing; stale commands should decay to a stop.
        if selected_time is None:
            selected_twist = Twist()
        else:
            age = (now - selected_time).nanoseconds / 1e9
            if age > self.source_timeout_sec:
                selected_twist = Twist()

        self._publish_source(selected_source)
        self.pub.publish(selected_twist)

    def _publish_source(self, source: str):
        if source != self.last_selected_source:
            self.get_logger().info(f'cmd_vel source -> {source}')
            self.last_selected_source = source
        msg = String()
        msg.data = source
        self.source_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
