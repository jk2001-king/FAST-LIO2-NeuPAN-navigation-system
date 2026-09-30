#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, PointCloud2


class FastLioTimedBridge(Node):
    def __init__(self):
        super().__init__(
            "fast_lio_timed_bridge",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        self.declare_parameter("scan_rate", 20.0)
        self.declare_parameter("num_scan", 16)

        self.scan_rate = float(self.get_parameter("scan_rate").value)
        self.num_scan = int(self.get_parameter("num_scan").value)

        qos_sub = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2, "/velodyne_points", self.lidar_cb, qos_sub
        )
        self.lidar_pub = self.create_publisher(
            PointCloud2, "/velodyne_points_fixed", qos_pub
        )

        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_cb, qos_sub)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_fixed", qos_pub)

        self.get_logger().info(
            "Timed bridge running: /velodyne_points -> /velodyne_points_fixed"
        )

    @staticmethod
    def _field_offsets(msg: PointCloud2):
        offsets = {field.name: field.offset for field in msg.fields}
        required = ("x", "y", "ring", "time")
        missing = [name for name in required if name not in offsets]
        if missing:
            raise RuntimeError(f"Missing fields in PointCloud2: {missing}")
        return offsets["x"], offsets["y"], offsets["ring"], offsets["time"]

    def fill_time_field(self, msg: PointCloud2) -> PointCloud2:
        ox, oy, oring, otime = self._field_offsets(msg)
        step = msg.point_step
        width = msg.width * msg.height
        data = bytearray(msg.data)

        has_nonzero_time = False
        for i in range(width):
            base = i * step
            t = struct.unpack_from("<f", data, base + otime)[0]
            if math.isfinite(t) and t > 0.0:
                has_nonzero_time = True
                break

        if not has_nonzero_time:
            first_yaw = [None] * self.num_scan
            last_time = [0.0] * self.num_scan

            for i in range(width):
                base = i * step
                x = struct.unpack_from("<f", data, base + ox)[0]
                y = struct.unpack_from("<f", data, base + oy)[0]
                ring = struct.unpack_from("<H", data, base + oring)[0]

                if ring >= self.num_scan:
                    continue

                yaw = math.degrees(math.atan2(y, x))
                if first_yaw[ring] is None:
                    first_yaw[ring] = yaw
                    offset_sec = 0.0
                else:
                    delta = first_yaw[ring] - yaw
                    if delta < 0.0:
                        delta += 360.0
                    offset_sec = delta / (360.0 * self.scan_rate)
                    if offset_sec < last_time[ring]:
                        offset_sec += 1.0 / self.scan_rate

                last_time[ring] = offset_sec
                struct.pack_into("<f", data, base + otime, float(offset_sec))

        new_msg = PointCloud2()
        new_msg.header = msg.header
        if not new_msg.header.frame_id:
            new_msg.header.frame_id = "velodyne"
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.is_dense = msg.is_dense
        new_msg.data = bytes(data)
        return new_msg

    def lidar_cb(self, msg: PointCloud2):
        try:
            fixed_msg = self.fill_time_field(msg)
            self.lidar_pub.publish(fixed_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to republish timed cloud: {exc}")

    def imu_cb(self, msg: Imu):
        fixed_msg = Imu()
        fixed_msg.header = msg.header
        if not fixed_msg.header.frame_id:
            fixed_msg.header.frame_id = "imu_link"
        fixed_msg.orientation = msg.orientation
        fixed_msg.orientation_covariance = msg.orientation_covariance
        fixed_msg.angular_velocity = msg.angular_velocity
        fixed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        fixed_msg.linear_acceleration = msg.linear_acceleration
        fixed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioTimedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
