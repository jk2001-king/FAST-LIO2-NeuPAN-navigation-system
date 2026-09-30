#!/usr/bin/env python3

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, Twist, Vector3

class AMCLOdometryPublisher():
  def __init__(self):
    rospy.init_node('amcl_odom_pub')
    self.last_time = rospy.Time.now()
    # Pub
    self.odom_pub = rospy.Publisher('amcl_odom', Odometry, queue_size=1)
    # Sub
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.AMCLCallback)
    # Param
    self.x      = 0.
    self.y      = 0.
    self.quat_x = 0.
    self.quat_y = 0.
    self.quat_z = 0.
    self.quat_w = 1.

  def OdometryMessage(self):
    odom_quat = self.quat_x, self.quat_y, self.quat_z, self.quat_w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(odom_quat)
    ##
    amcl_odom = Odometry()
    amcl_odom.header.stamp = rospy.Time.now()
    amcl_odom.header.frame_id = 'odom'
    amcl_odom.child_frame_id = 'base_link'
    amcl_odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
    amcl_odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    ##
    self.odom_pub.publish(amcl_odom)

  def AMCLCallback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    self.quat_x = msg.pose.pose.orientation.x
    self.quat_y = msg.pose.pose.orientation.y
    self.quat_z = msg.pose.pose.orientation.z
    self.quat_w = msg.pose.pose.orientation.w


if __name__=="__main__":
  odometry = AMCLOdometryPublisher()
  while not rospy.is_shutdown():
    odometry.OdometryMessage()
    rospy.Rate(100).sleep()
