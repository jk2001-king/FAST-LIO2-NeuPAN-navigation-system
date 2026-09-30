#!/usr/bin/env python3

from math import sin, cos
import tf
import rospy
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

class OdometryPublisher():
  def __init__(self):
    rospy.init_node('odom_pub')
    self.last_time = rospy.Time.now()
    # Pub
    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
    # Sub
    rospy.Subscriber('wheel_status', Int16MultiArray, self.UartCallback)
    # Param
    self.x   = 0.
    self.y   = 0.
    self.v   = 0.
    self.vx  = 0.
    self.vy  = 0.
    self.th  = 0.
    self.vth = 0.

  def OdometryMessage(self):
    current_time = rospy.Time.now()
    dt = (current_time - self.last_time).to_sec()
    self.vx  = self.v * cos(self.th)
    self.vy  = self.v * sin(self.th)
    self.x  += self.vx * dt
    self.y  += self.vy * dt
    self.th += self.vth * dt
    ##
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
    odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
    odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
    ##
    self.last_time = current_time
    return odom

  def UartCallback(self, msg):
    if msg.data[2] == 67:
      lv =  float(msg.data[3] - 33.0) /10.0 
    elif msg.data[2] == 87:
      lv = -float(msg.data[3] - 33.0) /10.0	
    else:
      lv = 0.
    if msg.data[4] == 67:		
      rv =  float(msg.data[5] - 33.0) /10.0
    elif msg.data[4] == 87:
      rv = -float(msg.data[5] - 33.0) /10.0
    else:
      rv = 0.
		##
    self.v   = float((lv + rv) /2 /3.6)
    self.vth = float((rv - lv) /3.6 /0.54)
    odom = self.OdometryMessage()
    self.odom_pub.publish(odom)


if __name__=="__main__":
  odometry = OdometryPublisher()
  rospy.spin()
