#!/usr/bin/env python3

import time, tf
import numpy as np
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

class WheelNavCommand():
  def __init__(self):
    rospy.init_node('wheel_cmd')
    # Pub
    self.navcmd_pub = rospy.Publisher('wheel_cmd', Int16MultiArray, queue_size=1)
    self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    self.turn_status_pub = rospy.Publisher('turn_status', Int16, queue_size=1)
    # Sub
    rospy.Subscriber('cmd_vel', Twist, self.VelCallback)
    rospy.Subscriber('wheel_status', Int16MultiArray, self.UartCallback)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, self.GoalCallback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.PoseCallback)
    rospy.Subscriber('laser_obstacle', Int16, self.ObsCallback)
    # Param
    self.wheel_separation = 0.54
    self.manual_chk = 0
    self.obstacle_chk = 0
    self.sturn_sw = False
    self.init_nav = False
    self.init_dist = 0.
    self.nav_count = 0

    self.cc_goal_x   = 0.
    self.cc_goal_y   = 0.
    self.cc_pose_x   = 0.
    self.cc_pose_y   = 0.
    self.cc_pose_yaw = 0.
    self.cc_distance = 0.
    self.cc_yawref   = 0.
    self.cc_yawerror = 0.

    self.l_forward  = False
    self.l_backward = False
    self.r_forward  = False
    self.r_backward = False

  def CalError(self):
    self.cc_x = self.cc_goal_x - self.cc_pose_x
    self.cc_y = self.cc_goal_y - self.cc_pose_y
    self.cc_distance = (self.cc_x**2 + self.cc_y**2)**0.5
    self.cc_yawref = np.arctan2(self.cc_y, self.cc_x)
    self.cc_yawerror = self.cc_yawref - self.cc_pose_yaw

  def CalVelocity(self, data):
    linear_x = data.linear.x
    angular_z = data.angular.z
    wheel_speed_left = (linear_x - angular_z * self.wheel_separation / 2) * 3.6
    wheel_speed_right = (linear_x + angular_z * self.wheel_separation / 2) * 3.6
    if wheel_speed_left > 0:
      ll = 0x43
    elif wheel_speed_left < 0:
      ll = 0x57
    else:
      ll = 0x53
    if wheel_speed_right > 0:
      rr = 0x43
    elif wheel_speed_right < 0:
      rr = 0x57
    else:
      rr = 0x53
    lv = abs(int(wheel_speed_left * 10)) + 0x21
    rv = abs(int(wheel_speed_right * 10)) + 0x21
    return ll, lv, rr, rv

  def ObstacleStop(self):
    lv = int(self.cr_lv*0.9)
    rv = int(self.cr_rv*0.9)
    ll = self.cr_ll
    rr = self.cr_rr
    if lv < 35:
      lv = 33
    if rv < 35:
      rv = 33
    if lv == 33:
      ll = 0x53
    if rv == 33:
      rr = 0x53
    print('obstacle detected !!')
    time.sleep(0.1)
    return ll, lv, rr, rv

  def StartTurn(self):
    if (abs(self.cc_yawerror) <= 3.14 and self.cc_yawerror >= 0) or (abs(self.cc_yawerror) > 3.14 and self.cc_yawerror < 0):
      ll = 87
      lv = 40
      rr = 67
      rv = lv
    else:
      ll = 67
      lv = 40
      rr = 87
      rv = lv
    if abs(self.cc_yawerror) < 0.5 or abs(self.cc_yawerror) > 5.78:
      self.sturn_sw = False
      lv = 33
      rv = 33
      print('sturn end')
    return ll, lv, rr, rv

  def WheelSmoothStarter(self, lv, rv):
    self.nav_count += 1
    ff = 0.5 + float(self.nav_count)/100
    print('nav_count',self.nav_count)
    print('ff',ff)
    if ff >= 1:
      self.init_nav = False
      self.nav_count = 0
    lv = int(lv*ff)
    rv = int(rv*ff)
    if lv < 33:
      lv = 33
    if rv < 33:
      rv = 33
    return lv, rv

  def WheelSmoothChanger(self, ll, lv, rr, rv):
    if ll == 67:
      self.l_forward = True
    elif ll == 87:
      self.l_backward = True
    if rr == 67:
      self.r_forward = True
    elif rr == 87:
      self.r_backward = True
    ##
    if self.l_backward == True and ll == 67:
      self.l_backward = False
      lv = 33
    elif self.l_forward == True and ll == 87:
      self.l_forward = False
      lv = 33
    if self.r_backward == True and rr == 67:
      self.r_backward = False
      rv = 33
    elif self.r_forward == True and rr == 87:
      self.r_forward = False
      rv = 33
    ##
    if lv == 33:
      ll = 0x53
    if rv == 33:
      rr = 0x53
    return ll, lv, rr, rv

  def VelCallback(self, msg):
    if self.manual_chk:
      print('manual mode')
      pass
    else:
      # Error Calculation
      self.CalError()
      # Obstacle Detection and Stop
      if self.obstacle_chk:
        ll, lv, rr, rv = self.ObstacleStop()
      else:
        # Start Turn
        if self.sturn_sw:
          ll, lv, rr, rv = self.StartTurn()
          self.turn_status_pub.publish(1)
        else:
          # Command Velocity Calculation
          ll, lv, rr, rv = self.CalVelocity(msg)
          self.turn_status_pub.publish(0)
          # Smooth Starter
          if self.init_nav:
            lv, rv = self.WheelSmoothStarter(lv, rv)
          else:
            pass

      ll, lv, rr, rv = self.WheelSmoothChanger(ll, lv, rr, rv)
      
      brk = 79
      navcmd = Int16MultiArray()
      navcmd.data = [ll, lv, rr, rv, brk]
      self.navcmd_pub.publish(navcmd)

  def UartCallback(self, msg):
    if msg.data[1] == 65:
      self.manual_chk = 0
    else:
      self.manual_chk = 1
    self.cr_ll = msg.data[2]
    self.cr_lv = msg.data[3]
    self.cr_rr = msg.data[4]
    self.cr_rv = msg.data[5]

  def GoalCallback(self, msg):
    print('goal_CB...')
    self.current_goal = msg
    self.cc_goal_x = msg.pose.position.x
    self.cc_goal_y = msg.pose.position.y
    self.init_dist = ((self.cc_goal_x-self.cc_pose_x)**2 + (self.cc_goal_y-self.cc_pose_y)**2)**0.5
    self.sturn_sw = True
    self.init_nav = True
    self.nav_count = 0

  def PoseCallback(self, msg):
    self.cc_pose_x = msg.pose.pose.position.x
    self.cc_pose_y = msg.pose.pose.position.y
    pose_quat = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(pose_quat)
    self.cc_pose_yaw = yaw

  def ObsCallback(self, msg):
    self.obstacle_chk = msg.data
    if self.obstacle_chk:
      self.init_nav = True
      self.nav_count = 0


if __name__=="__main__":
  navcmd = WheelNavCommand()
  rospy.spin()
