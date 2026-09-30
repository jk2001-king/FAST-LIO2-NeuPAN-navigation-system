#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

class CostmapCleaner():
  def __init__(self):
    rospy.init_node('costmap_cleaner')
    self.srv_call = rospy.ServiceProxy('move_base/clear_costmaps', Empty)

  def cleaning(self):
    rospy.wait_for_service('move_base/clear_costmaps')
    try:
      self.srv_call()
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)


if __name__=="__main__":
  cleaner = CostmapCleaner()
  while not rospy.is_shutdown():
    cleaner.cleaning()
    rospy.Rate(1).sleep()
