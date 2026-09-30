#!/usr/bin/env python3

import sys, select, termios, tty
import rospy
from std_msgs.msg import Int16

class ModeChanger():
  def __init__(self):
    rospy.init_node('mode')
    # Pub
    self.mode_pub = rospy.Publisher('mode_cmd', Int16, queue_size=1)
    # Param
    self.settings = termios.tcgetattr(sys.stdin)
    self.quit = False

  def getKey(self, key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist,_,_ = select.select([sys.stdin],[],[],key_timeout)
    if rlist:
      key = sys.stdin.read(1)
    else:
      key = ''
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,self.settings)
    return key

  def CmdPub(self):
    key = self.getKey(0.01)
    if key == 'a':
      print('[[[ Auto Mode ]]]\n')
      self.mode_pub.publish(Int16(65))
    elif key == 'm':
      print('[[[ Manual Mode ]]]\n')
      self.mode_pub.publish(Int16(77))
    elif key == "q":
      print("The end")
      self.quit = True
    else:
      pass


if __name__=="__main__":
  mode = ModeChanger()
  while not rospy.is_shutdown():
    mode.CmdPub()
    if mode.quit == True:
      break

