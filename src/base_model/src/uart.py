#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import Int16MultiArray, Int16

ser = serial.Serial(
  port     = '/dev/uart',
  baudrate = 115200,
  bytesize = serial.EIGHTBITS,
  parity   = serial.PARITY_NONE,
  stopbits = serial.STOPBITS_ONE,
)

class UARTCommunication():
  def __init__(self):
    rospy.init_node('uart')
    # Pub
    self.uart_pub = rospy.Publisher('wheel_status', Int16MultiArray, queue_size=1)
    # Sub
    rospy.Subscriber('wheel_cmd', Int16MultiArray, self.CmdCallback)
    rospy.Subscriber('mode_cmd', Int16, self.ModeCallback)
    # Param
    self.wheel_data = []
    self.stop_cmd = [83,33,83,33,79]

  def Checksum(self, ckdata):
    return (~(sum(ckdata))+1) & 0xFF

  def RX(self):
    self.wheel_data.append(ord(ser.read()) & 0xFF)
    if self.wheel_data[0] == 72:
      if self.wheel_data[-2:] == [13,10]:
        if self.wheel_data[-3] == self.Checksum(self.wheel_data[1:-3]):
          print('Status -------------------------------------')
          print(self.wheel_data)
          uart_msg = Int16MultiArray()
          uart_msg.data = self.wheel_data
          self.uart_pub.publish(uart_msg)
          self.mode = self.wheel_data[1]
          self.wheel_data = []
        else:
          self.wheel_data = []
      else:
        pass
    else:
      self.wheel_data = []

  def TX(self, wheel_cmd):
    cmd_data = [72] + wheel_cmd + [self.Checksum(wheel_cmd),13,10]
    for i in range(len(cmd_data)):
      ser.write(chr(cmd_data[i]).encode())
    print('Command ---------------------------------')
    print(cmd_data)

  def CmdCallback(self, msg):
    if self.mode == 65:
      wheel_cmd = [self.mode] + list(msg.data)
      self.TX(wheel_cmd)
    else:
      pass

  def ModeCallback(self, msg):
    self.mode = msg.data
    if self.mode == 65:
      print('\n\n[[[ Auto Mode ]]]')
      self.TX([self.mode] + self.stop_cmd)
    elif self.mode == 77:
      print('\n\n[[[ Manual Mode ]]]')
      self.TX([self.mode] + self.stop_cmd)
    else:
      pass


if __name__=="__main__":
  uart = UARTCommunication()
  try:
    while not rospy.is_shutdown():
      uart.RX()

  except KeyboardInterrupt:
    print('keyboard interrupt')
    
  finally:
    ser.close()
    pass
