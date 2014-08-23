#!/usr/bin/env python
import rospy, os
# Messages
from sensor_msgs.msg import Joy
# Baxter Interface for the grippers
import baxter_interface
from baxter_interface import CHECK_VERSION
# Priovr Utils
from priovr_interface.utils import *


class JoyGrippersController(object):
  def __init__(self):
    # Set-up grippers interface
    self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    self.left_gripper.calibrate()
    self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    self.right_gripper.calibrate()
    # Initial values
    self.prev_buttons = [0] * 4
    self.buttons = [False] * 4
    # Set-up publishers/subscribers
    rospy.Subscriber('/priovr/joysticks', Joy, self.joysticks_cb)
  
  def joysticks_cb(self, msg):
    # Check that the buttons where pressed / relased
    for i, previous in enumerate(self.prev_buttons):
      if (previous != msg.buttons[i]) and msg.buttons[i] == 1:
        self.buttons[i] = not self.buttons[i]
    # Open or close the grippers
    if self.buttons[RIGHT_C_BUTTON]:
      self.left_gripper.close()
    else:
      self.left_gripper.open()
    if self.buttons[RIGHT_Z_BUTTON]:
      self.right_gripper.close()
    else:
      self.right_gripper.open()


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  grippers = JoyGrippersController()
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
