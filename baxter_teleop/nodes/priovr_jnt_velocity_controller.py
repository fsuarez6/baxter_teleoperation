#!/usr/bin/env python

import rospy
#~ from math import pi
# Messages
from sensor_msgs.msg import JointState
# Baxter Python Interface
import baxter_interface
from baxter_interface import CHECK_VERSION
# Utils
from baxter_teleop.utils import *
from priovr_interface.utils import *

class JntVelController(object):
  def __init__(self):
    # Read the controllers parameters
    self.timeout = 0.1    # seconds
    # Set-up baxter interfaces
    self.left_arm = baxter_interface.Limb('left')
    self.right_arm = baxter_interface.Limb('right')
    # set_joint_velocities must be commanded at a rate great than the timeout specified by set_command_timeout. 
    # If the timeout is exceeded before a new set_joint_velocities command is received, the controller will switch 
    # modes back to position mode for safety purposes.
    self.left_arm.set_command_timeout(self.timeout)
    self.right_arm.set_command_timeout(self.timeout)
    # Initialize both arms at the same time
    self.left_timer = rospy.Timer(rospy.Duration(1), self.left_arm_init, oneshot=True)
    self.right_timer = rospy.Timer(rospy.Duration(1), self.right_arm_init, oneshot=True)
    # Get the joint names
    self.l_joint_names = self.left_arm.joint_names()
    self.r_joint_names = self.right_arm.joint_names()
    # Set-up publishers/subscribers
    rospy.Subscriber('/priovr/joint_states', JointState, self.joint_states_cb)
    rospy.spin()
  
  def left_arm_init(self, event):
    # Move to neutral (give some time to achive the target)
    self.left_arm.move_to_neutral(timeout=10.0)
    self.left_initialized = True
  
  def right_arm_init(self, event):
    # Move to neutral (give some time to achive the target)
    self.right_arm.move_to_neutral(timeout=10.0)
    self.right_initialized = True

  def joint_states_cb(self, msg):
    if rospy.is_shutdown():
      return
    # Start with empty velocities
    self.l_velocities = dict()
    self.r_velocities = dict()
    # populate the velocity commands
    for i, joint_name in enumerate(msg.name):
      if joint_name in self.l_joint_names:
        self.l_velocities[joint_name] = msg.velocity[i]
      elif joint_name in self.r_joint_names:
        self.r_velocities[joint_name] = msg.velocity[i]
    self.left_arm.set_joint_velocities(self.l_velocities)
    self.right_arm.set_joint_velocities(self.r_velocities)


# Main
if __name__ == '__main__':
  rospy.init_node('priovr_jnt_velocity_controller')
  priovr_jc = JntVelController()
