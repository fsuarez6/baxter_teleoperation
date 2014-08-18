#!/usr/bin/env python

import rospy, math
import numpy as np
# PyKDL
import PyKDL
from moveit_kinematics_interface.kdl_kinematics import Kinematics
from tf_conversions import posemath
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import PoseStamped


class CartesianController(object):
  def __init__(self, limb):
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      # TODO: Raise ros shutdown signal
      return
    self.publish_rate = read_parameter('~publish_rate', 100)
    # Set-up publishers/subscribers
    self.joint_cmd_pub = rospy.Publisher('/robot/limb/%s/joint_command' % limb, JointCommand)
    self.pose_pub = rospy.Publisher('/baxter/%s_gripper_pose' % limb, PoseStamped)
    rospy.Subscriber('/baxter/%s_ik_command' % limb, PoseStamped, self.ik_command_cb)
    # Start kinematics interface
    self.right_kinematics = Kinematics('%s_gripper' % limb, ik_solver='LMA')

  def shutdown(self):
    pass
    
  def ik_command_cb(self, msg):
    # TODO: Validate frame_id
    frame = posemath.fromMsg(msg.pose)
    ik_found = self.right_kinematics.set_from_ik(frame)
    print ik_found
    if ik_found >= 0:
      print self.right_kinematics.get_joint_positions()
      print self.right_kinematics.get_end_effector_transform()


if __name__ == '__main__':
  node_name = 'pykdl_cartesian_controller'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  right_arm = CartesianController('right')
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
