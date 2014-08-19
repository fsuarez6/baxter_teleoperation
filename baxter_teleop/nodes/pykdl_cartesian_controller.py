#!/usr/bin/env python

import rospy, math
import numpy as np
# PyKDL
import PyKDL
from moveit_kinematics_interface.kdl_kinematics import Kinematics
from tf_conversions import posemath
# Utils
from moveit_kinematics_interface.utils import kdl_to_numpy_array
from baxter_teleop.utils import read_parameter
# Messages
from baxter_core_msgs.msg import EndpointState, JointCommand
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class CartesianController(object):
  def __init__(self, limb):
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    self.publish_rate = read_parameter('~publish_rate', 100)
    self.frame_id = read_parameter('~frame_id', 'base')
    # Start kinematics interface
    self.arm_kinematics = Kinematics('%s_gripper' % limb, ik_solver='LMA')
    self.joint_names = self.arm_kinematics.get_joint_names()
    self.joint_positions = [None] * len(self.joint_names)
    # Set-up publishers/subscribers
    self.joint_cmd_pub = rospy.Publisher('/robot/limb/%s/joint_command' % limb, JointCommand)
    self.pose_pub = rospy.Publisher('/baxter/%s_gripper_pose' % limb, PoseStamped)
    rospy.Subscriber('/baxter/%s_ik_command' % limb, PoseStamped, self.ik_command_cb)
    rospy.Subscriber('/robot/joint_states', JointState, self.joint_states_cb)
    # Wait until the first joint_states msg is received
    rospy.loginfo('Waiting for /robot/joint_states topic')
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown() and None in self.joint_positions:
      rate.sleep()
    # Ensure a valid joint command
    self.joint_command = list(self.joint_positions)
    self.robot_pose = PoseStamped()
    rospy.Subscriber('/robot/limb/%s/endpoint_state' % limb, EndpointState, self.endpoint_state_cb)
    rospy.loginfo('%s arm controller initialized' % limb)

  def shutdown(self):
    pass
  
  def endpoint_state_cb(self, msg):
    # Publish robot pose
    self.robot_pose.pose = msg.pose
    self.robot_pose.header.stamp = rospy.Time.now()
    self.robot_pose.header.frame_id = self.frame_id
    self.pose_pub.publish(self.robot_pose)
    # Publish robot arm command
    cmd_msg = JointCommand()
    cmd_msg.mode = JointCommand().POSITION_MODE
    cmd_msg.names = self.joint_names
    cmd_msg.command = self.joint_command
    self.joint_cmd_pub.publish(cmd_msg)
    
  def joint_states_cb(self, msg):
    for i, name in enumerate(self.joint_names):
      if name in msg.name:
        self.joint_positions[i] = msg.position[msg.name.index(name)]
      
  def ik_command_cb(self, msg):
    # TODO: Validate msg.header.frame_id
    robot_frame = posemath.fromMsg(self.robot_pose)
    target_frame = posemath.fromMsg(msg.pose)
    seed = list(self.joint_positions)
    found = self.arm_kinematics.search_ik(target_frame, seed, timeout=0.01)
    # TODO: Check that the command changed enough
    changed_enough = True
    if found:
      self.joint_command = kdl_to_numpy_array(self.arm_kinematics.get_joint_positions())


if __name__ == '__main__':
  node_name = 'pykdl_cartesian_controller'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  right_arm = CartesianController('right')
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
