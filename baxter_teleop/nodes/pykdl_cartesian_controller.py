#!/usr/bin/env python

import rospy, math
import numpy as np
# PyKDL
import PyKDL
from tf_conversions import posemath
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_kdl_to_list
# URDF
from urdf_parser_py.urdf import URDF
# Baxter Python Interface
import baxter_interface
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from geometry_msgs.msg import PoseStamped


class CartesianController(object):
  def __init__(self, limb):
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    # Read the controllers parameters
    self.kp = PyKDL.JntArray(6)
    self.kd = PyKDL.JntArray(6)
    for i in range(6):
      self.kp[i] = 100
      self.kd[i] = 1
    self.publish_rate = read_parameter('~publish_rate', 100)
    self.frame_id = read_parameter('~frame_id', 'base')
    self.timeout = read_parameter('~timeout', 0.1)       # seconds
    # Set-up baxter interface
    self.arm_interface = baxter_interface.Limb(limb)
    # set_joint_velocities must be commanded at a rate great than the timeout specified by set_command_timeout.
    # If the timeout is exceeded before a new set_joint_velocities command is received, the controller will switch
    # modes back to position mode for safety purposes.
    self.arm_interface.set_command_timeout(self.timeout)
    # Baxter kinematics
    self.urdf = URDF.from_parameter_server(key='robot_description')
    self.tip_link = '%s_gripper' % limb
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    # Initialize the arm
    self.arm_interface.move_to_neutral(timeout=10.0)
    # Get the joint names
    self.joint_names = self.arm_interface.joint_names()
    self.num_joints = len(self.joint_names)
    # Set-up publishers/subscribers
    rospy.Subscriber('/baxter/%s_ik_command' % limb, PoseStamped, self.ik_command_cb)

  def shutdown(self):
    pass
  
  def get_kdl_angles(self):
    q = PyKDL.JntArray(self.num_joints)
    joint_angles = self.arm_interface.joint_angles()
    for i, name in enumerate(self.joint_names):
      q[i] = joint_angles[name]
    return q
  
  def get_kdl_velocities(self):
    qdot = PyKDL.JntArray(self.num_joints)
    joint_velocities = self.arm_interface.joint_velocities()
    for i, name in enumerate(self.joint_names):
      qdot[i] = joint_velocities[name]
    return qdot
  
  def jacobian_transpose(self, J):
    J_transpose = PyKDL.Jacobian(J.columns())
    for i in range(J.rows()):
      for j in range(J.columns()):
        J_transpose[j,i] = J[i,j]
    return J_transpose
  
  def ik_command_cb(self, msg):
    # TODO: Validate msg.header.frame_id
    x_target = posemath.fromMsg(msg.pose)
    # Get positions and velocities
    q = self.get_kdl_angles()
    qdot = self.get_kdl_velocities()
    # Compute the forward kinematics and Jacobian (at this location)
    x = posemath.fromMatrix(self.kinematics.forward(joint_kdl_to_list(q)))
    # Use the jacobian solver directly to get the PyKDL.Jacobian object
    J = PyKDL.Jacobian(self.num_joints)
    self.kinematics._jac_kdl.JntToJac(q, J)
    # TODO: Function to multiply a KDL::Jacobian with a KDL::JntArray to get a KDL::Twist, it should not be used 
    # to calculate the forward velocity kinematics, the solver classes are built for this purpose. J*q = t
    # Implement ChainFkSolverVel_recursive
    xdot = J * qdot
    # Calculate a Cartesian restoring wrench
    x_error = PyKDL.diff(x, x_target)
    wrench = PyKDL.Wrench(6)
    for i in range(6):
      wrench[i] = -(self.kp[i] * x_error[i] - self.kd[i] * xdot[i])
    # Convert the force into a set of joint torques
    tau = self.jacobian_transpose(J) * wrench
    print 'fuck!'


if __name__ == '__main__':
  node_name = 'pykdl_cartesian_controller'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  right_cc = CartesianController('right')
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
