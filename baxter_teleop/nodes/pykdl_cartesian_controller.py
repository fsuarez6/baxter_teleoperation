#!/usr/bin/env python

import rospy, math
import numpy as np
# PyKDL
import PyKDL
from tf_conversions import posemath
from pykdl_utils.kdl_kinematics import KDLKinematics, joint_kdl_to_list, joint_list_to_kdl, kdl_to_mat
# URDF
from urdf_parser_py.urdf import URDF
# Baxter Python Interface
import baxter_interface
# Utils
from baxter_teleop.utils import read_parameter
# Messages
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, UInt16


def baxter_to_kdl_frame(pose):
  if pose.has_key('orientation') and pose.has_key('position'):
    rot = PyKDL.Rotation.Quaternion(*pose['orientation'])
    vec = PyKDL.Vector(*pose['position'])
    return PyKDL.Frame(rot, vec)
  else:
    return None
  
def baxter_to_kdl_twist(twist):
  if twist.has_key('linear') and twist.has_key('angular'):
    vel = PyKDL.Vector(*twist['linear'])
    rot = PyKDL.Vector(*twist['angular'])
    return PyKDL.Twist(vel, rot)
  else:
    return None

class CartesianController(object):
  def __init__(self, limb):
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    # Read the controllers parameters
    gains = read_parameter('~gains', dict())
    self.kp = joint_list_to_kdl(gains['Kp'])
    self.kd = joint_list_to_kdl(gains['Kd'])
    self.publish_rate = read_parameter('~publish_rate', 500)
    self.frame_id = read_parameter('~frame_id', 'base')
    self.timeout = read_parameter('~timeout', 0.1)       # seconds
    # Set-up baxter interface
    self.arm_interface = baxter_interface.Limb(limb)
    # set_joint_torques must be commanded at a rate great than the timeout specified by set_command_timeout.
    # If the timeout is exceeded before a new set_joint_velocities command is received, the controller will switch
    # modes back to position mode for safety purposes.
    self.arm_interface.set_command_timeout(self.timeout)
    # Baxter kinematics
    self.urdf = URDF.from_parameter_server(key='robot_description')
    self.tip_link = '%s_gripper' % limb
    self.kinematics = KDLKinematics(self.urdf, self.frame_id, self.tip_link)
    self.fk_vel_solver = PyKDL.ChainFkSolverVel_recursive(self.kinematics.chain)
    # Adjust the publish rate of baxter's state
    joint_state_pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
    # Initialize the arm
    self.arm_interface.move_to_neutral(timeout=10.0)
    # Get the joint names
    self.joint_names = self.arm_interface.joint_names()
    self.num_joints = len(self.joint_names)
    # Set-up publishers/subscribers
    self.suppress_grav_comp = rospy.Publisher('/robot/limb/%s/suppress_gravity_compensation' % limb, Empty)
    joint_state_pub_rate.publish(int(self.publish_rate))
    rospy.Subscriber('/baxter/%s_ik_command' % limb, PoseStamped, self.ik_command_cb)
    rospy.loginfo('Running Cartesian controller for the %s arm' % limb)
    # Start torque controller timer
    self.cart_command = None
    self.torque_controller_timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.torque_controller_cb)

  def shutdown(self):
    self.torque_controller_timer.shutdown()
    self.arm_interface.exit_control_mode()

  def get_kdl_array_vel(self):
    out = PyKDL.JntArrayVel(self.num_joints)
    joint_angles = self.arm_interface.joint_angles()
    joint_velocities = self.arm_interface.joint_velocities()
    for i, name in enumerate(self.joint_names):
      out.q[i] = joint_angles[name]
      out.qdot[i] = joint_velocities[name]
    return out
  
  def jacobian_transpose(self, J):
    J_mat = kdl_to_mat(J)
    return J_mat.T
  
  def ik_command_cb(self, msg):
    self.cart_command = msg
    # Add stamp because the console app don't use it
    self.cart_command.header.stamp = rospy.Time.now()

  def torque_controller_cb(self, event):
    """
    Implementation of a Cartesian controller with PyKDL
    
    """
    if rospy.is_shutdown() or self.cart_command == None:
      return
    elapsed_time = rospy.Time.now() - self.cart_command.header.stamp
    if elapsed_time.to_sec() > self.timeout:
      return
    # TODO: Validate msg.header.frame_id
    # Create a copy of the message so it's safe thread
    x_target = posemath.fromMsg(self.cart_command.pose)
    # Get positions and velocities
    jnt_array_vel = self.get_kdl_array_vel()
    # Get the current forward kinematics of the robot
    x = baxter_to_kdl_frame(self.arm_interface.endpoint_pose())
    xdot = baxter_to_kdl_twist(self.arm_interface.endpoint_velocity())
    # Calculate a Cartesian restoring wrench
    x_error = PyKDL.diff(x_target, x)
    wrench = np.matrix(np.zeros(6)).T
    for i in range(len(wrench)):
      wrench[i] = -(self.kp[i] * x_error[i] + self.kd[i] * xdot[i])
    # Calculate the jacobian
    J = PyKDL.Jacobian(self.num_joints)
    self.kinematics._jac_kdl.JntToJac(jnt_array_vel.q, J)
    # Convert the force into a set of joint torques. tau = J^T * wrench
    tau = self.jacobian_transpose(J) * wrench
    # TODO: Suppress gravity compensation??
    #~ self.suppress_grav_comp.publish(Empty())
    # Populate the joint_torques
    joint_torques = dict()
    for i, name in enumerate(self.joint_names):
      joint_torques[name] = tau[i]
    try:
      self.arm_interface.set_joint_torques(joint_torques)
    except rospy.ROSException:
      pass


if __name__ == '__main__':
  node_name = 'pykdl_cartesian_controller'
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  right_cc = CartesianController('right')
  rospy.spin()
  rospy.loginfo('Shuting down [%s] node' % node_name)
