#!/usr/bin/env python

"""
    Control the joints of a robot using a skeleton tracker such as the
    OpenNI tracker package in junction with a Kinect RGB-D camera.
    
    Based on the Pi Robot Project: http://www.pirobot.org
    
    Copyright (c) 2014 Francisco Suarez-Ruiz.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from math import pi, copysign
import numpy as np
# PyKDL staff
import PyKDL as KDL
from tf_conversions import posemath
from math import atan2
# Messages
from k4w2_msgs.msg import SkeletonState, SkeletonRaw, RPY
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import PoseStamped

class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''

sr = SkeletonRaw()
LEFT_ARM = [sr.ShoulderLeft, sr.ElbowLeft, sr.WristLeft, sr.HandLeft]
RIGHT_ARM = [sr.ShoulderRight, sr.ElbowRight, sr.WristRight, sr.HandRight]

BODY_LINKS = ('head', 
              'neck', 
              'torso', 
              'left_shoulder', 
              'left_elbow', 
              'left_hand', 
              'right_shoulder', 
              'right_elbow', 
              'right_hand', 
              'left_hip', 
              'left_knee', 
              'left_foot', 
              'right_hip', 
              'right_knee', 
              'right_foot')

BODY_JOINTS = { 'neck':       {'parent':sr.SpineShoulder, 'own':sr.Neck,          'child':sr.Head},
                'l_shoulder': {'parent':sr.ShoulderRight, 'own':sr.ShoulderLeft,  'child':sr.ElbowLeft},
                'l_elbow':    {'parent':sr.ShoulderLeft,  'own':sr.ElbowLeft,     'child':sr.WristLeft}
                #~ 'r_shoulder': {'parent':sr.ShoulderLeft,  'own':sr.ShoulderRight, 'child':sr.ElbowRight}
              }

class TrackerJointController():
  def __init__(self):
    rospy.on_shutdown(self.shutdown)
    self.loginfo("Initializing Joint Controller Node...")
    # Read from the parameter server
    self.publish_rate = rospy.get_param('~joint_controller_rate', 30)
    self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
    # Validate body joint names
    for key in self.skel_to_joint_map.keys():
      if key not in BODY_JOINTS.keys():
        rospy.logwarn('Invalid body joint name [%s]' % key)
        del self.skel_to_joint_map[key]
    # TODO: Validate the robot joint names
    
    # Initial values
    self.relative_frame = dict()
    self.skeleton = SkeletonState()
    self.left_command = JointCommand()
    self.right_command = JointCommand()
    self.timer = None
    self.teleoperate = False
    self.calibrated = True

    # Setup publishers / subscribers
    self.left_command_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_command_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    rospy.Subscriber('kinect/skeleton', SkeletonState, self.cb_skeleton_handler)
    # Start the timer that will publish the robot commands
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.process_skeleton)
    
    # Debug
    self.pose_pub = rospy.Publisher('/test/pose', PoseStamped)
    self.l_shoulder_pub = rospy.Publisher('/test/l_shoulder', RPY)
    self.l_elbow_pub = rospy.Publisher('/test/l_elbow', RPY)
    self.l_wrist_pub = rospy.Publisher('/test/l_wrist', RPY)
    
    rate = rospy.Rate(self.publish_rate)
    while not rospy.is_shutdown():
      if self.confident(LEFT_ARM + RIGHT_ARM):
        if self.confident(LEFT_ARM + RIGHT_ARM):
          self.calibrated = True
          self.teleoperate = True
        elif self.arms_crossed():
          self.teleoperate = False
          self.calibrated = False
        elif self.calibrated:
          self.teleoperate = True
      else:
        self.calibrated = False
        self.teleoperate = False
      #~ print self.calibrated, self.teleoperate
      rate.sleep()
  
  def point_to_vector(self, point):
    return KDL.Vector(point.x, point.y, point.z)
  
  def process_skeleton(self, event):
    if not self.confident(LEFT_ARM + RIGHT_ARM):
      return
    self.left_command = JointCommand()
    self.right_command = JointCommand()
    self.left_command.mode = self.left_command.POSITION_MODE
    self.right_command.mode = self.right_command.POSITION_MODE
    for name in self.skel_to_joint_map.keys():
      parent = BODY_JOINTS[name]['parent']
      own = BODY_JOINTS[name]['own']
      child = BODY_JOINTS[name]['child']
      parent_pos = self.point_to_vector(self.skeleton.pose[parent].position)
      own_pos = self.point_to_vector(self.skeleton.pose[own].position)
      child_pos = self.point_to_vector(self.skeleton.pose[child].position)
      a = child_pos - own_pos   # Direction vector
      b = own_pos - parent_pos  # Reference vector
      angle = 2 * atan2((a * b.Norm() - a.Norm() * b).Norm(), (a * b.Norm() + a.Norm() * b).Norm())
      axis = (a * b) / (a.Norm() * b.Norm())
      rotation = KDL.Rotation.Rot(axis, angle)
      rpy = list(rotation.GetRPY())
      for i,joint in enumerate(self.skel_to_joint_map[name]):
        if joint != 'no_joint':
          joint_name = joint
          if '-' == joint[0]:
            rpy[i] *= -1.0
            joint_name = joint[1:]
          if 'left_' in joint_name:
            self.left_command.names.append(joint_name)
            self.left_command.command.append(rpy[i])
          elif 'right_' in joint_name:
            self.right_command.names.append(joint_name)
            self.right_command.command.append(rpy[i])
    if self.teleoperate:
    #~ if False:
      self.left_command_pub.publish(self.left_command)
      self.right_command_pub.publish(self.right_command)
    
  def arms_crossed(self):
    confident = self.confident(LEFT_ARM + RIGHT_ARM)
    elbows_sign = copysign(1.0, self.skeleton.pose[sr.ElbowRight].position.x - self.skeleton.pose[sr.ElbowLeft].position.x)
    hands_sign = copysign(1.0, self.skeleton.pose[sr.HandRight].position.x - self.skeleton.pose[sr.Left].position.x)
    return (confident and (elbows_sign != hands_sign))
    
  def psi_pose(self, tolerance):
    result = False
    confident = self.confident(LEFT_ARM + RIGHT_ARM)
    try:
      left_arm = [self.joint_rpy['l_shoulder'][1], self.joint_rpy['l_shoulder'][2], self.joint_rpy['l_elbow'][1]]
      right_arm = [self.joint_rpy['r_shoulder'][1], self.joint_rpy['r_shoulder'][2], self.joint_rpy['r_elbow'][1]]
      target = [0,0,-pi/2,0,0,pi/2]
      in_psi_pose = np.allclose(left_arm + right_arm, target, atol=tolerance)
      result = (confident and in_psi_pose)
    except KeyError:
      result = False
    return result

  def confident(self, joints):
    if not self.skeleton.tracking_state:    # Empty list
      return False
    traking_state = np.array(self.skeleton.tracking_state)[joints]
    return (sr.TrackingState_NotTracked not in traking_state and sr.TrackingState_Inferred not in traking_state) 

  def cb_skeleton_handler(self, msg):
    # TODO: Median filter?
    self.skeleton = msg

  def shutdown(self):
    self.loginfo('Shutting down Tracker Joint Controller Node.')
    # Stop the publisher timer
    self.timer.shutdown() 
  
  def loginfo(self, msg):
    rospy.loginfo(TextColors().OKBLUE + str(msg) + TextColors().ENDC)


if __name__ == '__main__':
  rospy.init_node('tracker_joint_controller', log_level=rospy.WARN)
  try:
    TrackerJointController()
  except rospy.ROSInterruptException:
    pass
