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
import PyKDL as KDL
from math import pi, copysign
import numpy as np
# Messages
from skeleton_markers.msg import Skeleton
from baxter_core_msgs.msg import JointCommand

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

BODY_JOINTS = { 'neck':       {'parent':'torso',          'child':'head'},
                'l_shoulder': {'parent':'torso',          'child':'left_shoulder'},
                'l_elbow':    {'parent':'left_shoulder',  'child':'left_elbow'},
                'l_wrist':    {'parent':'left_elbow',     'child':'left_hand'},
                'r_shoulder': {'parent':'torso',          'child':'right_shoulder'},
                'r_elbow':    {'parent':'right_shoulder', 'child':'right_elbow'},
                'r_wrist':    {'parent':'right_elbow',    'child':'right_hand'}
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
    self.joint_orientation = dict()
    self.joint_rpy = dict()
    self.skeleton = dict()
    self.skeleton['confidence'] = dict()
    self.skeleton['position'] = dict()
    self.skeleton['orientation'] = dict()
    self.left_command = JointCommand()
    self.right_command = JointCommand()
    self.timer = None
    self.teleoperate = False
    self.calibrated = True

    # Setup publishers / subscribers
    self.left_command_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_command_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    rospy.Subscriber('skeleton', Skeleton, self.cb_skeleton_handler)
    # Start the timer that will publish the robot commands
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.process_skeleton)
    
    rate = rospy.Rate(self.publish_rate)
    while not rospy.is_shutdown():
      if self.confident(BODY_LINKS[:7]):
        if self.psi_pose(0.5):
          self.calibrated = True
          self.teleoperate = False
        elif self.arms_crossed():
          self.teleoperate = False
          self.calibrated = False
        elif self.calibrated:
          self.teleoperate = True
      else:
        self.calibrated = False
        self.teleoperate = False
      rate.sleep()

  def process_skeleton(self, event):
    self.left_command = JointCommand()
    self.right_command = JointCommand()
    self.left_command.mode = self.left_command.POSITION_MODE
    self.right_command.mode = self.right_command.POSITION_MODE
    for name in self.skel_to_joint_map.keys():
      parent = BODY_JOINTS[name]['parent']
      child = BODY_JOINTS[name]['child']
      try:
        q_parent = self.skeleton['orientation'][parent]
        q_child = self.skeleton['orientation'][child]
        self.joint_orientation[name] = KDL.Rotation.Inverse(q_parent) * q_child
        self.joint_rpy[name] = self.joint_orientation[name].GetRPY()
        rpy = self.joint_rpy[name]
        for i,joint in enumerate(self.skel_to_joint_map[name]):
          if joint != 'no_joint':
            if 'left_' in joint:
              self.left_command.names.append(joint)
              self.left_command.command.append(rpy[i])
            elif 'right_' in joint:
              self.right_command.names.append(joint)
              self.right_command.command.append(rpy[i])
      except KeyError:
        pass
    if self.teleoperate:
      self.left_command_pub.publish(self.left_command)
      self.right_command_pub.publish(self.right_command)
    
  def arms_crossed(self):
    confident = self.confident(['left_elbow', 'right_elbow', 'left_hand', 'right_hand'])
    try:
      elbows_sign = copysign(1.0, self.skeleton['position']['left_elbow'].x() - self.skeleton['position']['right_elbow'].x())
      hands_sign = copysign(1.0, self.skeleton['position']['left_hand'].x() - self.skeleton['position']['right_hand'].x())
      return (confident and (elbows_sign != hands_sign))
    except KeyError:
      return False
    
  def psi_pose(self, tolerance):
    result = False
    confident = self.confident(['left_shoulder','right_shoulder','left_elbow','right_elbow','left_hand','right_hand'])
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
    try:
      for joint in joints:
          if self.skeleton['confidence'][joint] < 0.5:
              return False
      return True
    except KeyError:
      return False

  def cb_skeleton_handler(self, msg):
    for i, joint in enumerate(msg.name):
      self.skeleton['confidence'][joint] = msg.confidence[i]
      self.skeleton['position'][joint] = KDL.Vector(msg.position[i].x, msg.position[i].y, msg.position[i].z)
      self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[i].x, msg.orientation[i].y, 
                                                                    msg.orientation[i].z, msg.orientation[i].w)

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
