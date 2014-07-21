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
from math import acos, asin, pi
# Messages
from sensor_msgs.msg import JointState
from skeleton_markers.msg import Skeleton
from std_msgs.msg import Float64
from baxter_core_msgs.msg import JointCommand


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
    rospy.init_node('tracker_joint_controller')
    rospy.on_shutdown(self.shutdown)
    rospy.loginfo("Initializing Joint Controller Node...")
    # Read from the parameter server
    self.rate = rospy.get_param('~joint_controller_rate', 5)
    self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
    # Validate body joint names
    for key in self.skel_to_joint_map.keys():
      if key not in BODY_JOINTS.keys():
        rospy.logwarn('Invalid body joint name [%s]' % key)
        del self.skel_to_joint_map[key]
    # TODO: Validate the robot joint names
    
    # Initial values
    self.skeleton = dict()
    self.skeleton['confidence'] = dict()
    self.skeleton['position'] = dict()
    self.skeleton['orientation'] = dict()
    self.left_command = JointCommand()
    self.right_command = JointCommand()

    # Setup publishers / subscribers
    self.left_command_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_command_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
    
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      self.teleop_joints()
      self.left_command_pub.publish(self.left_command)
      self.right_command_pub.publish(self.right_command)
      rate.sleep()

  def teleop_joints(self):
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
        q_joint = KDL.Rotation.Inverse(q_parent) * q_child
        rpy = q_joint.GetRPY()
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

  def skeleton_handler(self, msg):
    for i, joint in enumerate(msg.name):
      self.skeleton['confidence'][joint] = msg.confidence[i]
      self.skeleton['position'][joint] = KDL.Vector(msg.position[i].x, msg.position[i].y, msg.position[i].z)
      self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[i].x, msg.orientation[i].y, 
                                                                    msg.orientation[i].z, msg.orientation[i].w)

  def shutdown(self):
    rospy.loginfo('Shutting down Tracker Joint Controller Node.')
        
if __name__ == '__main__':
  try:
    TrackerJointController()
  except rospy.ROSInterruptException:
    pass
