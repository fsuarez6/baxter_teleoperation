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
from sensor_msgs.msg import JointState
from skeleton_markers.msg import Skeleton
from std_msgs.msg import Float64
import PyKDL as KDL
from math import acos, asin, pi

class TrackerJointController():
  def __init__(self):
    rospy.init_node('tracker_joint_controller')
    rospy.on_shutdown(self.shutdown)    
    rospy.loginfo("Initializing Joint Controller Node...")
    self.rate = rospy.get_param('~joint_controller_rate', 5)
    rate = rospy.Rate(self.rate)
    
    # The namespace may be set by the servo controller (usually null)
    namespace = rospy.get_namespace()    
    self.joints = rospy.get_param(namespace + '/joints', '')
    self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
    self.use_real_robot = rospy.get_param('~use_real_robot', False)    
    self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.5)    
    self.tracker_commands = rospy.get_param('~tracker_commands', ['STOP', 'TELEOP_JOINTS'])     
    self.HALF_PI = pi / 2.0

    # Subscribe to the skeleton topic.
    rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
    
    # Store the current skeleton configuration in a local dictionary.
    self.skeleton = dict()
    self.skeleton['confidence'] = dict()
    self.skeleton['position'] = dict()
    self.skeleton['orientation'] = dict()

    # Use the joint_state publisher for a fake robot
    self.joint_state_pub = rospy.Publisher('/joint_states', JointState)
    
    while not rospy.is_shutdown():
      self.teleop_joints()
      self.joint_state_pub.publish(self.joint_state)
      rate.sleep()
          
  def teleop_joints(self):
      # Fixed Joints: Set position to 0 radians.
      try:
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_shoulder_roll_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_shoulder_roll_joint')] = 2
          
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_wrist_flex_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_wrist_flex_joint')] = 2
      except KeyError:
          pass
      
      try:
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_shoulder_roll_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_shoulder_roll_joint')] = 2
          
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_wrist_flex_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_wrist_flex_joint')] = 2
      except KeyError:
          pass
                  
      # Torso Rotation
      try:
          torso_quaternion = self.skeleton['orientation']['torso']
          torso_rpy = torso_quaternion.GetRPY()
          self.cmd_joints.position[self.cmd_joints.name.index(self.skel_to_joint_map['torso'])] = torso_rpy[1] / 2.0
          self.cmd_joints.velocity[self.cmd_joints.name.index('torso_joint')] = self.default_joint_speed
      except:
          pass
      
      # Head Pan
      try:
          head_quaternion = self.skeleton['orientation']['head']
          head_rpy = head_quaternion.GetRPY()
          self.cmd_joints.position[self.cmd_joints.name.index('head_pan_joint')] = head_rpy[1]
          self.cmd_joints.velocity[self.cmd_joints.name.index('head_pan_joint')] = self.default_joint_speed
      except:
          pass
      
      # Head Tilt
      try:
          self.cmd_joints.position[self.cmd_joints.name.index('head_tilt_joint')] = -head_rpy[0]
          self.cmd_joints.velocity[self.cmd_joints.name.index('head_tilt_joint')] = self.default_joint_speed
      except:
          pass
      
      # Left Arm
      try:
          left_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['left_shoulder']
          left_shoulder_elbow = self.skeleton['position']['left_elbow'] - self.skeleton['position']['left_shoulder']
          left_arm_elbow_flex_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_elbow']
          left_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']
          
          left_shoulder_neck.Normalize()
          left_shoulder_elbow.Normalize()
          lh = left_arm_elbow_flex_hand.Normalize()
          left_shoulder_hand.Normalize()

          left_arm_shoulder_lift_angle = asin(left_shoulder_elbow.y()) + self.HALF_PI
          left_arm_shoulder_pan_angle = asin(left_arm_elbow_flex_hand.x())
          left_arm_elbow_flex_angle = -acos(KDL.dot(left_shoulder_elbow, left_arm_elbow_flex_hand))
          left_arm_wrist_flex_angle = -left_arm_elbow_flex_angle / 2.0
          left_arm_elbow_flex_angle = left_arm_elbow_flex_angle / 4.0
          
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_shoulder_lift_joint')] = left_arm_shoulder_lift_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_shoulder_lift_joint')] = self.default_joint_speed
          
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_shoulder_pan_joint')] = -left_arm_shoulder_pan_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_shoulder_pan_joint')] = self.default_joint_speed
          
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_elbow_flex_joint')] = left_arm_elbow_flex_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_elbow_flex_joint')] = self.default_joint_speed                                   
      
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_wrist_flex_joint')] = left_arm_wrist_flex_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_wrist_flex_joint')] = self.default_joint_speed

          left_arm_shoulder_roll_angle = acos(KDL.dot(left_shoulder_elbow, left_shoulder_neck))
          self.cmd_joints.position[self.cmd_joints.name.index('left_arm_shoulder_roll_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_shoulder_roll_joint')] = self.default_joint_speed      
      except KeyError:
          pass
          
      # Right Arm
      try:
          right_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['right_shoulder']
          right_shoulder_elbow = self.skeleton['position']['right_elbow'] - self.skeleton['position']['right_shoulder']
          right_arm_elbow_flex_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_elbow']
          right_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']
          
          right_shoulder_neck.Normalize()
          right_shoulder_elbow.Normalize()
          rh = right_arm_elbow_flex_hand.Normalize()
          right_shoulder_hand.Normalize()

          right_arm_shoulder_lift_angle = -(asin(right_shoulder_elbow.y()) + self.HALF_PI)
          right_arm_shoulder_pan_angle = -asin(right_arm_elbow_flex_hand.x())
          right_arm_elbow_flex_angle = acos(KDL.dot(right_shoulder_elbow, right_arm_elbow_flex_hand))
          right_arm_wrist_flex_angle = -right_arm_elbow_flex_angle / 2.0
          right_arm_elbow_flex_angle = right_arm_elbow_flex_angle / 4.0
                          
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_shoulder_lift_joint')] = right_arm_shoulder_lift_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_shoulder_lift_joint')] = self.default_joint_speed
          
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_shoulder_pan_joint')] = right_arm_shoulder_pan_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_shoulder_pan_joint')] = self.default_joint_speed
          
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_elbow_flex_joint')] = right_arm_elbow_flex_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_elbow_flex_joint')] = self.default_joint_speed
          
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_wrist_flex_joint')] = right_arm_wrist_flex_angle
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_wrist_flex_joint')] = self.default_joint_speed

          right_arm_shoulder_roll_angle = -asin(right_shoulder_hand.x())
          self.cmd_joints.position[self.cmd_joints.name.index('right_arm_shoulder_roll_joint')] = 0
          self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_shoulder_roll_joint')] = self.default_joint_speed
      except KeyError:
        pass
          
  def skeleton_handler(self, msg):
    for index, joint in enumerate(msg.name):
      self.skeleton['confidence'][joint] = msg.confidence[i]
      self.skeleton['position'][joint] = KDL.Vector(msg.position[i].x, msg.position[i].y, msg.position[i].z)
      self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[i].x, msg.orientation[i].y, msg.orientation[i].z, msg.orientation[i].w)

  def shutdown(self):
    rospy.loginfo('Shutting down Tracker Joint Controller Node.')
        
if __name__ == '__main__':
  try:
    TrackerJointController()
  except rospy.ROSInterruptException:
    pass
