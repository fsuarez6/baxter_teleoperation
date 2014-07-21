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
from math import pi
from baxter_core_msgs.msg import JointCommand
        
if __name__ == '__main__':
  rospy.init_node('arms_to_zero')
  rospy.loginfo("Initializing arms_to_zero Node")
  left_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
  right_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
  joints = ['s0','s1','e0','e1','w0','w1','w2']
  left_msg = JointCommand()
  right_msg = JointCommand()
  left_msg.mode = JointCommand().POSITION_MODE
  right_msg.mode = JointCommand().POSITION_MODE
  zero_position = [0, 0, -pi/2, 0, pi/2, 0, 0]
  left_msg.command = zero_position
  right_msg.command = zero_position
  for joint in joints:
    left_msg.names.append('left_' + joint)
    right_msg.names.append('right_' + joint)
  start_time = rospy.Time.now()
  rospy.loginfo("Publishing command message for 5 seconds")
  while rospy.Time.now() < start_time + rospy.Duration(5.0):
    left_pub.publish(left_msg)
    right_pub.publish(right_msg)
    rospy.sleep(1/60.0)
  rospy.loginfo("Finishing arms_to_zero Node")
