#!/usr/bin/env python
import rospy
from math import pi
from baxter_core_msgs.msg import JointCommand
        
if __name__ == '__main__':
  rospy.init_node('arms_to_zero')
  rospy.loginfo('Initializing arms_to_zero Node')
  left_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
  right_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
  joints = ['s0','s1','e0','e1','w0','w1','w2']
  left_msg = JointCommand()
  right_msg = JointCommand()
  left_msg.mode = JointCommand().POSITION_MODE
  right_msg.mode = JointCommand().POSITION_MODE
  left_msg.command = [pi/4, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
  right_msg.command = [-pi/4, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
  for joint in joints:
    left_msg.names.append('left_' + joint)
    right_msg.names.append('right_' + joint)
  start_time = rospy.Time.now()
  duration = 10
  rate = rospy.Rate(100.0)
  rospy.loginfo('Publishing command message for %s seconds' % duration)
  while rospy.Time.now() < start_time + rospy.Duration(duration):
    if rospy.is_shutdown():
      break
    left_pub.publish(left_msg)
    right_pub.publish(right_msg)
    rate.sleep()
  rospy.loginfo('Finishing arms_to_zero Node')
