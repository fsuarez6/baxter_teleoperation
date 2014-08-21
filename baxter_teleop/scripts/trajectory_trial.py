#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Point, PoseStamped
import numpy as np

def point_to_array(point):
  return np.array(point.x, point.y, point.z)
        
if __name__ == '__main__':
  rospy.init_node('trajectory_trial')
  limb = 'right'
  ik_cmd_pub = rospy.Publisher('/baxter/%s_ik_command' % limb, PoseStamped)
  ros_msg = PoseStamped()
  ros_msg.header.frame_id = 'base'
  start_point = np.array([0.7, -0.7, 0.2])
  target_point = np.array([0.5, 0.0, 0.2])
  ros_msg.pose.orientation = Quaternion(0,1,0,0)
  rate = 100.0
  ros_rate = rospy.Rate(rate)   # Hz
  allowed_time = 5.0            # seconds
  segments = allowed_time * rate
  delta = (target_point - start_point) / segments
  cmd_point = start_point
  rospy.loginfo('Publishing %d commands in %d seconds' % (segments, allowed_time))
  start_time = rospy.Time.now()
  while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() <= allowed_time:
    ros_msg.pose.position = Point(*cmd_point)
    cmd_point += delta
    # Publish the msg
    ros_msg.header.stamp = rospy.Time.now()
    ik_cmd_pub.publish(ros_msg)
    ros_rate.sleep()
