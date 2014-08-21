#!/usr/bin/env python
import rospy
import numpy as np
# Messages
from geometry_msgs.msg import Point, PoseStamped, Quaternion
# Utils
from baxter_teleop.utils import read_parameter


class CartCoupling(object):
  def __init__(self, limb):
    if limb not in ['right', 'left']:
      rospy.logerr('Unknown limb name [%s]' % limb)
      return
    self.ik_cmd_pub = rospy.Publisher('/baxter/%s_ik_command' % limb, PoseStamped)
    rospy.Subscriber('/priovr/%s_wrist_pose' % limb[0], PoseStamped, self.pose_cb)
    rospy.spin()

  def pose_cb(self, msg):
    cmd_msg = msg
    cmd_msg.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)
    self.ik_cmd_pub.publish(cmd_msg)

# Main
if __name__ == '__main__':
  rospy.init_node('priovr_joint_controller')
  cc = CartCoupling('right')
  
