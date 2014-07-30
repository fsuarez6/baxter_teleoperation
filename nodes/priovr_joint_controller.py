#!/usr/bin/env python
import rospy, serial, sys, math
# Messages
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand


BAXTER_JOINTS = { 'left_s0': 0,
                  'left_s1': 0,
                  'left_e0': -math.pi/2,
                  'left_e1': 0,
                  'left_w0': 0,
                  'left_w1': 0,
                  'left_w2': 0,
                  'right_s0': 0,
                  'right_s1': 0,
                  'right_e0': math.pi/2,
                  'right_e1': 0,
                  'right_w0': 0,
                  'right_w1': 0,
                  'right_w2': 0
                }

class JointController(object):
  def __init__(self):
    # Set-up publishers/subscribers
    rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
    self.left_arm = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_arm = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    rospy.spin()

  def joint_states_cb(self, msg):
    # Define command messages
    left_msg = JointCommand()
    right_msg = JointCommand()
    left_msg.mode = JointCommand().POSITION_MODE
    right_msg.mode = JointCommand().POSITION_MODE
    # Populate command messages
    for i, joint_name in enumerate(msg.name):
      # Validate the joint name
      if joint_name not in BAXTER_JOINTS.keys():
        rospy.logwarn('Unknown joint: %s' % joint_name)
        continue
      if 'left_' in joint_name:
        left_msg.names.append(joint_name)
        left_msg.command.append(msg.position[i] + BAXTER_JOINTS[joint_name])
      elif 'right_' in joint_name:
        right_msg.names.append(joint_name)
        right_msg.command.append(msg.position[i] + BAXTER_JOINTS[joint_name])
    # Publish commands
    self.left_arm.publish(left_msg)
    self.right_arm.publish(right_msg)

  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

# Main
if __name__ == '__main__':
  rospy.init_node('priovr_joint_controller')
  priovr_jc = JointController()
