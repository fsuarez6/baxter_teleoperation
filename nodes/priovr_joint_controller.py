#!/usr/bin/env python
import rospy, serial, sys
from math import pi
# Messages
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand
# URDF
from urdf_parser_py.urdf import URDF


BAXTER_JOINTS = { 'left_s0':  {'initial': 0,      'factor': 1},
                  'left_s1':  {'initial': 0,      'factor': 1},
                  'left_e0':  {'initial': -pi/2,  'factor': 1},
                  'left_e1':  {'initial': 0,      'factor': 1},
                  'left_w0':  {'initial': 0,      'factor': 1},
                  'left_w1':  {'initial': 0,      'factor': 2},
                  'left_w2':  {'initial': 0,      'factor': 1},
                  'right_s0': {'initial': 0,      'factor': 1},
                  'right_s1': {'initial': 0,      'factor': 1},
                  'right_e0': {'initial': pi/2,   'factor': 1},
                  'right_e1': {'initial': 0,      'factor': 1},
                  'right_w0': {'initial': 0,      'factor': 1},
                  'right_w1': {'initial': 0,      'factor': 2},
                  'right_w2': {'initial': 0,      'factor': 1}
                }


class JointLimit(object):
  def __init__(self):
    self.lower = None
    self.upper = None
    self.effort = None
    self.velocity = None
  
  def __str__(self):
    msg = 'lower: [%s] upper: [%s] effort: [%s] velocity: [%s]' % (self.lower, self.upper, self.effort, self.velocity)
    return msg


class JointController(object):
  def __init__(self):
    # Get the joint limits
    self.joint_limits = dict()
    robot_urdf = URDF.from_parameter_server()
    for name, joint in robot_urdf.joint_map.items():
      if name in BAXTER_JOINTS.keys():
        self.joint_limits[name] = JointLimit()
        self.joint_limits[name].lower = joint.limit.lower
        self.joint_limits[name].upper = joint.limit.upper
        self.joint_limits[name].effort = joint.limit.effort
        self.joint_limits[name].velocity = joint.limit.velocity
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
      # Skip command if it's out of the joint limits
      lower = self.joint_limits[joint_name].lower
      upper = self.joint_limits[joint_name].upper
      if not (lower <= msg.position[i] <= upper):
        continue
      # Prepare the joint command
      q0 = BAXTER_JOINTS[joint_name]['initial']
      factor = BAXTER_JOINTS[joint_name]['factor']
      joint_cmd = (factor * msg.position[i]) + q0
      # Append command to the corresponding arm
      if 'left_' in joint_name:
        left_msg.names.append(joint_name)
        left_msg.command.append(joint_cmd)
      elif 'right_' in joint_name:
        right_msg.names.append(joint_name)
        right_msg.command.append(joint_cmd)
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
