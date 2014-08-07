#!/usr/bin/env python
import rospy, serial, sys
from math import pi
# Messages
from sensor_msgs.msg import JointState, Joy
from baxter_core_msgs.msg import JointCommand
# URDF
from urdf_parser_py.urdf import URDF
# Baxter Interface for the grippers
import baxter_interface
from baxter_interface import CHECK_VERSION


BAXTER_JOINTS = { 'left_s0':  {'offset': 0,      'factor': 1},
                  'left_s1':  {'offset': 0,      'factor': 1},
                  'left_e0':  {'offset': -pi/2,  'factor': 1},
                  'left_e1':  {'offset': 0,      'factor': 1},
                  'left_w0':  {'offset': 0,      'factor': 1},
                  'left_w1':  {'offset': 0,      'factor': 2},
                  'left_w2':  {'offset': 0,      'factor': 1},
                  'right_s0': {'offset': 0,      'factor': 1},
                  'right_s1': {'offset': 0,      'factor': 1},
                  'right_e0': {'offset': pi/2,   'factor': 1},
                  'right_e1': {'offset': 0,      'factor': 1},
                  'right_w0': {'offset': 0,      'factor': 1},
                  'right_w1': {'offset': 0,      'factor': 2},
                  'right_w2': {'offset': 0,      'factor': 1}
                }

# Joysticks globals
LEFT_Z_BUTTON = 0
LEFT_C_BUTTON = 1
RIGHT_Z_BUTTON = 2
RIGHT_C_BUTTON = 3
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 2
RIGHT_Y_AXIS = 3

def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)

class JointLimits(object):
  def __init__(self, joint):
    self.lower = joint.limit.lower
    self.upper = joint.limit.upper
    self.effort = joint.limit.effort
    self.velocity = joint.limit.velocity
  
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
        self.joint_limits[name] = JointLimits(joint)
    # Set-up grippers interface
    self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    self.right_gripper.calibrate()
    # Initial values
    self.buttons_prev = [0,0]
    self.c_bottom = False
    self.z_bottom = False
    self.right_w2_vel = 0
    self.right_w2_pos = BAXTER_JOINTS['right_w2']['offset']
    # Set-up publishers/subscribers
    self.left_arm = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
    self.right_arm = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
    rospy.Subscriber('/priovr/joint_states', JointState, self.joint_states_cb)
    rospy.Subscriber('/priovr/joysticks', Joy, self.joysticks_cb)
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
      q0 = BAXTER_JOINTS[joint_name]['offset']
      factor = BAXTER_JOINTS[joint_name]['factor']
      joint_cmd = (factor * msg.position[i]) + q0
      # Append command to the corresponding arm
      if 'left_' in joint_name:
        left_msg.names.append(joint_name)
        left_msg.command.append(joint_cmd)
      elif 'right_' in joint_name:
        right_msg.names.append(joint_name)
        right_msg.command.append(joint_cmd)
      # Update the right wrist position
      name = 'right_w2'
      self.right_w2_pos += self.right_w2_vel
      lower = self.joint_limits[name].lower
      upper = self.joint_limits[name].upper
      if (self.right_w2_pos < lower):
        self.right_w2_pos = lower
      if (self.right_w2_pos > upper):
        self.right_w2_pos = upper
      right_msg.names.append(name)
      right_msg.command.append(self.right_w2_pos)
          
    # Publish commands
    self.left_arm.publish(left_msg)
    self.right_arm.publish(right_msg)
    
  def joysticks_cb(self, msg):
    # Check that the buttons where pressed / relased
    if self.buttons_prev[0] != msg.buttons[RIGHT_C_BUTTON] or self.buttons_prev[1] != msg.buttons[RIGHT_Z_BUTTON]:
      if msg.buttons[RIGHT_C_BUTTON] == 1:
        self.c_bottom = not self.c_bottom
      if msg.buttons[RIGHT_Z_BUTTON] == 1:
        self.z_bottom = not self.z_bottom
      self.buttons_prev = [msg.buttons[RIGHT_C_BUTTON], msg.buttons[RIGHT_Z_BUTTON]]
      # Open or close the gripper
      if self.c_bottom:
        self.left_gripper.close()
      else:
        self.left_gripper.open()
      if self.z_bottom:
        self.right_gripper.close()
      else:
        self.right_gripper.open()
    # Update the right wrist velocity
    self.right_w2_vel = msg.axes[RIGHT_X_AXIS] * 0.02


# Main
if __name__ == '__main__':
  rospy.init_node('priovr_joint_controller')
  priovr_jc = JointController()
