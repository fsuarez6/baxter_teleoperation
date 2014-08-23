#! /usr/bin/env python
import rospy, math, numpy, PyKDL
from math import pi

# Globals
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

# Helper methods
def baxter_to_kdl_frame(pose):
  if pose.has_key('orientation') and pose.has_key('position'):
    rot = PyKDL.Rotation.Quaternion(*pose['orientation'])
    vec = PyKDL.Vector(*pose['position'])
    return PyKDL.Frame(rot, vec)
  else:
    return None
  
def baxter_to_kdl_twist(twist):
  if twist.has_key('linear') and twist.has_key('angular'):
    vel = PyKDL.Vector(*twist['linear'])
    rot = PyKDL.Vector(*twist['angular'])
    return PyKDL.Twist(vel, rot)
  else:
    return None
  
def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)
