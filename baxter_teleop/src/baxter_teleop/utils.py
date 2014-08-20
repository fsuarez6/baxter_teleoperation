#! /usr/bin/env python
import rospy, math
import numpy

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
def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)
