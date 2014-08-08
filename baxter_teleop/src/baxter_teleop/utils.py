#! /usr/bin/env python
import rospy, math
import numpy

# Helper methods
def nearly_equal(a,b,sig_fig=3):
  return (a==b or int(a*10**sig_fig) == int(b*10**sig_fig))

def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)

# Classes
class JointLimits(object):
  def __init__(self, joint):
    self.lower = joint.limit.lower
    self.upper = joint.limit.upper
    self.effort = joint.limit.effort
    self.velocity = joint.limit.velocity
  
  def __str__(self):
    msg = 'lower: [%s] upper: [%s] effort: [%s] velocity: [%s]' % (self.lower, self.upper, self.effort, self.velocity)
    return msg


