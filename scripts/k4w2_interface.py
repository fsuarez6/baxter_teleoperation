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

import rospy, socket, struct, time
# Messages
from skeleton_markers.msg import Skeleton

BODY_LINKS = ('head', 
              'neck', 
              'torso', 
              'left_shoulder', 
              'left_elbow', 
              'left_hand', 
              'right_shoulder', 
              'right_elbow', 
              'right_hand', 
              'left_hip', 
              'left_knee', 
              'left_foot', 
              'right_hip', 
              'right_knee', 
              'right_foot')

class K4W2Interface():
  node_name = 'K4W2 Interface'
  def __init__(self):
    rospy.on_shutdown(self.shutdown)
    rospy.loginfo("Initializing %s Node" % self.node_name)
    # Read from the parameter server
    self.publish_rate = rospy.get_param('~publish_rate', 60)
    # UDP
    self.read_port = int(self.read_parameter('~read_port', 6565))
    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))
    
    #~ rate = rospy.Rate(20) # hz
    while not rospy.is_shutdown():
      data = self.recv_timeout()
      if data:
        print(len(data))
        fmt = '=51h175f2h'
        msg = struct.unpack(fmt, data[:struct.calcsize(fmt)])
        print(msg[0:51])
        
      #~ rate.sleep()
  
  def recv_timeout(self, timeout=0.01):
    self.read_socket.setblocking(0)
    total_data=[]
    data=''
    begin=time.time()
    while 1:
      #if you got some data, then timeout break 
      if total_data and time.time()-begin>timeout:
        break
      #if you got no data at all, wait a little longer
      elif time.time()-begin>timeout*2:
        break
      try:
        data=self.read_socket.recv(8192)
        if data:
          total_data.append(data)
          begin=time.time()
      except:
        pass
    return ''.join(total_data)

  def shutdown(self):
    rospy.loginfo("Shutting down %s Node" % self.node_name)
    # Stop the publisher timer
    #~ self.timer.shutdown()
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

if __name__ == '__main__':
  rospy.init_node('k4w2_interface')
  try:
    K4W2Interface()
  except rospy.ROSInterruptException:
    pass
