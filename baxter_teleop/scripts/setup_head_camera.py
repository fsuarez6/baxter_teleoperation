#!/usr/bin/env python

"""
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

import rospy
from baxter_interface.camera import CameraController
        
if __name__ == '__main__':
  node_name = 'setup_head_camera'
  rospy.init_node(node_name)
  rospy.loginfo("Initializing %s Node" % node_name)
  # Close left and right cameras
  try:
    l_cam = CameraController('left_hand_camera')
    l_cam.close()
  except:
    pass
  try:
    r_cam = CameraController('right_hand_camera')
    r_cam.close()
  except:
    pass
  h_cam = CameraController('head_camera')
  h_cam.close()
  h_cam.resolution = (480, 300)
  h_cam.open()  
  rospy.loginfo("Finishing %s Node" % node_name)
