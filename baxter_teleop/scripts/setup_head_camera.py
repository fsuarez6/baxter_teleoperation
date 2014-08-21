#!/usr/bin/env python

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
