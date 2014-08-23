#!/usr/bin/env python
import rospy, os, argparse, subprocess
from baxter_interface.camera import CameraController

def connect_to_camera(name):
  cam = None
  try:
    cam = CameraController(name)
  except:
    pass
  return cam
  
def close_camera(cam):
  if cam:
    cam.close()

def open_camera(cam, res=(1280, 800)):
  if cam:
    cam.close()
    cam.resolution = res
    cam.open()

if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  # Parse arguments
  parser = argparse.ArgumentParser(description='%s' % node_name, add_help=True, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  action_grp = parser.add_mutually_exclusive_group(required=True)
  action_grp.add_argument('-r','--right',help='Enable right camera only',action='store_true')
  action_grp.add_argument('-l','--left',help='Enable left camera only',action='store_true')
  action_grp.add_argument('-he','--head',help='Enable head camera only',action='store_true')
  action_grp.add_argument('-b','--both',help='Enable left and right cameras',action='store_true')
  
  args = parser.parse_args(rospy.myargv()[1:])
  rospy.loginfo('Starting [%s] node' % node_name)
  
  # Enumerate cameras
  subprocess.check_output(['rosrun','baxter_tools','camera_control.py','-e']).rstrip()
  
  # Try to connect to the cameras
  l_cam = connect_to_camera('left_hand_camera')
  r_cam = connect_to_camera('right_hand_camera')
  h_cam = connect_to_camera('head_camera')
  
  # Close all the cameras
  close_camera(h_cam)
  close_camera(l_cam)
  close_camera(r_cam)
  
  # Open the corresponding camera(s)
  if args.both:
    open_camera(l_cam)
    open_camera(r_cam)
  elif args.right:
    open_camera(r_cam)
  elif args.left:
    open_camera(l_cam)
  elif args.head:
    open_camera(h_cam)

  rospy.loginfo('Shuting down [%s] node' % node_name)
