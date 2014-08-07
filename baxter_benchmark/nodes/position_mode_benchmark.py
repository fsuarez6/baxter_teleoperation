#!/usr/bin/env python
import rospy, random, os, datetime
from math import pi
import numpy as np
import scipy.io as sio
# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand
# Kinematics Services
from moveit_kinematics_interface.srv import GetStateMetrics, GetStateMetricsRequest, GetStateMetricsResponse
from moveit_kinematics_interface.srv import GetJointLimits, GetJointLimitsRequest, GetJointLimitsResponse
# Baxter Interface
from baxter_interface import Limb
# Joint Limits
from priovr_joint_controller import JointLimits, BAXTER_JOINTS, read_parameter


class PositionBenchmark:
  def __init__(self):
    # Read parameters
    self.arm_name = read_parameter('~limb', 'right')
    if self.arm_name not in ['left','right']:
      rospy.logerr('Unknown arm name: %s. Should be [left | right]' % self.arm_name)
      exit(1)
    self.frame_id = read_parameter('~frame_id', 'base')
    self.test_cases = read_parameter('~test_cases', 10)
    self.raw_mode = read_parameter('~raw_mode', False)
    rospy.loginfo('Reference frame: %s' % self.frame_id)
    # Set-up baxter interface
    self.limb = Limb(self.arm_name)
    self.joint_names = self.limb.joint_names()
    self.num_joints = len(self.joint_names)
    
  def joint_states_cb(self):
    pass
    
  def valid_pose(self, pose):
    return (pose.position.x > 0.3) and (pose.position.z < 0.7)
  
  def run(self):
    # Subscribe to the FK service
    fk_srv_name = read_parameter('~metrics_service', '/kinematics_services/get_fk_metrics')
    rospy.loginfo('Waiting for %s service' % fk_srv_name)
    rospy.wait_for_service(fk_srv_name)
    fk_srv = rospy.ServiceProxy(fk_srv_name, GetStateMetrics)
    # Subscribe to limits_service
    limits_srv_name = read_parameter('~metrics_service', '/kinematics_services/get_joint_limits')
    rospy.loginfo('Waiting for %s service' % limits_srv_name)
    rospy.wait_for_service(limits_srv_name)
    limits_srv = rospy.ServiceProxy(limits_srv_name, GetJointLimits)
    # Get the joint limits
    req = GetJointLimitsRequest()
    req.header.stamp = rospy.Time.now()
    req.header.frame_id = self.frame_id
    req.name = self.joint_names
    try:
      res = limits_srv(req)
    except rospy.ServiceException, e:
      rospy.logwarn('Service did not process request: %s' % str(e))
    min_positions = np.array(res.min_position) + 0.8
    max_positions = np.array(res.max_position) - 0.8
    rospy.loginfo('min_positions: %s' % str(min_positions))
    rospy.loginfo('max_positions: %s' % str(max_positions))
    rospy.loginfo('Generating [%d] test cases' % self.test_cases)
    # Numpy arrays to be sent to Matlab
    goal_angles = np.array([]) 
    goal_poses = np.array([])
    reached_angles = np.array([])
    reached_poses = np.array([])
    # FK Request message
    req = GetStateMetricsRequest()
    random_joints = [0] * self.num_joints
    joint_angles = [0] * self.num_joints
    current_pose = Pose()
    for i in range(self.test_cases):
      # Generate random poses until they are valid
      joint_commands = dict()
      current_pose.position.x = -0.5
      while not self.valid_pose(current_pose):
        # Check for shutdowns
        if rospy.is_shutdown():
          return
        for j, name in enumerate(self.joint_names):
          random_joints[j] = random.uniform(min_positions[j], max_positions[j])
          joint_commands[name] = random_joints[j]
        req.joint_states.header.stamp = rospy.Time.now()
        req.joint_states.header.frame_id = self.frame_id
        req.joint_states.name = self.joint_names
        req.joint_states.position = list(random_joints)
        try:
          res = fk_srv(req)
          if res.found_group:
            current_pose = res.pose
        except rospy.ServiceException, e:
          rospy.logwarn('Service did not process request: %s' % str(e))
      # Prepare goal data for matlab
      goal_angles = np.append(goal_angles, np.array(random_joints), 0)
      goal_poses = np.append(goal_poses, np.array(self.pose2list(current_pose)), 0)
      if (i % 10 == 0) and (self.test_cases >= 10):
        rospy.loginfo('[Benchmark] Evaluated: %d/%d' % (i, self.test_cases))
      # Send the command to the robot
      self.limb.move_to_joint_positions(joint_commands, Raw=self.raw_mode)
      # Prepare reached data for matlab
      angles_dict = self.limb.joint_angles()
      pose_dict = self.limb.endpoint_pose()
      for k, name in enumerate(self.joint_names):
        joint_angles[k] = angles_dict[name]
      reached_angles = np.append(reached_angles, np.array(joint_angles), 0)
      req.joint_states.position = list(joint_angles)
      try:
        res = fk_srv(req)
        if res.found_group:
          pose = res.pose
      except rospy.ServiceException, e:
        rospy.logwarn('Service did not process request: %s' % str(e))
      reached_poses = np.append(reached_poses, np.array(self.pose2list(pose)), 0)
    # Prepares the data to save it in a .mat file
    goal_angles = goal_angles.reshape((self.test_cases, self.num_joints))
    goal_poses = goal_poses.reshape((self.test_cases, 7))
    reached_angles = reached_angles.reshape((self.test_cases, self.num_joints))
    reached_poses = reached_poses.reshape((self.test_cases, 7))
    rospy.loginfo('[Benchmark] Evaluated: %d/%d' % (i+1, self.test_cases))
    # Add timestamp to the filename
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    mode = 'position_mode'
    if self.raw_mode:
      mode = 'raw_mode'
    filename = os.path.expanduser('~/benchmark/%s-%s.mat' % (mode, timestamp))
    # Try to make the dir (It should exists although)
    try:
      os.makedirs(os.path.split(filename)[0])
    except OSError, e:
      pass
    sio.savemat(filename, { 'goal_angles':goal_angles, 'goal_poses':goal_poses,
                            'reached_angles':reached_angles, 'reached_poses':reached_poses}, oned_as='column')
    rospy.loginfo('Results saved to: %s' % filename)

  def pose2list(self, pose):
    data = [0] * 7
    data[0] = pose.position.x
    data[1] = pose.position.y
    data[2] = pose.position.z
    data[3] = pose.orientation.x
    data[4] = pose.orientation.y
    data[5] = pose.orientation.z
    data[6] = pose.orientation.w
    return data

  def list2pose(self, data):
    pose = Pose()
    pose.orientation.w = data[0]
    pose.orientation.x = data[1]
    pose.orientation.y = data[2]
    pose.orientation.z = data[3]
    pose.position.x = data[4]
    pose.position.y = data[5]
    pose.position.z = data[6]
    return pose


if __name__ == '__main__':
  rospy.init_node('position_mode_benchmark')
  pmb = PositionBenchmark()
  pmb.run()
