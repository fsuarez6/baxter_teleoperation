#!/usr/bin/env python
import rospy, yaml, os, datetime
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
# Helpers
from baxter_teleop.utils import read_parameter


class RepeatabilityBenchmark:
  def __init__(self):
    # Read parameters
    self.arm_name = read_parameter('~limb', 'right')
    if self.arm_name not in ['left','right']:
      rospy.logerr('Unknown arm name: %s. Should be [left | right]' % self.arm_name)
      exit(1)
    self.frame_id = read_parameter('~frame_id', 'base')
    self.raw_mode = read_parameter('~raw_mode', False)
    self.repetitions = read_parameter('~repetitions', 25)
    rospy.loginfo('Reference frame: %s' % self.frame_id)
    # Set-up baxter interface
    self.limb = Limb(self.arm_name)
    self.joint_names = self.limb.joint_names()
    self.num_joints = len(self.joint_names)
    # Parse the yaml file to obtain the goal positions
    self.repeatability = read_parameter('~repeatability', dict())
    if not self.repeatability:
      rospy.logerr('[~repeatability] parameter not found')
      exit(1)
    self.test_cases = 0
    has_name = False
    key_word = 'position'
    for name in self.repeatability.keys():
      if len(self.repeatability[name]) !=  self.num_joints:
        rospy.logerr('Incorrect number of elements for [~repeatability/%s]' % name)
        exit(1)
      if key_word == name[:len(key_word)]:
        idx =  int(name[len(key_word):])
        if idx >= 0:
          self.test_cases += 1
        else:
          rospy.logwarn('Unexpected numbering for [~repeatability/%s]' % name)
          continue
      if name == 'name':
        has_name = True
    if self.test_cases == 0:
      rospy.logerr('Did not fount any position in the [~repeatability] parameter')
      exit(1)
    if not has_name:
      rospy.logerr('Did not fount name list in the [~repeatability] parameter')
      exit(1)

  def run(self):
    # Subscribe to the FK service
    fk_srv_name = read_parameter('~metrics_service', '/kinematics_services/get_fk_metrics')
    rospy.loginfo('Waiting for %s service' % fk_srv_name)
    rospy.wait_for_service(fk_srv_name)
    fk_srv = rospy.ServiceProxy(fk_srv_name, GetStateMetrics)
    # Get the goal angles from the repeatability dict
    key_word = 'position'
    goal_angles = [None] * self.test_cases
    for name, position in self.repeatability.items():
      if key_word == name[:len(key_word)]:
        idx =  int(name[len(key_word):])
        # sort the joint positions
        sorted_angles = [None] * self.num_joints
        for j, joint_name in enumerate(self.joint_names):
          data_idx = self.repeatability['name'].index(joint_name)
          sorted_angles[j] = self.repeatability[name][data_idx]
        goal_angles[idx] = sorted_angles
    # Include the repetitions
    self.test_cases *= self.repetitions
    one_repetition = list(goal_angles)
    for _ in range(self.repetitions - 1):
      goal_angles += one_repetition
    # Using Numpy arrays to be sent to Matlab
    goal_angles = np.array(goal_angles)
    goal_poses = np.array([])
    reached_angles = np.array([])
    reached_poses = np.array([])
    # FK Request message
    joint_angles = [0] * self.num_joints
    for i in range(self.test_cases):
      # Prepare the FK request message
      req = GetStateMetricsRequest()
      req.joint_states.header.stamp = rospy.Time.now()
      req.joint_states.header.frame_id = self.frame_id
      req.joint_states.name = self.joint_names
      req.joint_states.position = list(goal_angles[i][:])
      current_pose = Pose()
      try:
        res = fk_srv(req)
        if res.found_group:
          current_pose = res.pose
      except rospy.ServiceException, e:
        rospy.logwarn('Service did not process request: %s' % str(e))
      # Prepare goal data for matlab
      goal_poses = np.append(goal_poses, np.array(self.pose2list(current_pose)), 0)
      # Send the command to the robot
      joint_commands = dict()
      for j, name in enumerate(self.joint_names):
        joint_commands[name] = goal_angles[i][j]
      self.limb.move_to_joint_positions(joint_commands, Raw=self.raw_mode)
      rospy.loginfo('[Benchmark] Evaluated: %d/%d' % (i+1, self.test_cases))
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
    # Add timestamp to the filename
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    mode = 'repeatability_position'
    if self.raw_mode:
      mode = 'repeatability_raw'
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
  rospy.init_node('repeatability_benchmark')
  pmb = RepeatabilityBenchmark()
  pmb.run()
