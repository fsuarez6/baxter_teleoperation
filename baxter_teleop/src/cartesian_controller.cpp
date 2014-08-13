#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <eigen_conversions/eigen_msg.h>

// Messages
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>

// Flann
#include <float.h>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

// Joint limits
#include <joint_limits_interface/joint_limits_urdf.h>

// Moveit Kinematics Interface
#include <moveit/kinematics_interface/kinematics_interface.h>

using namespace moveit;

class CartesianController 
{
  private:
    // Ros
    ros::NodeHandle                           nh_, nh_private_;
    ros::Publisher                            control_publisher_;
    ros::Publisher                            pose_publisher_;
    ros::Subscriber                           end_point_sub_;
    ros::Subscriber                           motion_control_sub_;
    ros::Timer                                pose_timer_;
    // Kinematics
    KinematicsInterfacePtr                    ik_kinematics_;
    Eigen::Affine3d                           end_effector_pose_;
    std::map<std::string, joint_limits_interface::JointLimits> urdf_limits_;
    // flann
    flann::Index<flann::L2<float> >*          position_index_;
    std::map<std::string, flann::Matrix<float> >  metrics_db_;
    // Time
    ros::Time                                 last_state_print_;
    ros::Time                                 last_motion_print_;
    ros::Time                                 last_ik_time_;
    // Misc
    std::vector<std::string>                  joint_names_;
    std::string                               robot_namespace_;
    std::string                               model_frame_; 
    double                                    publish_rate_;
    double                                    position_error_;
    bool                                      raw_mode_;
    
  public:
    CartesianController(): 
      nh_private_("~")
    { 
      // Get parameters from the server
      nh_private_.param(std::string("position_error"), position_error_, 0.05);
      nh_private_.param(std::string("publish_rate"), publish_rate_, 100.0);
      std::string database_name, folder_key, file_key, planning_group;
      nh_private_.param(std::string("metrics_database"), database_name, std::string("ik_metrics"));
      nh_private_.param(std::string("folder_key"), folder_key, std::string("6575ecddff8e44245a6d08f7e6a232f6"));
      nh_private_.param(std::string("file_key"), file_key, std::string("dc12ea999aabf684e4dab37176899cd2"));
      nh_private_.param(std::string("planning_group"), planning_group, std::string("right_arm"));
      nh_private_.param(std::string("raw_mode"), raw_mode_, false);

      if (!nh_private_.hasParam("position_error"))
        ROS_WARN_STREAM("Parameter [~position_error] not found, using default: " << position_error_ << " m.");
      if (!nh_private_.hasParam("publish_rate"))
        ROS_WARN_STREAM("Parameter [~publish_rate] not found, using default: " << publish_rate_ << " Hz");
      if (!nh_private_.hasParam("metrics_database"))
        ROS_WARN_STREAM("Parameter [~metrics_database] not found, using default: " << database_name);
      if (!nh_private_.hasParam("folder_key"))
        ROS_WARN_STREAM("Parameter [~folder_key] not found, using default: " << folder_key);
      if (!nh_private_.hasParam("file_key"))
        ROS_WARN_STREAM("Parameter [~file_key] not found, using default: " << file_key);
      if (!nh_private_.hasParam("planning_group"))
        ROS_WARN_STREAM("Parameter [~planning_group] not found, using default: " << planning_group);
      if (!nh_private_.hasParam("raw_mode"))
        ROS_WARN_STREAM("Parameter [~raw_mode] not found, using default: " << raw_mode_);

      // Get robot namespace
      robot_namespace_ = ros::this_node::getNamespace();
      if (robot_namespace_.rfind("/") != robot_namespace_.length()-1)
        robot_namespace_.append("/");
      if (robot_namespace_.length() > 1) 
      {
        if (robot_namespace_[0] == robot_namespace_[1])
          robot_namespace_.erase(0, 1);
      }

      // Kinematic interfaces
      ik_kinematics_.reset(new KinematicsInterface());
      joint_names_ = ik_kinematics_->getActiveJointModelNames();
      model_frame_ = ik_kinematics_->getModelFrame();
      urdf_limits_ = ik_kinematics_->getJointLimits();

      // Setup publishers and subscribers
      std::string arm_name;
      if (planning_group.compare("left_arm") == 0)
        arm_name = "left";
      else if (planning_group.compare("right_arm") == 0)
        arm_name = "right";
      else
      {
        ROS_ERROR_STREAM("Unknown planning group:\n" << planning_group);
        ros::shutdown();
        return;
      }
      std::ostringstream topic_name;
      topic_name << "/robot/limb/" << arm_name << "/joint_command";
      control_publisher_ = nh_.advertise<baxter_core_msgs::JointCommand>(topic_name.str().c_str(), 1);
      topic_name.str(std::string());    // Clear
      topic_name << "/robot/limb/" << arm_name << "/endpoint_state";
      end_point_sub_ = nh_.subscribe(topic_name.str().c_str(), 1, &CartesianController::endPointStateCB, this);
      topic_name.str("/baxter/ik_command");
      motion_control_sub_ = nh_.subscribe(topic_name.str().c_str(), 1, &CartesianController::ikCommandCB, this); 
      topic_name.str("/baxter/pose");
      pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name.str().c_str(), 1);
      
      // Load the previously generated metrics
      std::string filename;
      filename = getFilename(database_name, folder_key, file_key);
      ROS_INFO_STREAM("Loading [metrics database] from:\n" << filename);
      ros::Time flann_start_time = ros::Time::now();
      try {
        flann::load_from_file(metrics_db_["positions"], filename, "positions");
        flann::load_from_file(metrics_db_["orientations"], filename, "orientations");
        flann::load_from_file(metrics_db_["metrics"], filename, "metrics");
        flann::load_from_file(metrics_db_["joint_states"], filename, "joint_states");
      }
      catch (...) {
        ROS_ERROR_STREAM("Failed loading [metrics database] from:\n" << filename);
        ros::shutdown();
        return;
      }
      // Load the previously generated index
      std::ostringstream index_file;
      index_file << getFolderName(folder_key) << "ik_metrics_index." << file_key << ".dat";
      flann::SavedIndexParams saved_params = flann::SavedIndexParams(index_file.str());
      position_index_ = new flann::Index<flann::L2<float> > (metrics_db_["positions"], saved_params);
      double elapsed_time = (ros::Time::now() - flann_start_time).toSec();
      ROS_INFO("[metrics database] successfully loaded in %.2f seconds", elapsed_time);

      // Setup timer for publishing pose at 60 Hz
      pose_timer_ = nh_.createTimer(ros::Duration(1.0/60), &CartesianController::publishPose, this);
      last_motion_print_ = ros::Time::now();
    }
    
    ~CartesianController(){}

    // publish pose only (for rviz visualization)
    void publishPose(const ros::TimerEvent& _event) 
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = model_frame_;
      tf::poseEigenToMsg(end_effector_pose_, pose_msg.pose);
      pose_msg.header.stamp = ros::Time::now();
      pose_publisher_.publish(pose_msg);
    }
    
    void endPointStateCB(const baxter_core_msgs::EndpointStateConstPtr& _msg) 
    {
      tf::poseMsgToEigen(_msg->pose, end_effector_pose_);
      // Debug
      if (ros::Time::now() - last_state_print_  >= ros::Duration(1.0))
      {
        last_state_print_ = ros::Time::now();
        ROS_DEBUG_STREAM("T [with respect base_link]:\n" << end_effector_pose_.matrix());
      }
    }
    
    void ikCommandCB(const geometry_msgs::PoseStampedConstPtr& _msg)
    { 
      // Validate the message frame_id
      if (_msg->header.frame_id != model_frame_)
      {
        ROS_WARN("ikCommandCB: frame_id [%s] received. Expected [%s]", _msg->header.frame_id.c_str(), model_frame_.c_str());
        return;
      }
      // Check that the IK command has changed enought
      Eigen::Affine3d goal_pose, current_pose = ik_kinematics_->getEndEffectorTransform();
      tf::poseMsgToEigen(_msg->pose, goal_pose);
      Eigen::Translation3d position_difference;
      double x_dist = goal_pose.translation().x() - current_pose.translation().x();
      double y_dist = goal_pose.translation().y() - current_pose.translation().y();
      double z_dist = goal_pose.translation().z() - current_pose.translation().z();
      double xyz_dist = sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2));
      bool small_change = false;
      if (xyz_dist < position_error_)
        small_change = true;
      // Get the latest robot state
      std::vector<double> joint_positions, cmd_joint_positions;
      ik_kinematics_->getJointPositions(joint_positions);
      bool found_ik = true;
      if (!small_change)
      {
        // Here 1 is the number of random restart and 0.001s is the allowed time after each restart
        bool found_ik = ik_kinematics_->setEndEffectorPose(_msg->pose, 1, 0.001);
      }
      // Get the new joint states for the arm
      if (found_ik)
        ik_kinematics_->getJointPositions(cmd_joint_positions);
      else
      {
        ROS_DEBUG("Did not find IK solution");
        // Determine nn closest XYZ points in the reachability database
        int max_nn = 1000, nearest_neighbors;
        flann::Matrix<float> query_pos(new float[3], 1, 3);
        query_pos[0][0] = _msg->pose.position.x;
        query_pos[0][1] = _msg->pose.position.y;
        query_pos[0][2] = _msg->pose.position.z;
        flann::Matrix<int> indices(new int[query_pos.rows*max_nn], query_pos.rows, max_nn);
        flann::Matrix<float> dists(new float[query_pos.rows*max_nn], query_pos.rows, max_nn);
        // do a knn search, using 128 checks
        // this->index_pos->knnSearch(query_pos, indices, dists, nn, flann::SearchParams(128));
        nearest_neighbors = indices.cols;
        float radius = pow(position_error_, 2);
        nearest_neighbors = position_index_->radiusSearch(query_pos, indices, dists, radius, flann::SearchParams(128));
        // Check that we found something
        if (nearest_neighbors <= 0) {
          ROS_INFO_THROTTLE(60, "Didn't find any shit. Query: [%.3f, %.3f, %.3f]", _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
          return; }
        nearest_neighbors = fmin(nearest_neighbors, max_nn);
        std::vector<float> score(nearest_neighbors), d_q(nearest_neighbors),
                           d_j(nearest_neighbors), d_xyz(nearest_neighbors);
        Eigen::Quaterniond q_actual(end_effector_pose_.rotation());
        Eigen::Quaterniond q;
        for (std::size_t i=0; i < nearest_neighbors; ++i)
        {
          // http://math.stackexchange.com/questions/90081/quaternion-distance
          q.w() = metrics_db_["orientations"][i][0];
          q.x() = metrics_db_["orientations"][i][1];
          q.y() = metrics_db_["orientations"][i][2];
          q.z() = metrics_db_["orientations"][i][3];
          d_q[i] = 1 - pow(q.dot(q_actual), 2.0);
          d_xyz[i] = sqrtf(dists[0][i]);
          d_j[i] = 0;
          for(std::size_t j=0; j < joint_names_.size(); ++j)
          {
            float j_error = fabs(joint_positions[i] - metrics_db_["joint_states"][indices[0][i]][j]);
            d_j[i] = j_error;
          }
          // Heuristics to improve the ik solution
          score[i] = d_xyz[i] + d_j[i];
          //~ score[i] = 0.5*d_q[i] + 0.5*d_j[i];
        }
        std::size_t choice_idx = std::min_element(score.begin(), score.end()) - score.begin();
        ROS_DEBUG("nn [%d] choice [%d] score [%f] d_q [%f] d_xyz [%f] d_j [%f]", nearest_neighbors, 
                    (int)choice_idx, score[choice_idx], d_q[choice_idx], d_xyz[choice_idx], d_j[choice_idx]);
        // Populate the new joint_positions
        for(std::size_t i=0; i < joint_names_.size(); ++i)
          cmd_joint_positions.push_back(metrics_db_["joint_states"][indices[0][choice_idx]][i]);
        ik_kinematics_->setJointPositions(cmd_joint_positions);
        // Debugging
        if (ros::Time::now() - last_motion_print_  >= ros::Duration(1.0))
        {
          last_motion_print_ = ros::Time::now();
          ROS_DEBUG_STREAM("Index [" << choice_idx << "] Distance [" << dists[0][choice_idx] << "]");
        }
      }
      // Command the robot to the new joint_values
      ROS_DEBUG("Joint names: %d, Joint values: %d", int(joint_names_.size()), int(cmd_joint_positions.size()));
      double elapsed_time = (ros::Time::now() - last_ik_time_).toSec();
      last_ik_time_ = ros::Time::now();
      double velocity;
      std::string joint;
      // Initialize the JointCommand msg. POSITION_MODE
      baxter_core_msgs::JointCommand cmd_msg;
      if (raw_mode_)
        cmd_msg.mode = cmd_msg.RAW_POSITION_MODE;
      else
        cmd_msg.mode = cmd_msg.POSITION_MODE;
      cmd_msg.names.resize(joint_names_.size());
      cmd_msg.command.resize(joint_names_.size());
      for(std::size_t i=0; i < joint_names_.size(); ++i)
      {
        // Limit the max velocity between iterations
        joint = joint_names_[i];
        if ( urdf_limits_.find(joint) == urdf_limits_.end() )
          continue;
        velocity = (cmd_joint_positions[i] - joint_positions[i])/elapsed_time;
        if (velocity > urdf_limits_[joint].max_velocity)
          cmd_joint_positions[i] = urdf_limits_[joint].max_velocity*elapsed_time + joint_positions[i];
        if (velocity < -urdf_limits_[joint].max_velocity)
          cmd_joint_positions[i] = -urdf_limits_[joint].max_velocity*elapsed_time + joint_positions[i];
        // Populate the command to each joint
        cmd_msg.names[i] = joint_names_[i];
        cmd_msg.command[i] = cmd_joint_positions[i];
        ROS_DEBUG("Joint %s: %f", cmd_msg.names[i].c_str(), cmd_msg.command[i]);
      }
      control_publisher_.publish(cmd_msg);
    }

    std::string getFolderName(const std::string& folder_key)
    {
      std::ostringstream folder_name;
      folder_name << getenv("HOME") << "/.openrave/robot." << folder_key << "/";
      return folder_name.str();
    }

    std::string getFilename(const std::string& database, const std::string& folder_key, const std::string& file_key)
    {
      std::ostringstream filename;
      filename << getFolderName(folder_key) << database << "." << file_key << ".pp";
      return filename.str();
    }
};

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_cart_controller");
  CartesianController cc;
  ros::spin();
  ros::shutdown();
  return 0;
}
