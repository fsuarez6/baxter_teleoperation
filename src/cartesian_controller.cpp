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
//~ #include "grips_msgs/GripsState.h"
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>

// Joint limits
#include <joint_limits_interface/joint_limits_urdf.h>

// Grips kinematic Interface
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
    KinematicsInterfacePtr                     ik_kinematics_;
    Eigen::Affine3d                           end_effector_pose_;
    std::map<std::string, joint_limits_interface::JointLimits> urdf_limits_;
    // Time
    ros::Time                                 last_state_print_;
    ros::Time                                 last_motion_print_;
    ros::Time                                 last_ik_time_;
    // Misc
    std::vector<std::string>                  joint_names_;
    std::string                               robot_namespace_;
    std::string                               model_frame_; 
    double                                   publish_rate_;
    double                                   position_error_;
    
  public:
    CartesianController(): 
      nh_private_("~")
    { 
      // Get parameters from the server
      nh_private_.param(std::string("publish_rate"), publish_rate_, 100.0);
      if (!nh_private_.hasParam("publish_rate"))
        ROS_WARN_STREAM("Parameter [~publish_rate] not found, using default: " << publish_rate_ << " Hz");      
      
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
      std::string topic_name;
      topic_name = "/robot/limb/left/joint_command";
      control_publisher_ = nh_.advertise<baxter_core_msgs::JointCommand>(topic_name.c_str(), 1);
      topic_name = "/robot/limb/left/endpoint_state";
      end_point_sub_ = nh_.subscribe(topic_name.c_str(), 1, &CartesianController::endPointStateCB, this);
      topic_name = "/baxter/ik_command";
      motion_control_sub_ = nh_.subscribe(topic_name.c_str(), 1, &CartesianController::ikCommandCB, this); 
      topic_name = "/baxter/pose";
      pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name.c_str(), 1);

      // Setup timer for publishing pose at 60 Hz
      pose_timer_ = nh_.createTimer(ros::Duration(1.0/60), &CartesianController::publishPose, this);

      last_motion_print_ = ros::Time::now();
    }
    
    ~CartesianController()
    {
    }
    
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
      // Get the latest robot state
      std::vector<double> current_joint_values;
      ik_kinematics_->getJointPositions(current_joint_values);
      std::ostringstream current_str;     
      current_str << "current_joint_values : [";
      for(std::size_t i=0; i < joint_names_.size(); ++i)
        current_str << current_joint_values[i] << " ";
      current_str << "]";
      // Here 1 is the number of random restart and 0.001s is the allowed time after each restart
      bool found_ik = ik_kinematics_->setEndEffectorPose(_msg->pose, 1, 0.001);
      // Get the new joint states for the arm
      std::vector<double> new_joint_values;
      if (found_ik)
        ik_kinematics_->getJointPositions(new_joint_values);
      else
      {
        return;
        //~ // Check that the IK command has changed enought
        //~ Eigen::Affine3d goal_pose, current_pose = ik_kinematics_->getEndEffectorTransform();
        //~ tf::poseMsgToEigen(_msg->pose, goal_pose);
        //~ double x_dist = goal_pose.translation().x() - current_pose.translation().x();
        //~ double y_dist = goal_pose.translation().y() - current_pose.translation().y();
        //~ double z_dist = goal_pose.translation().z() - current_pose.translation().z();
        //~ double xyz_dist = sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2));
        //~ 
        //~ if (xyz_dist < position_error_)
          //~ return;
        //~ 
        //~ ROS_DEBUG("Did not find IK solution");
        //~ // Determine nn closest XYZ points in the reachability database
        //~ int max_nn = 1000, nearest_neighbors;
        //~ flann::Matrix<float> query_pos(new float[3], 1, 3);
        //~ query_pos[0][0] = _msg->pose.position.x;
        //~ query_pos[0][1] = _msg->pose.position.y;
        //~ query_pos[0][2] = _msg->pose.position.z;
        //~ flann::Matrix<int> indices(new int[query_pos.rows*max_nn], query_pos.rows, max_nn);
        //~ flann::Matrix<float> dists(new float[query_pos.rows*max_nn], query_pos.rows, max_nn);
        //~ // do a knn search, using 128 checks
        //~ // this->index_pos->knnSearch(query_pos, indices, dists, nn, flann::SearchParams(128));
        //~ nearest_neighbors = indices.cols;
        //~ float radius = pow(position_error_, 2);
        //~ nearest_neighbors = position_index_->radiusSearch(query_pos, indices, dists, radius, flann::SearchParams(128));
        //~ // Check that we found something
        //~ if (nearest_neighbors <= 0) {
          //~ ROS_INFO_THROTTLE(60, "Didn't find any shit. Query: [%.3f, %.3f, %.3f]", _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z);
          //~ return; }
        //~ nearest_neighbors = fmin(nearest_neighbors, max_nn);
        //~ std::vector<float> score(nearest_neighbors), d_q(nearest_neighbors), 
                           //~ d_j(nearest_neighbors), d_xyz(nearest_neighbors);
        //~ Eigen::Quaterniond q_actual(end_effector_pose_.rotation());
        //~ Eigen::Quaterniond q;        
        //~ for (std::size_t i=0; i < nearest_neighbors; ++i)
        //~ {
          //~ // http://math.stackexchange.com/questions/90081/quaternion-distance
          //~ q.w() = metrics_db_["orientations"][i][0];
          //~ q.x() = metrics_db_["orientations"][i][1];
          //~ q.y() = metrics_db_["orientations"][i][2];
          //~ q.z() = metrics_db_["orientations"][i][3];
          //~ d_q[i] = 1 - pow(q.dot(q_actual), 2.0);
          //~ d_xyz[i] = sqrtf(dists[0][i]);
          //~ d_j[i] = 0;
          //~ for(std::size_t j=0; j < joint_names_.size(); ++j)
          //~ {
            //~ float j_error = fabs(current_joint_values[i] - metrics_db_["joint_states"][indices[0][i]][j]);
            //~ d_j[i] = fmax(d_j[i], j_error);
          //~ }
          //~ // score[i] = d_q[i] + d_xyz[i] + d_j[i];
          //~ score[i] = 0.5*d_q[i] + 0.5*d_j[i];
        //~ }
        //~ std::size_t choice_idx = std::min_element(score.begin(), score.end()) - score.begin();
        //~ ROS_DEBUG("nn [%d] choice [%d] score [%f] d_q [%f] d_xyz [%f] d_j [%f]", nearest_neighbors, 
                    //~ (int)choice_idx, score[choice_idx], d_q[choice_idx], d_xyz[choice_idx], d_j[choice_idx]);
        //~ // Populate the new joint_values
        //~ std::ostringstream new_str;
        //~ new_str << "new_joint_values : [";
        //~ for(std::size_t i=0; i < joint_names_.size(); ++i)
        //~ {
          //~ new_joint_values.push_back(metrics_db_["joint_states"][indices[0][choice_idx]][i]);
          //~ new_str << new_joint_values[i] << " ";            
        //~ }
        //~ new_str << "]";
        //~ ik_kinematics_->setJointPositions(new_joint_values);
        //~ // Debugging
        //~ if (ros::Time::now() - last_motion_print_  >= ros::Duration(1.0))
        //~ {
          //~ last_motion_print_ = ros::Time::now();
          //~ ROS_DEBUG_STREAM(current_str.str());
          //~ ROS_DEBUG_STREAM("Index [" << choice_idx << "] Distance [" << dists[0][choice_idx] << "]");
          //~ ROS_DEBUG_STREAM(new_str.str());
        //~ }
      }
      // Command the robot to the new joint_values
      ROS_DEBUG("Joint names: %d, Joint values: %d", int(joint_names_.size()), int(new_joint_values.size()));
      double elapsed_time = (ros::Time::now() - last_ik_time_).toSec();
      last_ik_time_ = ros::Time::now();
      double velocity;
      std::string joint;
      // Initialize the JointCommand msg. POSITION_MODE
      baxter_core_msgs::JointCommand cmd_msg;
      cmd_msg.mode = cmd_msg.POSITION_MODE;
      cmd_msg.names.resize(joint_names_.size());
      cmd_msg.command.resize(joint_names_.size());
      for(std::size_t i=0; i < joint_names_.size(); ++i)
      {
        // Check the max velocity
        joint = joint_names_[i];
        if ( urdf_limits_.find(joint) == urdf_limits_.end() )
          continue;
        
        velocity = (new_joint_values[i] - current_joint_values[i])/elapsed_time;
        if (velocity > urdf_limits_[joint].max_velocity)
          new_joint_values[i] = urdf_limits_[joint].max_velocity*elapsed_time + current_joint_values[i];
        if (velocity < -urdf_limits_[joint].max_velocity)
          new_joint_values[i] = -urdf_limits_[joint].max_velocity*elapsed_time + current_joint_values[i];
        // Populate the command to each joint
        cmd_msg.names[i] = joint_names_[i];
        cmd_msg.command[i] = new_joint_values[i];
        ROS_DEBUG("Joint %s: %f", joint_names_[i].c_str(), new_joint_values[i]);
      }
      control_publisher_.publish(cmd_msg);
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
