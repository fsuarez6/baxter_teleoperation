#include <ros/ros.h>

// Flann
#include <float.h>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

using namespace flann;

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

int main(int argc, char** argv)
{
  // Start the node
  ros::init (argc, argv, "build_metrics_index");
  ros::NodeHandle nh_private ("~");
  // Get parameters from the server
  std::string database_name, folder_key, file_key;
  
  nh_private.param(std::string("metrics_database"), database_name, std::string("ik_metrics"));
  if (!nh_private.hasParam("metrics_database"))
    ROS_WARN_STREAM("Parameter [~metrics_database] not found, using default: " << database_name);
    
  nh_private.param(std::string("folder_key"), folder_key, std::string("6575ecddff8e44245a6d08f7e6a232f6"));
  if (!nh_private.hasParam("folder_key"))
    ROS_WARN_STREAM("Parameter [~folder_key] not found, using default: " << folder_key);
    
  nh_private.param(std::string("file_key"), file_key, std::string("dc12ea999aabf684e4dab37176899cd2"));
  if (!nh_private.hasParam("file_key"))
    ROS_WARN_STREAM("Parameter [~file_key] not found, using default: " << file_key);

  std::string filename;
  filename = getFilename(database_name, folder_key, file_key);
  ROS_INFO_STREAM("Loading [metrics database] from:\n" << filename);
  ros::Time flann_start_time = ros::Time::now();
  Matrix<float> positions_mat;
  try {
    load_from_file(positions_mat, filename, "positions");
  }
  catch (...) {
    ROS_ERROR_STREAM("Failed loading [metrics database] from:\n" << filename);
    ros::shutdown();
    return -1;
  }
  // construct an randomized kd-tree index using 4 kd-trees
  Index<L2<float> > index(positions_mat, flann::KDTreeIndexParams(4));
  index.buildIndex();
  double elapsed_time = (ros::Time::now() - flann_start_time).toSec();
  ROS_INFO("[metrics database] successfully loaded in %.2f seconds", elapsed_time);
  
  // example of a query
  int nn = 100; 
  flann::Matrix<float> query(new float[3], 1, 3);
  query[0][0] = 0.075;
  query[0][1] = 0.4;
  query[0][2] = 0.6;
  Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
  Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
  // do a knn search, using 128 checks
  index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
  ROS_INFO("Example query: [%.3f, %.1f, %.1f] found [%d] nearest_neighbors", query[0][0], query[0][1], query[0][2], (int)indices.cols);
  
  // save the index
  std::ostringstream index_file;
  index_file << getFolderName(folder_key) << "ik_metrics_index." << file_key << ".dat";
  index.save(index_file.str());
  ROS_INFO_STREAM("Saved [metrics index] to:\n" << index_file.str());

  delete[] positions_mat.ptr();
  delete[] query.ptr();
  delete[] indices.ptr();
  delete[] dists.ptr();
  
  return 0;
}
