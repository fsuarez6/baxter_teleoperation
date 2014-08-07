cmake_minimum_required(VERSION 2.8.3)
project(baxter_openrave)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  baxter_interface
  baxter_tools
)

catkin_package(CATKIN_DEPENDS
  baxter_core_msgs
  baxter_interface
  baxter_tools
  collada_urdf
  moveit_ikfast
  urdfdom
)


include_directories(${catkin_INCLUDE_DIRS})
