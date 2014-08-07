cmake_minimum_required(VERSION 2.8.3)
project(baxter_benchmark)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  baxter_interface
  baxter_tools
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  baxter_core_msgs
  baxter_interface
  baxter_tools
)


include_directories(${catkin_INCLUDE_DIRS})
