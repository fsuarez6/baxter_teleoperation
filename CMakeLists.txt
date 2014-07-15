cmake_minimum_required(VERSION 2.8.3)
project(baxter_teleop)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  baxter_interface
  baxter_tools
  moveit_kinematics_interface
  rate_position_controller
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  baxter_core_msgs
  baxter_description
  baxter_interface
  baxter_moveit_config
  baxter_tools
  moveit_kinematics_interface
  rate_position_controller
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(cartesian_controller src/cartesian_controller.cpp)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(cartesian_controller baxter_core_msgs_gencpp)
