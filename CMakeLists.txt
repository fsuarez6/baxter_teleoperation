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
  collada_urdf
  image_view
  moveit_ikfast
  moveit_kinematics_interface
  rate_position_controller
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(HDF5 REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(build_metrics_index src/build_metrics_index.cpp)
target_link_libraries(build_metrics_index ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HDF5_LIBRARIES})
add_dependencies(build_metrics_index grips_msgs_gencpp)

add_executable(cartesian_controller src/cartesian_controller.cpp)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(cartesian_controller baxter_core_msgs_gencpp)
