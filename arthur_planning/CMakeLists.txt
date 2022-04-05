cmake_minimum_required(VERSION 3.0.2)
project(arthur_planning)

find_package(catkin REQUIRED COMPONENTS
interactive_markers
moveit_core
moveit_ros_planning
moveit_ros_planning_interface
moveit_ros_perception
rviz_visual_tools
moveit_visual_tools
pluginlib
geometric_shapes
pcl_ros
pcl_conversions
rosbag
tf2_ros
tf2_eigen
tf2_geometry_msgs
message_generation
Eigen3 REQUIRED
std_msgs
geometry_msgs
trajectory_msgs
roscpp
rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(include  ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) 
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_message_files(
  FILES
  arthur_traj.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs geometry_msgs trajectory_msgs
  LIBRARIES ${PROJECT_NAME}    # Added this line
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


add_executable(trajectory_test src/trajectory_test.cpp)
target_link_libraries(trajectory_test ${catkin_LIBRARIES})
add_dependencies(trajectory_test arthur_planning_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(traj_test src/traj_test.cpp)
target_link_libraries(traj_test ${catkin_LIBRARIES})


