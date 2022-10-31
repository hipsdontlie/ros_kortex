cmake_minimum_required(VERSION 3.0.2)

project(arthur_watchdog)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
geometry_msgs
tf
std_msgs
roslaunch
moveit_msgs
trajectory_msgs
urdf
message_generation
)

include_directories(include  ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) 
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_message_files(
  FILES
  inputs.msg
  perception.msg
  controls.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime rospy roscpp std_msgs geometry_msgs trajectory_msgs
  # LIBRARIES ${PROJECT_NAME}    # Added this line
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
#   DEPENDS
#     EIGEN3
)

include_directories( 
  arthur_watchdog/include
  ${catkin_INCLUDE_DIRS}
) 

add_executable(watchdog_main include/inputs.hpp
                            include/perception.hpp
                            # include/controls.hpp
                            main.cpp
                            inputs.cpp
                            perception.cpp
                            # controls.cpp
                            )
# add_dependencies(inputs ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(watchdog_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(watchdog_main ${catkin_LIBRARIES})