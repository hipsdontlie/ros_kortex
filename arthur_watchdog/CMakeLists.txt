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
)

# find_package(orocos_kdl REQUIRED
# urdf_model REQUIRED
# Eigen 3 3.3 REQUIRED
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

# catkin_package(
#   INCLUDE_DIRS arthur_watchdog/include ${Eigen3_INCLUDE_DIRS}
#   # LIBRARIES ${PROJECT_NAME}   # Added this line
# #   CATKIN_DEPENDS arthur_planning message_runtime roscpp std_msgs geometry_msgs tf kdl_parser urdf eigen
# #   DEPENDS orocos_kdl urdf_model
# )

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

# catkin_install_python(PROGRAMS wrench_control/scripts/ftsensor_listener.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install
# foreach(dir arms grippers)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

# add_executable(ee_tf_listener wrench_control/ee_tf_listener.cpp)
# add_dependencies(ee_tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ee_tf_listener ${catkin_LIBRARIES})

# add_executable(real_dummy_pelvis_broadcaster priority_control/real_dummy_pelvis_broadcaster.cpp)
# add_dependencies(real_dummy_pelvis_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(real_dummy_pelvis_broadcaster ${catkin_LIBRARIES})

# add_executable(tf_listener priority_control/tf_listener.cpp)
# add_dependencies(tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(tf_listener ${catkin_LIBRARIES})

# add_executable(wrench_controller wrench_control/wrench_controller.cpp)
# add_dependencies(wrench_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(wrench_controller ${catkin_EXPORTED_TARGETS})
# add_dependencies(wrench_controller arthur_planning_generate_messages_cpp)
# target_link_libraries(wrench_controller ${catkin_LIBRARIES})

add_executable(inputs include/inputs.hpp
                        inputs.cpp)
# add_dependencies(inputs ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(inputs ${catkin_EXPORTED_TARGETS})
target_link_libraries(inputs ${catkin_LIBRARIES})