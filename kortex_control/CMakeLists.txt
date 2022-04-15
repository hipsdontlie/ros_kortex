cmake_minimum_required(VERSION 3.0.2)

project(kortex_control)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
geometry_msgs
tf
std_msgs
roslaunch
moveit_msgs
trajectory_msgs
arthur_planning
)

include_directories(include ${catkin_INCLUDE_DIRS}) 

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}   # Added this line
  CATKIN_DEPENDS arthur_planning message_runtime roscpp std_msgs geometry_msgs tf 
  DEPENDS
)

catkin_install_python(PROGRAMS wrench_control/scripts/ftsensor_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install
foreach(dir arms grippers)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(ee_tf_listener wrench_control/ee_tf_listener.cpp)
add_dependencies(ee_tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ee_tf_listener ${catkin_LIBRARIES})

add_executable(wrench_controller wrench_control/wrench_controller.cpp)
add_dependencies(wrench_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(wrench_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(wrench_controller arthur_planning_generate_messages_cpp)
target_link_libraries(wrench_controller ${catkin_LIBRARIES})