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
kdl_parser
urdf
)

find_package(orocos_kdl REQUIRED
urdf_model REQUIRED
Eigen 3 3.3 REQUIRED
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS priority_control/include ${Eigen3_INCLUDE_DIRS}
  # LIBRARIES ${PROJECT_NAME}   # Added this line
  CATKIN_DEPENDS arthur_planning message_runtime roscpp std_msgs geometry_msgs tf kdl_parser urdf eigen
  DEPENDS orocos_kdl urdf_model
)

include_directories( 
  priority_control/include
  ${catkin_INCLUDE_DIRS}
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

add_executable(real_dummy_pelvis_broadcaster priority_control/real_dummy_pelvis_broadcaster.cpp)
add_dependencies(real_dummy_pelvis_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_dummy_pelvis_broadcaster ${catkin_LIBRARIES})

add_executable(tf_listener priority_control/tf_listener.cpp)
add_dependencies(tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(wrench_controller wrench_control/wrench_controller.cpp)
add_dependencies(wrench_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(wrench_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(wrench_controller arthur_planning_generate_messages_cpp)
target_link_libraries(wrench_controller ${catkin_LIBRARIES})

add_executable(master_controller priority_control/master_controller.cpp
                                  priority_control/include/arthur_robot_model.hpp
                                  priority_control/arthur_robot_model.cpp
                                  priority_control/include/task.hpp
                                  priority_control/task.cpp
                                  priority_control/include/priority_controller.hpp
                                  priority_control/priority_controller.cpp
                                  priority_control/include/pelvis_alignment_task.hpp
                                  priority_control/pelvis_alignment_task.cpp
                                  priority_control/include/camera_alignment_task.hpp
                                  priority_control/camera_alignment_task.cpp
                                  )
add_dependencies(master_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(master_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(master_controller ${catkin_LIBRARIES})