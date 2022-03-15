search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=/7dof/gen3.srdf.xacro
robot_name_in_srdf=gen3
moveit_config_pkg=gen3_moveit_config
robot_name=gen3
planning_group_name=arm
ikfast_plugin_pkg=gen3_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=end_effector_link
ikfast_output_path=/home/oem/arthur_ws/src/ros_kortex/kortex_description/robots/gen3_arm_ikfast_plugin/src/gen3_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
