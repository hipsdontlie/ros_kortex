#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>

using namespace std::chrono;

const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpp_test");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  std::string default_planning_pipeline;
    node_handle.getParam("move_group/default_planning_pipeline", default_planning_pipeline);
  ROS_INFO("Current planning pipeline: %s", default_planning_pipeline.c_str());

  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  // visual_tools.deleteAllMarkers();

  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.0;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // const moveit::core::JointModelGroup* joint_model_group =
  //     move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  ROS_INFO_NAMED("Arthur's Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  std::vector<double> joint_values;
  current_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("end_effector_link");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  geometry_msgs::Pose target_msg;
  tf::poseEigenToMsg(end_effector_state, target_msg);
  ROS_INFO_STREAM("Pose position: (" << target_msg.position.x << " " << target_msg.position.y << " " << target_msg.position.z << ")" << std::endl);
  // geometry_msgs::Pose target_pose1;
  // // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = target_msg.position.x - 0.2;
  // target_pose1.position.y = target_msg.position.y;
  // target_pose1.position.z = target_msg.position.z;
  // move_group_interface.setPoseTarget(target_pose1);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_msg);

  geometry_msgs::Pose target_pose3 = target_msg;

  target_pose3.position.x -= 0.1;

  // target_pose3.position.x = 0.409954;
  // target_pose3.position.y = 0.199208;
  // target_pose3.position.z = 0.292865;
  waypoints.push_back(target_pose3);  // down

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  auto start = high_resolution_clock::now();

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
 
  std::cout << "Time taken by function: "
        << duration.count() << " microseconds" << std::endl;

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();

  ros::shutdown();
  return 0;

}



