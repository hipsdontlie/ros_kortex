#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Vector3.h>
// #include <arthur_planning/arthur_traj.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Float64.h>
#include <kortex_driver/SendJointSpeedsCommand.h>
#include <sensor_msgs/JointState.h>
#include <boost/bind.hpp>
#include <vector>
#include <array>

#include "arthur_robot_model.hpp"
#include "task.hpp"

using namespace priority_control;

bool sendJointSpeeds(ros::ServiceClient joint_speed_commander, Eigen::VectorXd q_vel, std::shared_ptr<ArthurRobotModel> robot)
{
    kortex_driver::SendJointSpeedsCommandRequest joint_speed_command;
    for (size_t i = 0; i < robot->nj(); ++i)
    {
        kortex_driver::JointSpeed joint_speed;
        joint_speed.duration = 0;
        joint_speed.joint_identifier = i;
        joint_speed.value = q_vel(i);
        joint_speed_command.input.joint_speeds.push_back(joint_speed);
    }

    kortex_driver::SendJointSpeedsCommand srv;
    srv.request = joint_speed_command;
    return joint_speed_commander.call(srv);
}

void jointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg, KDL::JntArray& q_pos, std::shared_ptr<ArthurRobotModel> robot) {
    for (size_t i = 0 ; i < robot->nj(); ++i)
    {
        q_pos(i) = msg->position[i];
    }
}

void errorCallback(const geometry_msgs::Transform::ConstPtr &msg, geometry_msgs::Transform& error) {
    error = (*msg);
}

Eigen::VectorXd pid_controller(geometry_msgs::Transform& error)
{
    std::array<double, 6> Kp = {1, 1, 1, 1, 1, 1};
    std::array<double, 6> Ki = {0, 0, 0, 0, 0, 0};
    std::array<double, 6> Kd = {0, 0, 0, 0, 0, 0};
    Eigen::VectorXd twist = Eigen::VectorXd::Zero(6);
    twist(0) = error.translation.x;
    twist(1) = error.translation.y;
    twist(2) = error.translation.z;
    tf::Matrix3x3(tf::Quaternion(error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w)).inverse().getRPY(twist(3), twist(4), twist(5));
    for (int i = 0; i < 6; ++i)
    {
        twist(i) = Kp[i] * twist(i);
    }
    return twist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_controller");
    ros::NodeHandle node;
    double controller_rate_ = 40.0; // Rate of ros node (hz)
    double controller_time_step_ = 1.0 / controller_rate_;
    ros::Rate rate(controller_rate_);

    std::string robot_description_;
    node.getParam("/master_controller/robot_description", robot_description_);
    std::string base_frame_;
    node.getParam("/master_controller/base_frame", base_frame_);
    std::string tip_frame_;
    node.getParam("/master_controller/tip_frame", tip_frame_);

    std::array<bool, 6> task_dof_= {true, true, true, true, true, true};
    
    std::shared_ptr<ArthurRobotModel> robot_ = std::make_shared<ArthurRobotModel>(robot_description_, base_frame_, tip_frame_);
    std::shared_ptr<Task> task_ = std::make_shared<Task>(robot_, task_dof_, tip_frame_);

    KDL::JntArray q_pos_ = KDL::JntArray(robot_->nj());
    Eigen::VectorXd q_vel_ = Eigen::VectorXd::Zero(robot_->nj());

    geometry_msgs::Transform error_;

    // Subs/Pubs
    ros::Subscriber joint_state_sub_ = node.subscribe<sensor_msgs::JointState>("/my_gen3/joint_states", 1, boost::bind(&jointPositionCallback, _1, q_pos_, robot_));
    ros::Subscriber tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/tool_frame_to_dummy_pelvis", 1, boost::bind(&errorCallback, _1, error_));
    ros::ServiceClient joint_speed_commander_ = node.serviceClient<kortex_driver::SendJointSpeedsCommand>("/my_gen3/base/send_joint_speeds_command");

    // // sends wrench commands to kortex_driver service
    // ros::ServiceClient wrench_commander = node.serviceClient<kortex_driver::SendWrenchCommand>("/my_gen3/base/send_wrench_command");
    // // sends speeds to reamer
    // ros::Publisher reamer_commander = node.advertise<std_msgs::Int16>("/reamer_speed", 1);
    // // sub to get current reamer velocity
    // ros::Subscriber reamer_sub = node.subscribe<std_msgs::Float64>("/reamer_velocity", 1, boost::bind(&reamerVelCallback, _1, &reamerVel));
    // // sub to get ee pose wrt base_link
    // ros::Subscriber transform_sub = node.subscribe<geometry_msgs::Transform>("/my_gen3/ee_tf", 1, boost::bind(&tfCallback, _1, xyzrpy, &retract_xyzrpy, &currentRot, velocities, &time, &dynamicComp, &finished, &qForTrajEval));
    // // sub to get pelvis orientation wrt base_link
    // ros::Subscriber pelvis_transform_sub = node.subscribe<geometry_msgs::PoseStamped>("/reaming_end_point", 1, boost::bind(&pelvisTFCallback, _1, &pelvis_xyzrpy, &pelvisRot));
    // // sub to get trajectory information
    // ros::Subscriber trajectory_sub = node.subscribe<arthur_planning::arthur_traj>("/my_gen3/arthur_traj", 1, boost::bind(&trajCallback, _1, &traj, &current_waypoint, &trajNum, &desiredRots, wrench_commander, reamer_commander, &planned, &reamerVel));
    // // sub to see dynamic comp
    // ros::Subscriber dynamic_compensation_listener = node.subscribe<std_msgs::Bool>("/pelvis_error", 1, boost::bind(&dynamicCompTrigger, _1, &dynamicComp, &startPlanner, &finished, &planned, wrench_commander, reamer_commander, &reamerVel));
    // // sub to get forces at end-effector
    // ros::Subscriber force_listener = node.subscribe<geometry_msgs::Vector3>("/sensor_force", 1, boost::bind(&forceSensorCallback, _1, forces));
    // // pub to notify planner to start planning
    // ros::Publisher notify_planner = node.advertise<std_msgs::Bool>("/start_planning", 1);
    // // pub to send actual pose to trajectory evaluator
    // ros::Publisher traj_eval_pub = node.advertise<geometry_msgs::Pose>("/actual_traj", 1);

    ros::spinOnce();
    while (ros::ok())
    {
        task_->update_task(q_pos_, pid_controller(error_));
        task_->compute_kinematic_matrices(robot_->identity_matrix());
        q_vel_ = task_->pseudoinverse_jacobian() * task_->task_twist();
        // TODO: Check if velocity and positions are valid
        sendJointSpeeds(joint_speed_commander_, q_vel_, robot_);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};