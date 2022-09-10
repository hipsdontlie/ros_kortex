#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <arthur_planning/arthur_traj.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <kortex_driver/SendWrenchCommand.h>
#include <boost/bind.hpp>
#include <vector>
#include <array>

#include "arthur_robot_model.hpp"

using namespace priority_control;

bool compute_jacobian(std::shared_ptr<ArthurRobotModel> robot, KDL::JntArray q_pos, KDL::Jacobian jacobian)
{
    if (robot->jac_solver_->JntToJac(q_pos, jacobian) < 0)
    {
        ROS_ERROR("Could not compute jacobian!");
        // TODO: Stop Controller or Send Message to Watchdog
        return false;
    }
    // TODO: Rotate jacobian?
    // TODO: Crop Jacobian?
    // TODO: Multiple Jacobian by weighting matrices?
    return true;
}

bool compute_pseudoinverse_jacobian(std::shared_ptr<ArthurRobotModel> robot, KDL::Jacobian unweighted_jacobian)
{

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
    
    std::shared_ptr<ArthurRobotModel> robot_ = std::make_shared<ArthurRobotModel>(robot_description_, base_frame_, tip_frame_);

    KDL::Jacobian jacobian_ = KDL::Jacobian(robot_->nj());
    KDL::JntArray q_pos_ = KDL::JntArray(robot_->nj());
    Eigen::VectorXd q_vel_ = Eigen::VectorXd::Zero(robot_->nj());
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
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};