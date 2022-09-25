#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
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

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
double maxLinearVelocity = 0.01;
double maxAngularVelocity = 1.57;

bool sendJointSpeeds(ros::ServiceClient joint_speed_commander, Eigen::VectorXd q_vel, std::shared_ptr<ArthurRobotModel> robot)
{
    kortex_driver::SendJointSpeedsCommandRequest joint_speed_command;
    for (size_t i = 0; i < robot->nj(); ++i)
    {
        kortex_driver::JointSpeed joint_speed;
        joint_speed.duration = 0;
        joint_speed.joint_identifier = i;
        joint_speed.value = 180.0*q_vel(i)/M_PI;
        joint_speed_command.input.joint_speeds.push_back(joint_speed);
    }

    kortex_driver::SendJointSpeedsCommand srv;
    srv.request = joint_speed_command;
    return joint_speed_commander.call(srv);
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg, KDL::JntArray* q_pos, std::shared_ptr<ArthurRobotModel> robot) {
    for (size_t i = 0 ; i < robot->nj(); ++i)
    {
        q_pos->data(i) = msg->position[i];
        // q_vel->data(i) = msg->velocity[i];
    }
}

void errorCallback(const geometry_msgs::Transform::ConstPtr &msg, geometry_msgs::Transform* error) {
    (*error) = (*msg);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr &msg, Eigen::VectorXd* twist) {
    (*twist)(0) = (*msg).linear.x;
    (*twist)(1) = (*msg).linear.y;
    (*twist)(2) = (*msg).linear.z;
    (*twist)(3) = (*msg).angular.x;
    (*twist)(4) = (*msg).angular.y;
    (*twist)(5) = (*msg).angular.z;
}

Eigen::VectorXd pidController(geometry_msgs::Transform& error, const Eigen::VectorXd& twist)
{
    std::array<double, 6> Kp = {1, 1, 1, 1, 1, 1};
    std::array<double, 6> Ki = {0, 0, 0, 0, 0, 0};
    std::array<double, 6> Kd = {0, 0, 0, 0, 0, 0};
    Eigen::VectorXd twist_command = Eigen::VectorXd::Zero(6);
    twist_command(0) = error.translation.x;
    twist_command(1) = error.translation.y;
    twist_command(2) = error.translation.z;
    tf::Matrix3x3(tf::Quaternion(error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w)).getRPY(twist_command(3), twist_command(4), twist_command(5));

    for (int i = 0; i < 6; ++i)
    {
        twist_command(i) = Kp[i] * twist_command(i) - Kd[i]*twist(i);
    }
    
    double linearVel = sqrt(twist_command(0)*twist_command(0) + twist_command(1)*twist_command(1) + twist_command(2)*twist_command(2));
    double angularVel = sqrt(twist_command(3)*twist_command(3) + twist_command(4)*twist_command(4) + twist_command(5)*twist_command(5));
    if (linearVel > maxLinearVelocity)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            twist_command(i) = maxLinearVelocity* twist_command(i) / linearVel;
        }
    }
    if (angularVel > maxAngularVelocity)
    {
        for (size_t i = 3; i < 6; ++i)
        {
            twist_command(i) = maxAngularVelocity* twist_command(i) / angularVel;
        }
    }
    std::cout << "*****************" << std::endl;
    std::cout << twist_command << std::endl;
    return twist_command;
}

double jointLimitDampingFunction(double x)
{
    return 0.5 - 0.5*tanh((1 / (1-x)) - (1/x));
}

void computeJointLimitAvoidance(Eigen::MatrixXd& Wq, Eigen::VectorXd& F, const KDL::JntArray& q_pos, std::shared_ptr<ArthurRobotModel> robot)
{
    for (size_t i = 0; i < robot->nj(); ++i)
    {
        if (q_pos(i) >= robot->lower_joint_limit()[i] && q_pos(i) < robot->lower_damping_threshold()[i])
        {
            Wq(i,i) = jointLimitDampingFunction((robot->lower_damping_threshold()[i] - q_pos(i)) / (robot->lower_damping_threshold()[i] - robot->lower_joint_limit()[i]));
            F(i) = robot->joint_limit_force_max()[i] * (robot->lower_damping_threshold()[i] - q_pos(i)) / (robot->lower_damping_threshold()[i] - robot->lower_joint_limit()[i]);
        }
        else if (q_pos(i) > robot->upper_damping_threshold()[i] && q_pos(i) <= robot->upper_joint_limit()[i])
        {
            Wq(i,i) = jointLimitDampingFunction((q_pos(i) - robot->upper_damping_threshold()[i]) / (robot->upper_joint_limit()[i] - robot->upper_damping_threshold()[i]));
            F(i) = robot->joint_limit_force_max()[i] * (robot->upper_damping_threshold()[i] - q_pos(i)) / (robot->upper_joint_limit()[i] - robot->upper_damping_threshold()[i]);
        }
        else if (q_pos(i) >= robot->lower_damping_threshold()[i] && q_pos(i) <= robot->upper_damping_threshold()[i])
        {
            Wq(i,i) = 1;
            F(i) = 0;
        }
        else
        {
            Wq(i,i) = 0;
            F(i) = 0;
        }
    }
}

bool validJointVel(Eigen::VectorXd q_vel, KDL::JntArray q_pos, std::shared_ptr<ArthurRobotModel> robot, double dt)
{
    KDL::JntArray q_pos_next = KDL::JntArray(robot->nj());
    for (size_t i = 0; i < robot->nj(); ++i)
    {
        q_pos_next(i) = q_pos(i) + q_vel(i)*dt;
    }
    
    for (size_t i = 0; i < robot->nj(); ++i)
    {
        if (abs(q_vel(i)) > robot->joint_vel_limit()[i])
        {
            std::cout << abs(q_vel(i)) << std::endl;
            std::cout << robot->joint_vel_limit()[i] << std::endl;
            ROS_WARN("At Hard Joint Velocity Limit for Joint: %ld", i+1);
            return false;
        }
        if (q_pos_next(i) > robot->upper_joint_limit()[i] || q_pos_next(i) < robot->lower_joint_limit()[i])
        {
            ROS_WARN("At Hard Joint Position Limit for Joint: %ld", i+1);
            return false;
        }
    }

    KDL::Jacobian jac_next = KDL::Jacobian(robot->nj());
    if (robot->jac_solver_->JntToJac(q_pos_next, jac_next) < 0)
    {
        ROS_ERROR("Could not compute jacobian!");
        // TODO: Stop Controller or Send Message to Watchdog
        return false;
    }

    robot->svd_fast_solver_->compute(jac_next.data);
    int basic_rank = robot->svd_fast_solver_->rank();
    Eigen::MatrixXd Wq_temp = Eigen::MatrixXd::Identity(robot->nj(), robot->nj());
    Eigen::VectorXd F_temp = Eigen::VectorXd::Zero(robot->nj());
    computeJointLimitAvoidance(Wq_temp, F_temp, q_pos_next, robot);
    robot->svd_fast_solver_->compute(jac_next.data*Wq_temp);
    int joint_lim_rank = robot->svd_fast_solver_->rank();
    if (basic_rank >= 6 && joint_lim_rank < 6)
    {
        return false;
    }
    return true;
}

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    signal(SIGINT, mySigIntHandler);
    tf::TransformListener listener;

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    double controller_rate_ = 40.0; // Rate of ros node (hz)
    double controller_time_step_ = 1.0 / controller_rate_;
    ros::Rate rate(controller_rate_);

    std::string robot_description_;
    node.getParam("/master_controller/robot_description", robot_description_);
    std::string base_frame_;
    node.getParam("/master_controller/base_frame", base_frame_);
    std::string tip_frame_;
    node.getParam("/master_controller/tip_frame", tip_frame_);
    std::string target_frame_;
    node.getParam("/master_controller/target_frame", target_frame_);

    std::array<bool, 6> task_dof_= {true, true, true, true, true, true};
    
    std::shared_ptr<ArthurRobotModel> robot_ = std::make_shared<ArthurRobotModel>(robot_description_, base_frame_, tip_frame_);
    std::shared_ptr<Task> task_ = std::make_shared<Task>(robot_, task_dof_, tip_frame_);

    KDL::JntArray q_pos_ = KDL::JntArray(robot_->nj());
    Eigen::VectorXd q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
    Eigen::VectorXd twist_ = Eigen::VectorXd::Zero(ArthurRobotModel::CARTESIAN_DOF);

    geometry_msgs::Transform error_;
    error_.translation.x = 0;
    error_.translation.y = 0;
    error_.translation.z = 0;
    error_.rotation.x = 0;
    error_.rotation.y = 0;
    error_.rotation.z = 0;
    error_.rotation.w = 1;
    
    Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
    Eigen::VectorXd Joint_Limit_Force = Eigen::VectorXd::Zero(robot_->nj());

    // Subs/Pubs
    ros::Subscriber joint_state_sub_ = node.subscribe<sensor_msgs::JointState>("/my_gen3/joint_states", 1, boost::bind(&jointStateCallback, _1, &q_pos_, robot_));
    ros::Subscriber tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/tool_frame_to_dummy_pelvis", 1, boost::bind(&errorCallback, _1, &error_));
    ros::Subscriber tracking_twist_sub_ = node.subscribe<geometry_msgs::Twist>("/tf/twist/tool_frame_to_dummy_pelvis", 1, boost::bind(&twistCallback, _1, &twist_));
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

    listener.waitForTransform(tip_frame_, target_frame_,
                              ros::Time::now(), ros::Duration(100.0));

    ros::spinOnce();
    while (!g_request_shutdown)
    {
        Eigen::VectorXd desired_twist = pidController(error_, twist_);
        if (std::isnan(desired_twist(0)) || std::isnan(desired_twist(1)) || std::isnan(desired_twist(2)) || std::isnan(desired_twist(3)) || std::isnan(desired_twist(4)) || std::isnan(desired_twist(5)))
        {
            std::cout << "Desired twist is nan!" << std::endl;
        }
        else
        {
            task_->update_task(q_pos_, desired_twist);
            computeJointLimitAvoidance(Wq, Joint_Limit_Force, q_pos_, robot_);
            task_->compute_kinematic_matrices(Wq);
            q_vel_command_ = task_->pseudoinverse_jacobian() * task_->task_twist() +
                ((robot_->identity_matrix() - task_->pseudoinverse_jacobian()*task_->task_jacobian()) *
                (robot_->identity_matrix() - Wq) * Joint_Limit_Force);
            if (!validJointVel(q_vel_command_, q_pos_, robot_, controller_time_step_))
            {
                q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
            }
            // std::cout << "-----------------------" << std::endl;
            // std::cout << q_vel_command_ << std::endl;
            // TODO: Check if velocity and positions are valid
            sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
        }
        ros::spinOnce();
        rate.sleep();
    }

    q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
    sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
    std::cout << "Shutting controller down!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    ros::shutdown();
    return 0;
};