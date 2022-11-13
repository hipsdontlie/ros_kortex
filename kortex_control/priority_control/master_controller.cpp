#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <kortex_driver/SendJointSpeedsCommand.h>
#include <sensor_msgs/JointState.h>
#include <boost/bind.hpp>
#include <vector>
#include <array>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "arthur_robot_model.hpp"
#include "task.hpp"
#include "priority_controller.hpp"
#include "pelvis_alignment_task.hpp"
#include "camera_alignment_task.hpp"

using namespace priority_control;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

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

void pelvisErrorCallback(const geometry_msgs::Transform::ConstPtr &msg, geometry_msgs::Transform* error) {
    (*error) = (*msg);
}

void pelvisTwistCallback(const geometry_msgs::Twist::ConstPtr &msg, Eigen::VectorXd* twist) {
    (*twist)(0) = (*msg).linear.x;
    (*twist)(1) = (*msg).linear.y;
    (*twist)(2) = (*msg).linear.z;
    (*twist)(3) = (*msg).angular.x;
    (*twist)(4) = (*msg).angular.y;
    (*twist)(5) = (*msg).angular.z;
}

void cameraErrorCallback(const geometry_msgs::Transform::ConstPtr &msg, geometry_msgs::Transform* error) {
    (*error) = (*msg);
}

void cameraTwistCallback(const geometry_msgs::Twist::ConstPtr &msg, Eigen::VectorXd* twist) {
    (*twist)(0) = (*msg).linear.x;
    (*twist)(1) = (*msg).linear.y;
    (*twist)(2) = (*msg).linear.z;
    (*twist)(3) = (*msg).angular.x;
    (*twist)(4) = (*msg).angular.y;
    (*twist)(5) = (*msg).angular.z;
}

void controllerFlagCallback(const std_msgs::Bool::ConstPtr &msg, bool* controller_flag)
{
    (*controller_flag) = msg->data;
}

void clearFaultCallback(const std_msgs::Empty::ConstPtr &msg, std::shared_ptr<PriorityController>* controller)
{
    (*controller)->reset_fault();
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
    std::string ee_marker_frame_;
    node.getParam("/master_controller/ee_marker_frame", ee_marker_frame_);

    robot_model_loader::RobotModelLoader robot_model_loader_("collision_robot_description");
    const moveit::core::RobotModelPtr& kinematic_model_ = robot_model_loader_.getModel();
    std::shared_ptr<planning_scene::PlanningScene>planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);

    // std::array<bool, 6> task_dof_= {true, true, true, true, true, true};
    
    std::shared_ptr<ArthurRobotModel> robot_ = std::make_shared<ArthurRobotModel>(robot_description_, base_frame_, tip_frame_, planning_scene_);
    // std::shared_ptr<Task> task_ = std::make_shared<Task>(robot_, task_dof_, tip_frame_);
    std::shared_ptr<PelvisAlignmentTask> pelvis_task_ = std::make_shared<PelvisAlignmentTask>(robot_, tip_frame_);
    std::shared_ptr<CameraAlignmentTask> camera_task_ = std::make_shared<CameraAlignmentTask>(robot_, ee_marker_frame_);
    std::shared_ptr<PriorityController> priority_controller_ = std::make_shared<PriorityController>(robot_, controller_time_step_);
    priority_controller_->addTask(pelvis_task_, 1);
    priority_controller_->addTask(camera_task_, 2);
    

    KDL::JntArray q_pos_ = KDL::JntArray(robot_->nj());
    Eigen::VectorXd q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
    Eigen::VectorXd pelvis_twist_ = Eigen::VectorXd::Zero(ArthurRobotModel::CARTESIAN_DOF);
    Eigen::VectorXd camera_twist_ = Eigen::VectorXd::Zero(ArthurRobotModel::CARTESIAN_DOF);

    geometry_msgs::Transform pelvis_error_;
    pelvis_error_.translation.x = 0;
    pelvis_error_.translation.y = 0;
    pelvis_error_.translation.z = 0;
    pelvis_error_.rotation.x = 0;
    pelvis_error_.rotation.y = 0;
    pelvis_error_.rotation.z = 0;
    pelvis_error_.rotation.w = 1;

    geometry_msgs::Transform camera_error_;
    camera_error_.translation.x = 0;
    camera_error_.translation.y = 0;
    camera_error_.translation.z = 0;
    camera_error_.rotation.x = 0;
    camera_error_.rotation.y = 0;
    camera_error_.rotation.z = 0;
    camera_error_.rotation.w = 1;

    std_msgs::Float64 singularity_measure_;
    singularity_measure_.data = 1.0;
    bool controller_enabled_ = false;
    std_msgs::Bool hit_joint_lim_;
    std_msgs::Bool fault_;
    hit_joint_lim_.data = false;
    
    // Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(robot_->nj(), robot_->nj());
    // Eigen::VectorXd Joint_Limit_Force = Eigen::VectorXd::Zero(robot_->nj());

    // Subs/Pubs
    ros::Publisher pelvis_error_metrics_pub_ = node.advertise<std_msgs::Float64MultiArray>("/controls_error", 1);
    ros::Publisher camera_error_metrics_pub_ = node.advertise<std_msgs::Float64MultiArray>("/error_metrics/camera", 1);
    ros::Publisher singularity_measure_pub_ = node.advertise<std_msgs::Float64>("/controls_singularity", 1);
    ros::Publisher joint_limit_pub_ = node.advertise<std_msgs::Bool>("/controls_jlimits", 1);
    ros::Publisher controller_fault_ = node.advertise<std_msgs::Bool>("/controller_fault", 1);
    ros::Subscriber clear_fault_sub_ = node.subscribe<std_msgs::Empty>("/controller_clear_fault", 1, boost::bind(&clearFaultCallback, _1, &priority_controller_));
    ros::Subscriber controller_flag_sub_ = node.subscribe<std_msgs::Bool>("/controller_flag", 1, boost::bind(&controllerFlagCallback, _1, &controller_enabled_));
    ros::Subscriber joint_state_sub_ = node.subscribe<sensor_msgs::JointState>("/my_gen3/joint_states", 1, boost::bind(&jointStateCallback, _1, &q_pos_, robot_));
    // ros::Subscriber pelvis_tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/tool_tip_frame_to_dummy_pelvis", 1, boost::bind(&pelvisErrorCallback, _1, &pelvis_error_));
    // ros::Subscriber pelvis_tracking_twist_sub_ = node.subscribe<geometry_msgs::Twist>("/tf/twist/tool_tip_frame_to_dummy_pelvis", 1, boost::bind(&pelvisTwistCallback, _1, &pelvis_twist_));
    ros::Subscriber pelvis_tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/tool_tip_frame_to_reaming_pose", 1, boost::bind(&pelvisErrorCallback, _1, &pelvis_error_));
    ros::Subscriber pelvis_tracking_twist_sub_ = node.subscribe<geometry_msgs::Twist>("/tf/twist/tool_tip_frame_to_reaming_pose", 1, boost::bind(&pelvisTwistCallback, _1, &pelvis_twist_));
    // ros::Subscriber camera_tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/ee_marker_frame_to_dummy_camera", 1, boost::bind(&cameraErrorCallback, _1, &camera_error_));
    // ros::Subscriber camera_tracking_twist_sub_ = node.subscribe<geometry_msgs::Twist>("/tf/twist/ee_marker_frame_to_dummy_camera", 1, boost::bind(&cameraTwistCallback, _1, &camera_twist_));
    ros::Subscriber camera_tracking_frame_sub_ = node.subscribe<geometry_msgs::Transform>("/tf/ee_marker_frame_to_camera", 1, boost::bind(&cameraErrorCallback, _1, &camera_error_));
    ros::Subscriber camera_tracking_twist_sub_ = node.subscribe<geometry_msgs::Twist>("/tf/twist/ee_marker_frame_to_camera", 1, boost::bind(&cameraTwistCallback, _1, &camera_twist_));
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
    // // sub to get forces at end-effectorstd_msgs::Float64MultiArrayforceSensorCallback, _1, forces));
    // // pub to notify planner to start planning
    // ros::Publisher notify_planner = node.advertise<std_msgs::Bool>("/start_planning", 1);
    // // pub to send actual pose to trajectory evaluator
    // ros::Publisher traj_eval_pub = node.advertise<geometry_msgs::Pose>("/actual_traj", 1);


    // listener.waitForTransform(tip_frame_, target_frame_,
    //                           ros::Time::now(), ros::Duration(1.0));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    ros::spinOnce();
    while (!g_request_shutdown)
    {
        try{
            transformStamped = tfBuffer.lookupTransform("camera", "reaming_pose",
                                ros::Time(0));
            transformStamped = tfBuffer.lookupTransform("camera", "ee_marker_frame",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        // Eigen::VectorXd desired_twist = pidController(pelvis_error_, pelvis_twist_, pelvis_error_metrics_pub_);
        // if (std::isnan(desired_twist(0)) || std::isnan(desired_twist(1)) || std::isnan(desired_twist(2)) || std::isnan(desired_twist(3)) || std::isnan(desired_twist(4)) || std::isnan(desired_twist(5)))
        // {
        //     std::cout << "Desired twist is nan!" << std::endl;
        // }
        // else
        // {
        //     task_->update_task(desired_twist);
        //     computeJointLimitAvoidance(Wq, Joint_Limit_Force, q_pos_, robot_);
        //     task_->compute_kinematic_matrices(q_pos_, Wq, robot_->identity_matrix());
        //     q_vel_command_ = task_->pseudoinverse_jacobian() * task_->task_twist() +
        //         ((robot_->identity_matrix() - task_->pseudoinverse_jacobian()*task_->task_jacobian()) *
        //         (robot_->identity_matrix() - Wq) * Joint_Limit_Force);
        //     if (!validJointVel(q_vel_command_, q_pos_, robot_, controller_time_step_))
        //     {
        //         q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
        //     }
        //     // std::cout << "-----------------------" << std::endl;
        //     // std::cout << q_vel_command_ << std::endl;
        //     // TODO: Check if velocity and positions are valid
        //     sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
        // }
        // std::cout << "Check 1" << std::endl;
        // std::cout << "+++++++++++++++++" << std::endl;
        // std::cout << robot_->segnr2name(robot_->name2segnr(ee_marker_frame_)) << std::endl;
        // std::cout << robot_->segnr2name(robot_->name2segnr(tip_frame_)) << std::endl;
        // ROS_WARN("Transform Found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        bool successful_update = true;
        
        if(!pelvis_task_->updateError(q_pos_, pelvis_error_, pelvis_twist_, pelvis_error_metrics_pub_))
        {
            successful_update = false;
            ROS_WARN("Pelvis error could not be updated!!!");
        }
        if(!camera_task_->updateError(q_pos_, camera_error_, camera_twist_, camera_error_metrics_pub_))
        {
            successful_update = false;
            ROS_WARN("Camera error could not be updated!!!");
        }
        // std::cout << "Check 2" << std::endl;
        if (!priority_controller_->computeJointVelocityCommand(q_pos_))
        {
            successful_update = false;
            ROS_WARN("Could not compute next velocity command!!!");
        }
        // std::cout << "Check 3" << std::endl;

        singularity_measure_.data = 1.0 / priority_controller_->manipulability(q_pos_);
        singularity_measure_pub_.publish(singularity_measure_);
        hit_joint_lim_.data = priority_controller_->hit_joint_limit();
        joint_limit_pub_.publish(hit_joint_lim_);

        if (controller_enabled_ && successful_update)
        {
            q_vel_command_ = priority_controller_->getJointVelocityCommand();
        }
        else
        {
            q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
        }

        if (!successful_update)
        {
            fault_.data = true;
            controller_fault_.publish(fault_);
        }
        else
        {
            fault_.data = false;
            controller_fault_.publish(fault_);
        }
        
        sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
        ros::spinOnce();
        rate.sleep();
    }

    q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
    sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
    std::cout << "Shutting controller down!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    ros::shutdown();
    return 0;
};