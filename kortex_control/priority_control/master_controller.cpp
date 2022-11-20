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

    
    std::shared_ptr<ArthurRobotModel> robot_ = std::make_shared<ArthurRobotModel>(robot_description_, base_frame_, tip_frame_, planning_scene_);
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

    bool in_fault_ = false;

    std_msgs::Float64 singularity_measure_;
    singularity_measure_.data = 1.0;
    bool controller_enabled_ = false;
    std_msgs::Bool hit_joint_lim_;
    std_msgs::Bool fault_;
    hit_joint_lim_.data = false;
    std_msgs::Bool start_reaming_;
    start_reaming_.data = false;
    std_msgs::Bool start_dynamic_comp_;
    start_dynamic_comp_.data = false;

    bool start_reaming_button_ = false;


    // Subs/Pubs
    ros::Publisher pelvis_error_metrics_pub_ = node.advertise<std_msgs::Float64MultiArray>("/controls_error", 1);
    ros::Publisher camera_error_metrics_pub_ = node.advertise<std_msgs::Float64MultiArray>("/error_metrics/camera", 1);
    ros::Publisher singularity_measure_pub_ = node.advertise<std_msgs::Float64>("/controls_singularity", 1);
    ros::Publisher joint_limit_pub_ = node.advertise<std_msgs::Bool>("/controls_jlimits", 1);
    ros::Publisher controller_fault_ = node.advertise<std_msgs::Bool>("/controller_fault", 1);
    ros::Publisher start_reaming_pub_ = node.advertise<std_msgs::Bool>("/start_reaming/command", 1);
    ros::Publisher start_dynamic_comp_pub_ = node.advertise<std_msgs::Bool>("/start_dynamic_compensation/command", 1);
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

        node.getParam("ui_start_reaming", start_reaming_button_);
    
        bool successful_update = true;
        
        if(!pelvis_task_->updateError(q_pos_, pelvis_error_, pelvis_twist_, pelvis_error_metrics_pub_))
        {
            successful_update = false;
            ROS_WARN("Pelvis error could not be updated!!!");
        }
        if (pelvis_task_->errorAboveThreshold())
        {
            start_reaming_.data = false & start_reaming_button_;
            start_dynamic_comp_.data = true & start_reaming_button_;
        }
        else
        {
            start_reaming_.data = true & start_reaming_button_;
            start_dynamic_comp_.data = false & start_reaming_button_;
        }
        if(!camera_task_->updateError(q_pos_, camera_error_, camera_twist_, camera_error_metrics_pub_))
        {
            successful_update = false;
            ROS_WARN("Camera error could not be updated!!!");
        }
        if (!priority_controller_->computeJointVelocityCommand(q_pos_))
        {
            successful_update = false;
            ROS_WARN("Could not compute next velocity command!!!");
        }

        start_reaming_pub_.publish(start_reaming_);
        start_dynamic_comp_pub_.publish(start_dynamic_comp_);

        singularity_measure_.data = 1.0 / priority_controller_->manipulability(q_pos_);
        singularity_measure_pub_.publish(singularity_measure_);
        hit_joint_lim_.data = priority_controller_->hit_joint_limit();
        joint_limit_pub_.publish(hit_joint_lim_);
        fault_.data = !successful_update;
        controller_fault_.publish(fault_);

        if (controller_enabled_ && successful_update && !in_fault_)
        {
            q_vel_command_ = priority_controller_->getJointVelocityCommand();
            sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
        }
        else if (!in_fault_)
        {
            q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
            in_fault_ = sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_);
            ROS_WARN("Attempting to stop controller because of fault!");
            // ROS_WARN("Controller in fault!");
            // in_fault_ = true;
        }
        else if (controller_enabled_ && successful_update && in_fault_)
        {
            in_fault_ = false;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    q_vel_command_ = Eigen::VectorXd::Zero(robot_->nj());
    while (!sendJointSpeeds(joint_speed_commander_, q_vel_command_, robot_))
    {
        // std::cout << "Attempting to shut controller down!" << std::endl;
    }
    std::cout << "Shutting controller down!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    ros::shutdown();
    return 0;
};