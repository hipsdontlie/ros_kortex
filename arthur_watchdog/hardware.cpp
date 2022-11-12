#include "include/hardware.hpp"


//controls alignment error
void Hardware::reamer_speed(const std_msgs::Float64::ConstPtr& reamerSpeed_msg)
{
  prevTime_reamerSpeed = ros::Time::now().toSec();
  if (reamerSpeed_msg->data > 500.0)
  {
    ROS_INFO("Reamer speed too high!\n");
    hardware_flag = false;
  }
  else
  { 
    // std::cout<<error_msg->data<<std::endl;
    ROS_INFO("Reamer speed within limits! Continue...\n");
    hardware_flag = true;
  }
}

void Hardware::load_applied(const std_msgs::Float64::ConstPtr& loadApplied_msg)
{
  prevTime_loadApplied = ros::Time::now().toSec();
  if (loadApplied_msg->data > 20.0)
  {
    ROS_INFO("Load very high! Please retract\n");
    hardware_flag = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Load within limit...\n");
    hardware_flag = true;
  }
}

void Hardware::ream_percent(const std_msgs::Float64::ConstPtr& reamPercent_msg)
{
  prevTime_reamPercent = ros::Time::now().toSec();
  if (reamPercent_msg->data > 100.0)
  {
    ROS_INFO("Goal point reached. Reaming percentage above 100. Stopping..\n");
    hardware_flag = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Reaming percentage below 100. Reaming...\n");
    hardware_flag = true;
  }
}

void Hardware::current_drawn(const std_msgs::Float64::ConstPtr& currentDrawn_msg)
{
  prevTime_currentDrawn = ros::Time::now().toSec();
  if (currentDrawn_msg->data > 20.0)
  {
    ROS_INFO("Current drawn very high! Please retract\n");
    hardware_flag = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Current drawn within limit...\n");
    hardware_flag = true;
  }
}


