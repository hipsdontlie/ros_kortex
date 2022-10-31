#include "include/controls.hpp"


//controls alignment error
void Controls::error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg)
{
  // value = error_msg->data.front();
  if (error_msg->data.front() > 1.0)
  {
    ROS_INFO("Alignment error too high!\n");
  }
  else
  { 
    // std::cout<<error_msg->data<<std::endl;
    ROS_INFO("Alignment error within threshold! Continue...\n");
  }
}

void Controls::singularity_check(const std_msgs::Float64::ConstPtr& singularity_msg)
{
  if (singularity_msg->data > 0.9)
  {
    ROS_INFO("Very close to singularity! Please realign!\n");
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Singularity within limits...\n");
  }
}

void Controls::joint_limits(const std_msgs::Bool::ConstPtr& jlimits_msg)
{
  if (jlimits_msg->data == true)
  {
    ROS_INFO("Very close to joint limits! Please realign!\n");
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Joints within limits...\n");
  }
}

