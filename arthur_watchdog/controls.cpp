#include "include/controls.hpp"


//controls alignment error
void Controls::error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg)
{
  prevTime_error = ros::Time::now().toSec();
  double trans_error = error_msg->data[0];
  double orientation_error = error_msg->data[1];
  if (trans_error > 30.0)
  {
    ROS_INFO("Translation error is too high!\n");
  }
  else
  { 
    // std::cout<<error_msg->data<<std::endl;
    ROS_INFO("Translation error within threshold! Continue...\n");
  }
}

void Controls::singularity_check(const std_msgs::Float64::ConstPtr& singularity_msg)
{
  prevTime_singularity = ros::Time::now().toSec();
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
  prevTime_jlimits = ros::Time::now().toSec();
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

