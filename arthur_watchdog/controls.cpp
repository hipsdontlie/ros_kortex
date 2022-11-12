#include "include/controls.hpp"


//controls alignment error
void Controls::error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg)
{
  prevTime_error = ros::Time::now().toSec();
  double trans_error = error_msg->data[0];
  double orientation_error = error_msg->data[1];
  if (trans_error > 150.0 || orientation_error > 45.0)
  {
    if(trans_error > 150.0)
    {
      trans_bool = false;
      ROS_INFO("Translation error is too high!\n");
    }
    if(orientation_error > 45.0)
    {
      orien_bool = false;
      ROS_INFO("Orientation error is too high!\n");
    }
  }
  else
  { 
    // std::cout<<error_msg->data<<std::endl;
    ROS_INFO("Translation/Orientation error within threshold! Continue...\n");
    // controller_flag = true;
    trans_bool = true;
    orien_bool = true;
  }
}

void Controls::singularity_check(const std_msgs::Float64::ConstPtr& singularity_msg)
{
  prevTime_singularity = ros::Time::now().toSec();
  if (singularity_msg->data < 0.05)
  {
    ROS_INFO("Very close to singularity! Please realign!\n");
    // controller_flag = false;
    singularity_bool = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Singularity within limits...\n");
    // controller_flag = true;
    singularity_bool = true;
  }
}

void Controls::joint_limits(const std_msgs::Bool::ConstPtr& jlimits_msg)
{
  prevTime_jlimits = ros::Time::now().toSec();
  if (jlimits_msg->data == true)
  {
    ROS_INFO("Very close to joint limits! Please realign!\n");
    // controller_flag = false;
    jlimits_bool = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Joints within limits...\n");
    // controller_flag = true;
    jlimits_bool = true;
  }
}

void Controls::controller_fault(const std_msgs::Bool::ConstPtr& controls_fault)
{
  if (controls_fault->data == true)
  {
    ROS_INFO("Controller has fault! Please realign!\n");
    // controller_flag = false;
    controlsFault_bool = false;
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Joints within limits...\n");
    // controller_flag = true;
    controlsFault_bool = true;
  }
}


