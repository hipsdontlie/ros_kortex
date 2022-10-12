#include "include/controls.hpp"


//controls alignment error
void Controls::error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg)
{
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

