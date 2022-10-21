#include "include/hardware.hpp"


//controls alignment error
void Hardware::reamer_speed(const std_msgs::Float64::ConstPtr& reamerSpeed_msg)
{
  if (reamerSpeed_msg->data > 500.0)
  {
    ROS_INFO("Reamer speed too high!\n");
  }
  else
  { 
    // std::cout<<error_msg->data<<std::endl;
    ROS_INFO("Reamer speed within limits! Continue...\n");
  }
}

void Hardware::load_applied(const std_msgs::Float64::ConstPtr& loadApplied_msg)
{
  if (loadApplied_msg->data > 20.0)
  {
    ROS_INFO("Load very high! Please retract\n");
  }
  else
  { 
    // std::cout<<singularity_msg->data<<std::endl;
    ROS_INFO("Load within limit...\n");
  }
}


