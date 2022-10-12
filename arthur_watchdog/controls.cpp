#include "include/controls.hpp"

bool flg = false;

//perception rmse error
void Controls::error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg)
{
  if (error_msg->data > rmse_thresh->data)
  {
    ROS_INFO("RMSE error too high. Try again!\n");
    flg = true;
  }
  else
  { 
    std::cout<<error_msg->data<<std::endl;
    ROS_INFO("RMSE error is low! Continue...\n");
    // flg = true;
  }
}