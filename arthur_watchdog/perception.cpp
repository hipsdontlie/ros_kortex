#include "include/perception.hpp"

bool flg = false;

//perception rmse error
void Perception::perception_eval(const std_msgs::Float64::ConstPtr& rmse_msg)
{
  if (rmse_msg->data > rmse_thresh->data)
  {
    ROS_INFO("RMSE error too high. Try again!\n");
    flg = true;
  }
  else
  { 
    std::cout<<rmse_msg->data<<std::endl;
    ROS_INFO("RMSE error is low! Continue...\n");
    // flg = true;
  }
}