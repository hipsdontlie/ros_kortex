#include "include/inputs.hpp"
#include<arthur_watchdog/arthur_watch.h>



void Inputs::pelvisCallback(const geometry_msgs::PoseStamped::ConstPtr& pelvis_msg)
    {
      ros::Duration pelvis_freq(0.025);
      prevTime = ros::Time::now();

      if (pelvis_msg->pose.position.x==0 || pelvis_msg->pose.position.y==0 || pelvis_msg->pose.position.z==0)
      {
        ROS_INFO("Pelvis marker not visible\n");
        pelvis_visible = true;
      }
      else if ((currTime - prevTime) > pelvis_freq)
      {
        ROS_INFO("Pelvis marker stream dropped below 40Hz\n");
        pelvis_visible = false;
      }
      else
      {
        ROS_INFO("Pelvis marker is visible\n");
        pelvis_visible = false;
      }
    }

    //reaming end point monitor
void Inputs::cupCallback(const geometry_msgs::PoseStamped::ConstPtr& cup_msg)
{
  if (cup_msg->pose.position.x==0 || cup_msg->pose.position.y==0 || cup_msg->pose.position.z==0)
  {
    ROS_INFO("Reaming end point unavailable.\n");
  }
  else
  {
    ROS_INFO("Reaming end point determined!\n");
  }
}


//end-effector marker monitor
void Inputs::eeCallback(const geometry_msgs::PoseStamped::ConstPtr& ee_msg)
{
  if (ee_msg->pose.position.x==0 || ee_msg->pose.position.y==0 || ee_msg->pose.position.z==0)
  {
    ROS_INFO("End effector not visible\n");
    ee_visible = false;
  }
  else
  {
    ROS_INFO("End effector visible\n");
    ee_visible = true;
  }
}