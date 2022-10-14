#include "include/inputs.hpp"
#include<arthur_watchdog/inputs.h>



void Inputs::pelvisCallback(const geometry_msgs::PoseStamped::ConstPtr& pelvis_msg)
    {
      ros::Duration pelvis_freq(0.025);
      prevTime_pelvis = ros::Time::now().toSec();

    //   if (pelvis_msg->pose.position.x==0 || pelvis_msg->pose.position.y==0 || pelvis_msg->pose.position.z==0)
    //   {
    //     ROS_INFO("Pelvis marker not visible\n");
    //     pelvis_visible = true;
    //   }
      if ((currTime_pelvis - prevTime_pelvis) > pelvis_freq.toSec())
      {
        ROS_INFO("Pelvis marker stream dropped below 40Hz\n");
        pelvis_visible = false;
      }
      else
      {
        ROS_INFO("Pelvis marker is visible\n");
        pelvis_visible = true;
      }
    }

//reaming end point monitor
void Inputs::rpCallback(const geometry_msgs::PoseStamped::ConstPtr& rp_msg)
{   
    ros::Duration rp_freq(0.025);
    prevTime_rp = ros::Time::now().toSec();
    if ((currTime_ee - prevTime_ee) > rp_freq.toSec())
    {
        ROS_INFO("Registration probe not visible.\n");
        probe_visible = false;
    }
    else
    {
        ROS_INFO("Registration probe visible\n");
        probe_visible = true;
    }
}


//end-effector marker monitor
void Inputs::eeCallback(const geometry_msgs::PoseStamped::ConstPtr& ee_msg)
{
    prevTime_ee = ros::Time::now().toSec();
    ros::Duration ee_freq(0.025);
    if ((currTime_ee - prevTime_ee) > ee_freq.toSec())
    {
        ROS_INFO("End effector not visible\n");
        std::cout<<currTime_ee - prevTime_ee<<std::endl;
        ee_visible = false;
    }
    else
    {
        ROS_INFO("End effector visible\n");
        ee_visible = true;
    }
}