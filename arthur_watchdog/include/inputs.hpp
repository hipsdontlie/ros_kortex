#pragma once

#include<iostream>
#include<vector>
#include<string>
#include<experimental/filesystem>
#include<time.h>
using std::experimental::filesystem::exists;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Inputs
{
    public:
        //pelvis marker monitor
        void pelvisCallback(const geometry_msgs::PoseStamped::ConstPtr& pelvis_msg);

        //reaming end point monitor
        void cupCallback(const geometry_msgs::PoseStamped::ConstPtr& cup_msg);

        //end-effector marker monitor
        void eeCallback(const geometry_msgs::PoseStamped::ConstPtr& ee_msg);        

        //registration probe monitor
        void rpCallback(const geometry_msgs::PoseStamped::ConstPtr& rp_msg);   

    
        ros::Time currTime;
        ros::Time prevTime;
        ros::Time freq;
        bool ee_visible = false;
        bool probe_visible = false;
        bool pelvis_visible = false;
        bool robot_connected = false;
    
};