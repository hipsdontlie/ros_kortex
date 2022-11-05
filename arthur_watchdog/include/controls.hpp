#pragma once

#include<iostream>
#include<vector>
#include<string>
#include<experimental/filesystem>
#include<time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include<std_msgs/Bool.h>
// #include<std_msgs/doub
using std::experimental::filesystem::exists;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Controls
{
    public:
        //pelvis marker monitor
        void joint_limits(const std_msgs::Bool::ConstPtr& jlimits_msg);

        void singularity_check(const std_msgs::Float64::ConstPtr& singularity_msg);

        void error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg);

        void controller_fault(const std_msgs::Bool::ConstPtr& controls_fault);

        double currTime_error;
        double prevTime_error;
        double currTime_singularity;
        double prevTime_singularity;
        double currTime_jlimits;
        double prevTime_jlimits;
        bool controller_flag;
        bool trans_bool;
        bool orien_bool;
        bool singularity_bool;
        bool jlimits_bool;
        bool controlsFault_bool;



};