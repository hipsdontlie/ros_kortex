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

class Hardware
{
    public:
        //pelvis marker monitor
        void reamer_speed(const std_msgs::Float64::ConstPtr& reamerSpeed_msg);

        void load_applied(const std_msgs::Float64::ConstPtr& loadApplied_msg);

        void ream_percent(const std_msgs::Float64::ConstPtr& reamPercent_msg);

        void current_drawn(const std_msgs::Float64::ConstPtr& currentDrawn_msg);

        bool hardware_flag;
        bool reamerSpeed_printed = false;
        bool loadApplied_printed = false;
        bool reamPercent_printed = false;
        bool currentDrawn_printed = false;

        double currTime_reamerSpeed;
        double prevTime_reamerSpeed;
        double currTime_loadApplied;
        double prevTime_loadApplied;
        double currTime_reamPercent;
        double prevTime_reamPercent;
        double currTime_currentDrawn;
        double prevTime_currentDrawn;
        // void error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg);

};