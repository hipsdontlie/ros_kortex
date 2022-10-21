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

        // void error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg);

};