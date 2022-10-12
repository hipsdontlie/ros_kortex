#include<iostream>
#include<vector>
#include<string>
#include<experimental/filesystem>
#include<time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
using std::experimental::filesystem::exists;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Controls
{
    public:
        //pelvis marker monitor
        void joint_limits(const std_msgs::Float64::ConstPtr& jlimits_msg);

        void singularity_check(const std_msgs::Float64::ConstPtr& singularity_msg);

        void error_check(const std_msgs::Float64MultiArray::ConstPtr& error_msg);

};