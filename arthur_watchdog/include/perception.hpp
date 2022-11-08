#include<iostream>
#include<vector>
#include<string>
#include<experimental/filesystem>
#include<time.h>
#include <std_msgs/Float64.h>
using std::experimental::filesystem::exists;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Perception
{
    public:
        //pelvis marker monitor
        void perception_eval(const std_msgs::Float64::ConstPtr& percep_msg);

        std::shared_ptr<std_msgs::Float64> rmse_thresh = std::make_shared<std_msgs::Float64> ();

        bool rmse_error = true;
        // bool rmse_error = false;

        bool percep_printed = false;


    
};