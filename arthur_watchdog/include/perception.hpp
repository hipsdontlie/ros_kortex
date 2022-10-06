#include<iostream>
#include<vector>
#include<string>
#include<experimental/filesystem>
#include<time.h>
using std::experimental::filesystem::exists;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

class Perception
{
    public:
        //pelvis marker monitor
        void perception_eval(const geometry_msgs::PoseStamped::ConstPtr& percep_msg);

    
};