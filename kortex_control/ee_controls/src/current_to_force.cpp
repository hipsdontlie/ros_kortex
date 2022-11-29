#include "ros/ros.h"
#include <boost/bind.hpp>
#include "std_msgs/Float64.h"

void currentToForceCallback(const std_msgs::Float64::ConstPtr& msg, ros::Publisher* force_pub)
{
    std_msgs::Float64 force_msg;
    force_msg.data = msg->data * (0.284518 / 4.6) * 0.6494350416 * 2.0 * M_PI / 0.005;

    force_pub->publish(force_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "current_to_force_node");

    ros::NodeHandle n;

    ros::Publisher force_pub = n.advertise<std_msgs::Float64>("/end_effector_force", 1);
    ros::Subscriber current_sub = n.subscribe<std_msgs::Float64>("/hardware_current/data", 1, boost::bind(&currentToForceCallback, _1, &force_pub));

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}