#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <arthur_planning/arthur_traj.h>


static bool new_traj = true;
static tf::Vector3 translation = tf::Vector3();
static tf::Matrix3x3 mRot = tf::Matrix3x3();
static int current_waypoint = 0;

void tfCallback(const geometry_msgs::Transform msg) {
    tf::Quaternion q = tf::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
    mRot.setRotation(q);
    translation.setValue(msg.translation.x, msg.translation.y, msg.translation.z);
}

// void trajCallback(const arthur_planning::arthur_traj msg) {

// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "wrench_controller");

    ros::NodeHandle node;

    // ros::Subscriber transform_sub = node.subscribe("/my_gen3/ee_tf", 1, tfCallback);
    // ros::Subscriber trajectory_sub = node.subscribe("/my_gen3/arthur_traj", 1, trajCallback)
    // ros::ServiceClient wrench_commander = node.serviceClient<kortex_driver::SendWrenchCommand>("/my_gen3/base/send_wrench_command")

    ros::Rate rate(40.0);
    while (ros::ok()){
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};