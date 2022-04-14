#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "ee_tf_listener");

  ros::NodeHandle node;

  ros::Publisher transform_pub = node.advertise<geometry_msgs::Transform>("/my_gen3/ee_tf", 1);

  tf::TransformListener listener;

  ros::Rate rate(40.0);
  while (ros::ok()){
    tf::StampedTransform stamped_transform;
    try{
      listener.lookupTransform("base_link", "tool_frame",  
                               ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Transform transform;
    transform.translation.x = stamped_transform.getOrigin().x();
    transform.translation.y = stamped_transform.getOrigin().y();
    transform.translation.z = stamped_transform.getOrigin().z();
    tf::Quaternion q;
    stamped_transform.getBasis().getRotation(q);
    transform.rotation.x = q[0];
    transform.rotation.y = q[1];
    transform.rotation.z = q[2];
    transform.rotation.w = q[3];
    
    transform_pub.publish(transform);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};