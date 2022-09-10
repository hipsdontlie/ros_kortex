/**
 * @file tf_listener.cpp
 * @author Anthony Kyu (akyu@andrew.cmu.edu)
 * Team Members: Parker Hill, Kaushik Balasundar, Sundaram Seivur, Gunjan Sethi
 * @brief This listens for the end-effector transform frame that is broadcasted and then publishes it.
 * @version 1.0
 * @date 2022-04-06
 * 
 * @copyright Hipster Copyright (c) 2022
 * 
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;
  std::string source_frame_;
  node.getParam("/tf_listener/source_frame", source_frame_);
  std::string target_frame_;
  node.getParam("/tf_listener/target_frame", target_frame_);
  std::string topic_name_ = "tf/" + source_frame_ + "_to_" + target_frame_;
  ros::Publisher transform_pub = node.advertise<geometry_msgs::Transform>(topic_name_, 1);

  tf::TransformListener listener;

  ros::Rate rate(1000.0);
  while (ros::ok()){
    // Gets the broadcasted frame called target_frame_
    tf::StampedTransform stamped_transform;
    try{
      listener.lookupTransform(source_frame_, target_frame_,  
                               ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Puts frame into transform struct to be published
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
    
    // publish transform to be used
    transform_pub.publish(transform);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};