#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

float t; 
void dummyPoseBroadcaster(float t){
  
  ROS_INFO("Sending dummy tf!");
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "dummy_pelvis";
  double pi = 3.14159265359;
  t = t * pi / 180; 
  float x = 0.25*sin(t);
  float y = 0.25*sin(t);
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.15);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);


}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "dummy_pelvis_pose");
  ROS_INFO("Started dummy tf node!");
  ros::NodeHandle node;
  
  
  while (ros::ok()){
    t += 1;
    ros::Rate r(5); // 10 hz
    dummyPoseBroadcaster(t);
    r.sleep();
  }

  return 0;
};