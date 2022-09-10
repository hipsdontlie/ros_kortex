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
  transformStamped.transform.translation.x = 0.757;
  transformStamped.transform.translation.y = 0.001;
  transformStamped.transform.translation.z = 0.433;
  //-0.001, -0.434, -0.756
  //[0.500, 0.500, 0.500, -0.500]
  tf2::Quaternion q;
  q.setRPY(3.142, 0, 0);
//   transformStamped.transform.rotation.x = q.x();
//   transformStamped.transform.rotation.y = q.y();
//   transformStamped.transform.rotation.z = q.z();
//   transformStamped.transform.rotation.w = q.w();
  transformStamped.transform.rotation.x = 0.5;
  transformStamped.transform.rotation.y = 0.5;
  transformStamped.transform.rotation.z = 0.5;
  transformStamped.transform.rotation.w = 0.5;
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