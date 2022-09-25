#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, 0.2, 0.2) );
  tf::Quaternion q;
  q.setEuler(0, -M_PI/2.0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "pelvis", "dummy_pelvis"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "real_dummy_pelvis_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/pelvis_pose", 1, &poseCallback);

  ros::spin();
  return 0;
};