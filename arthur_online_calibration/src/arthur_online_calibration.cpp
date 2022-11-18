#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>


void baseToEEMarkerFrameCallback(const geometry_msgs::Transform::ConstPtr& msg, tf::Transform* transform){
  transform->setOrigin(tf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
  transform->setRotation(tf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));
}

void eeMarkerFrameToEEMarkerCallback(const geometry_msgs::Transform::ConstPtr& msg, tf::Transform* transform){
  transform->setOrigin(tf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
  transform->setRotation(tf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));
}

void eeMarkerToCameraCallback(const geometry_msgs::Transform::ConstPtr& msg, tf::Transform* transform){
  transform->setOrigin(tf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
  transform->setRotation(tf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));
}

void broadcastBaseToCamera(tf::TransformBroadcaster& br, tf::Transform base_to_ee_marker_frame, tf::Transform ee_marker_frame_to_ee_marker, tf::Transform ee_marker_to_camera)
{
  ee_marker_frame_to_ee_marker.getOrigin().setZero();
  tf::Transform base_to_camera = base_to_ee_marker_frame * ee_marker_frame_to_ee_marker * ee_marker_to_camera;
//   tf::Transform base_to_camera = base_to_ee_marker_frame * ee_marker_frame_to_ee_marker;
//   base_to_camera = base_to_camera * ee_marker_to_camera;
//   std::cout << base_to_camera.getOrigin().getX() << ", ";
//   std::cout << base_to_camera.getOrigin().getY() << ", ";
//   std::cout << base_to_camera.getOrigin().getZ() << ", ";
//   std::cout << base_to_camera.getRotation().getX() << ", ";
//   std::cout << base_to_camera.getRotation().getY() << ", ";
//   std::cout << base_to_camera.getRotation().getZ() << ", ";
//   std::cout << base_to_camera.getRotation().getW() << std::endl;
  br.sendTransform(tf::StampedTransform(base_to_camera, ros::Time::now(), "base_link", "camera"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "arthur_online_calibration");

  ros::NodeHandle node_;

  double calibration_rate_ = 40.0; // Rate of ros node (hz)
  ros::Rate rate(calibration_rate_);
  tf::TransformBroadcaster br_;

  tf::Transform base_to_ee_marker_frame_ = tf::Transform::getIdentity();
  tf::Transform ee_marker_frame_to_ee_marker_ = tf::Transform::getIdentity();
  ee_marker_frame_to_ee_marker_.setRotation(tf::Quaternion(0.0034195, -0.0097427, 0.995, 0.09933));
  tf::Transform ee_marker_to_camera_ = tf::Transform::getIdentity();

  ros::Subscriber base_to_ee_marker_frame_sub_ = node_.subscribe<geometry_msgs::Transform>("/tf/base_link_to_ee_marker_frame", 1, boost::bind(&baseToEEMarkerFrameCallback, _1, &base_to_ee_marker_frame_));
//   ros::Subscriber ee_marker_frame_to_ee_marker_sub_ = node_.subscribe<geometry_msgs::Transform>("/tf/ee_marker_frame_to_ee_marker", 1, boost::bind(&eeMarkerFrameToEEMarkerCallback, _1, &ee_marker_frame_to_ee_marker_));
  ros::Subscriber ee_marker_to_camera_sub_ = node_.subscribe<geometry_msgs::Transform>("/tf/ee_marker_to_camera", 1, boost::bind(&baseToEEMarkerFrameCallback, _1, &ee_marker_to_camera_));


  while (ros::ok())
  {
    broadcastBaseToCamera(br_, base_to_ee_marker_frame_, ee_marker_frame_to_ee_marker_, ee_marker_to_camera_);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};