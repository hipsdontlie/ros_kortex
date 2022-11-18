#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <cmath>
#include <vector>
#include <array>

double Wc = 62.831853; // cutoff frequency in rad/s
double K = std::tan(M_PI * Wc);
double norm = 1 / (K*K*K + 2*K*K + 2*K + 1);
double a0 = K*K*K*norm;
double a1 = 3 * a0;
double a2 = a1; 
double a3 = a0;
double b1 = (3*K*K*K + 2*K*K - 2*K - 3) * norm;
double b2 = (3*K*K*K - 2*K*K - 2*K + 3) * norm;
double b3 = (K*K*K - 2*K*K + 2*K - 1) * norm;

std::array<double, 7> z1 = {0, 0, 0, 0, 0, 0, 0};
std::array<double, 7> z2 = {0, 0, 0, 0, 0, 0, 0};
std::array<double, 7> z3 = {0, 0, 0, 0, 0, 0, 0};

void baseToEEMarkerFrameCallback(const geometry_msgs::Transform::ConstPtr& msg, tf::Transform* transform){
  transform->setOrigin(tf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
  transform->setRotation(tf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));
}

void eeMarkerToCameraCallback(const geometry_msgs::Transform::ConstPtr& msg, tf::Transform* transform){
  transform->setOrigin(tf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
  transform->setRotation(tf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));
}

void broadcastBaseToCamera(tf::TransformBroadcaster& br, tf::Transform base_to_ee_marker_frame, tf::Transform ee_marker_frame_to_ee_marker, tf::Transform ee_marker_to_camera, int& counter)
{
  // ee_marker_frame_to_ee_marker.getOrigin().setZero();
  tf::Transform base_to_camera = base_to_ee_marker_frame * ee_marker_frame_to_ee_marker * ee_marker_to_camera;
  std::array<double, 7> input = {base_to_camera.getOrigin().getX(),
                                base_to_camera.getOrigin().getY(),
                                base_to_camera.getOrigin().getZ(),
                                base_to_camera.getRotation().getX(),
                                base_to_camera.getRotation().getY(),
                                base_to_camera.getRotation().getZ(),
                                base_to_camera.getRotation().getW()};
  std::array<double, 7> output = {0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 7; ++i)
  {
    output[i] = input[i] * a0 + z1[i];
    z1[i] = input[i] * a1 + z2[i] - b1 * output[i];
    z2[i] = input[i] * a2 + z3[i] - b2 * output[i];
    z3[i] = input[i] * a3 - b3 * output[i];
  }
  if (counter < 3)
  {
    ++counter;
  }
  else
  {
    tf::Transform output_transform = tf::Transform::getIdentity();
    output_transform.setOrigin(tf::Vector3(output[0], output[1], output[2]));
    output_transform.setRotation(tf::Quaternion(output[3], output[4], output[5], output[6]));
    br.sendTransform(tf::StampedTransform(output_transform, ros::Time::now(), "base_link", "camera"));
  }
//   tf::Transform base_to_camera = base_to_ee_marker_frame * ee_marker_frame_to_ee_marker;
//   base_to_camera = base_to_camera * ee_marker_to_camera;
//   std::cout << base_to_camera.getOrigin().getX() << ", ";
//   std::cout << base_to_camera.getOrigin().getY() << ", ";
//   std::cout << base_to_camera.getOrigin().getZ() << ", ";
//   std::cout << base_to_camera.getRotation().getX() << ", ";
//   std::cout << base_to_camera.getRotation().getY() << ", ";
//   std::cout << base_to_camera.getRotation().getZ() << ", ";
//   std::cout << base_to_camera.getRotation().getW() << std::endl;
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "arthur_online_calibration");

  ros::NodeHandle node_;

  double calibration_rate_ = 40.0; // Rate of ros node (hz)
  ros::Rate rate(calibration_rate_);
  tf::TransformBroadcaster br_;

  tf::Transform base_to_ee_marker_frame_ = tf::Transform::getIdentity();
  tf::Transform ee_marker_frame_to_ee_marker_ = tf::Transform::getIdentity();
  // ee_marker_frame_to_ee_marker_.setRotation(tf::Quaternion(0.0034195, -0.0097427, 0.995, 0.09933));
  ee_marker_frame_to_ee_marker_.setRotation(tf::Quaternion(0, 0, 0.9950581, 0.0992947));
  tf::Transform ee_marker_to_camera_ = tf::Transform::getIdentity();

  ros::Subscriber base_to_ee_marker_frame_sub_ = node_.subscribe<geometry_msgs::Transform>("/tf/base_link_to_ee_marker_frame", 1, boost::bind(&baseToEEMarkerFrameCallback, _1, &base_to_ee_marker_frame_));
  ros::Subscriber ee_marker_to_camera_sub_ = node_.subscribe<geometry_msgs::Transform>("/tf/ee_marker_to_camera", 1, boost::bind(&baseToEEMarkerFrameCallback, _1, &ee_marker_to_camera_));

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  bool transforms_exist_ = false;
  int lowpass_counter_ = 0;

  while (ros::ok())
  {
    if(!transforms_exist_)
    {
      try
      {
        tfBuffer.lookupTransform("base_link", "ee_marker_frame", ros::Time(0));
        tfBuffer.lookupTransform("ee_marker", "camera", ros::Time(0));
      }
      catch (tf2::TransformException &ex) 
      {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
      }
      transforms_exist_ = true;
    }
    
    broadcastBaseToCamera(br_, base_to_ee_marker_frame_, ee_marker_frame_to_ee_marker_, ee_marker_to_camera_, lowpass_counter_);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};