#include "include/inputs.hpp"


void Inputs::pelvisCallback(const geometry_msgs::PoseStamped::ConstPtr& pelvis_msg)
    {
      ros::Time pelvis_freq(0.025);
      if (pelvis_msg->pose.position.x==0 || pelvis_msg->pose.position.y==0 || pelvis_msg->pose.position.z==0)
      {
        ROS_INFO("Pelvis marker not visible\n");
      }
      else if (pelvis_msg->header.stamp > pelvis_freq)
      {
        ROS_INFO("Pelvis marker stream dropped below 40Hz\n");
      }
      else
      {
        ROS_INFO("Pelvis marker is visible\n");
      }
    }

    //reaming end point monitor
void Inputs::cupCallback(const geometry_msgs::PoseStamped::ConstPtr& cup_msg)
{
  if (cup_msg->pose.position.x==0 || cup_msg->pose.position.y==0 || cup_msg->pose.position.z==0)
  {
    ROS_INFO("Reaming end point unavailable.\n");
  }
  else
  {
    ROS_INFO("Reaming end point determined!\n");
  }
}


//end-effector marker monitor
void Inputs::eeCallback(const geometry_msgs::PoseStamped::ConstPtr& ee_msg)
{
  if (ee_msg->pose.position.x==0 || ee_msg->pose.position.y==0 || ee_msg->pose.position.z==0)
  {
    ROS_INFO("End effector not visible\n");
  }
  else
  {
    ROS_INFO("End effector visible\n");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  Inputs inputs;

  ros::Subscriber sub = n.subscribe("pelvis_pose", 1000, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber cup_sub = n.subscribe("initial_reaming_point", 1000, &Inputs::cupCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1000, &Inputs::eeCallback, &inputs);


  ros::spin();


  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
