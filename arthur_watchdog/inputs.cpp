#include "include/inputs.hpp"
#include "include/perception.hpp"

// #include <arthur_watchdog/arthur_watch.h>

ros::Time prevTime;
ros::Time currTime;
// std_msgs::Float64 rmse_thresh;
std::shared_ptr<std_msgs::Float64> rmse_thresh;
void Inputs::pelvisCallback(const geometry_msgs::PoseStamped::ConstPtr& pelvis_msg)
    {
      ros::Duration pelvis_freq(0.025);
      prevTime = ros::Time::now();

      if (pelvis_msg->pose.position.x==0 || pelvis_msg->pose.position.y==0 || pelvis_msg->pose.position.z==0)
      {
        ROS_INFO("Pelvis marker not visible\n");
      }
      else if ((currTime - prevTime) > pelvis_freq)
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

//perception rmse error
void Perception::perception_eval(const std_msgs::Float64::ConstPtr& rmse_msg)
{
  if (rmse_msg->data > rmse_thresh->data)
  {
    ROS_INFO("RMSE error too high. Try again!\n");
  }
  else
  {
    ROS_INFO("RMSE error is slow! Continue...\n");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  Inputs inputs;
  Perception perception;

  rmse_thresh->data = 1.0;


  ros::Subscriber sub = n.subscribe("pelvis_pose", 1000, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber cup_sub = n.subscribe("initial_reaming_point", 1000, &Inputs::cupCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1000, &Inputs::eeCallback, &inputs);

  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1000, &Perception::perception_eval, &perception);


  while(ros::ok())
  {
    currTime = ros::Time::now();

    if(currTime - prevTime > ros::Duration(0.025))
      std::cout<<"Pelvis marker not visible"<<std::endl;

    ros::spinOnce();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
