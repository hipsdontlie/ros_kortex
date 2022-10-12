#include "include/inputs.hpp"
#include "include/perception.hpp"
#include "include/controls.hpp"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  Inputs inputs;
  Perception perception;
  Controls controls;

  perception.rmse_thresh->data = 1.0;


  ros::Subscriber sub = n.subscribe("pelvis_pose", 1000, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber cup_sub = n.subscribe("initial_reaming_point", 1000, &Inputs::cupCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1000, &Inputs::eeCallback, &inputs);

  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1000, &Perception::perception_eval, &perception);

  ros::Subscriber ee_control_sub = n.subscribe("control_error", 1000, &Controls::error_check, &controls);


  while(ros::ok())
  {
    inputs.currTime = ros::Time::now();

    if(inputs.currTime - inputs.prevTime > ros::Duration(0.025))
      std::cout<<"Pelvis marker frequency dropped below 40Hz\n"<<std::endl;

    // if (flg)
    //   break;

    ros::spinOnce();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
