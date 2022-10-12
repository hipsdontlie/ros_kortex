#include "include/inputs.hpp"
#include "include/perception.hpp"
#include "include/controls.hpp"
#include<arthur_watchdog/inputs.h>
#include<arthur_watchdog/perception.h>
#include<std_msgs/Bool.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  bool controller_flag = false;

  Inputs inputs;
  Perception perception;
  Controls controls;

  perception.rmse_thresh->data = 1.0;


  ros::Subscriber sub = n.subscribe("pelvis_pose", 1000, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber cup_sub = n.subscribe("initial_reaming_point", 1000, &Inputs::cupCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1000, &Inputs::eeCallback, &inputs);

  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1000, &Perception::perception_eval, &perception);

  // ros::Subscriber ee_control_sub = n.subscribe("control_error", 1000, &Controls::error_check, &controls);

  ros::Publisher inputs_pub = n.advertise<arthur_watchdog::inputs>("input_health", 1000);
  ros::Publisher percep_pub = n.advertise<arthur_watchdog::perception>("perception_health", 1000);
  ros::Publisher controllerFlag_pub = n.advertise<std_msgs::Bool>("controller_flag", 1000);




  while(ros::ok())
  {
    arthur_watchdog::inputs input_msg;
    arthur_watchdog::perception percep_msg;
    std_msgs::Bool control_msg;

    inputs.currTime = ros::Time::now();

    if(inputs.currTime - inputs.prevTime > ros::Duration(0.025))
      std::cout<<"Pelvis marker frequency dropped below 40Hz\n"<<std::endl;

    // if (flg)
    //   break;

    input_msg.ee_visible = inputs.ee_visible;
    input_msg.pelvis_visible = inputs.pelvis_visible;
    inputs_pub.publish(input_msg);

    percep_msg.rmse_error = perception.rmse_error;

    if (inputs.pelvis_visible==true && perception.rmse_error==true){
      controller_flag = true;
      control_msg.data = controller_flag;
    }
    else{
      controller_flag = false;
      control_msg.data = controller_flag;
    }

    controllerFlag_pub.publish(control_msg);


    ros::spinOnce();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
