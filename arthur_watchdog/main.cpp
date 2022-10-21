#include "include/inputs.hpp"
#include "include/perception.hpp"
#include "include/controls.hpp"
#include "include/hardware.hpp"
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
  Hardware hardware;

  perception.rmse_thresh->data = 1.0;

  //subscribers
  ros::Subscriber sub = n.subscribe("pelvis_pose", 1000, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber rp_sub = n.subscribe("probe_pose", 1000, &Inputs::rpCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1000, &Inputs::eeCallback, &inputs);

  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1000, &Perception::perception_eval, &perception);

  ros::Subscriber errors_sub = n.subscribe("controls_error", 1000, &Controls::error_check, &controls);
  
  ros::Subscriber singularity_sub = n.subscribe("controls_singularity", 1000, &Controls::singularity_check, &controls);

  ros::Subscriber jlimits_sub = n.subscribe("controls_jlimits", 1000, &Controls::joint_limits, &controls);

  ros::Subscriber reamerSpeed_sub = n.subscribe("controls_jlimits", 1000, &Controls::joint_limits, &controls);

  ros::Subscriber loadApplied_sub = n.subscribe("controls_jlimits", 1000, &Controls::joint_limits, &controls);




  // ros::Subscriber ee_control_sub = n.subscribe("control_error", 1000, &Controls::error_check, &controls);

  //publishers
  ros::Publisher inputs_pub = n.advertise<arthur_watchdog::inputs>("input_health", 1000);
  ros::Publisher percep_pub = n.advertise<arthur_watchdog::perception>("perception_health", 1000);
  ros::Publisher controllerFlag_pub = n.advertise<std_msgs::Bool>("controller_flag", 1000);

  ros::Rate loop_rate(1000);


  while(ros::ok())
  {
    arthur_watchdog::inputs input_msg;
    arthur_watchdog::perception percep_msg;
    std_msgs::Bool control_msg;

    inputs.currTime_pelvis = ros::Time::now().toSec();
    inputs.currTime_ee = ros::Time::now().toSec();
    inputs.currTime_rp = ros::Time::now().toSec();

    if(inputs.currTime_pelvis - inputs.prevTime_pelvis > ros::Duration(0.025).toSec())
    {
      // std::cout<<"Pelvis marker not visible"<<std::endl;
      ROS_INFO("Pelvis not visible!\n");
      inputs.pelvis_visible = false;
    }

    if(inputs.currTime_ee - inputs.prevTime_ee > ros::Duration(0.025).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("End-effector not visible!\n");
      inputs.ee_visible = false;
    }
    // if (flg)
    //   break;

    if(inputs.currTime_rp - inputs.prevTime_rp > ros::Duration(0.025).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("Registration probe not visible!\n");
      inputs.probe_visible = false;
    }

    input_msg.ee_visible = inputs.ee_visible;
    input_msg.pelvis_visible = inputs.pelvis_visible;
    input_msg.probe_visible = inputs.probe_visible;
    inputs_pub.publish(input_msg);

    percep_msg.rmse_error = perception.rmse_error;
    percep_pub.publish(percep_msg);

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
    loop_rate.sleep();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
