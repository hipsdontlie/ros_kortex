#include "include/inputs.hpp"
#include "include/perception.hpp"
#include "include/controls.hpp"
#include "include/hardware.hpp"
#include<arthur_watchdog/inputs.h>
#include<arthur_watchdog/perception.h>
#include<std_msgs/Bool.h>
#include <fstream>
#include<std_msgs/Empty.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  std::ofstream fw("/home/mrsd-team-c/arthur_ws/CPlusPlusSampleFile.txt", std::ofstream::out);

  bool controller_flag = false;

  Inputs inputs;
  Perception perception;
  Controls controls;
  Hardware hardware;

  perception.rmse_thresh->data = 1.0;

  //subscribers
  ros::Subscriber sub = n.subscribe("pelvis_pose", 1, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber rp_sub = n.subscribe("probe_pose", 1, &Inputs::rpCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1, &Inputs::eeCallback, &inputs);

  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1, &Perception::perception_eval, &perception);

  ros::Subscriber errors_sub = n.subscribe("controls_error", 1, &Controls::error_check, &controls);
  
  ros::Subscriber singularity_sub = n.subscribe("controls_singularity", 1, &Controls::singularity_check, &controls);

  ros::Subscriber jlimits_sub = n.subscribe("controls_jlimits", 1, &Controls::joint_limits, &controls);

  ros::Subscriber reamerSpeed_sub = n.subscribe("controls_jlimits", 1, &Controls::joint_limits, &controls);

  ros::Subscriber loadApplied_sub = n.subscribe("controls_jlimits", 1, &Controls::joint_limits, &controls);




  // ros::Subscriber ee_control_sub = n.subscribe("control_error", 1000, &Controls::error_check, &controls);

  //publishers
  ros::Publisher inputs_pub = n.advertise<arthur_watchdog::inputs>("input_health", 1);
  ros::Publisher percep_pub = n.advertise<arthur_watchdog::perception>("perception_health", 1);
  ros::Publisher controllerFlag_pub = n.advertise<std_msgs::Bool>("controller_flag", 1);
  ros::Publisher eStop_pub = n.advertise<std_msgs::Empty>("my_gen3/in/emergency_stop", 1);

  ros::Rate loop_rate(1000);


  while(ros::ok())
  {
    arthur_watchdog::inputs input_msg;
    arthur_watchdog::perception percep_msg;
    std_msgs::Bool control_msg;
    std_msgs::Empty eStop_msg;
    eStop_msg = {};

    inputs.currTime_pelvis = ros::Time::now().toSec();
    inputs.currTime_ee = ros::Time::now().toSec();
    inputs.currTime_rp = ros::Time::now().toSec();

    if(inputs.currTime_pelvis - inputs.prevTime_pelvis > ros::Duration(0.035).toSec())
    {
      // std::cout<<"Pelvis marker not visible"<<std::endl;
      ROS_INFO("Pelvis not visible!\n");
      inputs.pelvis_visible = false;
      if (fw.is_open())
      {
          fw << inputs.pelvis_visible << "\n";
      }
        fw.close();
      eStop_pub.publish(eStop_msg);
    }
    
    //if you stop receiving 
    if(inputs.currTime_ee - inputs.prevTime_ee > ros::Duration(0.035).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("End-effector not visible!\n");
      inputs.ee_visible = false;
      eStop_pub.publish(eStop_msg);
    }

    if(inputs.currTime_rp - inputs.prevTime_rp > ros::Duration(0.035).toSec())
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

    if (inputs.pelvis_visible==true && inputs.ee_visible==true && perception.rmse_error==true)
    {
      controller_flag = true;
      control_msg.data = controller_flag;
      ROS_INFO("Setting controller flag to true");
    }
    else
    {
      controller_flag = false;
      control_msg.data = controller_flag;
    }


    if(controls.currTime_error - controls.prevTime_error > ros::Duration(0.033).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("Controller error publisher dropped below 30Hz!\n");
      controller_flag = false;
    }

    if(controls.currTime_singularity - controls.prevTime_singularity > ros::Duration(0.033).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("Controller singularity publisher dropped below 30Hz!\n");
      controller_flag = false;
    }

    if(controls.currTime_jlimits - controls.prevTime_jlimits > ros::Duration(0.033).toSec())
    {
      // std::cout<<"End-effector not visible"<<std::endl;
      ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
      controller_flag = false;
    }

    controllerFlag_pub.publish(control_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
