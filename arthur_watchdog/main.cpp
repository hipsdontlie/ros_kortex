#include "include/inputs.hpp"
#include "include/perception.hpp"
#include "include/controls.hpp"
#include "include/hardware.hpp"
#include<arthur_watchdog/inputs.h>
#include<arthur_watchdog/perception.h>
#include<arthur_watchdog/controls.h>
#include<std_msgs/Bool.h>
#include <fstream>
#include<std_msgs/Empty.h>
#include<time.h>
#include <tf/transform_listener.h>

// bool faults_cleared = false;

// void clearFaults_callback(const std_msgs::Bool::ConstPtr& msg, ros::Publisher* clearFaults_pub)
// {
  
  
//   if(msg->data)
//   {
//     faults_cleared = true;
//     clearFaults_pub->publish(eStop_clearFaults_msg);
//   }
//   else
//   {
//     faults_cleared = false;
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pelvis_pose_listener");
  ros::NodeHandle n;

  std::ofstream fw("/home/mrsd-team-c/arthur_ws/ArthurLogs.txt", std::ofstream::out);

  // bool controller_flag = false;

  Inputs inputs;
  Perception perception;
  Controls controls;
  Hardware hardware;
  // bool printed = false;
  bool ui_clear_faults;
  bool eStop_published_bool = false;

  std_msgs::Bool eStop_bool_msg;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  perception.rmse_thresh->data = 1.0;


    //*********************************************** publishers ********************************************************
  
  ros::Publisher inputs_pub = n.advertise<arthur_watchdog::inputs>("input_health", 1);
  ros::Publisher percep_pub = n.advertise<arthur_watchdog::perception>("perception_health", 1);
  ros::Publisher controllerFlag_pub = n.advertise<std_msgs::Bool>("controller_flag", 1);
  ros::Publisher controlsMsg_pub = n.advertise<arthur_watchdog::controls>("controller_health", 1);
  ros::Publisher hardwareFlag_pub = n.advertise<std_msgs::Bool>("hardware_flag/command", 1);
  ros::Publisher eStop_pub = n.advertise<std_msgs::Empty>("my_gen3/in/emergency_stop", 1);
  ros::Publisher eStop_clearFaults_pub = n.advertise<std_msgs::Empty>("my_gen3/in/clear_faults", 1);
  ros::Publisher controller_clearFaults_pub = n.advertise<std_msgs::Empty>("controller_clear_fault", 1);
  ros::Publisher eStop_flag_pub = n.advertise<std_msgs::Bool>("eStop_flag", 1);
  ros::Time current_time_estop;

  //********************************************** subscribers *************************************************

  //inputs to the system subscribers
  // ros::Subscriber sub = n.subscribe("pelvis_pose", 1, &Inputs::pelvisCallback, &inputs);
  ros::Subscriber sub = n.subscribe("pelvis_pose", 1, &Inputs::pelvisCallback, &inputs);

  ros::Subscriber rp_sub = n.subscribe("probe_pose", 1, &Inputs::rpCallback, &inputs);

  ros::Subscriber ee_sub = n.subscribe("end_effector_pose", 1, &Inputs::eeCallback, &inputs);

  //perception subsystem subscriber
  ros::Subscriber rmse_sub = n.subscribe("percep_rmse", 1, &Perception::perception_eval, &perception);

  //controls subsystem subscribers
  ros::Subscriber errors_sub = n.subscribe("controls_error", 1, &Controls::error_check, &controls);
  
  ros::Subscriber singularity_sub = n.subscribe("controls_singularity", 1, &Controls::singularity_check, &controls);

  ros::Subscriber jlimits_sub = n.subscribe("controls_jlimits", 1, &Controls::joint_limits, &controls);

  ros::Subscriber controlFault_sub = n.subscribe("controller_fault", 1, &Controls::controller_fault, &controls);

  //hardware/end-effector subscribers
  // ros::Subscriber reamerSpeed_sub = n.subscribe("hardware_force/data", 1, &Hardware::reamer_speed, &hardware);

  // ros::Subscriber loadApplied_sub = n.subscribe("hardware_reamerSpeed/data", 1, &Hardware::load_applied, &hardware);

  ros::Subscriber reamPercent_sub = n.subscribe("hardware_reamPercent/data", 1, &Hardware::ream_percent, &hardware);

  // ros::Subscriber currentDrawn_sub = n.subscribe("hardware_current/data", 1, &Hardware::current_drawn, &hardware);

  // ros::Subscriber clearFaults_sub = n.subscribe<std_msgs::Bool>("clearFaults", 1, boost::bind(&clearFaults_callback, _1, &clearFaults_pub));





  // ros::Subscriber ee_control_sub = n.subscribe("control_error", 1000, &Controls::error_check, &controls);

  ros::Rate loop_rate(100);



  while(ros::ok())
  {
    arthur_watchdog::inputs input_msg;
    arthur_watchdog::perception percep_msg;
    arthur_watchdog::controls controls_msg;
    std_msgs::Bool controlsFlag_msg;
    std_msgs::Bool hardware_msg;
    std_msgs::Empty eStop_msg;
    std_msgs::Empty controller_clearFault_msg;
    std_msgs::Empty eStop_clearFaults_msg;
    eStop_msg = {};
    eStop_clearFaults_msg = {};
    controller_clearFault_msg = {};

  // *********************************** inputs ********************************************

    inputs.currTime_pelvis = ros::Time::now().toSec();
    inputs.currTime_ee = ros::Time::now().toSec();
    inputs.currTime_rp = ros::Time::now().toSec();
    controls.currTime_error = ros::Time::now().toSec();
    controls.currTime_jlimits = ros::Time::now().toSec();
    controls.currTime_singularity = ros::Time::now().toSec();


    try{
            transformStamped = tfBuffer.lookupTransform("base_link", "ee_marker_frame",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

    if(inputs.currTime_pelvis - inputs.prevTime_pelvis > ros::Duration(0.035).toSec())
    {
      // ROS_INFO("Pelvis not visible!\n");
      inputs.pelvis_visible = false;
      if (fw.is_open() && !inputs.pelvis_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Pelvis marker is not visible\n";
        inputs.pelvis_printed = true;
      }
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
      }

      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          n.setParam("ui_clear_faults", false);
          current_time_estop = ros::Time::now();
         
        }
      }
        
    }
    
    //if you stop receiving 
    if(inputs.currTime_ee - inputs.prevTime_ee > ros::Duration(0.035).toSec())
    {
      // ROS_INFO("End-effector not visible!\n");
      inputs.ee_visible = false;
      if (fw.is_open() && !inputs.ee_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "End-effector marker is not visible\n";
        inputs.ee_printed = true;
      }
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        std::cout<<"Estop bool ee: "<<eStop_published_bool<<std::endl;
      }

      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();


          std::cout<<"Estop bool ee: "<<eStop_published_bool<<std::endl;
        }
      }
    }

    if(inputs.currTime_rp - inputs.prevTime_rp > ros::Duration(0.035).toSec())
    {
      // ROS_INFO("Registration probe not visible!\n");
      inputs.probe_visible = false;
    }

    //************************ publish inputs message ******************************************
    input_msg.ee_visible = inputs.ee_visible;
    input_msg.pelvis_visible = inputs.pelvis_visible;
    input_msg.probe_visible = inputs.probe_visible;
    inputs_pub.publish(input_msg);

    if(inputs.pelvis_visible && inputs.pelvis_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "Pelvis is visible \n";

      inputs.pelvis_printed = false;
    }

    if(inputs.ee_visible && inputs.ee_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "End-effector is visible \n";

      inputs.ee_printed = false;
    }

    // ***************************************** end of inputs ********************************************

    // ********************************* perception ******************************************

    percep_msg.rmse_error = perception.rmse_error;
    // eStop_pub.publish(eStop_msg);
    percep_pub.publish(percep_msg);

    if (fw.is_open() && !perception.rmse_error && !perception.percep_printed)
    {
      // fw << inputs.pelvis_visible << "\n";
      fw << "Registration RMSE error is higher than 0.3 \n";
      perception.percep_printed = true;
    }

    // ************************************* end of perception **********************************************

    // std::cout<<"Control trans error: "<<controls.trans_bool<<std::endl; 
    // std::cout<<"Control orientation error: "<<controls.orien_bool<<std::endl; 
    // std::cout<<"Control singularity error: "<<controls.singularity_bool<<std::endl; 
    // std::cout<<"Control joint limits error: "<<controls.jlimits_bool<<std::endl; 
    
    // ******************************************* controls ******************************************************
    
    if (inputs.pelvis_visible && inputs.ee_visible && perception.rmse_error 
        && controls.trans_bool && controls.orien_bool && controls.jlimits_bool 
        && controls.singularity_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10) && !eStop_published_bool)
    // if (inputs.pelvis_visible && perception.rmse_error)
    {
      controls.controller_flag = true;
      hardware.hardware_flag = false;
      controlsFlag_msg.data = controls.controller_flag;
      // ROS_INFO("Setting controller flag to true");
      if (fw.is_open() && controls.controllerFlag_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controller flag set to true\n";
        controls.controllerFlag_printed = false;
      }

      if(!controls.controlsFault_bool)
        controller_clearFaults_pub.publish(controller_clearFault_msg);
    }
    else
    {
      controls.controller_flag = false;
      // hardware.hardware_flag = true;
      controlsFlag_msg.data = controls.controller_flag;
      if (fw.is_open() && !controls.controllerFlag_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controller flag couldn't be set to true because downstream processes are unhealthy\n";
        controls.controllerFlag_printed = true;
      }
    }


    if(controls.currTime_error - controls.prevTime_error > ros::Duration(1).toSec())
    {
      // ROS_INFO("Controller error publisher dropped below 30Hz!\n");
      controls.trans_bool = false;
      controls.orien_bool = false;
      if (fw.is_open() && !controls.error_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controller error publisher dropped below 30Hz\n";
        controls.error_printed = true;
      }
    }

    if(controls.currTime_singularity - controls.prevTime_singularity > ros::Duration(1).toSec())
    {
    // hardware.hardware_flag = true;
      controls.singularity_bool = false;
      controls.controller_flag = false;
      if (fw.is_open() && !controls.singularity_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controller singularity publisher dropped below 30Hz\n";
        controls.singularity_printed = true;
      }
    }

    if(controls.currTime_jlimits - controls.prevTime_jlimits > ros::Duration(1).toSec())
    {
      // ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
      controls.jlimits_bool = false;
      controls.controller_flag = false;
      if (fw.is_open() && !controls.jlimits_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controller joint limits publisher dropped below 30Hz\n";
        controls.jlimits_printed = true;
      }
    }
    
    //publish controller flag
    controls_msg.singularity_bool = controls.singularity_bool;
    controls_msg.jlimits_bool = controls.jlimits_bool;
    controls_msg.controlsFault_bool = controls.controlsFault_bool;
    controls_msg.orien_bool = controls.orien_bool;
    controls_msg.trans_bool = controls.trans_bool;

    controllerFlag_pub.publish(controlsFlag_msg);
    controlsMsg_pub.publish(controls_msg);

    if(controls.trans_bool == false)
    {
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        // std::cout<<"Estop bool trans error: "<<eStop_published_bool<<std::endl;
      }
      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();
        

          std::cout<<"Estop bool trans error: "<<eStop_published_bool<<std::endl;
        }
      }

      if(fw.is_open() && !controls.transError_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controls translation error is high\n";
        controls.transError_printed = true;
      }
    }   
    if(controls.trans_bool && fw.is_open() && controls.transError_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "Controls translation error is within limits\n";
      controls.transError_printed = false;
    }

    if(controls.orien_bool == false)
    {
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        // std::cout<<"Estop bool orien error: "<<eStop_published_bool<<std::endl;
      }
      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();


          std::cout<<"Estop bool orien error: "<<eStop_published_bool<<std::endl;
        }
      }
      
      if(fw.is_open() && !controls.orienError_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Controls orientation error is high\n";
        controls.orienError_printed = true;
      }
    } 

    if(controls.singularity_bool == false)
    {
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        // std::cout<<"Estop bool singularity: "<<eStop_published_bool<<std::endl;
      }
      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();


          std::cout<<"Estop bool singularity: "<<eStop_published_bool<<std::endl;
        }
      }
      
      if(fw.is_open() && !controls.singularity_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Arthur at singularity\n";
        controls.orienError_printed = true;
      }
    } 

    if(controls.jlimits_bool == false)
    {
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        // std::cout<<"Estop bool jlimits: "<<eStop_published_bool<<std::endl;
      }
      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();


          std::cout<<"Estop bool jlimits: "<<eStop_published_bool<<std::endl;
        }
      }
      
      if(fw.is_open() && !controls.jlimits_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Arthur at joint limits\n";
        controls.orienError_printed = true;
      }
    } 

    if(controls.orien_bool && fw.is_open() && controls.orienError_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "Controls orientation error is within limits\n";
      controls.orienError_printed = false;
    }

    if(controls.singularity_bool == false && fw.is_open() && !controls.singularityBool_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "Arthur at singularity\n";
      controls.singularityBool_printed = true;
    }   

    if(controls.singularity_bool && fw.is_open() && controls.singularityBool_printed)
    {
      time_t rawtime;
      struct tm * timeinfo;
      char st [128];
      
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
      fw << st <<"     ";
      fw << "Arthur out of singularity\n";
      controls.singularityBool_printed = false;
    } 

    if(controls.controlsFault_bool == false)
    {
      if(!eStop_published_bool && (ros::Time::now() - current_time_estop) > ros::Duration(10))
      {
        eStop_pub.publish(eStop_msg);
        eStop_published_bool = true;
        eStop_bool_msg.data = eStop_published_bool;
        hardware.hardware_flag = true;
        // std::cout<<"Estop bool control fault: "<<eStop_published_bool<<std::endl;
      }
      if(n.getParam("ui_clear_faults", ui_clear_faults))
      {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          std::cout<<"Estop bool control fault: "<<eStop_published_bool<<std::endl;
          current_time_estop = ros::Time::now();
        }
      }
    } 

  if (inputs.pelvis_visible && inputs.ee_visible && perception.rmse_error 
        && controls.trans_bool && controls.orien_bool && controls.jlimits_bool 
        && controls.singularity_bool && n.getParam("ui_clear_faults", ui_clear_faults))
        {
        if(ui_clear_faults)
        {
          eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
          n.setParam("ui_clear_faults", false);
          // ROS_INFO("Published estop!");
          // hardware.hardware_flag = false;
          eStop_published_bool = false;
          eStop_bool_msg.data = eStop_published_bool;
          current_time_estop = ros::Time::now();
        }
      }

  if (inputs.pelvis_visible && inputs.ee_visible && perception.rmse_error 
    && controls.trans_bool && controls.orien_bool && controls.jlimits_bool 
    && controls.singularity_bool && n.getParam("ui_clear_faults", ui_clear_faults))
    {
    if(ui_clear_faults)
    {
      eStop_clearFaults_pub.publish(eStop_clearFaults_msg);
      n.setParam("ui_clear_faults", false);
      // ROS_INFO("Published estop!");
      // hardware.hardware_flag = false;
      eStop_published_bool = false;
      eStop_bool_msg.data = eStop_published_bool;
      current_time_estop = ros::Time::now();
    }
  }

  // ****************************************** end of controls ********************************************************

  //*************************************** end-effector **********************************************

    if(hardware.hardware_flag && fw.is_open() && hardware.hardwareFlag_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Hardware flag set to true\n";
        hardware.hardwareFlag_printed = false;
      }

    if(hardware.hardware_flag==false && fw.is_open() && !hardware.hardwareFlag_printed)
      {
        time_t rawtime;
        struct tm * timeinfo;
        char st [128];
        
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
        fw << st <<"     ";
        fw << "Hardware flag set to false\n";
        hardware.hardwareFlag_printed = true;
      }

    // if(hardware.currTime_reamerSpeed - hardware.prevTime_reamerSpeed > ros::Duration(0.2).toSec())
    //   {
    //     // ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
    //     hardware.hardware_flag = true;
    //     if (fw.is_open() && !hardware.reamerSpeed_printed)
    //     {
    //       time_t rawtime;
    //       struct tm * timeinfo;
    //       char st [128];
          
    //       time (&rawtime);
    //       timeinfo = localtime (&rawtime);
    //       strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
    //       fw << st <<"     ";
    //       fw << "End-effector speed publisher dropped below 30Hz\n";
    //       hardware.reamerSpeed_printed = true;
    //     }
    //   }

    // if(hardware.currTime_loadApplied - hardware.prevTime_loadApplied > ros::Duration(0.2).toSec())
    //   {
    //     // ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
    //     hardware.hardware_flag = true;
    //     if (fw.is_open() && !hardware.loadApplied_printed)
    //     {
    //       time_t rawtime;
    //       struct tm * timeinfo;
    //       char st [128];
          
    //       time (&rawtime);
    //       timeinfo = localtime (&rawtime);
    //       strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
    //       fw << st <<"     ";
    //       fw << "End-effector load applied publisher dropped below 30Hz\n";
    //       hardware.loadApplied_printed = true;
    //     }
    //   }

    // if(hardware.currTime_currentDrawn - hardware.prevTime_currentDrawn > ros::Duration(0.2).toSec())
    //   {
    //     // ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
    //     hardware.hardware_flag = true;
    //     if (fw.is_open() && !hardware.currentDrawn_printed)
    //     {
    //       time_t rawtime;
    //       struct tm * timeinfo;
    //       char st [128];
          
    //       time (&rawtime);
    //       timeinfo = localtime (&rawtime);
    //       strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
    //       fw << st <<"     ";
    //       fw << "End-effector current drawn publisher dropped below 30Hz\n";
    //       hardware.currentDrawn_printed = true;
    //     }
    //   }

    if(hardware.currTime_reamPercent - hardware.prevTime_reamPercent > ros::Duration(0.2).toSec())
      {
        // ROS_INFO("Controller joint limits publisher dropped below 30Hz!\n");
        // hardware.hardware_flag = true;
        if (fw.is_open() && !hardware.reamPercent_printed)
        {
          time_t rawtime;
          struct tm * timeinfo;
          char st [128];
          
          time (&rawtime);
          timeinfo = localtime (&rawtime);
          strftime (st,128,"Date: %y-%m-%d  Time: %I:%M:%S",timeinfo);
          fw << st <<"     ";
          fw << "End-effector reaming percentage publisher dropped below 30Hz\n";
          hardware.reamPercent_printed = true;
        }
      }

    hardware_msg.data = hardware.hardware_flag;
    hardwareFlag_pub.publish(hardware_msg);

    eStop_flag_pub.publish(eStop_bool_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  // vector<string> file_to_check = {"pelvis_point_cloud"};

  // exists(file_to_check[0]) ? cout << "Exists\n" : cout << "Doesn't exist\n";

  return 0;
}
