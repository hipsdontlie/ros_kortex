/*
Main high level and low level control state machine
Author: Kaushik Balasundar
*/
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "currentSensor.h"
#include "motorControl.h"

//Current sensor pin 
byte currentSensorPin = A0;

//Reamer Motor Pins 
byte PWMPinReamerMotor = 6;
byte DIRPinReamerMotor = 5;
byte ENCAPinReamerMotor = 21;
byte ENCBPinReamerMotor = 20;

//Linear Actuator Motor Pins
byte PWMPinLinearActMotor = 7;
byte DIRPinLinearActMotor = 8;
byte ENCAPinLinearActMotor = 18;
byte ENCBPinLinearActMotor = 19;

//Limit switch pins
byte LimSwitchPin1 = 2;
byte LimSwitchPin2 = 3;

//Initialize motor interrupt pins 
volatile static int MotorControl::encoderValue_ReamerMotor_ = 0;
volatile static int MotorControl::encoderValue_LinearActMotor_ = 0;
volatile static byte MotorControl::ENCBPinReamerMotor_ = ENCBPinReamerMotor;
volatile static byte MotorControl::ENCAPinReamerMotor_ = ENCAPinReamerMotor;
volatile static byte MotorControl::ENCBPinLinearActMotor_ = ENCBPinLinearActMotor;
volatile static byte MotorControl::ENCAPinLinearActMotor_ = ENCA_Pin_LinearActMotor;

// ROS Definitions
ros::NodeHandle nh;

// ROS messages 
std_msgs::Float64 curr_1;
std_msgs::Float64 rpmReamerMotor;
std_msgs::Float64 rpmLinearActuatorMotor;

// ROS publishers
ros::Publisher pubCurrentSensor("hardware_current/data", &curr_1);
ros::Publisher pubLinearActMotorSpeed("linear_actuator_speed/data",&rpmLinearActuatorMotor);
ros::Publisher pubReamerMotorSpeed("hardware_reamerSpeed/data",&rpmReamerMotor);
ros::Publisher pubForce("hardware_force/data",&force);
ros::Publisher pubReamingPercentage("hardware_reamPercent/data",&force);

//Current sensor 
currentSensor currSensor(currentSensorPin);

//Reamer motor 
MotorControl reamerMotor(PWM_Pin_ReamerMotor , DIR_Pin_ReamerMotor, 1);

//Linear Actuator motor 
MotorControl linearActuator(PWM_Pin_LinearActMotor, DIR_Pin_LinearActMotor, 2);

//Low level controller timers 
unsigned long rpmTimerM1 = 0;
unsigned long rpmTimerM2 = 0;

// ROS Subscribers 
ros::Subscriber<std_msgs::Int16> subReamerMotor("reamer_speed/command", &changespeed_ReamerMotor);
ros::Subscriber<std_msgs::Int16> subLinearActuatorMotor("linear_actuator_speed/command", &changespeed_LinearActuatorMotor);
ros::Subscriber<std_msgs::Bool> subReamingCmd("start_reaming/command", &getReamingCmd);
ros::Subscriber<std_msgs::Bool> subDynamicCompCmd("start_dynamic_compensation/command", &getDynamicCompCmd);
ros::Subscriber<std_msgs::Bool> subCalibrateCmd("start_ee_calibration/command", &getCalibrationCmd);
ros::Subscriber<std_msgs::Bool> subWatchdogCmd("hardware_flag/command", &getWatchDogCmd);

// Boolean flags for low level control
volatile static bool MotorControl::watchDogStop_ = false;
volatile static bool MotorControl::limitSwitchStop_ = false;

// Limit switch interrupr service routine 
void triggerLimSwitch(){
  nh.loginfo("Limit switch hit, stopping!");
  MotorControl::LimitSwitchStop_ = true;
  reamerMotor.stop();
  linearActuator.stop();
}

void setup(){

    //Serial port for debugging
    Serial.begin(57600);

    //ROS Setup
    nh.initNode();
    nh.advertise(pubCurrentSensor);

    // Motor control setup
    reamerMotor.setPIDVelConstants(0.15,0.03,0);
    reamerMotor.setPIDPosConstants(0.15,0.03,0);
    LinearActMotor.setPIDVelConstants(0.15,0.03,0);
    LinearActMotor.setPIDPosConstants(0.15,0.03,0);

    // Limit switch setup
    pinMode(LimSwitchPin1, INPUT_PULLUP);
    pinMode(LimSwitchPin2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin1), triggerLimSwitch, HIGH);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin2), triggerLimSwitch, HIGH);

}

void loop(){







/*--------------------------LOW LEVEL CONTOROL-----------------------------*/

  // Get the current readings from the current sensor 
  nh.spinOnce();
  float current = currSensor.getCurrent();
  curr_1.data = current;
  pub_C1.publish(&curr_1);

  // Set LimitSwitch interrupt flag to false
  MotorControl::LimitSwitchStop_ = false;

  // Reamer Motor Speed Control
  if (((millis()-rpmTimerM1)) > 400){
    float rpm = reamerMotor.pidVelocityControl(100);
    rpmTimerM1 = millis();

    // Publish the rpm of the reamer motor
    rpmReamerMotor.data = rpm;
    pubReamerMotorSpeed.publish(&rpmReamerMotor);
  }

  // Reamer Motor Speed Control
  if (((millis()-rpmTimerM2)) > 400){
    float rpm = LinearActMotor.pidVelocityControl(100);
    rpmTimerM2 = millis();

    // Publish the rpm of the LinearAct motor
    pubLinearActMotorSpeed.data = rpm;
    pubLinearActMotorSpeed.publish(&rpmLinearActuatorMotor);
  }



}

