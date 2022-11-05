/*
Main high level state machine and low level controls
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
volatile static byte MotorControl::ENCB_Pin_ReamerMotor_ = ENCBPinReamerMotor;
volatile static byte MotorControl::ENCA_Pin_ReamerMotor_ = ENCAPinReamerMotor;
volatile static byte MotorControl::ENCB_Pin_LinearActMotor_ = ENCBPinLinearActMotor;
volatile static byte MotorControl::ENCA_Pin_LinearActMotor_ = ENCAPinLinearActMotor;

// ROS Definitions
ros::NodeHandle nh;

// ROS messages 
std_msgs::Float64 current;
std_msgs::Float64 rpmReamerMotor;
std_msgs::Float64 force;
std_msgs::Float64 rpmLinearActuatorMotor;
std_msgs::Float64 reamingPercentage; 

// ROS publishers
ros::Publisher pubCurrentSensor("hardware_current/data", &current);
ros::Publisher pubLinearActMotorSpeed("linear_actuator_speed/data",&rpmLinearActuatorMotor);
ros::Publisher pubReamerMotorSpeed("hardware_reamerSpeed/data",&rpmReamerMotor);
ros::Publisher pubForce("hardware_force/data",&force);
ros::Publisher pubReamingPercentage("hardware_reamPercent/data",&reamingPercentage);

//Current sensor 
currentSensor currSensor(currentSensorPin);

//Reamer motor 
MotorControl reamerMotor(PWMPinReamerMotor , DIRPinReamerMotor, 1);
int reamerMotorCommand = 0;

//Linear Actuator motor 
MotorControl linearActuator(PWMPinLinearActMotor, DIRPinLinearActMotor, 2);
int linearActuatorMotorCommand = 0;

//Low level controller timers 
unsigned long rpmTimerM1 = 0;
unsigned long rpmTimerM2 = 0;

// Controller flags
bool startReaming = false;
bool dynamicCompensation = false;

// Enum for which low level controller to use
enum controlType {positionControl, speedControl};  
controlType ReamerMotorControlType = speedControl;
controlType LinearActMotorControlType = speedControl;

/*------------------------------------------ROS Callbacks -------------------------------------------*/ 

//Callback for reamer motor speed command 
void changespeed_ReamerMotor(const std_msgs::Int16& cmdReamerMotorSpeed){

  // TODO: Does this really need to be done? PID might take care of this. Test...
  // reamerMotorSpeed = map(abs(velocity_ReamerMotor.data),0,601,0,255);
  reamerMotorCommand = cmdReamerMotorSpeed.data;
  //Switch Controllers
  ReamerMotorControlType = speedControl;
  reamerMotor.initPID();

}

//Callback for linear actuator motor speed command 
void changespeed_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorSpeed){
  linearActuatorMotorCommand = cmdLinearActuatorMotorSpeed.data;

  //Switch Controllers
  LinearActMotorControlType = speedControl;
  linearActuator.initPID();
}

//Callback for reamer motor position command 
void changepos_ReamerMotor(const std_msgs::Int16& cmdReamerMotorPos){

  
  reamerMotorCommand = cmdReamerMotorPos.data;

  //Switch Controllers
  ReamerMotorControlType = positionControl;
  reamerMotor.initPID();
}

//Callback for linear actuator motor position command 
void changepos_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorPos){
  linearActuatorMotorCommand = cmdLinearActuatorMotorPos.data;

  //Switch Controllers
  LinearActMotorControlType = positionControl;
  linearActuator.initPID();
}

// Callback for start reaming command
void getReamingCmd(const std_msgs::Bool& reamingCmd){
  if(reamingCmd.data == true)
    startReaming  = true;
  else
    startReaming = false;
}

//Callback for dynamic compensation command
void getDynamicCompCmd(const std_msgs::Bool& dynamicCompensationCmd){
  dynamicCompensation = dynamicCompensationCmd.data;
}

// Callback for calibration command
// TODO: Ensure calibration process happens only once 
void getCalibrateCmd(const std_msgs::Bool& calibrateCmd){
  if(calibrateCmd.data == true){
    reamerMotor.calibrate();
    linearActuator.calibrate();
  }
}

// Callback for watchdog command
void getWatchdogCmd(const std_msgs::Bool& watchdogCmd){
  if(watchdogCmd.data == true)
    MotorControl::watchDogStop_ = true;
  
  else
    MotorControl::watchDogStop_ = false;
}


/*------------------------------------------End of ROS Callbacks -------------------------------------------*/ 

// ROS Subscribers 
ros::Subscriber<std_msgs::Int16> subReamerMotorVelCmd("reamer_speed/command", &changespeed_ReamerMotor);
ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorVelCmd("linear_actuator_speed/command", &changespeed_LinearActuatorMotor);
ros::Subscriber<std_msgs::Int16> subReamerMotorPosCmd("reamer_position/command", &changepos_ReamerMotor);
ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorPosCmd("linear_actuator_position/command", &changepos_LinearActuatorMotor);
ros::Subscriber<std_msgs::Bool> subReamingCmd("start_reaming/command", &getReamingCmd);
ros::Subscriber<std_msgs::Bool> subDynamicCompCmd("start_dynamic_compensation/command", &getDynamicCompCmd);
ros::Subscriber<std_msgs::Bool> subCalibrateCmd("start_ee_calibration/command", &getCalibrateCmd);
ros::Subscriber<std_msgs::Bool> subWatchdogCmd("hardware_flag/command", &getWatchdogCmd);

// Boolean flags for low level control
volatile static bool MotorControl::watchDogStop_ = false;
volatile static bool MotorControl::limitSwitchStop_ = false;

// Limit switch interrupr service routine 
void triggerLimSwitch(){
  // nh.loginfo("Limit switch hit, stopping!");
  MotorControl::limitSwitchStop_ = true;
  reamerMotor.stopMotor();
  linearActuator.stopMotor();
}


void setup(){

    //Serial port for debugging
    Serial.begin(57600);

    //ROS Setup
    nh.initNode();

    //Subscribers
    nh.subscribe(subReamerMotorVelCmd);
    nh.subscribe(subLinearActuatorMotorVelCmd);
    nh.subscribe(subReamerMotorPosCmd);
    nh.subscribe(subLinearActuatorMotorPosCmd);
    nh.subscribe(subReamingCmd);
    nh.subscribe(subDynamicCompCmd);
    nh.subscribe(subCalibrateCmd);
    nh.subscribe(subWatchdogCmd);


    nh.advertise(pubCurrentSensor);
    nh.advertise(pubReamerMotorSpeed);

    // Motor control setup
    reamerMotor.setPIDVelConstants(0.15,0.03,0);
    reamerMotor.setPIDPosConstants(0.3,0,1);
    linearActuator.setPIDVelConstants(0.15,0.03,0);
    linearActuator.setPIDPosConstants(0.3,0,0.1);

    // Limit switch setup
    pinMode(LimSwitchPin1, INPUT_PULLUP);
    pinMode(LimSwitchPin2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin1), triggerLimSwitch, RISING);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin2), triggerLimSwitch, RISING);
}

void loop(){

/*-------------------------------------HIGH LEVEL CONTROL-------------------------------------*/




/*-------------------------------------LOW LEVEL CONTROL-------------------------------------*/

  // Get the current readings from the current sensor 
  nh.spinOnce();
  // float current_value = currSensor.getCurrent();
  // current.data = current_value;
  // pubCurrentSensor.publish(&current);
  bool limswitch1 = digitalRead(LimSwitchPin1);
  bool limswitch2 = digitalRead(LimSwitchPin2);
  // if(!limswitch1)
  //   Serial.println("Lim switch 1");
  // if(!limswitch2)
  //   Serial.println("Lim switch 2");

  // Set LimitSwitch interrupt flag to false
  if(digitalRead(LimSwitchPin1) && digitalRead(LimSwitchPin2))
    MotorControl::limitSwitchStop_ = false;
  
  
  // Reamer Motor Speed Control
  if ((((millis()-rpmTimerM1)) > 400) && (ReamerMotorControlType == speedControl)){
    float rpm = reamerMotor.getMotorRPM();
    reamerMotor.pidSpeedControl(reamerMotorCommand);
    rpmTimerM1 = millis();
    
    // Serial.print("RPM: ");
    // Serial.println(rpm);


    // Publish the rpm of the reamer motor
    rpmReamerMotor.data = rpm;
    pubReamerMotorSpeed.publish(&rpmReamerMotor);
  }

  // Linear Actuator Motor Speed Control
  if ((((millis()-rpmTimerM2)) > 400) && (LinearActMotorControlType == speedControl)){
    float rpm = linearActuator.getMotorRPM();
    linearActuator.pidSpeedControl(linearActuatorMotorCommand);
    Serial.print("RPM: ");
    Serial.println(rpm);
    rpmTimerM2 = millis();

    // // Publish the rpm of the LinearAct motor
    // rpmLinearActuatorMotor.data = rpm;
    // pubLinearActMotorSpeed.publish(&rpmLinearActuatorMotor);
  }


  // Reamer Motor Position Control
  if ((((millis()-rpmTimerM1)) > 400) && (ReamerMotorControlType == positionControl)){

    float pidout = reamerMotor.pidPositionControl(reamerMotorCommand);
    rpmTimerM1 = millis();

    // Serial.print("Error: ");
    // Serial.println(pidout);


    // // Publish the rpm of the reamer motor
    // rpmReamerMotor.data = rpm;
    // pubReamerMotorSpeed.publish(&rpmReamerMotor);
  }

  // Linear Actuator Motor Position Control
  if ((((millis()-rpmTimerM2)) > 400) && (LinearActMotorControlType == positionControl)){

    float pidout = linearActuator.pidPositionControl(linearActuatorMotorCommand);
    rpmTimerM2 = millis();
    Serial.print(" Linear Actuator Current Position: ");
    Serial.println(linearActuator.getMotorPos());

    Serial.print("Error: ");
    Serial.println(pidout);


    // Publish the rpm of the reamer motor
    // rpmLinearActMotorControlType.data = rpm;
    // pubLinearActMotorControlTypeSpeed.publish(&rpmLinearActMotorControlType);
  }


}

