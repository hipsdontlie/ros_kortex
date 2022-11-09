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
byte ENCAPinLinearActMotor = 19;
byte ENCBPinLinearActMotor = 18;

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
std_msgs::Int16 posLinearActuatorMotor;

// ROS publishers
// ros::Publisher pubCurrentSensor("hardware_current/data", &current);
// ros::Publisher pubLinearActMotorSpeed("linear_actuator_speed/data",&rpmLinearActuatorMotor);
// ros::Publisher pubReamerMotorSpeed("hardware_reamerSpeed/data",&rpmReamerMotor);
// ros::Publisher pubLinearActuatorMotorPos("linear_actuator_pos/data",&posLinearActuatorMotor);
// ros::Publisher pubForce("hardware_force/data",&force);
// ros::Publisher pubReamingPercentage("hardware_reamPercent/data",&reamingPercentage);

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
bool startReamingProcess = true;
bool dynamicCompensation = false;
bool reachedEnd = false;
bool posCalibration = false;

// Enum for which low level controller to use
enum controlType {positionControl, speedControl};  
controlType ReamerMotorControlType = speedControl;
controlType LinearActMotorControlType = positionControl;

// Enum for high level controller states 
enum states 
{
  CALIBRATE,
  WAITFORCMD,
  MOVEUNTILCONTACT,
  STARTREAMING,
  DYNAMICCOMP,
  TESTING,
  DONEREAMING
};

enum states currentState = CALIBRATE;

// High level controller parameters 
float forceSetPoint = 0.65;
float errorForce = 0; 
float errorPrevForce = 0;
float KpForce, KiForce, KdForce = 0;
float currentTimeForce, previousTimeForce = 0;
float errorProportionalForce, errorIntegralForce, errorDerivativeForce = 0;
float PIDOutForce = 0;
float deltaTForce = 0;
int cmd = 0;
float timerForce1 = 0;
float timerForce2 = 0;
float currentSampleTime = 0;
float forceValue = 0;
int linActPos = 0;

/*------------------------------------------ROS Callbacks -------------------------------------------*/ 

//Callback for reamer motor speed command 
// void changespeed_ReamerMotor(const std_msgs::Int16& cmdReamerMotorSpeed){

//   // TODO: Does this really need to be done? PID might take care of this. Test...
//   // reamerMotorSpeed = map(abs(velocity_ReamerMotor.data),0,601,0,255);
//   reamerMotorCommand = cmdReamerMotorSpeed.data;
//   //Switch Controllers
//   ReamerMotorControlType = speedControl;
//   currentState = TESTING;
//   reamerMotor.initPID();

// }

// //Callback for linear actuator motor speed command 
// void changespeed_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorSpeed){
//   linearActuatorMotorCommand = cmdLinearActuatorMotorSpeed.data;
//   currentState = TESTING;

//   //Switch Controllers
//   LinearActMotorControlType = speedControl;
//   linearActuator.initPID();
// }

// //Callback for reamer motor position command 
// void changepos_ReamerMotor(const std_msgs::Int16& cmdReamerMotorPos){

//   reamerMotorCommand = cmdReamerMotorPos.data;
//   currentState = TESTING;

//   //Switch Controllers
//   ReamerMotorControlType = positionControl;
//   reamerMotor.initPID();
// }

// //Callback for linear actuator motor position command 
// void changepos_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorPos){
//   linearActuatorMotorCommand = cmdLinearActuatorMotorPos.data;
//   currentState = TESTING;

//   //Switch Controllers
//   LinearActMotorControlType = positionControl;
//   linearActuator.initPID();
// }

// // Callback for start reaming command
// void getReamingCmd(const std_msgs::Bool& reamingCmd){
//   // nh.loginfo("Start reaming callback!");
//   if(reamingCmd.data == true)
//     startReamingProcess  = true;
//   else
//     startReamingProcess = false;
// }

// // //Callback for dynamic compensation command
// void getDynamicCompCmd(const std_msgs::Bool& dynamicCompensationCmd){
//   dynamicCompensation = dynamicCompensationCmd.data;
// }

// // Callback for calibration command
// // TODO: Ensure calibration process happens only once 
// // void getCalibrateCmd(const std_msgs::Bool& calibrateCmd){
// //   if(calibrateCmd.data == true)
// //     currentState = CALIBRATE;
// // }

// // Callback for watchdog command
// void getWatchdogCmd(const std_msgs::Bool& watchdogCmd){
//   if(watchdogCmd.data == true)
//     MotorControl::watchDogStop_ = true;
  
//   else
//     MotorControl::watchDogStop_ = false;
// }

/*------------------------------------------End of ROS Callbacks -------------------------------------------*/ 

// ROS Subscribers 
// ros::Subscriber<std_msgs::Int16> subReamerMotorVelCmd("reamer_speed/command", &changespeed_ReamerMotor);
// ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorVelCmd("linear_actuator_speed/command", &changespeed_LinearActuatorMotor);
// ros::Subscriber<std_msgs::Int16> subReamerMotorPosCmd("reamer_position/command", &changepos_ReamerMotor);
// ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorPosCmd("linear_actuator_position/command", &changepos_LinearActuatorMotor);
// ros::Subscriber<std_msgs::Bool> subReamingCmd("start_reaming/command", &getReamingCmd);
// ros::Subscriber<std_msgs::Bool> subDynamicCompCmd("start_dynamic_compensation/command", &getDynamicCompCmd);
// ros::Subscriber<std_msgs::Bool> subCalibrateCmd("start_ee_calibration/command", &getCalibrateCmd);
// ros::Subscriber<std_msgs::Bool> subWatchdogCmd("hardware_flag/command", &getWatchdogCmd);

// Boolean flags for low level control
volatile static bool MotorControl::watchDogStop_ = false;
volatile static bool MotorControl::limitSwitchStop_ = false;

// Limit switch interrupr service routine 
void triggerLimSwitch(){
  // nh.loginfo("Limit switch hit, stopping!");
  MotorControl::limitSwitchStop_ = true;
  if(!currentState == CALIBRATE){
    linearActuator.stopMotor();
    reamerMotor.stopMotor();
  }
}

// Calibrate Motors 

void calibrateMotors (){

  if(!(digitalRead(LimSwitchPin1))){
    reachedEnd = true;
  }

  if(digitalRead(LimSwitchPin1) && !reachedEnd){
    // nh.loginfo("Calibration in progress...");
    // Serial.println("Calibration in progress...");
    // reamerMotor.runMotorBackward(50);
    linearActuator.runMotorBackward(50);
  }
  

  else{

    if(reachedEnd && !posCalibration){
      // nh.loginfo("Reached end...");
      // reamerMotor.runMotorForwardUnsafe(255);
      linearActuator.runMotorForwardUnsafe(255);
      delay(500);
      // reamerMotor.stopMotor();
      // linearActuator.stopMotor();
      posCalibration = true;
      MotorControl::encoderValue_ReamerMotor_ = 0;
      MotorControl::encoderValue_LinearActMotor_ = 0;
    }

    else{
      ReamerMotorControlType = positionControl;
      LinearActMotorControlType = positionControl;
      // reamerMotorCommand = 500;
      linearActuatorMotorCommand = 350;
      // nh.loginfo("Repositioning...");
      if(linearActuator.getMotorPos() > 300)
        currentState =  WAITFORCMD; 
    }       
  }
}


float forceController(float forceValue){
  
    //Compute error terms 
    currentTimeForce = micros();
    deltaTForce = currentTimeForce - previousTimeForce; 
    previousTimeForce = currentTimeForce;

    errorForce = forceSetPoint-forceValue;
    errorProportionalForce = errorForce;
    errorDerivativeForce = (errorForce - errorPrevForce)/deltaTForce;
    errorIntegralForce += errorForce*deltaTForce;
    
    //Update previous values
    errorPrevForce = errorForce;
    //Get PID output 
    PIDOutForce = (KpForce*errorProportionalForce) + (KdForce*errorDerivativeForce) + (KiForce*errorIntegralForce);
      // Serial.print("Error proportional: ");
      // Serial.println(errorForce);
      //Threshold the output 
    return PIDOutForce;
  
}

void setup(){

    //Serial port for debugging
    Serial.begin(57600);

    //ROS Setup
    // nh.initNode();

    //Subscribers
    // nh.subscribe(subReamerMotorVelCmd);
    // nh.subscribe(subLinearActuatorMotorVelCmd);
    // nh.subscribe(subReamerMotorPosCmd);
    // nh.subscribe(subLinearActuatorMotorPosCmd);
    // nh.subscribe(subReamingCmd);
    // nh.subscribe(subDynamicCompCmd);
    // nh.subscribe(subCalibrateCmd);
    // nh.subscribe(subWatchdogCmd);

    //Publishers
    // nh.advertise(pubCurrentSensor);
    // nh.advertise(pubReamerMotorSpeed);

    // Motor control setup
    reamerMotor.setPIDVelConstants(0.15,0.03,0);
    reamerMotor.setPIDPosConstants(0.3,0,1);
    linearActuator.setPIDVelConstants(0.15,0.03,0);
    linearActuator.setPIDPosConstants(0.5,0,0.1);

    // Limit switch setup
    pinMode(LimSwitchPin1, INPUT_PULLUP);
    pinMode(LimSwitchPin2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin1), triggerLimSwitch, RISING);
    attachInterrupt(digitalPinToInterrupt(LimSwitchPin2), triggerLimSwitch, RISING);

  // High level controller gains 
    KpForce = 250;
    KdForce = 0;
    KiForce = 0; 
}

void loop(){

/*-------------------------------------HIGH LEVEL CONTROL-------------------------------------*/
  // Get the current readings from the current sensor 
  // nh.spinOnce();
  // float* forceValues;
  if(millis() - currentSampleTime > 10){
    forceValue = currSensor.getCurrent();
    linActPos = linearActuator.getMotorPos();
    currentSampleTime  = millis();
    Serial.print("--------------------------------------------------------Current: ");
    Serial.println(forceValue);
    Serial.print("Motor Position: ");
    Serial.println(linActPos);
  }

  // Serial.print("Current state is: ");
  // Serial.println(currentState);

  // int currentRaw = currSensor.getRaw();
  
  // Serial.println(currentRaw);
  // current.data = currentValue;
  // pubCurrentSensor.publish(&current);

  // if(!digitalRead(LimSwitchPin1))
    // Serial.println("Limit switch 1 is on!");

  // if(!digitalRead(LimSwitchPin2))
    // Serial.println("Limit switch 2 is on!");

  // Serial.print(currentState);
  switch (currentState) {
    
    // Calibrate linear actuator position 
    case CALIBRATE:
      Serial.println("Calibrate!");
      calibrateMotors();
      break;

    //Wait until you get the actuation signal from arm controller
    case WAITFORCMD: 
      
      // nh.loginfo("Waiting for command...");
      // Serial.println("Waiting...");
      reamerMotor.stopMotor();
      linearActuator.stopMotor();
      // Hold position at calibration position
      ReamerMotorControlType = speedControl;
      LinearActMotorControlType = positionControl;
      reamerMotorCommand = 0;
      linearActuatorMotorCommand = 300;
      if(startReamingProcess == true){
          currentState = MOVEUNTILCONTACT;
      }
      break;

    //Actuate motor until contact is made with the pelvis
    case MOVEUNTILCONTACT:

      // Serial.println("Move until contact!");
      // nh.loginfo("Moving until contact with bone...");

      if ((millis()-timerForce1) > 10){
        // Serial.print("Current: ");
        // Serial.println(forceValue);
        float forceOut = forceController(forceValue);
        Serial.print("===============================================================output from force controller: ");
        Serial.println(forceOut);
        LinearActMotorControlType = speedControl;
        ReamerMotorControlType = speedControl;
        linearActuatorMotorCommand = int(forceOut);
        reamerMotorCommand = 0;
        timerForce1 = millis();
      }

      // Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      if(forceValue >= forceSetPoint && ){
          // Serial.println(currentState);
          // currentState = STARTREAMING;
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Force threshold Exceeded! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          Serial.print("Current at force thresh exceeded: ");
          Serial.println(forceValue);
          Serial.print("Current at forcesetpoint: ");
          Serial.println(forceSetPoint);
          Serial.print("-------------------------------------------State changed to : ");
          currentState = DONEREAMING;
          Serial.println(currentState);
          linearActuatorMotorCommand = 0;
      }

      break;

    //Ream as long as pelvis error is within thresholds and goal has not been reached 
    case STARTREAMING:
      // Serial.print("####################################################### Current force: ");
      // Serial.println(forceValue);
      Serial.println("Start reaming...");
      ReamerMotorControlType = speedControl;
      LinearActMotorControlType = speedControl;
      if ((millis()-timerForce2) > 400){
        linearActuatorMotorCommand = forceController(forceValue);
        reamerMotorCommand = 300;
        timerForce2 = millis();
      }
      
      break;

    // Dynamic compensation - change state back to 1 after performing compensation routine 
    case DYNAMICCOMP:
      
      Serial.println("Dynamic comp!");
      // currentState = WAITFORCMD;
      break;

    // Goal has been reached, stop reaming! 
    case DONEREAMING: 
      
      // Serial.println("Done reaming!");
      reamerMotor.stopMotor();
      linearActuator.stopMotor();
      // nh.loginfo("Done reaming!");
      break;

    case TESTING:
      // Serial.println("Testing!");
      ReamerMotorControlType = speedControl;
      reamerMotorCommand = 100;
      // nh.loginfo("Testing state...");
      break;
  
    default:
      
      // nh.loginfo("Invalid state, stopping reaming!");
      // Serial.println("Invalid state!");
      reamerMotor.stopMotor();
      linearActuator.stopMotor();
      break;
  }


/*-------------------------------------LOW LEVEL CONTROL-------------------------------------*/


  // Serial.println("Low level controller!");
  // Set LimitSwitch interrupt flag to false
  if(digitalRead(LimSwitchPin1) && digitalRead(LimSwitchPin2))
    // Serial.println("Both limit swtiches not engaged!");
    MotorControl::limitSwitchStop_ = false;

  
  if(!digitalRead(LimSwitchPin1) || !digitalRead(LimSwitchPin2)){
    MotorControl::limitSwitchStop_ = true;
    Serial.println("One of the limit switches engaged!");
  }   

  // Reamer Motor Speed Control
  if ((((millis()-rpmTimerM1)) > 400) && (ReamerMotorControlType == speedControl)){
    float rpm = reamerMotor.getMotorRPM();
    // reamerMotor.pidSpeedControl(reamerMotorCommand);
    reamerMotor.pidSpeedControl(reamerMotorCommand);
    rpmTimerM1 = millis();

    // Publish the rpm of the reamer motor
    // rpmReamerMotor.data = rpm;
    // pubReamerMotorSpeed.publish(&rpmReamerMotor);
  }

  // Linear Actuator Motor Speed Control
  if ((((millis()-rpmTimerM2)) > 400) && (LinearActMotorControlType == speedControl)){
    float rpm = linearActuator.getMotorRPM();
    linearActuator.pidSpeedControl(linearActuatorMotorCommand);
    rpmTimerM2 = millis();
    // nh.loginfo("Low level speed control...");
    // Publish the rpm of the LinearAct motor
    // rpmLinearActuatorMotor.data = rpm;
    // pubLinearActMotorSpeed.publish(&rpmLinearActuatorMotor);
  }


  // Reamer Motor Position Control
  if ((((millis()-rpmTimerM1)) > 400) && (ReamerMotorControlType == positionControl)){
    int currPos = reamerMotor.getMotorPos();
    int posCurr = reamerMotor.pidPositionControl(reamerMotorCommand);
    rpmTimerM1 = millis();
  }

  // Linear Actuator Motor Position Control
  if ((((millis()-rpmTimerM2)) > 400) && (LinearActMotorControlType == positionControl)){
    // nh.loginfo("Linear Actuator Low level Position control...");
    // Serial.println(linearActuatorMotorCommand);

    int currPos = linearActuator.getMotorPos();
    linearActuator.pidPositionControl(linearActuatorMotorCommand);
    rpmTimerM2 = millis();

    // Publish the rpm of the reamer motor
    // posLinearActuatorMotor.data = currPos;
    // pubLinearActuatorMotorPos.publish(&posLinearActuatorMotor);
  }


}

