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
#include <movingAvg.h>
#include <Encoder.h>

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

//Encoders
Encoder reamerMotorEnc(ENCBPinReamerMotor, ENCAPinReamerMotor);
Encoder linearActMotorEnc(ENCBPinLinearActMotor, ENCAPinLinearActMotor);
float REAMERMOTORPPR = 659.232;
float LINEARACTMOTORPPR = 230.7;

//Limit switch pins
byte LimSwitchPin1 = 2;
byte LimSwitchPin2 = 3;

//Initialize motor interrupt pins
// volatile static int MotorControl::encoderValue_ReamerMotor_ = 0;
// volatile static int MotorControl::encoderValue_LinearActMotor_ = 0;
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
std_msgs::Int16 controllerStatus;


// ROS publishers
ros::Publisher pubCurrentSensor("hardware_current/data", &current);
// ros::Publisher pubLinearActMotorSpeed("linear_actuator_speed/data",&rpmLinearActuatorMotor);
// ros::Publisher pubReamerMotorSpeed("hardware_reamerSpeed/data",&rpmReamerMotor);
// ros::Publisher pubLinearActuatorMotorPos("linear_actuator_pos/data",&posLinearActuatorMotor);
// ros::Publisher pubForce("hardware_force/data",&force);
ros::Publisher pubReamingPercentage("hardware_reamPercent/data",&reamingPercentage);
// ros::Publisher pubControllerStatus("hardware_controllerStatus/data", &controllerStatus);

//Current sensor
currentSensor currSensor(currentSensorPin);
movingAvg currentSensorAvg(10);

//Reamer motor
MotorControl reamerMotor(PWMPinReamerMotor, DIRPinReamerMotor, 1, REAMERMOTORPPR);
int reamerMotorCommand = 0;

//Linear Actuator motor
MotorControl linearActuator(PWMPinLinearActMotor, DIRPinLinearActMotor, 2, LINEARACTMOTORPPR);
int linearActuatorMotorCommand = 0;

//Low level controller timers
unsigned long rpmTimerM1 = 0;
unsigned long rpmTimerM2 = 0;

// Controller flags
bool startReamingProcess = false;
bool dynamicCompensation = false;
bool reachedEnd = false;
bool posCalibration = false;
bool reamAtEndPoint = false;
bool reamAtEndPointTimerFlag = false;
bool doneReaming = false;

// Enum for which low level controller to use
enum controlType { positionControl,
                   speedControl };
controlType ReamerMotorControlType = speedControl;
controlType LinearActMotorControlType = positionControl;

// Enum for high level controller states
enum states {
  CALIBRATE,
  WAITFORCMD,
  MOVEUNTILCONTACT,
  STARTREAMING,
  DYNAMICCOMP,
  TESTING,
  HANDLELIMSWITCH,
  DONEREAMING
};

enum states currentState = CALIBRATE;
enum states previousState = CALIBRATE;

// High level controller parameters
float forceSetPoint = 0.4;
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
float forceMovingAvg = 0;
int forceValueRaw = 0;
int reamingEndPoint = 40;
int reamingStartPoint = 30;
float prevrpm1, prevrpm2 = 0;
float startReamingTimer = 0;
float reamAtEndPointTimer = 0;

/*------------------------------------------ROS Callbacks -------------------------------------------*/

//Callback for reamer motor speed command
void changespeed_ReamerMotor(const std_msgs::Int16& cmdReamerMotorSpeed) {

  // TODO: Does this really need to be done? PID might take care of this. Test...
  // reamerMotorSpeed = map(abs(velocity_ReamerMotor.data),0,601,0,255);
  reamerMotorCommand = cmdReamerMotorSpeed.data;
  //Switch Controllers
  ReamerMotorControlType = speedControl;
  currentState = TESTING;
  reamerMotor.initPID();
}

//Callback for linear actuator motor speed command
void changespeed_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorSpeed) {
  linearActuatorMotorCommand = cmdLinearActuatorMotorSpeed.data;
  currentState = TESTING;

  //Switch Controllers
  LinearActMotorControlType = speedControl;
  linearActuator.initPID();
}

//Callback for reamer motor position command
void changepos_ReamerMotor(const std_msgs::Int16& cmdReamerMotorPos) {

  reamerMotorCommand = cmdReamerMotorPos.data;
  currentState = TESTING;

  //Switch Controllers
  ReamerMotorControlType = positionControl;
  reamerMotor.initPID();
}

//Callback for linear actuator motor position command
void changepos_LinearActuatorMotor(const std_msgs::Int16& cmdLinearActuatorMotorPos) {
  linearActuatorMotorCommand = cmdLinearActuatorMotorPos.data;
  currentState = TESTING;

  //Switch Controllers
  LinearActMotorControlType = positionControl;
  linearActuator.initPID();
}

// Callback for start reaming command
void getReamingCmd(const std_msgs::Bool& reamingCmd) {
  if (reamingCmd.data == true && !dynamicCompensation) {
    // nh.loginfo("Start reaming!");
    startReamingProcess = true;
    // currentState = MOVEUNTILCONTACT;
  }

  else {
    startReamingProcess = false;
    // currentState = WAITFORCMD;
  }
}

// Callback for dynamic compensation command
void getDynamicCompCmd(const std_msgs::Bool& dynamicCompensationCmd) {
  if (dynamicCompensationCmd.data) {
    // nh.loginfo("Started Dynamic Compensation!");
    dynamicCompensation = true;
    if(!doneReaming)
      currentState = DYNAMICCOMP;
  }

  else 
    dynamicCompensation = false;
    
}

// Callback for watchdog command
void getWatchdogCmd(const std_msgs::Bool& watchdogCmd) {

  if (watchdogCmd.data == true) {
    MotorControl::watchDogStop_ = true;
    currentState = WAITFORCMD;
  }

  else {
    MotorControl::watchDogStop_ = false;
  }
}

/*------------------------------------------End of ROS Callbacks -------------------------------------------*/
// ROS Subscribers
// ros::Subscriber<std_msgs::Int16> subReamerMotorVelCmd("reamer_speed/command", &changespeed_ReamerMotor);
// ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorVelCmd("linear_actuator_speed/command", &changespeed_LinearActuatorMotor);
// ros::Subscriber<std_msgs::Int16> subReamerMotorPosCmd("reamer_position/command", &changepos_ReamerMotor);
// ros::Subscriber<std_msgs::Int16> subLinearActuatorMotorPosCmd("linear_actuator_position/command", &changepos_LinearActuatorMotor);
ros::Subscriber<std_msgs::Bool> subReamingCmd("start_reaming/command", &getReamingCmd);
ros::Subscriber<std_msgs::Bool> subDynamicCompCmd("start_dynamic_compensation/command", &getDynamicCompCmd);
ros::Subscriber<std_msgs::Bool> subWatchdogCmd("hardware_flag/command", &getWatchdogCmd);

/*------------------------------------------Helper Functions-------------------------------------------*/


// Boolean flags for low level control
volatile static bool MotorControl::watchDogStop_ = false;
volatile static bool MotorControl::limitSwitchStop1_ = false;
volatile static bool MotorControl::limitSwitchStop2_ = false;


// Limit switch interrupr service routine
void triggerLimSwitch1() {

  MotorControl::limitSwitchStop1_ = true;

  if (!currentState == CALIBRATE){
    
    currentState = HANDLELIMSWITCH;
  }

}

void resetGains(){

  //Position control gains 
  linearActuator.errorIntegralPos_ = 0;
  linearActuator.errorProportionalPos_ = 0;
  linearActuator.errorDerivativePos_ = 0;
  reamerMotor.errorProportionalPos_ = 0; 
  reamerMotor.errorDerivativePos_ = 0; 
  reamerMotor.errorIntegralPos_ = 0; 

  //Velocity control gains
  linearActuator.errorIntegralVel_ = 0;
  linearActuator.errorProportionalVel_ = 0;
  linearActuator.errorDerivativeVel_ = 0;
  reamerMotor.errorProportionalVel_ = 0; 
  reamerMotor.errorDerivativeVel_ = 0; 
  reamerMotor.errorIntegralVel_ = 0; 
}

void triggerLimSwitch2() {

  MotorControl::limitSwitchStop2_ = true;

  if (!currentState == CALIBRATE){
    
    currentState = HANDLELIMSWITCH;
  }
}

// Ticks to distance conversions

double ticksTomm(int ticks) {
  double mm = 5.0 * double(ticks) / (230.7);
  return mm;
}

int mmtoTicks(double mm) {

  int ticks = mm * (230.7) / 5.0;
  return ticks;
}

double rpmTommPerSec(double rpm, double pitch) {
  return rpm * (pitch / (60.0));
}

double mmPerSecToRpm(double mmpersec, double ppr, double pitch) {
  return mmpersec / (pitch / (60.0));
}


// Calibrate Motors

void calibrateMotors() {

  if (!(digitalRead(LimSwitchPin1))) {
    reachedEnd = true;
  }

  if (digitalRead(LimSwitchPin1) && !reachedEnd) {
    LinearActMotorControlType = speedControl;
    linearActuatorMotorCommand = -200;
  }


  else {

    if (reachedEnd && !posCalibration) {
      // nh.loginfo("Reached end...");
      // reamerMotor.runMotorForwardUnsafe(255);
      // LinearActMotorControlType = positionControl;
      // linearActuatorMotorCommand =200;
      // linearActuatorMotorCommand = 0;
      reamerMotorEnc.write(0);
      linearActMotorEnc.write(0);
      linearActuator.runMotorForwardUnsafe(100);
      delay(200);
      // reamerMotor.stopMotor();
      // linearActuator.stopMotor();
      posCalibration = true;
      
    }

    else {
      // ReamerMotorControlType = positionControl;
      LinearActMotorControlType = positionControl;
      // reamerMotorCommand = 500;
      linearActuatorMotorCommand = 5;
      // nh.loginfo("Repositioning...");
      if (linearActMotorEnc.read() >= ticksTomm(5.0))
        currentState = WAITFORCMD;
    }
  }
}


float forceController(float forceValue) {

  //Compute error terms
  currentTimeForce = micros();
  deltaTForce = currentTimeForce - previousTimeForce;
  previousTimeForce = currentTimeForce;

  errorForce = forceSetPoint - forceValue;
  errorProportionalForce = errorForce;
  errorDerivativeForce = (errorForce - errorPrevForce) / deltaTForce;
  errorIntegralForce += errorForce * deltaTForce;

  //Update previous values
  errorPrevForce = errorForce;
  //Get PID output
  PIDOutForce = (KpForce * errorProportionalForce) + (KdForce * errorDerivativeForce) + (KiForce * errorIntegralForce);
  // Serial.print("Error proportional: ");
  // Serial.println(errorForce);
  //Threshold the output
  return PIDOutForce;
}

double getReamingPercentage() {
  double reamingPercentage = (ticksTomm(linearActMotorEnc.read()/reamingEndPoint)) * 100;
  return reamingPercentage;
}

void setup() {

  //Serial port for debugging
  Serial.begin(57600);
  // currentSensorAvg.begin();
  //ROS Setup
  nh.initNode();

  //Subscribers
  // nh.subscribe(subReamerMotorVelCmd);
  // nh.subscribe(subLinearActuatorMotorVelCmd);
  // nh.subscribe(subReamerMotorPosCmd);
  // nh.subscribe(subLinearActuatorMotorPosCmd);
  nh.subscribe(subReamingCmd);
  nh.subscribe(subDynamicCompCmd);
  nh.subscribe(subWatchdogCmd);

  //Publishers
  nh.advertise(pubCurrentSensor);
  // nh.advertise(pubReamerMotorSpeed);
  nh.advertise(pubReamingPercentage);
  // nh.advertise(pubControllerStatus);

  // Motor control setup
  reamerMotor.setPIDVelConstants(0.22, 0.67, 0);
  reamerMotor.setPIDPosConstants(0.6, 0.15, 0.1);
  linearActuator.setPIDVelConstants(1.54, 0, 0.1);
  linearActuator.setPIDPosConstants(0.20, 0.000001, 1.5);

  // Limit switch setup
  pinMode(LimSwitchPin1, INPUT_PULLUP);
  pinMode(LimSwitchPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LimSwitchPin1), triggerLimSwitch1, RISING);
  attachInterrupt(digitalPinToInterrupt(LimSwitchPin2), triggerLimSwitch2, RISING);

  // High level controller gains
  KpForce = 40;
  KdForce = 1;
  KiForce = 0;
}

void loop() {

  // Get the current readings from the current sensor
  nh.spinOnce();
  if (millis() - currentSampleTime >5) {
    forceValue = currSensor.getCurrentMovingAvg();
  }

  //Publishing some key parameters
  current.data = forceValue;
  pubCurrentSensor.publish(&current);

  //Publishing reaming percentage 
  reamingPercentage.data = getReamingPercentage();
  pubReamingPercentage.publish(&reamingPercentage);

  /*-------------------------------------HIGH LEVEL CONTROL-------------------------------------*/

  // if (ticksTomm(linearActMotorEnc.read()) >= reamingEndPoint && forceValue <= 0.22) {
  //   // Serial.println("Reached reaming end point!");
  //   
  //   currentState = DONEREAMING;
  //   linearActuatorMotorCommand = 0;
  //   reamerMotorCommand = 0;
  // }

  switch (currentState) {

    // Calibrate linear actuator position
    case CALIBRATE:
      controllerStatus.data = 0;
      calibrateMotors();
      break;

    //Wait until you get the actuation signal from arm controller
    case WAITFORCMD:

      nh.loginfo("Waiting for command...");
      controllerStatus.data = 1;
      reamerMotor.stopMotor();
      linearActuator.stopMotor();

      // Hold position at calibration position
      ReamerMotorControlType = speedControl;
      LinearActMotorControlType = speedControl;
      reamerMotorCommand = 0;
      linearActuatorMotorCommand = 0;

      // nh.loginfo("Waiting for start reaming command...");
      if (startReamingProcess == true && !MotorControl::watchDogStop_ && !doneReaming){
        currentState = MOVEUNTILCONTACT;
      }
      break;

    //Actuate motor until contact is made with the pelvis
    case MOVEUNTILCONTACT:
      controllerStatus.data = 2;

      nh.loginfo("Moving until contact with bone...");
      if ((millis() - timerForce1) > 10) {
        float forceOut = forceController(forceValue);
        LinearActMotorControlType = speedControl;
        ReamerMotorControlType = speedControl;;
        linearActuatorMotorCommand = int(forceOut);
        reamerMotorCommand = 0;
        timerForce1 = millis();
      }

      if (forceValue >= forceSetPoint && ticksTomm(linearActMotorEnc.read()) > reamingEndPoint - 10) {
        
        currentState = STARTREAMING;
        startReamingTimer = millis();
        linearActuatorMotorCommand = 0;
      }

      if(ticksTomm(linearActMotorEnc.read()) > reamingStartPoint){
        currentState = STARTREAMING;
      }

      if (dynamicCompensation) {
        
        previousState = currentState;
        currentState = DYNAMICCOMP;
      }

      break;

    //Ream as long as pelvis error is within thresholds and goal has not been reached
    case STARTREAMING:

      nh.loginfo("Start reaming...");
      controllerStatus.data = 3;
      if (MotorControl::watchDogStop_) {
        
        currentState = WAITFORCMD;
      }
      // Serial.println("Start reaming...");
      ReamerMotorControlType = speedControl;
      LinearActMotorControlType = speedControl;
      if ((millis() - timerForce2) > 400) {
        linearActuatorMotorCommand = forceController(forceValue);
        reamerMotorCommand = 1000;
        // reamerMotor.runMotorForward(255);
        timerForce2 = millis();
      }

      if (ticksTomm(linearActMotorEnc.read()) >= reamingEndPoint) {
       
        //Hold position if force is still high 

        // if(forceValue >= 0.25){
        //   //Hold position of linear actuator at the reaming end point
        //   nh.loginfo("Reached end point but bone still left to ream!");
        //   LinearActMotorControlType = positionControl;
        //   linearActuatorMotorCommand = reamingEndPoint;
        // }

        if(reamAtEndPointTimerFlag == false){
          reamAtEndPointTimer = millis();          
          reamAtEndPointTimerFlag = true;
        }

        if(millis()/1000 - reamAtEndPointTimer/1000 <= 15){
          nh.loginfo("Reached end point, reaming for 15 seconds!");
          ReamerMotorControlType = speedControl;
          LinearActMotorControlType = speedControl;
          linearActuatorMotorCommand = 0; 
          reamerMotorCommand = 1000;
        }

        else{
          currentState = DONEREAMING;
          linearActuatorMotorCommand = 0;
          reamerMotorCommand = 0;
        }

      }

      if (dynamicCompensation == true) {
        currentState = DYNAMICCOMP;
      }

      //Retract if reamer gets stuck
      if (reamerMotor.getMotorRPM(reamerMotorEnc.read(), REAMERMOTORPPR) == 0 && ((startReamingTimer - millis()) > 1000)) {
        nh.loginfo("Stuck! Attempting to get unstuck!");
        LinearActMotorControlType = positionControl;
        linearActuatorMotorCommand = reamingStartPoint;
        forceSetPoint -= 0.2;
        reamingStartPoint -= 10;
        currentState = MOVEUNTILCONTACT;
      }

      break;

    // Dynamic compensation - change state back to 1 after performing compensation routine
    case DYNAMICCOMP:
    
    resetGains();    
    nh.loginfo("Dynamic Compensation!");
     if(doneReaming){
        currentState = DONEREAMING;
     }
     
     else{
      controllerStatus.data = 4;
      reamAtEndPointTimerFlag = false;
      LinearActMotorControlType = positionControl;
      ReamerMotorControlType = speedControl;
      reamerMotorCommand = 0;
      linearActuatorMotorCommand = 5;
      if(dynamicCompensation == false){
        currentState = WAITFORCMD;        
      }

     } 

      break;

    case HANDLELIMSWITCH:

      nh.loginfo("Handling limit switch");
      
      //Stop the reamer motor
      ReamerMotorControlType = speedControl;
      LinearActMotorControlType = speedControl;
      reamerMotorCommand = 0;
      reamerMotor.stopMotor();

      //If limit switch 1 is hit, move the linear acutator forward 
      if(MotorControl::limitSwitchStop1_){
        linearActuator.stopMotor();
        linearActuatorMotorCommand = 0;
        linearActuator.runMotorForwardUnsafe(200);
        delay(200);
        
      }


      //If limit switch 2 is hit, move the linear acutator backward 
      if(MotorControl::limitSwitchStop2_){
        linearActuator.stopMotor();
        linearActuatorMotorCommand = 0;
        linearActuator.runMotorBackwardUnsafe(200);
        delay(200);
      }

      currentState = WAITFORCMD;
      break;


    // Goal has been reached, stop reaming!
    case DONEREAMING:
      controllerStatus.data = 5;
      nh.loginfo("Done reaming!");
      doneReaming = true;
      LinearActMotorControlType = positionControl;
      ReamerMotorControlType = speedControl;
      reamerMotorCommand = 0;
      linearActuator.errorIntegralPos_ = 0;
      linearActuator.errorProportionalPos_ = 0;
      linearActuator.errorDerivativePos_ = 0;
      linearActuatorMotorCommand = 15;
      reamerMotor.stopMotor();
      currentState = DONEREAMING;
      break;

    case TESTING:
      controllerStatus.data = 6;
      LinearActMotorControlType = speedControl;
      linearActuatorMotorCommand = 2;
      // nh.loginfo("Testing state...");
      break;

    default:
      controllerStatus.data = -1;
      nh.loginfo("Invalid state, stopping reaming!");
      reamerMotor.stopMotor();
      linearActuator.stopMotor();
      break;
  }


  /*-------------------------------------LOW LEVEL CONTROL-------------------------------------*/

  // pubControllerStatus.publish(&controllerStatus);
  // reamerMotorCommand = 100;
  // ReamerMotorControlType = speedControl;

  // Set LimitSwitch interrupt flag to false

  if (!digitalRead(LimSwitchPin1))
    MotorControl::limitSwitchStop1_ = true;
  else
    MotorControl::limitSwitchStop1_ = false;


  if (!digitalRead(LimSwitchPin2))
    MotorControl::limitSwitchStop2_ = true;
  else
    MotorControl::limitSwitchStop2_ = false;


  // Reamer Motor Speed Control
  if ((((millis() - rpmTimerM1)) > 10) && (ReamerMotorControlType == speedControl)) {
    // reamerMotor.pidSpeedControl(reamerMotorCommand);
    long int encValue = reamerMotorEnc.read();
    float rpm = reamerMotor.getMotorRPM(encValue, REAMERMOTORPPR);
    if (abs(rpm - prevrpm1) > 600) {
      rpm = prevrpm1;
    } else {
      prevrpm1 = rpm;
    }
    float pidOut = reamerMotor.pidSpeedControl(rpm, reamerMotorCommand);
    rpmTimerM1 = millis();

    // Publish the rpm of the reamer motor
    // rpmReamerMotor.data = rpm;
    // pubReamerMotorSpeed.publish(&rpmReamerMotor);
  }


  // Linear Actuator Motor Speed Control
  if ((((millis() - rpmTimerM2)) > 10) && (LinearActMotorControlType == speedControl)){
    
    long int encValue = linearActMotorEnc.read();
    float rpm = linearActuator.getMotorRPM(encValue, LINEARACTMOTORPPR);
    if (abs(rpm - prevrpm2) > 600) {
      rpm = prevrpm2;
    } else {
      prevrpm2 = rpm;
    }

    float rpmCmd = mmPerSecToRpm(linearActuatorMotorCommand, LINEARACTMOTORPPR, 5);
    linearActuator.pidSpeedControl(rpm, rpmCmd);
    rpmTimerM2 = millis();
    // nh.loginfo("Low level speed control...");
    // Publish the rpm of the LinearAct motor

    // rpmLinearActuatorMotor.data = rpm;
    // pubLinearActMotorSpeed.publish(&rpmLinearActuatorMotor);
  }


  // Reamer Motor Position Control
  if ((((millis() - rpmTimerM1)) > 10) && (ReamerMotorControlType == positionControl)) {

    int currPos = reamerMotorEnc.read();
    int ticks = mmtoTicks(reamerMotorCommand);
    int posCurr = reamerMotor.pidPositionControl(currPos, ticks);
    rpmTimerM1 = millis();
  }

  // Linear Actuator Motor Position Control
  if ((((millis() - rpmTimerM2)) > 10) && (LinearActMotorControlType == positionControl)) {
    // nh.loginfo("Linear Actuator Low level Position control...");
    // Serial.println(linearActuatorMotorCommand);

    int currPos = linearActMotorEnc.read();
    int ticks = mmtoTicks(linearActuatorMotorCommand);
    linearActuator.pidPositionControl(currPos, ticks);
    rpmTimerM2 = millis();

    // Publish the rpm of the reamer motor
    // posLinearActuatorMotor.data = currPos;
    // pubLinearActuatorMotorPos.publish(&posLinearActuatorMotor);
  }
}