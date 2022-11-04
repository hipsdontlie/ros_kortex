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
byte PWM_Pin_ReamerMotor = 6;
byte DIR_Pin_ReamerMotor = 5;
byte ENCA_Pin_ReamerMotor = 21;
byte ENCB_Pin_ReamerMotor = 20;

//Linear Actuator Motor Pins
byte PWM_Pin_LinearActMotor = 7;
byte DIR_Pin_LinearActMotor = 8;
byte ENCA_Pin_LinearActMotor = 18;
byte ENCB_Pin_LinearActMotor = 19;


//Limit switch pins
byte LimSwitch_Pin_1 = 2;
byte LimSwitch_Pin_2 = 3;

//Initialize interrupt pins 
volatile static int MotorControl::encoderValue_ReamerMotor_ = 0;
volatile static int MotorControl::encoderValue_LinearActMotor_ = 0;
volatile static byte MotorControl::ENCB_Pin_ReamerMotor_ = ENCB_Pin_ReamerMotor;
volatile static byte MotorControl::ENCA_Pin_ReamerMotor_ = ENCA_Pin_ReamerMotor;
volatile static byte MotorControl::ENCB_Pin_LinearActMotor_ = ENCB_Pin_LinearActMotor;
volatile static byte MotorControl::ENCA_Pin_LinearActMotor_ = ENCA_Pin_LinearActMotor;


// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 curr_1;
ros::Publisher pub_C1("current", &curr_1);

//Current sensor 
currentSensor currSensor(currentSensorPin);

//Reamer motor 
MotorControl reamerMotor(PWM_Pin_ReamerMotor , DIR_Pin_ReamerMotor, 1);


//Linear Actuator motor 
MotorControl linearActuator(PWM_Pin_LinearActMotor, DIR_Pin_LinearActMotor, 2);


//Low level controller timers 
unsigned long rpm_timer_M1 = 0;
unsigned long rpm_timer_M2 = 0;

void setup(){

    Serial.begin(57600);
    nh.initNode();
    reamerMotor.setPIDVelConstants(0.15,0.03,0);
    reamerMotor.setPIDPosConstants(0.15,0.03,0);
    nh.advertise(pub_C1);
    
}

void loop(){

// Get the current readings from the current sensor 
    // nh.spinOnce();
    // float current = currSensor.getCurrent();
    // curr_1.data = current;
    // pub_C1.publish(&curr_1);

// Test PID Controls

  if (((millis()-rpm_timer_M1)) > 400){
    int out = reamerMotor.pidPositionControl(100);
    rpm_timer_M1 = millis();
    Serial.println(out);
  }



}

