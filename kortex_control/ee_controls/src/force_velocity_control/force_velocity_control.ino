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
static int encoderValue_ReamerMotor = 0;
static int encoderValue_LinearActMotor = 0;
static int encPIN_ReamerMotor // TODO
static int encPIN_ReamerMotor // TODO

//Limit switch pins
byte LimSwitch_Pin_1 = 2;
byte LimSwitch_Pin_2 = 3;

// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 curr_1;
ros::Publisher pub_C1("current", &curr_1);

//Current sensor 
currentSensor currSensor(currentSensorPin);

//Reamer motor 
MotorControl reamerMotor(PWM_Pin_ReamerMotor, DIR_Pin_ReamerMotor, ENCA_Pin_ReamerMotor, ENCB_Pin_ReamerMotor, LimSwitch_Pin_1, LimSwitch_Pin_2);

void setup(){
    Serial.begin(57600);
    nh.initNode();
    nh.advertise(pub_C1);

}

void loop(){

// Get the current readings from the current sensor 
    nh.spinOnce();
    float current = currSensor.getCurrent();
    curr_1.data = current;
    pub_C1.publish(&curr_1);

// Test motor 
}

