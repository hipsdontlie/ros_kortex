#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "currentSensor.h"
#include "motorControl.h"
//Current sensor pin 
#define currentSensorPin  A0

//Reamer Motor Pins 
#define PWM_Pin_ReamerMotor 6
#define DIR_Pin_ReamerMotor 5
#define ENCA_Pin_ReamerMotor 21
#define ENCB_Pin_ReamerMotor 20

// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 curr_1;
ros::Publisher pub_C1("current", &curr_1);

//Current sensor 
currentSensor currSensor(currentSensorPin);

//Reamer motor 
MotorControl reamerMotor(PWM_Pin_ReamerMotor, DIR_Pin_ReamerMotor, ENCA_Pin_ReamerMotor, ENCB_Pin_ReamerMotor);

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

