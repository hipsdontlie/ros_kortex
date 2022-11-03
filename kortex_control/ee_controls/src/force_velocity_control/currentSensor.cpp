/*
Arduino Library for interfacing the current sensor
Author: Kaushik Balasundar
*/

#include "currentSensor.h"

/*
 @brief currentSensor constructor that calls init() 
*/

currentSensor::currentSensor(byte currentSensorPin){
  pin_ = currentSensorPin;
  init();
}

/*
@brief Initialize the currentSensor with the desired pin
*/

void currentSensor::init(){
    pinMode(pin_, INPUT);
}

/*
@brief Get the voltage reading, and convert to the current readings based on calibrated constants 
*/

double currentSensor::getCurrent(){
    current_ = (analogRead(pin_)-510)/14;
    return current_;
}


