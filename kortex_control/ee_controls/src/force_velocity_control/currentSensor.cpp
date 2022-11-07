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

float currentSensor::getCurrent(){
    currentRaw_ = 0;
    currentRawAvg_ = 0;
    current_ = 0;
    // Average filtering
    for(int i=0 ; i<50; i++){
      currentRaw_ += getRaw();
    }

    currentRawAvg_ = currentRaw_/50;
    current_ = (float(currentRawAvg_)-510)/14;

    return abs(current_);
}

int currentSensor::getRaw(){
    return analogRead(pin_);
}


