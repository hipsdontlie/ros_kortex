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
@brief Get the voltage reading, and convert to the average current readings based on calibrated constants 
*/

float currentSensor::getCurrentAvg(){
    currentRaw_ = 0;
    currentRawAvg_ = 0;
    current_ = 0;
    numSamples_ = 20; 
    // Average filtering
    for(int i=0 ; i<numSamples_ && getRaw() > 0 && getRaw() < 1023; i++){
      currentRaw_ = currentRaw_ + getRaw();
    }

    currentRawAvg_ = float(currentRaw_)/numSamples_;
    current_ = (float(currentRawAvg_)-510.0)/14.0;

    return -current_;
}

/*
@brief Get the voltage reading, and convert to the current readings based on calibrated constants 
*/

float currentSensor::getCurrent(){
    currentRaw_ = 0;
    currentRawAvg_ = 0;
    current_ = 0;
    numSamples_ = 20; 
    currentRaw_ = getRaw();
    current_ = (float(currentRaw_)-510.0)/14.0;
    return -current_;
}

/*
@brief Get the raw voltage readings
*/

int currentSensor::getRaw(){
    return analogRead(pin_);
}


