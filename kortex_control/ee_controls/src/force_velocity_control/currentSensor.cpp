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

double currentSensor::getCurrentAvg(){
    currentRaw_ = 0;
    currentRawAvg_ = 0;
    current_ = 0;
    numSamples_ = 50; 
    // Average filtering
    for(int i=0 ; i<numSamples_ ; i++){
      currentRaw_ = currentRaw_ + getRaw();
      delay(3);
    }

    currentRawAvg_ = double(currentRaw_)/numSamples_;
    current_ = (2.5 - (currentRawAvg_ * (5.0 / 1024.0)) )/0.068;
    return current_;
}

/*
@brief Get the voltage reading, and convert to the average current readings with a moving average filter  
*/

double currentSensor::getCurrentMovingAvg(){
    // Moving Average filtering
    sum_ = sum_ - allReadings[idx_];
    current_ = (2.5 - (getRaw()* (5.0 / 1024.0)) )/0.068;
    allReadings[idx_] = current_; 
    sum_ = sum_ + current_;
    idx_ = (idx_ + 1) % WINDOWSIZE;
    movingAverage_ = sum_/((double)WINDOWSIZE);
    return movingAverage_;
}

/*
@brief Get the voltage reading, and convert to the current readings based on calibrated constants 
*/

double currentSensor::getCurrent(){
    currentRaw_ = 0;
    currentRawAvg_ = 0;
    current_ = 0;
    numSamples_ = 20; 
    currentRaw_ = getRaw();
    current_ = (double(currentRaw_)-510.0)/14.0;
    return -current_;
}

/*
@brief Get the raw voltage readings
*/

int currentSensor::getRaw(){
    return analogRead(pin_);
}


