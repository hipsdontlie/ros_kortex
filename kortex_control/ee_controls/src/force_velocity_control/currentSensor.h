/*
Arduino Library for interfacing the current sensor
AUTHOR: Kaushik Balasundar
*/


#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H
#define WINDOWSIZE 100
#include <Arduino.h>

class currentSensor{

        /* -------------------------------- Private Variables -------------------------------- */

    private:
        //Analog pin
        byte pin_;
    
    
    public:

        /* -------------------------------- Public Variables -------------------------------- */

        float current_;
        int currentRaw_;
        float currentRawAvg_; 
        int numSamples_;
        int allReadings[WINDOWSIZE];
        float movingAverage_; 
        int sum_;
        int idx_;


        // float testArr[100];
        

        /* -------------------------------- Public Members ----------------------------------- */
        
        // Setup pin for current sensor and call init()
        currentSensor(byte currentSensorPin);

        // Setup the analogPin for reading from current sensor 
        void init();


        // Get current 
        float getCurrent(); 

        // Get current 
        float getCurrentAvg(); 

        //Get moving average current 
        float getCurrentMovingAvg();

        //Publish current to a ROS topic 
        void publishCurrent();

        //Get raw sensor readings 
        int getRaw();

};

#endif