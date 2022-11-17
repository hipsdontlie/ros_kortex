/*
Arduino Library for interfacing the current sensor
AUTHOR: Kaushik Balasundar
*/


#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H
#define WINDOWSIZE 200
#include <Arduino.h>

class currentSensor{

        /* -------------------------------- Private Variables -------------------------------- */

    private:
        //Analog pin
        byte pin_;
    
    
    public:

        /* -------------------------------- Public Variables -------------------------------- */

        double current_;
        int currentRaw_;
        double currentRawAvg_; 
        int numSamples_;
        double allReadings[WINDOWSIZE];
        double movingAverage_; 
        double sum_;
        int idx_;


        // float testArr[100];
        

        /* -------------------------------- Public Members ----------------------------------- */
        
        // Setup pin for current sensor and call init()
        currentSensor(byte currentSensorPin);

        // Setup the analogPin for reading from current sensor 
        void init();


        // Get current 
        double getCurrent(); 

        // Get current 
        double getCurrentAvg(); 

        //Get moving average current 
        double getCurrentMovingAvg();

        //Publish current to a ROS topic 
        void publishCurrent();

        //Get raw sensor readings 
        int getRaw();

};

#endif