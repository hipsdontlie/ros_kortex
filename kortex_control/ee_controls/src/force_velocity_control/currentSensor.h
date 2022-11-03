/*
Arduino Library for interfacing the current sensor
AUTHOR: Kaushik Balasundar
*/


#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

#include <Arduino.h>

class currentSensor{

        /* -------------------------------- Private Variables -------------------------------- */

    private:
        //Analog pin
        byte pin_;
    
    
    public:

        /* -------------------------------- Public Variables -------------------------------- */

        double current_; 

        /* -------------------------------- Public Members ----------------------------------- */
        
        // Setup pin for current sensor and call init()
        currentSensor(byte currentSensorPin);

        // Setup the analogPin for reading from current sensor 
        void init();

        // Get current 
        double getCurrent(); 

        //Publish current to a ROS topic 
        void publishCurrent();

};

#endif