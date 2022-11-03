/*

Arduino Library for low level position and velocity control of the end-effector linear actuator motor and the reaming motor 

*/


#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <ros.h> 
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


class MotorControl{

    private:
    /* -------------------------------- Private Variables -------------------------------- */

        //Pins needed for communication
        byte PWM_Pin_; 
        byte DIR_Pin_; 
        volatile static byte ENCB_Pin_;
        volatile static byte ENCA_Pin_;
        volatile static byte LIM_Switch_1_;
        volatile static byte LIM_Switch_2_;

    public:
    /* -------------------------------- Public Variables -------------------------------- */
        
        //PID Position control parameters
        int Kp_pos_, Kd_pos_, Ki_pos_;
        int PIDOutPos_, posCurr_, posPrev_;

        //PID Velocity control parameters 
        int Kp_vel_, Kd_vel_, Ki_vel_;
        int PIDOutVel_;
        double rpmPrev_,rpmCurr_,rpmTimer_, tempPosCurr_, tempPosPrev_;

        //Other generic PID parameters
        volatile static int encoderValue_;
        double currentTime_, previousTime_, deltaT_;
        double errorIntegral_, errorDerivative_, errorProportional_;

        //Some stopping variables 
        volatile static bool watchDogStop_, limitSwitchStop_;

        /* -------------------------------- Public Members ----------------------------------- */
      
        // Setup pins and call init()
        MotorControl(byte PWM_Pin_, byte DIRin, byte ENCA_Pin, byte ENCB_Pin, byte LIM_Switch_1, byte LIM_Switch_2);

        // Setup the appropriate pinMode and turn off the motor (default state)
        void init();
        
        // Power on the motor using analogWrite 
        void runMotor(int analogValue);

        // Power off the motor
        void stopMotor();

        // Get RPM of the motor 
        double getMotorRPM();

        // Get current position 
        int getMotorPos();

        //Encoder interrupt service routine (interrupt service routines need to be static) 
        static void encoder(int encoderVal);

        //PID position control 
        void pidPositionControl(int targetTick);

        //PID velocity control 
        void pidVelocityControl(int targetVelocity);
        
        //Calibrate motor 
        void calibrateMotor(); 

        //Set PID position control constants 
        void setPIDPosConstants(double Kp, double Ki, double Kd);

        //Set PID velocity control constants 
        void setPIDVelConstants(double Kp, double Ki, double Kd);

        //LimitSwitch trigger interrupt service routine (interrupt service routines need to be static)
        static void triggerLimitSwitch();

};

#endif