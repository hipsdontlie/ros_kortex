/*

Arduino Library for low level position and velocity control of the end-effector linear actuator motor and the reaming motor 

*/


#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro



class MotorControl{

    private:
    /* -------------------------------- Private Variables -------------------------------- */

        //Pins needed for communication
        byte PWM_Pin_; 
        byte DIR_Pin_; 
        int whichMotor_;


    public:
    /* -------------------------------- Public Variables -------------------------------- */
        
        //PID Position control parameters
        float Kp_pos_, Kd_pos_, Ki_pos_;
        float PIDOutPos_;
        int posCurr_, posPrev_;
        float errorIntegralPos_, errorDerivativePos_, errorProportionalPos_, errorCurrPos_, errorPrevPos_;
        float currentTimePos_, previousTimePos_, deltaTPos_;


        //PID Velocity control parameters 
        float Kp_vel_, Kd_vel_, Ki_vel_;
        int PIDOutVel_;
        int cmd_;
        float rpmPrev_,rpmCurr_,rpmTimer_, tempPosCurr_, tempPosPrev_;
        float errorIntegralVel_, errorDerivativeVel_, errorProportionalVel_, errorCurrVel_, errorPrevVel_;
        float currentTimeVel_, previousTimeVel_, deltaTVel_;


        //Other generic PID parameters
        int encoderValue_;
        volatile static int encoderValue_ReamerMotor_;
        volatile static int encoderValue_LinearActMotor_;

        //Some stopping variables for safety 
        volatile static bool watchDogStop_, limitSwitchStop_;

        //Interrupt pins 
        volatile static byte ENCB_Pin_ReamerMotor_;
        volatile static byte ENCA_Pin_ReamerMotor_;
        volatile static byte ENCB_Pin_LinearActMotor_;
        volatile static byte ENCA_Pin_LinearActMotor_;

        /* -------------------------------- Public Members ----------------------------------- */
      
        // Setup pins and call init()
        MotorControl(byte PWM_Pin_, byte DIRin, int whichMotor);

        // Setup the appropriate pinMode and turn off the motor (default state)
        void init();
        
        // Actuate motor forward using analogWrite 
        void runMotorForward(int analogValue);

        // Actuate motor backward using analogWrite 
        void runMotorBackward(int analogValue);

        // Power off the motor
        void stopMotor();

        // Get RPM of the motor 
        float getMotorRPM();

        // Get current position 
        int getMotorPos();

        //Encoder interrupt service routine (interrupt service routines need to be static) 
        static void ReamerMotorEncoder();

        //Encoder interrupt service routine (interrupt service routines need to be static) 
        static void LinearActMotorEncoder();

        //Setup encoder pins 
        void setupInterruptPins();

        //PID position control 
        int pidPositionControl(int targetPos);

        //PID speed control 
        int pidSpeedControl(int targetSpeed);
        
        //Calibrate motor 
        void calibrate(); 

        //Set PID position control constants 
        void setPIDPosConstants(float Kp, float Ki, float Kd);

        //Set PID velocity control constants 
        void setPIDVelConstants(float Kp, float Ki, float Kd);

        //LimitSwitch trigger interrupt service routine (interrupt service routines need to be static)
        static void triggerLimitSwitch();

        //Initialize PID
        void initPID();


};

#endif