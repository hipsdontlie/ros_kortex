/*
Arduino Library for interfacing Servocity motors with PID position and velocity control
Author: Kaushik Balasundar
*/
#include "motorControl.h"

/*
 @brief motorControl constructor that calls init() 
*/
MotorControl::MotorControl(byte PWM_Pin, byte DIR_Pin, byte ENCA_Pin, byte ENCB_Pin, byte LIM_Switch_1, byte LIM_Switch_2){

    PWM_Pin_ = PWM_Pin;
    DIR_Pin_ = DIR_Pin;
    ENCB_Pin_ = ENCB_Pin;
    ENCA_Pin_ = ENCA_Pin_;
    LIM_Switch_1_ = LIM_Switch_1;
    LIM_Switch_2_ = LIM_Switch_2;
    init();
}

/*
@brief Initialize the motor with the desired pin
*/
MotorControl::init(){

    //Motor 
    pinMode(PWM_Pin_, OUTPUT);
    pinMode(DIR_Pin_, OUTPUT);
    pinMode(ENCA_Pin_, INPUT_PULLUP);
    pinMode(ENCB_Pin_, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_Pin_), encoder, FALLING);
    stopMotor();
    
    //Limit switches 
    pinMode(LIM_Switch_1_, INPUT_PULLUP);
    pinMode(LIM_Switch_2_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIM_1), triggerLimSwitch, HIGH);
    attachInterrupt(digitalPinToInterrupt(LIM_2), triggerLimSwitch, HIGH);
    
    //Set PID position control parameters to 0
    Kp_pos_, Kd_pos_, Ki_pos_, PIDOutPos_, posPrev_,posCurr_  = 0;

    //Set PIDVelocity control parameters to 0
    Kp_vel_, Kd_vel_, Ki_vel_, PIDOutVel_, rpmPrev_, rpmCurr_, rpmTimer_ = 0;

    //Set other generic control parameters to 0
    encoderValue_, currentTime_, previousTime_, deltaT_, errorIntegral_, errorDerivative_, errorProportional_ = 0;

    //Set stopping variables to false
    watchDogStop_, limitSwitchStop_ = false;
}

/*
@brief Stop the motor
*/
void MotorControl::stopMotor(){
    analogWrite(PWM_Pin_, 0);
    return;
}

/*
@brief Interrupt function for encoder position 
*/
void MotorControl::encoder(){
    if (digitalRead(ENCB_Pin_) == HIGH)
        encoderValue_++;
    else
        encoderValue_--;  
    return;
}

/*
@brief Returns the current encoder position
*/
int MotorControl::getMotorPos(){
    return encoderValue_;
}

/*
@brief Returns the current motor rpm
*/
double MotorControl::getMotorRPM(){
    currentTime_ = micros();
    deltaT_ = ((float) (currentTime_-previousTime_)/1000000); //TODO: Find the right constant 
    posCurr_ = encoderValue_;
    rpmCurr_ = float(pos_-posPrev_)/deltaT_*0.37;
    posPrev_ = posCurr_;
    previousTime_ = micros();
    rpmTimer_ = millis();
    return rpmCurr_;
}

/*
@brief Actuate motors until limit switches are triggered and move a small amount 
*/
void MotorControl::calibrateMotor(){

    return;
}

/*
@brief PID position control to a targetPos
*/
void MotorControl::pidPositionControl(int targetPos){


    return;
}

/*
@brief PID velocity control to a targetVel
*/
void MotorControl::pidVelocityControl(int targetVelocity){

    errorProportional_ = float(targetVelocity)-abs(velCurr_);
    errorDerivative_ = (prev_rpm_-vel_)/deltaT_;
    prevRPM_ = vel_;
    errorIntegral_ += (float(set_val_)-abs(vel_))*deltaT_;
    PIDOutVel_ = PIDOutVel_ + int((Kp_vel_*errorProportional_+Kd_vel_*errorDerivative_+Ki_vel_*errorIntegral_));
    if (PIDOutVel_ > 255){
        PIDOutVel_ = 255;
    }

    if (PIDOutVel_ < 0){
        PIDOutVel_ = 0;
    }

    analogWrite(PWM_Pin_, PIDOutVel_);
    return;
}

/*
@brief Set PID position control constants 
*/
void MotorControl::setPIDPosConstants(double Kp, double Ki, double Kd){
    Kp_pos_ = Kp;
    Kd_pos_ = Kd; 
    Ki_pos_ = Ki;
}

/*
@brief Set PID velocity control constants 
*/
void MotorControl::setPIDPosConstants(double Kp, double Ki, double Kd){
    Kp_vel_ = Kp;
    Kd_vel_ = Kd; 
    Ki_vel_ = Ki;
}


/*
@brief Limit switch interrupt function
*/
void MotorControl::triggerLimSwitch(){
  limitSwitchStop_ = true;
  stopMotors();
}


