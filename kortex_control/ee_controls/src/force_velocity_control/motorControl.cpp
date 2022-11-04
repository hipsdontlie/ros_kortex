/*
Arduino Library for interfacing Servocity motors with PID position and velocity control
Author: Kaushik Balasundar
*/
#include "motorControl.h"

/*
 @brief motorControl constructor that calls init() 
*/
MotorControl::MotorControl(byte PWM_Pin, byte DIR_Pin, int whichMotor){

    PWM_Pin_ = PWM_Pin;
    DIR_Pin_ = DIR_Pin;
    whichMotor_ = whichMotor;
    init();

}

/*
@brief Initialize the motor with the desired pin
*/
void MotorControl::init(){

    //Generic Motor Pins 
    pinMode(PWM_Pin_, OUTPUT);
    pinMode(DIR_Pin_, OUTPUT);

    //Reamer end-effector interrupt pins
    pinMode(ENCB_Pin_ReamerMotor_, INPUT);
    pinMode(ENCA_Pin_ReamerMotor_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_Pin_ReamerMotor_), ReamerMotorEncoder, FALLING);

    //Linear Actuator end-effector interrupt pins 
    pinMode(ENCB_Pin_LinearActMotor_, INPUT);
    pinMode(ENCA_Pin_LinearActMotor_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_Pin_LinearActMotor_), LinearActMotorEncoder, FALLING);  
  
   
    //Set PID position control parameters to 0
    Kp_pos_, Kd_pos_, Ki_pos_, PIDOutPos_, posPrev_,posCurr_, cmd_ = 0;

    //Set PIDVelocity control parameters to 0
    Kp_vel_, Kd_vel_, Ki_vel_, PIDOutVel_, rpmPrev_, rpmCurr_, rpmTimer_, tempPosCurr_, tempPosPrev_ = 0;

    //Set other generic control parameters to 0
    currentTime_, previousTime_, deltaT_, errorIntegral_, errorDerivative_, errorProportional_ = 0;

    //Set encoder value to 0
    encoderValue_ = 0;

    
}

/*
@brief Run the motor forward using analogWrite 
*/
void MotorControl::runMotorForward(int analogValue){
  digitalWrite(DIR_Pin_, LOW);
  analogWrite(PWM_Pin_, analogValue);
  return;
}

/*
@brief Run the motor backward using analogWrite 
*/
void MotorControl::runMotorBackward(int analogValue){
  digitalWrite(DIR_Pin_, HIGH);
  analogWrite(PWM_Pin_, analogValue);
  return;
}

/*
@brief Stop the motor
*/
void MotorControl::stopMotor(){
    analogWrite(PWM_Pin_, 0);
    return;
}

void MotorControl::setupInterruptPins(){


}

/*
@brief Interrupt service routine function for encoder position of reamer motor
*/
void MotorControl::ReamerMotorEncoder(){
    if (digitalRead(ENCB_Pin_ReamerMotor_) == HIGH)
        encoderValue_ReamerMotor_++;
    else
        encoderValue_ReamerMotor_--;  
    return;
}

/*
@brief Interrupt service routine function for encoder position of reamer motor
*/
void MotorControl::LinearActMotorEncoder(){
    if (digitalRead(ENCB_Pin_LinearActMotor_) == HIGH)
        encoderValue_LinearActMotor_++;
    else
        encoderValue_LinearActMotor_--;  
    return;
}

/*
@brief Returns the current encoder position
*/
int MotorControl::getMotorPos(){

    if(whichMotor_ == 1)
        return encoderValue_ReamerMotor_;
    
    else return encoderValue_LinearActMotor_;
}

/*
@brief Returns the current motor rpm
*/
float MotorControl::getMotorRPM(){
    
    if(whichMotor_ == 1){
        encoderValue_ = encoderValue_ReamerMotor_;
    }
    else{
        encoderValue_ = encoderValue_LinearActMotor_;  
    } 

    currentTime_ = micros();
    deltaT_ = ((float) (currentTime_-previousTime_)/1000000); //TODO: Find the right constant 
    tempPosCurr_ = encoderValue_;
    rpmCurr_ = float(tempPosCurr_-tempPosPrev_)/deltaT_*0.37;
    tempPosPrev_ = tempPosCurr_;
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
int MotorControl::pidPositionControl(int targetPos){

    if(whichMotor_ == 1)
        posCurr_ = encoderValue_ReamerMotor_; 
    else 
        posCurr_ = encoderValue_LinearActMotor_;

    //Compute error terms 
    errorProportional_ = float(targetPos)-posCurr_;
    errorDerivative_ = (posPrev_-posCurr_)/deltaT_;
    errorIntegral_ += (float(targetPos)-posCurr_)*deltaT_;

    //Update prevPos to currPos
    posPrev_ = posCurr_;
    
    //Get PID output 
    PIDOutPos_ = PIDOutPos_ + int((Kp_pos_*errorProportional_ + Kd_pos_*errorDerivative_ + Ki_pos_*errorIntegral_));
 
    //Threshold the output 
    if (abs(PIDOutVel_) > 255)
        cmd_ = 255;
    else if (PIDOutVel_ < 0)
        cmd_ = abs(PIDOutVel_);
    else
        cmd_ = PIDOutVel_;
    
    //Actuate motor with the output value based on direction of targetPos
    if(targetPos > 0)
      runMotorForward(cmd_);
    else
      runMotorBackward(cmd_);

    return PIDOutPos_;

}

/*
@brief PID velocity control to a targetVel
*/
int MotorControl::pidVelocityControl(int rpmTarget){

    //Update the RPM of the motor
    getMotorRPM();

    //Compute error terms 
    errorProportional_ = float(rpmTarget)-rpmCurr_;
    errorDerivative_ = (rpmPrev_-rpmCurr_)/deltaT_;
    errorIntegral_ += (float(rpmTarget)-rpmCurr_)*deltaT_;

    //Update previous RPM to current RPM
    rpmPrev_ = rpmCurr_;

    //Get PID output 
    PIDOutVel_ = PIDOutVel_ + int((Kp_vel_*errorProportional_ + Kd_vel_*errorDerivative_ + Ki_vel_*errorIntegral_));
    
    //Threshold the output 
    if (abs(PIDOutVel_) > 255)
        cmd_ = 255;
    else if (PIDOutVel_ < 0)
        cmd_ = abs(PIDOutVel_);
    else
        cmd_ = PIDOutVel_;

    //Actuate motor with the output value based on direction of targetVek
    if(rpmTarget > 0)
      runMotorForward(cmd_);
    else
      runMotorBackward(cmd_);

    return PIDOutVel_;
}

/*
@brief Set PID position control constants 
*/
void MotorControl::setPIDPosConstants(float Kp, float Ki, float Kd){
    Kp_pos_ = Kp;
    Kd_pos_ = Kd; 
    Ki_pos_ = Ki;
}

/*
@brief Set PID velocity control constants 
*/
void MotorControl::setPIDVelConstants(float Kp, float Ki, float Kd){
    Kp_vel_ = Kp;
    Kd_vel_ = Kd; 
    Ki_vel_ = Ki;
}


/*
@brief Limit switch interrupt function
*/
// void MotorControl::triggerLimitSwitch(){
//   limitSwitchStop_ = true;
//   analogWrite(ENC_B_)
// }


