/*
Arduino Library for interfacing Servocity motors with PID position and velocity control
Author: Kaushik Balasundar
*/
#include "motorControl.h"

/*
@brief motorControl constructor that calls init() 
*/
MotorControl::MotorControl(byte PWM_Pin, byte DIR_Pin, int whichMotor, int PPR){

    PWM_Pin_ = PWM_Pin;
    DIR_Pin_ = DIR_Pin;
    whichMotor_ = whichMotor;
    PPR_ = PPR;
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
    // pinMode(ENCB_Pin_ReamerMotor_, INPUT);
    // pinMode(ENCA_Pin_ReamerMotor_, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ENCA_Pin_ReamerMotor_), ReamerMotorEncoder, FALLING);
    // ReamerMotorEncoder(ENCB_Pin_ReamerMotor_, ENCA_Pin_ReamerMotor_);
    
    //Linear Actuator end-effector interrupt pins 
    // pinMode(ENCB_Pin_LinearActMotor_, INPUT);
    // pinMode(ENCA_Pin_LinearActMotor_, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ENCA_Pin_LinearActMotor_), LinearActMotorEncoder, FALLING);  
    // ReamerMotorEncoder(ENCB_Pin_LinearActMotor_, ENCA_Pin_LinearActMotor_);
    
   
    //Set PID position control parameters to 0
    Kp_pos_, Kd_pos_, Ki_pos_, PIDOutPos_, posPrev_,posCurr_, cmd_ = 0;

    //Set PIDVelocity control parameters to 0
    Kp_vel_, Kd_vel_, Ki_vel_, PIDOutVel_, rpmPrev_, rpmCurr_, rpmTimer_, tempPosCurr_, tempPosPrev_, numSteps_ = 0;

    //Set other generic control parameters to 0
    currentTimePos_, previousTimePos_, deltaTPos_, errorIntegralPos_, errorDerivativePos_, errorProportionalPos_, errorPrevPos_, errorCurrPos_ = 0;
    currentTimeVel_, previousTimeVel_, deltaTVel_, errorIntegralVel_, errorDerivativeVel_, errorProportionalVel_, errorPrevVel_, errorCurrVel_ = 0;

    //Set encoder value to 0
    encoderValue_ = 0;

    //Calibration stuff
    reachedEnd = false;

    
}

/*
@brief Run the motor forward using analogWrite with limitswitch and watchdog checks
*/
void MotorControl::runMotorForward(int analogValue){
  
  if(!limitSwitchStop1_ && !watchDogStop_ && !controllerFlagStop_){
    digitalWrite(DIR_Pin_, LOW);
    analogWrite(PWM_Pin_, analogValue);
    return;
  }

  else{
    stopMotor();
  }

}

/*
@brief Run the motor forward using analogWrite without any checks (only use for calibration!)
*/
void MotorControl::runMotorForwardUnsafe(int analogValue){
  digitalWrite(DIR_Pin_, LOW);
  analogWrite(PWM_Pin_, analogValue);
  return;
}

/*
@brief Run the motor backward using analogWrite without any checks (only use for calibration!)
*/
void MotorControl::runMotorBackwardUnsafe(int analogValue){
  digitalWrite(DIR_Pin_, HIGH);
  analogWrite(PWM_Pin_, analogValue);
  return;
}



/*
@brief Run the motor backward using analogWrite 
*/
void MotorControl::runMotorBackward(int analogValue){
  if(!limitSwitchStop2_ && !watchDogStop_ && !controllerFlagStop_){
    digitalWrite(DIR_Pin_, HIGH);
    analogWrite(PWM_Pin_, analogValue);
  }
  return;
}

/*
@brief Stop the motor
*/
void MotorControl::stopMotor(){
    analogWrite(PWM_Pin_, 0);
    return;
}

/*
@brief Interrupt service routine function for encoder position of reamer motor
*/
// void MotorControl::readReamerMotorEncoder(){
//     encoderValue_ReamerMotor_ = ReamerMotorEncoder.read();
// }

/*
@brief Interrupt service routine function for encoder position of reamer motor
*/
// void MotorControl::readLinearActMotorEncoder(){
//     encoderValue_LinearActMotor_ = LinearActMotorEncoder.read();
// }


/*
@brief Returns the current motor rpm
*/
float MotorControl::getMotorRPM(long int encoderValue, float ppr){
    
    currentTimeVel_ = micros();
    deltaTVel_ = ((float) (currentTimeVel_-previousTimeVel_)/1000000.0); 
    numSteps_ = 60.0/deltaTVel_;
    tempPosCurr_ = encoderValue;
    rpmCurr_ = float(tempPosCurr_-tempPosPrev_) * numSteps_ / ppr; 
    tempPosPrev_ = tempPosCurr_;
    previousTimeVel_ = currentTimeVel_;
    return rpmCurr_;
}


/*
@brief PID position control to a targetPos
*/
int MotorControl::pidPositionControl(long int encoderValue, int targetPos){
    
    currentTimePos_ = micros();
    deltaTPos_ = currentTimePos_ - previousTimePos_; 
    previousTimePos_ = currentTimePos_;
    posCurr_ = encoderValue;
             
     //Compute error terms 
    errorCurrPos_ = float(targetPos)-posCurr_;
    errorProportionalPos_ = errorCurrPos_;
    errorDerivativePos_ = (errorCurrPos_ - errorPrevPos_)/deltaTPos_;
    errorIntegralPos_ = errorIntegralPos_ + errorCurrPos_*deltaTPos_;
    
    //Update previous values
    errorPrevPos_ = errorCurrPos_;
    
    //Get PID output 
    PIDOutPos_ =  int((Kp_pos_*errorProportionalPos_) + (Kd_pos_*errorDerivativePos_) + (Ki_pos_*errorIntegralPos_));
 
    //Threshold the output 
    if (abs(PIDOutPos_) > 255)
        cmd_ = 255;
  
    else if (PIDOutPos_ < 0)
        cmd_ = abs(PIDOutPos_);

    // else if (PIDOutPos_ > -10 && PIDOutPos_ < 10)
    //     stopMotor();

    // else if(abs(PIDOutPos_) > 5 && abs(PIDOutPos_) < 60)
    //     cmd_ = 60;

    else
        cmd_ = PIDOutPos_;
    
    //Actuate motor with the output value based on direction of targetPos
    if(PIDOutPos_ > 0)
      runMotorForward(cmd_);
    else
      runMotorBackward(cmd_);

    return posCurr_;

}

/*
@brief PID velocity control to a targetVel
*/
int MotorControl::pidSpeedControl(int rpmCurr, int rpmTarget){

    if(rpmTarget == 0){
      stopMotor();
      return 0;
    }

     //Compute error terms 
    errorProportionalVel_ = float(rpmTarget)-rpmCurr;
    errorDerivativeVel_ = (rpmPrev_-rpmCurr)/deltaTVel_;
    errorIntegralVel_ += (float(rpmTarget)-rpmCurr)*deltaTVel_;

    //Update previous RPM to current RPM
    rpmPrev_ = rpmCurr;

    //Get PID output 
    PIDOutVel_ = int((Kp_vel_*errorProportionalVel_ + Kd_vel_*errorDerivativeVel_ + Ki_vel_*errorIntegralVel_));
    
    //Threshold the output 
    if(whichMotor_ == 2)
    {
      if (abs(PIDOutVel_) > 50)
          cmd_ = 50;
      else if (PIDOutVel_ < 0)
          cmd_ = abs(PIDOutVel_);

      // else if(abs(PIDOutVel_) > 0 && abs(PIDOutVel_) < 50)
      //     cmd_ = 50; //Minimum speed to keep the linear actuator moving 

      else
          cmd_ = PIDOutVel_;
    } 

    else{
          
      if (abs(PIDOutVel_) > 255)
          cmd_ = 255;
      else if (PIDOutVel_ < 0)
          cmd_ = abs(PIDOutVel_);

      // else if(abs(PIDOutVel_) > 0 && abs(PIDOutVel_) < 50)
      //     cmd_ = 50; //Minimum speed to keep the linear actuator moving 

      else
          cmd_ = PIDOutVel_;
    } 
    

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
@brief Initialize PID params  
*/

void MotorControl::initPID(){
    
    //Set other generic control parameters to 0
    currentTimePos_, previousTimePos_, deltaTPos_, errorIntegralPos_, errorDerivativePos_, errorProportionalPos_, errorPrevPos_, errorCurrPos_, PIDOutPos_ = 0;
    currentTimeVel_, previousTimeVel_, deltaTVel_, errorIntegralVel_, errorDerivativeVel_, errorProportionalVel_, errorPrevVel_, errorCurrVel_, PIDOutVel_ = 0;
}