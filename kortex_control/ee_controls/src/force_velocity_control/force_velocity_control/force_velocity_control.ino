#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 rpm_ReamerMotor;
std_msgs::Float64 rpm_LinearActuatorMotor;
std_msgs::Float64 curr_1;
ros::Publisher pub_ReamerMotor("reamer_velocity",&rpm_ReamerMotor);
ros::Publisher pub_LinearActuatorMotor("linear_actuator_velocity",&rpm_LinearActuatorMotor);
ros::Publisher pub_C1("current",&curr_1);



// Pin Definitions
byte CURR_1 = A0;


//Limit switch pins 
int LIM_1 = 2; 
int LIM_2 = 3;
bool stop = false;

// Current sensor pin
int CURR = 1;

// Reamer Motor 
int PWM_Pin_ReamerMotor = 6;
int DIR_Pin_ReamerMotor = 5;
int ENCA_Pin_ReamerMotor = 21;
int ENCB_Pin_ReamerMotor = 20;
float encoderValue_ReamerMotor = 0;

// Linear Actuator Motor 
int PWM_Pin_LinearActuatorMotor = 7;
int DIR_Pin_LinearActuatorMotor = 8;
int ENCB_Pin_LinearActuatorMotor = 18;
int ENCA_Pin_LinearActuatorMotor = 19;
float encoderValue_LinearActuatorMotor = 0;

float currentTime_ReamerMotor = micros();
float previousTime_ReamerMotor = micros();
float pos_ReamerMotor = 0;
float posPrev_ReamerMotor = 0;
float deltaT_ReamerMotor = 0;

float currentTime_LinearActuatorMotor = micros();
float previousTime_LinearActuatorMotor = micros();
float pos_LinearActuatorMotor = 0;
float posPrev_LinearActuatorMotor = 0;
float deltaT_LinearActuatorMotor = 0;

// PID variables for reamer motor 
float vel_ReamerMotor = 0;
int set_val_ReamerMotor = 0;
float PID_P_ReamerMotor = 0.15;
float PID_I_ReamerMotor = 0.03;
float PID_D_ReamerMotor = 0.0;
float error_integral_ReamerMotor = 0;
float error_derivative_ReamerMotor = 0;
float error_proportional_ReamerMotor = 0;
float prev_rpm_ReamerMotor = 0;
int val_ReamerMotor = 0;


// PID variables for linear actuator motor 
float curr_val_LinearActuatorMotor = 0;
int set_val_LinearActuatorMotor = 0;
float prev_val_LinearActuatorMotor;
float PID_P_LinearActuatorMotor = 0.15;
float PID_I_LinearActuatorMotor = 0.03;
float PID_D_LinearActuatorMotor = 0.0;
float error_integral_LinearActuatorMotor = 0;
float error_derivative_LinearActuatorMotor = 0;
float error_proportional_LinearActuatorMotor = 0;
float prev_rpm_LinearActuatorMotor = 0;
int val_LinearActuatorMotor = 0;
float val_LinearActuatorMotor_map = 0;
float vel_LinearActuatorMotor = 0;

// Timing variables
unsigned long pid_timer_ReamerMotor = 0;
unsigned long rpm_timer_ReamerMotor = 0;
unsigned long pid_timer_LinearActuatorMotor = 0;
unsigned long rpm_timer_LinearActuatorMotor = 0;

// End-effector controls variables 
bool startReaming = false; 
bool startDynamicComp = false;
bool startCalibration = false;
bool calibrationInProgress = false;
float absPosReamerMotor = 0;


enum states 
{
  CALIBRATE,
  WAITFORCMD,
  MOVEUNTILCONTACT,
  STARTREAMING,
  DYNAMICCOMP,
  DONEREAMING
};

enum states currentState = WAITFORCMD;

// ROS Callback functions 
void changevelocity_ReamerMotor( const std_msgs::Int16& velocity_ReamerMotor){
  stop = false;
  if ((millis() - pid_timer_ReamerMotor) > 400) {
    if(velocity_ReamerMotor.data > 0){
      digitalWrite(DIR_Pin_ReamerMotor,LOW);
    }
    else{
      digitalWrite(DIR_Pin_ReamerMotor,HIGH);
    }
    set_val_ReamerMotor = velocity_ReamerMotor.data;
    val_ReamerMotor = map(abs(velocity_ReamerMotor.data),0,601,0,255);
    // analogWrite(PWM_Pin_ReamerMotor,val_ReamerMotor);
    //  delay(400);
    pid_timer_ReamerMotor = millis();
  }
}

void changevelocity_LinearActuatorMotor(const std_msgs::Int16& velocity_LinearActuatorMotor){
  stop = false;
  if ((millis() - pid_timer_LinearActuatorMotor) > 400) {
    if(velocity_LinearActuatorMotor.data > 0){
      digitalWrite(DIR_Pin_LinearActuatorMotor,LOW);
    }
    else{
      digitalWrite(DIR_Pin_LinearActuatorMotor,HIGH);
    }
    set_val_LinearActuatorMotor = velocity_LinearActuatorMotor.data;
    val_LinearActuatorMotor = map(abs(velocity_LinearActuatorMotor.data),0,116,0,255);
    // analogWrite(PWM_Pin_LinearActuatorMotor,val_LinearActuatorMotor);
    //  delay(400);
    pid_timer_LinearActuatorMotor = millis();
  }
  
}

// Function to get the trigger to start reaming from arm controls
void getReamingCmd(const std_msgs::Bool& reamingCmd){
  startReaming = reamingCmd.data;
  currentState = STARTREAMING;
}

// Function to get trigger to begin dynamic compensation from arm controls
void getDynamicCompCmd(const std_msgs::Bool& dynamicCompCmd){
  startDynamicComp = dynamicCompCmd.data;
  if(startDynamicComp){
    startReaming = false;
     
  }
}

// Function to get trigger to begin calirbation from arm controls
void getCalibrationCmd(const std_msgs::Bool& calibrationCmd){
  nh.loginfo("Inside callback");
  startCalibration = calibrationCmd.data;
  if(startCalibration){
    currentState = CALIBRATE;
  }
}


// ROS Subscribers 
ros::Subscriber<std_msgs::Int16> sub_ReamerMotor("reamer_speed", &changevelocity_ReamerMotor);
ros::Subscriber<std_msgs::Int16> sub_LinearActuatorMotor("linear_actuator_speed", &changevelocity_LinearActuatorMotor);
ros::Subscriber<std_msgs::Bool> sub_reaming_cmd("start_reaming", &getReamingCmd);
ros::Subscriber<std_msgs::Bool> sub_dynamic_comp_cmd("start_dynamic_compensation", &getDynamicCompCmd);
ros::Subscriber<std_msgs::Bool> sub_calibrate_cmd("start_ee_calibration", &getCalibrationCmd);

// Helper functions 
void stopMotors(){
  set_val_ReamerMotor = 0;
  analogWrite(PWM_Pin_LinearActuatorMotor, 0); 
}

void triggerLimSwitch(){
  nh.loginfo("Stopping!");
  stop = true;
  stopMotors();
}

void calibrateLinearActuatorMotor(){

  //Actuate until limit switch has been reached 
  if(!stop){
    nh.loginfo("Sending command to motor...");
    calibrationInProgress = true;
    digitalWrite(DIR_Pin_LinearActuatorMotor,HIGH);
    analogWrite(PWM_Pin_LinearActuatorMotor, 100); 
  }

  else{
    digitalWrite(DIR_Pin_LinearActuatorMotor,LOW);
    analogWrite(PWM_Pin_LinearActuatorMotor, 0); 
    encoderValue_LinearActuatorMotor = 0;
    set_val_LinearActuatorMotor = 1245;
    currentState = WAITFORCMD;
    calibrationInProgress = false;
  }

}


void setup()
{
  Serial.begin(57600);

  pinMode(CURR_1,INPUT);
  pinMode(PWM_Pin_ReamerMotor, OUTPUT);
  pinMode(PWM_Pin_LinearActuatorMotor, OUTPUT);
  pinMode(DIR_Pin_ReamerMotor, OUTPUT);
  pinMode(DIR_Pin_LinearActuatorMotor, OUTPUT);

  pinMode(ENCA_Pin_ReamerMotor, INPUT_PULLUP);
  pinMode(ENCA_Pin_LinearActuatorMotor, INPUT_PULLUP);

  pinMode(ENCB_Pin_ReamerMotor, INPUT);
  pinMode(ENCB_Pin_LinearActuatorMotor, INPUT);

  analogWrite(PWM_Pin_ReamerMotor,LOW);
  analogWrite(PWM_Pin_LinearActuatorMotor,LOW);

  // attachPCINT(digitalPinToPCINT(ENCA_Pin),encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Pin_ReamerMotor), encoder_ReamerMotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Pin_LinearActuatorMotor), encoder_LinearActuatorMotor, FALLING); 

  //Limit switches 
  pinMode(LIM_1, INPUT_PULLUP);
  pinMode(LIM_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIM_1), triggerLimSwitch, HIGH);
  attachInterrupt(digitalPinToInterrupt(LIM_2), triggerLimSwitch, HIGH);


//  nh.getHardware()->setBaud(9600);

  nh.initNode();
  nh.subscribe(sub_ReamerMotor);
  nh.advertise(pub_ReamerMotor);
  nh.subscribe(sub_LinearActuatorMotor);
  nh.subscribe(sub_reaming_cmd);
  nh.subscribe(sub_dynamic_comp_cmd);
  nh.subscribe(sub_calibrate_cmd);
  nh.advertise(pub_LinearActuatorMotor);
  nh.advertise(pub_C1);
  pid_timer_ReamerMotor = millis();
  pid_timer_ReamerMotor = millis();
  rpm_timer_ReamerMotor = millis();
  rpm_timer_LinearActuatorMotor = millis();

}

void loop()
{
  // Serial.println("here");
  nh.spinOnce();

  // High level task-controller functions 

  switch (currentState) {
    
    // Calibrate linear actuator position 
    case CALIBRATE:
      calibrateLinearActuatorMotor();    
      break;

    //Wait until you get the actuation signal from arm controller
    case WAITFORCMD: 
      
      nh.loginfo("Waiting for command...");
      break;

    //Actuate motor until contact is made with the pelvis
    case MOVEUNTILCONTACT:
      // actuateUntilContact();
      currentState = STARTREAMING;
      break;

    //Ream as long as pelvis error is within thresholds and goal has not been reached 
    case STARTREAMING:
      
      break;

    // Dynamic compensation - change state back to 1 after performing compensation routine 
    case DYNAMICCOMP:
      
      currentState = WAITFORCMD;
      break;

    // Goal has been reached, stop reaming! 
    case DONEREAMING: 
      
      stopMotors();
      nh.loginfo("Done reaming!");
      break;

  
    default:
      
      nh.loginfo("Invalid state, stopping reaming!");
      stopMotors();
      break;
}

  // Low level motor velocity controllers
  // if(stop){
  //   nh.loginfo("Limit Switch triggered, stopping!");
  // }
 
  if (((millis()-rpm_timer_ReamerMotor)) > 100 && (!stop) && (!calibrationInProgress)){
    getPos_LinearActuatorMotor();
    pidControl_LinearActuatorMotor();
    rpm_timer_LinearActuatorMotor = millis();
  }

  if (((millis()-rpm_timer_LinearActuatorMotor)) > 100 && (!stop)){
    getRPM_LinearActuatorMotor();
    pidControl_LinearActuatorMotor();
    rpm_timer_LinearActuatorMotor = millis();
  }

  
}

void getCurrent(){
  double current = (analogRead(CURR_1)-510)/14;
  curr_1.data = current;
  pub_C1.publish(&curr_1);
}

void getRPM_ReamerMotor(){
  currentTime_ReamerMotor = micros();
  deltaT_ReamerMotor = ((float) (currentTime_ReamerMotor-previousTime_ReamerMotor)/1000000); //TODO: Find the right constant 
  pos_ReamerMotor = encoderValue_ReamerMotor;
  vel_ReamerMotor = float(pos_ReamerMotor-posPrev_ReamerMotor)/deltaT_ReamerMotor*0.37;
  rpm_ReamerMotor.data = -vel_ReamerMotor;
  pub_ReamerMotor.publish(&rpm_ReamerMotor);
  posPrev_ReamerMotor = pos_ReamerMotor;
  previousTime_ReamerMotor = micros();
  rpm_timer_ReamerMotor = millis();
  //  delay(100);
}

void getPos_LinearActuatorMotor(){
  curr_val_LinearActuatorMotor = encoderValue_LinearActuatorMotor;
}

void getRPM_LinearActuatorMotor(){
  currentTime_LinearActuatorMotor = micros();
  deltaT_LinearActuatorMotor = ((float) (currentTime_LinearActuatorMotor-previousTime_LinearActuatorMotor)/1000000);
  pos_LinearActuatorMotor = encoderValue_LinearActuatorMotor;
  vel_LinearActuatorMotor = float(pos_LinearActuatorMotor-posPrev_LinearActuatorMotor)/deltaT_LinearActuatorMotor*0.37;
  rpm_LinearActuatorMotor.data = -vel_LinearActuatorMotor;
  pub_LinearActuatorMotor.publish(&rpm_LinearActuatorMotor);
  posPrev_LinearActuatorMotor = pos_LinearActuatorMotor;
  previousTime_LinearActuatorMotor = micros();
  rpm_timer_LinearActuatorMotor = millis();
  //  delay(100);
}

void encoder_ReamerMotor(){
  // Serial.println("Inside encoder");
  if (digitalRead(ENCB_Pin_ReamerMotor) == HIGH)
    encoderValue_ReamerMotor++;
  else
    encoderValue_ReamerMotor--;  
}

void encoder_LinearActuatorMotor(){
  // Serial.println("Inside encoder");
  if (digitalRead(ENCB_Pin_LinearActuatorMotor)== HIGH)
    encoderValue_LinearActuatorMotor++;
  else
    encoderValue_LinearActuatorMotor--;  
}


// void pidVelocityControl_ReamerMotor(){
//   error_proportional_ReamerMotor = float(set_val_ReamerMotor)-abs(vel_ReamerMotor);
//   error_derivative_ReamerMotor = (prev_rpm_ReamerMotor-vel_ReamerMotor)/deltaT_ReamerMotor;
//   prev_rpm_ReamerMotor = vel_ReamerMotor;
//   error_integral_ReamerMotor = (float(set_val_ReamerMotor)-abs(vel_ReamerMotor))*deltaT_ReamerMotor;
//   val_ReamerMotor = val_ReamerMotor + int((PID_P_ReamerMotor*error_proportional_ReamerMotor+PID_D_ReamerMotor*error_derivative_LinearActuatorMotor+PID_I_ReamerMotor*error_integral_ReamerMotor));
//   if (val_ReamerMotor > 255){
//     val_ReamerMotor = 255;
//   }
//   if (val_ReamerMotor < 0){
//     val_ReamerMotor = 0;
//   }
    
//   analogWrite(PWM_Pin_ReamerMotor, val_ReamerMotor);
// }

// TODO: Fix this
void pidControl_LinearActuatorMotor(){
  error_proportional_LinearActuatorMotor = float(set_val_LinearActuatorMotor)-abs(curr_val_LinearActuatorMotor);
  error_derivative_LinearActuatorMotor = (prev_val_LinearActuatorMotor-curr_val_LinearActuatorMotor)/deltaT_LinearActuatorMotor;
  prev_val_LinearActuatorMotor = curr_val_LinearActuatorMotor;
  error_integral_LinearActuatorMotor += (float(set_val_LinearActuatorMotor)-abs(curr_val_LinearActuatorMotor))*deltaT_LinearActuatorMotor;
  val_LinearActuatorMotor = val_LinearActuatorMotor + int((PID_P_LinearActuatorMotor*error_proportional_LinearActuatorMotor+PID_D_LinearActuatorMotor*error_derivative_LinearActuatorMotor+PID_I_LinearActuatorMotor*error_integral_LinearActuatorMotor));
  if (val_LinearActuatorMotor > 50){
    val_LinearActuatorMotor = 50;
  }
  if (val_LinearActuatorMotor < 0){
    val_LinearActuatorMotor = 0;
  }

  val_LinearActuatorMotor_map = map(val_LinearActuatorMotor,0,116,0,255);
  const char* valstr = String(val_LinearActuatorMotor_map).c_str();
  analogWrite(PWM_Pin_LinearActuatorMotor, val_LinearActuatorMotor_map);
}


void pidControl_ReamerMotor(){
  error_proportional_ReamerMotor = float(set_val_ReamerMotor)-abs(vel_ReamerMotor);
  error_derivative_ReamerMotor = (prev_rpm_ReamerMotor-vel_ReamerMotor)/deltaT_ReamerMotor;
  prev_rpm_ReamerMotor = vel_ReamerMotor;
  error_integral_ReamerMotor += (float(set_val_ReamerMotor)-abs(vel_ReamerMotor))*deltaT_ReamerMotor;
  val_ReamerMotor = val_ReamerMotor + int((PID_P_ReamerMotor*error_proportional_ReamerMotor+PID_D_ReamerMotor*error_derivative_ReamerMotor+PID_I_ReamerMotor*error_integral_ReamerMotor));
  if (val_ReamerMotor > 255){
    val_ReamerMotor = 255;
  }
  if (val_ReamerMotor < 0){
    val_ReamerMotor = 0;
  }  
  analogWrite(PWM_Pin_ReamerMotor, val_ReamerMotor);
}
