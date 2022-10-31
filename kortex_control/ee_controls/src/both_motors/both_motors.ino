#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <PinChangeInterrupt.h> // From Library "PinChangeInterrupt" by NicoHood

// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 rpm_M1;
std_msgs::Float64 rpm_M2;
ros::Publisher pub_M1("reamer_velocity",&rpm_M1);
ros::Publisher pub_M2("linear_actuator_velocity",&rpm_M2);


// Pin Definitions


//Limit switch pins 
int LIM_1 = 2; 
int LIM_2 = 3;
bool stop = false;

// Reamer Motor 
int PWM_Pin_M1 = 6;
int DIR_Pin_M1 = 5;
int ENCA_Pin_M1 = 21;
int ENCB_Pin_M1 = 20;
float encoderValue_M1 = 0;

// Linear Actuator Motor 
int PWM_Pin_M2 = 7;
int DIR_Pin_M2 = 8;
int ENCB_Pin_M2 = 18;
int ENCA_Pin_M2 = 19;
float encoderValue_M2 = 0;

float currentTime_M1 = micros();
float previousTime_M1 = micros();
float pos_M1 = 0;
float posPrev_M1 = 0;
float deltaT_M1 = 0;

float currentTime_M2 = micros();
float previousTime_M2 = micros();
float pos_M2 = 0;
float posPrev_M2 = 0;
float deltaT_M2 = 0;

// PID variables for reamer motor 
float vel_M1 = 0;
int set_val_M1 = 0;
float PID_P_M1 = 0.15;
float PID_I_M1 = 0.03;
float PID_D_M1 = 0.0;
float error_integral_M1 = 0;
float error_derivative_M1 = 0;
float error_proportional_M1 = 0;
float prev_rpm_M1 = 0;
int val_M1 = 0;

// PID variables for end-effector motor 
float vel_M2 = 0;
int set_val_M2 = 0;
float PID_P_M2 = 0.15;
float PID_I_M2 = 0.03;
float PID_D_M2 = 0.0;
float error_integral_M2 = 0;
float error_derivative_M2 = 0;
float error_proportional_M2 = 0;
float prev_rpm_M2 = 0;
int val_M2 = 0;

// Timing variables
unsigned long pid_timer_M1 = 0;
unsigned long rpm_timer_M1 = 0;

unsigned long pid_timer_M2 = 0;
unsigned long rpm_timer_M2 = 0;

//Limit switches
// pinMode(BUTTON_PIN, INPUT);


void changevelocity_M1( const std_msgs::Int16& velocity_M1){
  stop = false;
  if ((millis() - pid_timer_M1) > 400) {
    if(velocity_M1.data > 0){
      digitalWrite(DIR_Pin_M1,LOW);
    }
    else{
      digitalWrite(DIR_Pin_M1,HIGH);
    }
    set_val_M1 = velocity_M1.data;
    val_M1 = map(abs(velocity_M1.data),0,601,0,255);
    // analogWrite(PWM_Pin_M1,val_M1);
    //  delay(400);
    pid_timer_M1 = millis();
  }
  
  
}

void changevelocity_M2( const std_msgs::Int16& velocity_M2){
  stop = false;
  if ((millis() - pid_timer_M2) > 400) {
    if(velocity_M2.data > 0){
      digitalWrite(DIR_Pin_M2,LOW);
    }
    else{
      digitalWrite(DIR_Pin_M2,HIGH);
    }
    set_val_M2 = velocity_M2.data;
    val_M2 = map(abs(velocity_M2.data),0,116,0,255);
    // analogWrite(PWM_Pin_M2,val_M2);
    //  delay(400);
    pid_timer_M2 = millis();
  }
  
}

void triggerLimSwitch(){
  stop = true;
  set_val_M1 = 0;
  set_val_M2 = 0;
  analogWrite(PWM_Pin_M1,0);
  analogWrite(PWM_Pin_M2,0);
}


ros::Subscriber<std_msgs::Int16> sub_M1("reamer_speed", &changevelocity_M1);
ros::Subscriber<std_msgs::Int16> sub_M2("linear_actuator_speed", &changevelocity_M2);

void setup()
{
  Serial.begin(57600);
  pinMode(PWM_Pin_M1, OUTPUT);
  pinMode(PWM_Pin_M2, OUTPUT);
  pinMode(DIR_Pin_M1, OUTPUT);
  pinMode(DIR_Pin_M2, OUTPUT);

  pinMode(ENCA_Pin_M1, INPUT_PULLUP);
  pinMode(ENCA_Pin_M2, INPUT_PULLUP);

  pinMode(ENCB_Pin_M1, INPUT);
  pinMode(ENCB_Pin_M2, INPUT);

  analogWrite(PWM_Pin_M1,LOW);
  analogWrite(PWM_Pin_M2,LOW);

  // attachPCINT(digitalPinToPCINT(ENCA_Pin),encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Pin_M1), encoder_M1, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Pin_M2), encoder_M2, FALLING); 

  //Limit switches 
  pinMode(LIM_1, INPUT_PULLUP);
  pinMode(LIM_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIM_1), triggerLimSwitch, RISING);
  attachInterrupt(digitalPinToInterrupt(LIM_2), triggerLimSwitch, RISING);


//  nh.getHardware()->setBaud(9600);

  nh.initNode();
  nh.subscribe(sub_M1);
  nh.advertise(pub_M1);
  nh.subscribe(sub_M2);
  nh.advertise(pub_M2);
  pid_timer_M1 = millis();
  pid_timer_M1 = millis();
  rpm_timer_M1 = millis();
  rpm_timer_M2 = millis();

}

void loop()
{
  // Serial.println("here");
  nh.spinOnce();

  if(stop){
    nh.loginfo("Stopping!");
  }

  else{
    nh.loginfo("Not Stopping!");
  }
  
  if (((millis()-rpm_timer_M1)) > 100 && (!stop)){
    getRPM_M1();
    pidControl_M1();
    rpm_timer_M1 = millis();
  }

  if (((millis()-rpm_timer_M2)) > 100 && (!stop)){
    getRPM_M2();
    pidControl_M2();
    rpm_timer_M2 = millis();
  }

  
}

void getRPM_M1(){
  currentTime_M1 = micros();
  deltaT_M1 = ((float) (currentTime_M1-previousTime_M1)/1000000);
  pos_M1 = encoderValue_M1;
  vel_M1 = float(pos_M1-posPrev_M1)/deltaT_M1*0.37;
  rpm_M1.data = -vel_M1;
  pub_M1.publish(&rpm_M1);
  posPrev_M1 = pos_M1;
  previousTime_M1 = micros();
  rpm_timer_M1 = millis();
  //  delay(100);
}

void getRPM_M2(){
  currentTime_M2 = micros();
  deltaT_M2 = ((float) (currentTime_M2-previousTime_M2)/1000000);
  pos_M2 = encoderValue_M2;
  vel_M2 = float(pos_M2-posPrev_M2)/deltaT_M2*0.37;
  rpm_M2.data = -vel_M2;
  pub_M2.publish(&rpm_M2);
  posPrev_M2 = pos_M2;
  previousTime_M2 = micros();
  rpm_timer_M2 = millis();
  //  delay(100);
}
void encoder_M1(){
  // Serial.println("Inside encoder");
  if (digitalRead(ENCB_Pin_M1) == HIGH)
    encoderValue_M1++;
  else
    encoderValue_M1--;  
}

void encoder_M2(){
  // Serial.println("Inside encoder");
  if (digitalRead(ENCB_Pin_M2)== HIGH)
    encoderValue_M2++;
  else
    encoderValue_M2--;  
}


void pidControl_M1(){
  error_proportional_M1 = float(set_val_M1)-abs(vel_M1);
  error_derivative_M1 = (prev_rpm_M1-vel_M1)/deltaT_M1;
  prev_rpm_M1 = vel_M1;
  error_integral_M1 = (float(set_val_M1)-abs(vel_M1))*deltaT_M1;
  val_M1 = val_M1 + int((PID_P_M1*error_proportional_M1+PID_D_M1*error_derivative_M2+PID_I_M1*error_integral_M1));
  if (val_M1 > 255){
    val_M1 = 255;
  }
  if (val_M1 < 0){
    val_M1 = 0;
  }
    
  analogWrite(PWM_Pin_M1, val_M1);
}


void pidControl_M2(){
  error_proportional_M2 = float(set_val_M2)-abs(vel_M2);
  error_derivative_M2 = (prev_rpm_M2-vel_M2)/deltaT_M2;
  prev_rpm_M2 = vel_M2;
  error_integral_M2 = (float(set_val_M2)-abs(vel_M2))*deltaT_M2;
  val_M2 = val_M2 + int((PID_P_M2*error_proportional_M2+PID_D_M2*error_derivative_M2+PID_I_M2*error_integral_M2));
  if (val_M2 > 255){
    val_M2 = 255;
  }
  if (val_M2 < 0){
    val_M2 = 0;
  }
    
  analogWrite(PWM_Pin_M2, val_M2);
}
