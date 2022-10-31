#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <PinChangeInterrupt.h> // From Library "PinChangeInterrupt" by NicoHood

// ROS Definitions
ros::NodeHandle nh;
std_msgs::Float64 rpm;
ros::Publisher pub("reamer_velocity",&rpm);

// Pin Definitions
int PWM_Pin = 6;
int DIR_Pin = 5;
int ENCA_Pin = 21;
int ENCB_Pin = 20;
float encoderValue = 0;
float currentTime = micros();
float previousTime = micros();
float pos = 0;
float posPrev = 0;
float deltaT = 0;

// PID variables
float vel = 0;
int set_val = 0;
float PID_P = 0.15;
float PID_I = 0.03;
float PID_D = 0.0;
float error_integral = 0;
float error_derivative = 0;
float error_proportional = 0;
float prev_rpm = 0;
int val = 0;

// Timing variables
unsigned long pid_timer = 0;
unsigned long rpm_timer = 0;

void changevelocity( const std_msgs::Int16& velocity){
  if ((millis() - pid_timer) > 400) {
    if(velocity.data > 0){
      digitalWrite(DIR_Pin,LOW);
    }
    else{
      digitalWrite(DIR_Pin,HIGH);
    }
    set_val = velocity.data;
    val = map(abs(velocity.data),0,601,0,255);
    analogWrite(PWM_Pin,val);
    //  delay(400);
    pid_timer = millis();
  }
  
  
}

ros::Subscriber<std_msgs::Int16> sub("reamer_speed", &changevelocity);

void setup()
{
  Serial.begin(57600);
  pinMode(PWM_Pin, OUTPUT);
  pinMode(DIR_Pin, OUTPUT);
  pinMode(ENCA_Pin, INPUT_PULLUP);
  pinMode(ENCB_Pin, INPUT);
  analogWrite(PWM_Pin,LOW);
  // attachPCINT(digitalPinToPCINT(ENCA_Pin),encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_Pin), encoder, FALLING);
//  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pid_timer = millis();
  rpm_timer = millis();
}

void loop()
{
  // Serial.println("here");
  nh.spinOnce();
  
  if ((millis()-rpm_timer) > 100){
    getRPM();
    Serial.println("here 2");
    pidControl();
    rpm_timer = millis();
  }
}

void getRPM(){
  currentTime = micros();
  deltaT = ((float) (currentTime-previousTime)/1000000);
  pos = encoderValue;
  vel = float(pos-posPrev)/deltaT*0.37;
  rpm.data = -vel;
  pub.publish(&rpm);
  posPrev = pos;
  previousTime = micros();
  rpm_timer = millis();
  //  delay(100);
}

void encoder(){
  // Serial.println("Inside encoder");
  if (digitalRead(ENCB_Pin) == HIGH)
    encoderValue++;
  else
    encoderValue--;  
}
void pidControl(){
  error_proportional = float(set_val)-abs(vel);
  error_derivative = (prev_rpm-vel)/deltaT;
  prev_rpm = vel;
  error_integral = (float(set_val)-abs(vel))*deltaT;
  val = val + int((PID_P*error_proportional+PID_D*error_derivative+PID_I*error_integral));
  if (val > 255){
    val = 255;
  }
  if (val < 0){
    val = 0;
  }
    
  analogWrite(PWM_Pin, val);
}