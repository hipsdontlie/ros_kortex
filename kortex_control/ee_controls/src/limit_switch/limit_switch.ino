#define LIMIT_SWITCH_PIN_1 2
#define LIMIT_SWITCH_PIN_2 3
bool stop1 = false;
bool stop2 = false;
 

void setup() {
  Serial.begin(9600);
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN_1), triggerLimSwitch1, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN_2), triggerLimSwitch2, RISING);

}

void triggerLimSwitch1(){
  stop1 = true;
}
 

 void triggerLimSwitch2(){
  stop2 = true;
}
 
void loop() {



  if(stop1){
    Serial.println("Activated lim switch 1!");
  }

  else{
    Serial.println("Not activated lim switch 1!");
  }

  if(stop2){
    Serial.println("Activated lim switch 2!");
  }

    else{
    Serial.println("Not activated lim switch 2!");
  }

}