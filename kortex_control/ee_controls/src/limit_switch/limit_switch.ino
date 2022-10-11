#define LIMIT_SWITCH_PIN_1 2
#define LIMIT_SWITCH_PIN_2 3
bool stop = false;

void setup() {
  Serial.begin(9600);
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN_1), triggerLimSwitch, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN_2), triggerLimSwitch, RISING);

}

void triggerLimSwitch(){
  stop = true;
}
 
void loop() {
 
  // if (digitalRead(LIMIT_SWITCH_PIN_2) == HIGH)
  // {
  //   Serial.println("Activated 1!");
  // }
 
  // else
  // {
  //   Serial.println("Not activated.");
  // }

  if(stop){
    Serial.println("STOP!");
  }
  else{
    Serial.println("Safe!");
  }
   
}