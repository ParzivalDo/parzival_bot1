#include <Arduino.h>
#include <initializer_list>
#include "buttons.h"
#include "motion.h"
#include "start_trigger.h"
Button startButton(cfg::SW2),stopButton(cfg::SW3);
Motion motion;StartTrigger trigger;
enum class State {IDLE,ARMED,DELAY_START,RUNNING,DONE,FAULT};
State state=State::IDLE;bool sensorOK=false;uint32_t waitSince=0;
void setup(){
 motors::begin();encoders::begin();startButton.begin();stopButton.begin();pinMode(cfg::LED,OUTPUT);
 Serial1.setRx(PA10);Serial1.setTx(PA9);Serial1.begin(115200);
 if(cfg::CALIBRATION_ONLY){motors::coast();Serial1.println("CALIBRATION: push forward 1000 mm; z resets origin");}
 else {sensorOK=trigger.begin();if(!sensorOK){state=State::FAULT;Serial1.println("FORWARD_INIT_FAILED");}}
 Serial1.println("MouseStraight1m: SW2 arm; cover Forward; SW3 stop");
}
void loop(){
 static EncoderCounts calibrationOrigin{};static uint32_t logged=0;
 bool sw2=startButton.pressed();stopButton.pressed();
 if(cfg::CALIBRATION_ONLY){
  if(Serial1.available() && Serial1.read()=='z')calibrationOrigin=encoders::snapshot();
  if(millis()-logged>=250){logged=millis();auto c=encoders::snapshot();
   Serial1.print("RAW_L=");Serial1.print(c.left-calibrationOrigin.left);
   Serial1.print(" RAW_R=");Serial1.print(c.right-calibrationOrigin.right);
   Serial1.print(" badL=");Serial1.print(c.invalidLeft);Serial1.print(" badR=");Serial1.println(c.invalidRight);}
  return;
 }
 if(digitalRead(cfg::SW3)==LOW){motion.stop();state=State::IDLE;}
 else if(sw2){
  if(state==State::ARMED||state==State::DELAY_START||state==State::RUNNING){motion.stop();state=State::IDLE;}
  else if(sensorOK){trigger.reset();state=State::ARMED;Serial1.println("ARMED");}
 }
 if(state==State::ARMED && trigger.covered()){
  // Recheck cancel after a potentially blocking I2C read.
  if(digitalRead(cfg::SW3)==HIGH && digitalRead(cfg::SW2)==HIGH){waitSince=millis();state=State::DELAY_START;}
 }
 if(state==State::DELAY_START && millis()-waitSince>=800){motion.start();state=State::RUNNING;}
 if(state==State::RUNNING){motion.update();
  if(!motion.active){state=motion.done?State::DONE:State::FAULT;
   Serial1.println(motion.done?"DONE":motion.fault);}}
 bool lit=state==State::RUNNING||state==State::DONE;
 if(state==State::ARMED||state==State::DELAY_START)lit=(millis()/300)%2;
 if(state==State::FAULT)lit=(millis()/100)%2;
 digitalWrite(cfg::LED,lit?LOW:HIGH);
 // Small bounded UART line; no I2C or delays during motion.
 if(millis()-logged>=250){logged=millis();
  Serial1.print("L=");Serial1.print(motion.leftMM,1);Serial1.print(" R=");Serial1.print(motion.rightMM,1);
  Serial1.print(" PWM=");Serial1.print(motion.dutyL,0);Serial1.print(',');Serial1.println(motion.dutyR,0);}
}
