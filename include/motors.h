#pragma once
#include <HardwareTimer.h>
#include "config.h"
#include "control.h"
// GPIO PWM: TIM2 IRQ 20 kHz, 100 slices => PWM 200 Hz, 1% resolution.
// Avoids implicit TIM1 complementary-output polarity on PB13/PB14.
namespace motors {
extern HardwareTimer timer;
extern volatile uint8_t dutyL,dutyR,phase;
extern volatile uint16_t lease;
extern volatile bool expired;
constexpr uint32_t MASK=(1UL<<14)|(1UL<<13);
inline void tick(){
 if(lease) {if(--lease==0){dutyL=dutyR=0;expired=true;}}
 uint32_t high=0;
 if(phase<dutyL)high|=1UL<<14;
 if(phase<dutyR)high|=1UL<<13;
 GPIOB->BSRR=high|((MASK & ~high)<<16);
 if(++phase==100)phase=0;
}
inline void zero(){
 uint32_t mask=__get_PRIMASK();__disable_irq();
 dutyL=dutyR=0;lease=0;GPIOB->BSRR=MASK<<16;__set_PRIMASK(mask);
}
inline void stop(){ // short brake: IN1=IN2=HIGH on TB6612
 zero();for(uint32_t p:std::initializer_list<uint32_t>{cfg::L1,cfg::L2,cfg::R1,cfg::R2})digitalWrite(p,HIGH);
}
inline void coast(){ // only for hand-push encoder calibration
 zero();for(uint32_t p:std::initializer_list<uint32_t>{cfg::L1,cfg::L2,cfg::R1,cfg::R2})digitalWrite(p,LOW);
 // TB6612 coast requires both direction pins LOW and PWM HIGH.
 uint32_t mask=__get_PRIMASK();__disable_irq();dutyL=dutyR=100;__set_PRIMASK(mask);
}
inline void arm(){
 zero();expired=false;
 digitalWrite(cfg::L1,cfg::REVERSE_MOTOR_L?LOW:HIGH);
 digitalWrite(cfg::L2,cfg::REVERSE_MOTOR_L?HIGH:LOW);
 digitalWrite(cfg::R1,cfg::REVERSE_MOTOR_R?LOW:HIGH);
 digitalWrite(cfg::R2,cfg::REVERSE_MOTOR_R?HIGH:LOW);
}
inline void command(float l,float r){
 uint32_t mask=__get_PRIMASK();__disable_irq();
 if(!expired){dutyL=(uint8_t)(bound(l,0,cfg::MAX_DUTY)+0.5f);
 dutyR=(uint8_t)(bound(r,0,cfg::MAX_DUTY)+0.5f);lease=2000;} //100 ms
 __set_PRIMASK(mask);
}
inline void begin(){
 for(uint32_t p:std::initializer_list<uint32_t>{cfg::PWM_L,cfg::PWM_R,cfg::L1,cfg::L2,cfg::R1,cfg::R2}){
  digitalWrite(p,LOW);pinMode(p,OUTPUT);
 }
 stop();timer.setOverflow(20000,HERTZ_FORMAT);timer.attachInterrupt(tick);timer.resume();
}
}
