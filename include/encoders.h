#pragma once
#include "config.h"
#include "control.h"
struct EncoderCounts {int32_t left,right; uint32_t invalidLeft,invalidRight;};
namespace encoders {
extern volatile int32_t left,right;
extern volatile uint32_t badL,badR;
extern volatile uint8_t oldL,oldR;
inline uint8_t readL(){uint32_t p=GPIOA->IDR;return ((p>>1)&1)*2+(p&1);}
inline uint8_t readR(){uint32_t p=GPIOA->IDR;return ((p>>6)&1)*2+((p>>7)&1);}
inline void irqL(){uint8_t n=readL();if((oldL^n)==3)++badL;left+=quadrature(oldL,n);oldL=n;}
inline void irqR(){uint8_t n=readR();if((oldR^n)==3)++badR;right+=quadrature(oldR,n);oldR=n;}
inline EncoderCounts snapshot(){
 uint32_t mask=__get_PRIMASK();__disable_irq();
 EncoderCounts c{left,right,badL,badR};__set_PRIMASK(mask);return c;
}
inline void begin(){
 for(uint32_t p:std::initializer_list<uint32_t>{cfg::LA,cfg::LB,cfg::RA,cfg::RB})pinMode(p,INPUT_PULLUP);
 oldL=readL();oldR=readR();
 attachInterrupt(digitalPinToInterrupt(cfg::LA),irqL,CHANGE);
 attachInterrupt(digitalPinToInterrupt(cfg::LB),irqL,CHANGE);
 attachInterrupt(digitalPinToInterrupt(cfg::RA),irqR,CHANGE);
 attachInterrupt(digitalPinToInterrupt(cfg::RB),irqR,CHANGE);
}
}
