#pragma once
#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"
class StartTrigger {
 VL53L0X sensor;uint8_t hits=0;uint32_t last=0;bool ok=false;
 public:
 bool begin(){
  // Release JTAG pins PB3/PB4/PA15, preserve PA13/PA14 SWD.
  __HAL_RCC_AFIO_CLK_ENABLE();__HAL_AFIO_REMAP_SWJ_NOJTAG();
  for(uint32_t p:std::initializer_list<uint32_t>{PB3,PB4,PA11,PA12,cfg::FORWARD}){digitalWrite(p,LOW);pinMode(p,OUTPUT);}
  Wire.setSDA(PB7);Wire.setSCL(PB6);Wire.begin();Wire.setClock(100000);
  delay(10);digitalWrite(cfg::FORWARD,HIGH);delay(10);
  sensor.setTimeout(60);ok=sensor.init();
  if(ok){ok=sensor.setMeasurementTimingBudget(20000);sensor.startContinuous(40);}
  return ok;
 }
 void reset(){hits=0;last=millis();}
 bool covered(){
  if(!ok||millis()-last<40)return false;
  last=millis();
  uint16_t mm=sensor.readRangeContinuousMillimeters();
  if(sensor.timeoutOccurred()||mm<10||mm>50)hits=0;else ++hits;
  return hits>=3;
 }
};
