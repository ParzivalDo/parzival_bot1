#pragma once
#include <Arduino.h>
class Button {
 uint32_t pin,changed=0;bool raw=false,stable=false;
 public:
 explicit Button(uint32_t p):pin(p){}
 void begin(){pinMode(pin,INPUT_PULLUP);raw=stable=digitalRead(pin)==LOW;}
 bool pressed(){
  bool n=digitalRead(pin)==LOW;if(n!=raw){raw=n;changed=millis();}
  if(raw!=stable && millis()-changed>=25){stable=raw;return stable;}return false;
 }
};
