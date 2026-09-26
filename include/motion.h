#pragma once
#include "encoders.h"
#include "motors.h"
class Motion {
 EncoderCounts origin{},last{};SpeedPI piL,piR;
 uint32_t lastUs=0,startMs=0,movedL=0,movedR=0;
 float speed=0,vL=0,vR=0;
 public:
 bool active=false,done=false;const char* fault=nullptr;
 float leftMM=0,rightMM=0,dutyL=0,dutyR=0;
 void stop(){active=false;motors::stop();dutyL=dutyR=0;piL.reset();piR.reset();}
 void fail(const char* why){fault=why;stop();}
 void start(){
  origin=last=encoders::snapshot();piL.reset();piR.reset();
  leftMM=rightMM=speed=vL=vR=dutyL=dutyR=0;fault=nullptr;done=false;
  lastUs=micros();startMs=movedL=movedR=millis();motors::arm();active=true;
 }
 void update(){
  if(!active)return;
  const uint32_t now=millis(),runMs=now-startMs;
  // Duration starts with motor control, excluding the Forward trigger delay.
  // Check every loop, not only at the 100 Hz control tick.
  if(runMs>=cfg::DRIVE_TIME_MS){done=true;stop();return;}
  uint32_t nowUs=micros(),elapsed=nowUs-lastUs;if(elapsed<cfg::CONTROL_US)return;
  if(elapsed>50000){fail("CONTROL_LATE");return;}lastUs=nowUs;
  float dt=elapsed*1e-6f;auto c=encoders::snapshot();
  float dL=(c.left-last.left)*cfg::ENCODER_SIGN_L/cfg::COUNTS_PER_MM_L;
  float dR=(c.right-last.right)*cfg::ENCODER_SIGN_R/cfg::COUNTS_PER_MM_R;
  if(c.left!=last.left)movedL=now;
  if(c.right!=last.right)movedR=now;
  last=c;
  leftMM=(c.left-origin.left)*cfg::ENCODER_SIGN_L/cfg::COUNTS_PER_MM_L;
  rightMM=(c.right-origin.right)*cfg::ENCODER_SIGN_R/cfg::COUNTS_PER_MM_R;
  vL+=0.35f*(dL/dt-vL);vR+=0.35f*(dR/dt-vR);
  if(motors::expired){fail("PWM_WATCHDOG");return;}
  if(leftMM < -2 || rightMM < -2){fail("ENCODER_DIRECTION");return;}
  if(fabsf(vL)>cfg::OVERSPEED_MM_S || fabsf(vR)>cfg::OVERSPEED_MM_S){fail("OVERSPEED");return;}
  if(c.invalidLeft-origin.invalidLeft>20||c.invalidRight-origin.invalidRight>20){fail("ENCODER_NOISE");return;}
  // No distance endpoint and no skew-triggered stop: both wheels keep correcting.
  const float secondsLeft=(cfg::DRIVE_TIME_MS-runMs)*0.001f;
  const float brakingCap=cfg::DECEL_MM_S2*secondsLeft;
  const float baseCap=bound(brakingCap,0,cfg::SPEED_MM_S);
  speed=bound(speed+cfg::ACCEL_MM_S2*dt,0,baseCap);
  auto refs=synchronizedTargets(speed,leftMM-rightMM,vL,vR,cfg::SPEED_MM_S,
     cfg::MAX_SYNC_MM_S,cfg::STRAIGHT_KP,cfg::SYNC_SPEED_KP,cfg::SYNC_SOFT_MM);
  // Apply final deceleration to EACH wheel, including its sync correction.
  float targetL=bound(refs.left,0,brakingCap),targetR=bound(refs.right,0,brakingCap);
  if(targetL<=1)movedL=now;
  if(targetR<=1)movedR=now;
  if(runMs>1200 && (now-movedL>800||now-movedR>800)){fail("STALL_OR_NO_ENCODER");return;}
  dutyL=piL.update(targetL,vL,dt,cfg::SPEED_KP,cfg::SPEED_KI,cfg::FF_STATIC,cfg::FF_PER_SPEED,cfg::MAX_DUTY);
  dutyR=piR.update(targetR,vR,dt,cfg::SPEED_KP,cfg::SPEED_KI,cfg::FF_STATIC,cfg::FF_PER_SPEED,cfg::MAX_DUTY);
  motors::command(dutyL,dutyR);
 }
};