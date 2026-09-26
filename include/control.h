#pragma once
#include <cmath>
#include <cstdint>
inline float bound(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
// Positive sequence: 00 -> 01 -> 11 -> 10 -> 00; reverse gives -1.
inline int quadrature(uint8_t oldAB,uint8_t newAB){
 static const int8_t table[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
 return table[(oldAB<<2)|newAB];
}
struct SpeedPI {
 float integral=0;
 void reset(){integral=0;}
 float update(float target,float measured,float dt,float kp,float ki,
              float ffStatic,float ffSpeed,float maxDuty){
  if(target<=0){reset();return 0;}
  float e=target-measured;
  float candidate=bound(integral+ki*e*dt,-maxDuty,maxDuty);
  float ff=ffStatic+ffSpeed*target;
  float raw=ff+kp*e+candidate;
  if((raw>=0 && raw<=maxDuty)||(raw>maxDuty && e<0)||(raw<0 && e>0)) integral=candidate;
  return bound(ff+kp*e+integral,0,maxDuty);
 }
};
inline float profileSpeed(float remaining,float prior,float dt,float top,float accel,float decel){
 return bound(std::sqrt(2.0f*decel*bound(remaining,0,100000)),0,
              bound(prior+accel*dt,0,top));
}

struct WheelTargets {float left,right;};
inline WheelTargets synchronizedTargets(float base,float positionError,
 float leftSpeed,float rightSpeed,float top,float maxCorrection,
 float positionGain,float velocityGain,float softError){
 float correction=bound(positionGain*positionError+velocityGain*(leftSpeed-rightSpeed),
                        -maxCorrection,maxCorrection);
 WheelTargets refs{bound(base-correction,0,top+maxCorrection),
                   bound(base+correction,0,top+maxCorrection)};
 // Above the soft threshold, make the leading wheel slower than the measured
 // lagging wheel, rather than asking both to accelerate into a growing skew.
 if(positionError>softError)
  refs.left=bound(refs.left,0,bound(rightSpeed-positionGain*(positionError-softError),0,top));
 else if(positionError<-softError)
  refs.right=bound(refs.right,0,bound(leftSpeed-positionGain*(-positionError-softError),0,top));
 return refs;
}