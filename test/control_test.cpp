#include <initializer_list>
#include "control.h"
#include <cassert>
#include <cstdio>
int main(){
 int n=0;uint8_t a=0;for(uint8_t b:{1,3,2,0}){n+=quadrature(a,b);a=b;}assert(n==4);
 n=0;for(uint8_t b:{2,3,1,0}){n+=quadrature(a,b);a=b;}assert(n==-4);
 assert(quadrature(0,3)==0);assert(quadrature(1,1)==0);
 for(uint8_t a=0;a<4;++a)for(uint8_t b=0;b<4;++b){
  assert(quadrature(a,b)==-quadrature(b,a));
  if(a==b || (a^b)==3)assert(quadrature(a,b)==0);
 }
 // A left wheel ahead must get a lower requested speed.
 float correction=bound(2*(10.0f-5.0f),-20,20);
 assert(80-correction < 80+correction);
 SpeedPI pi;for(int i=0;i<10000;++i)assert(pi.update(80,0,.01,.08,.12,10,.05,25)<=25);
 assert(pi.integral<25);assert(pi.update(0,0,.01,.08,.12,10,.05,25)==0);
 float v=profileSpeed(1000,0,.01,80,100,100);assert(fabsf(v-1)<.001);
 assert(profileSpeed(0,80,.01,80,100,100)==0);
 // Closed-loop asymmetric first-order motor simulation, real mm calibration.
 float l=0,r=0,vl=0,vr=0,s=0;SpeedPI pl,pr;
 for(int i=0;i<2500;++i){float rem=1000-(l+r)/2;if(rem<=1)break;
  s=profileSpeed(rem,s,.01,80,100,100);float corr=bound(2*(l-r),-20,20);
  float dl=pl.update(bound(s-corr,0,100),vl,.01,.08,.12,10,.05,25);
  float dr=pr.update(bound(s+corr,0,100),vr,.01,.08,.12,10,.05,25);
  vl+=.1f*(bound((dl-8)*10,0,300)-vl);vr+=.1f*(bound((dr-9)*9,0,300)-vr);
  l+=vl*.01f;r+=vr*.01f;
 }
 assert(fabsf(l-r)<5);assert((l+r)/2>=999 && (l+r)/2<1002);
 std::printf("PASS: quadrature, PI saturation/reset, profile, straight simulation: L=%.2f R=%.2f\n",l,r);
}
