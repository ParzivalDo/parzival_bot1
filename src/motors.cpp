#include "motors.h"
namespace motors {
HardwareTimer timer(TIM2);
volatile uint8_t dutyL=0,dutyR=0,phase=0;
volatile uint16_t lease=0;
volatile bool expired=false;
}
