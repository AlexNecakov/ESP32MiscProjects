#ifndef OENC_H
#define OENC_H
#define ADC_THRESH 900
#define WHEEL_CIRC 0.251
#define WHEEL_SEGMENTS 12
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void countChanges(int adcReading);
float getspeed(int dir,float dt);
#endif
