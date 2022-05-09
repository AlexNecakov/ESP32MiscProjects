#include "oEnc.h"
int color_changes = 0;
int lastVal = 0;

void countChanges(int adcReading)
{
    int currval = adcReading < ADC_THRESH;
    color_changes += currval != lastVal;
    lastVal = currval;

    vTaskDelay(10 / portTICK_RATE_MS);
}
float getspeed(int dir, float dt)
{
    float rotations = (float)color_changes / (float)WHEEL_SEGMENTS;
    float speedRPM = ((float)rotations / (dt)) * 60.0;
    //   printf("Color changes %d\n", color_changes);
    color_changes = 0;
    float ret_val = (float)dir * (float)speedRPM * (1.0 / 60.0) * WHEEL_CIRC;

    return ret_val;
}
