#ifndef CRAWLER_H
#define CRAWLER_H
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#define STEER_HIGH 2200
#define STEER_MID 1500
#define STEER_LOW 800
#define ESC_GPIO (14)   // GPIO connects to the PWM signal line
#define STEER_GPIO (32) // GPIO connects to the PWM signal line

#define ACCEL_TASK_DELAY 500
#define STEER_TASK_DELAY 500

#define ESC_MID_DEFAULT 1400
#define STATE_DRIVE 1
#define STATE_NEUTRAL 0
#define STATE_REVERSE -1


void initPWM();
void steerSetPWM(int pwmUS);

void steerReset();

void steerLeft();

void steerRight();

void driveSetPWM(int pwmUS);

void driveCalibrate(int neutralUS);

void driveNeutral();

void driveForward(int pwmUS);

void driveReverse(int pwmUS);

void driveBrake();

void setState( int state);
int getState();
void setmidpoint(int mid);
int getmidpoint();
#endif
