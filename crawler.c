#include "crawler.h"
int escMidPoint = ESC_MID_DEFAULT;
int stateCurrent = STATE_NEUTRAL;

void initPWM()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, STEER_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_config_t pwm_config1 = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config1);

    vTaskDelay(3000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler (3s)
    driveCalibrate(ESC_MID_DEFAULT);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Give yourself time to turn on crawler (3s)
}
void steerSetPWM(int pwmUS)
{
    mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, pwmUS);
    vTaskDelay(STEER_TASK_DELAY / portTICK_PERIOD_MS);
}

void steerReset()
{
    steerSetPWM(STEER_MID);
}

void steerLeft()
{
    steerSetPWM(STEER_HIGH);
}

void steerRight()
{
    steerSetPWM(STEER_LOW);
}

void driveSetPWM(int pwmUS)
{
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwmUS);
    vTaskDelay(ACCEL_TASK_DELAY / portTICK_PERIOD_MS);
}

void driveCalibrate(int neutralUS)
{
    driveSetPWM(neutralUS);
    vTaskDelay(2600 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
    escMidPoint = neutralUS;
    stateCurrent = STATE_NEUTRAL;
}

void driveNeutral()
{
    driveSetPWM(escMidPoint);
    stateCurrent = STATE_NEUTRAL;
}

void driveForward(int pwmUS)
{
    driveSetPWM(escMidPoint + pwmUS);
    stateCurrent = STATE_DRIVE;
}

void driveReverse(int pwmUS)
{
    driveSetPWM(escMidPoint - pwmUS);
    driveNeutral();
    driveSetPWM(escMidPoint - pwmUS);
    stateCurrent = STATE_REVERSE;
}

void driveBrake()
{
    if (stateCurrent == STATE_DRIVE)
    {
        driveSetPWM(escMidPoint - 100);
    }
    else
    {
        driveNeutral();
    }
    stateCurrent = STATE_NEUTRAL;
}





void setState( int state){
  stateCurrent = state;
}
int getState() {return stateCurrent;}
void setmidpoint(int mid){ escMidPoint = mid;}
int getmidpoint(){return escMidPoint;}
