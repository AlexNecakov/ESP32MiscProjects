#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_types.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"
// #include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "soc/rmt_reg.h"

#include "i2chelp.h"
#include "i2cdisp.h"
#include "i2clidar.h"
#include "crawler.h"
#include "oEnc.h"
#include "UARTesp.h"
#include "UDPhelp.h"
#define GOOD_CRIMP_ADDR 0x52
#define RED_JUMPER_ADDR 0x53
#define TWO_HALVES_ADDR 0x54
#define FRONT_LIDAR_ADDR GOOD_CRIMP_ADDR
#define LEFT_LIDAR_ADDR TWO_HALVES_ADDR
#define RIGHT_LIDAR_ADDR RED_JUMPER_ADDR

#define BREAK_LIGHT_GPIO 13

// recive task
uint16_t D_R = 0;
uint16_t D_L = 0;

// Deadzone PID Steer
#define deadzone_steer 10

// PID Steer Parameter
#define STEER_KP 0.5
#define STEER_KI 0
#define STEER_KD 0

#define MAX_ERROR_SIDE 200
#define MAX_SERVO_PWM 700 // update caps
#define CENTER_PWM 1500

//
#define DISTANCE_BRAKEPOINT 30
#define BRAKE_SPEED_THRESHOLD 0

// PID Drive Parameter
#define DRIVE_KP 0.8
#define DRIVE_KI 1
#define DRIVE_KD 0

// TIMER
#define TIMER_DIVIDER 16 //  Hardware timer clock divider

#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds

#define TIMER_INTERVAL_IN_SEC 0.1

#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

#define DT 0.1 //(number of seconds of timer) // update

// ADC
#define ADC_BITWIT ADC_WIDTH_BIT_10
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 250

// PID
#define SPEED_SETPOINT 0.3

#define IPTARTGET "192.168.1.130"
static xQueueHandle timer_queue;
// A simple structure to pass "events" to main task
typedef struct
{
    int flag; // flag for enabling stuff in timer task
} timer_event_t;

// using single adc unit, switch channel/attenuation per sensor

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channelOptical = ADC_CHANNEL_0;

static const adc_atten_t attenOptical = ADC_ATTEN_DB_11;

static const adc_unit_t unit = ADC_UNIT_1;

float speedMS = 0;
int break_flag = 0;
void initI2C()
{
    printf("Initializing I2C\n");
    initI2CMaster();
    initDisplay();

    vTaskDelay(22 / portTICK_PERIOD_MS);
}

void initLED()
{
    gpio_reset_pin(BREAK_LIGHT_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BREAK_LIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BREAK_LIGHT_GPIO, 0);
}

////////////////////////////////////////////////////////////////////////////// ADC
void initADC()
{
    // Check if Two Point or Vref are burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
        printf("eFuse Two Point: Supported\n");
    else
        printf("eFuse Two Point: NOT supported\n");

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
        printf("eFuse Vref: Supported\n");
    else
        printf("eFuse Vref: NOT supported\n");

    adc1_config_width(ADC_BITWIT);
    printf("Inited ADC\n");
}

uint32_t sampleADC()
{
    uint32_t adc_reading = 0;

    adc1_config_channel_atten(channelOptical, attenOptical);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, attenOptical, ADC_BITWIT, DEFAULT_VREF, adc_chars);
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channelOptical);
    }
    adc_reading /= NO_OF_SAMPLES;
    // printf("adc val %d\n", adc_reading);
    free(adc_chars);
    return (adc_reading == 0) ? 1 : adc_reading;
}

void loopPulseCounter()
{
    while (1)
    {
        countChanges(sampleADC());
    }
}

//////////////////////////////////////////////////////////////////////////////////// Timer
void IRAM_ATTR timer_group0_isr(void *para)
{
    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void initTimer()
{
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // Select and initialize basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_IN_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

static void loopAlarm(void *arg)
{
    while (1)
    {
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1)
        {
            speedMS = getspeed(1, TIMER_INTERVAL_IN_SEC);
            // printf("Speed %.2f\n", speedMS);
        }
    }
}

////////////////////////////////////////////////////////////////////////////// PID

void loopSteer()
{
    // PID parameters tuning
    float distanceLeft = 0;
    float distanceRight = 0;
    float output_dist = 0;
    float error_center = 0;
    float previous_error_center = 0;
    float derivative_center = 0;
    float integral_center = 0;
    int output_pwm = 0;

    // measured_value  == distance cm
    while (1)
    {
        // poll sensors
        distanceLeft = D_L;
        distanceRight = D_R;
        // printf("DISTANCE L: %.2f \n", distanceLeft);
        // printf("DISTANCE R: %.2f \n", distanceRight);

        // PID
        error_center = distanceRight - distanceLeft;
        // printf("ERROR CENTER: %.2f \n", error_center);

        integral_center = integral_center + error_center * DT;
        derivative_center = (error_center - previous_error_center) / DT;

        output_dist = STEER_KP * error_center + STEER_KI * integral_center + STEER_KD * derivative_center;
        // printf("output dist %.2f\n", output_dist);
        // Setting error to PID
        if (output_dist <= -MAX_ERROR_SIDE)
        {
            output_pwm = -MAX_SERVO_PWM;
        }
        else if (output_dist < -deadzone_steer)
        {
            output_pwm = (int)(output_dist * (MAX_SERVO_PWM / (MAX_ERROR_SIDE - deadzone_steer)) - deadzone_steer);
        }
        else if (output_dist < deadzone_steer)
        {
            output_pwm = 0;
        }
        else if (output_dist < MAX_ERROR_SIDE)
        {
            output_pwm = (int)(output_dist * (MAX_SERVO_PWM / (MAX_ERROR_SIDE - deadzone_steer)) + deadzone_steer);
        }
        else
        {
            output_pwm = MAX_SERVO_PWM;
        }

        output_pwm = -1 * output_pwm + CENTER_PWM;
        steerSetPWM(output_pwm);
        // printf("OUTPUT SERVO PWM: %d \n", output_pwm);
        // printf("Error: %fcm\n", error_center);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void loopDrive()
{
    // from external ir encoder
    int top_speed_pwm = 200;
    float top_speed_Ms = 2;
    int distanceFront = readLidar(FRONT_LIDAR_ADDR);

    // for cruise control
    float previous_error_speed = 0;
    float derivative_speed = 0;
    float integral_speed = 0;
    float output_speed = 0;
    int output_pwm = 0;

    // measured_value  == distance cm
    while (1)
    {
        // poll sensors
        distanceFront = readLidar((uint8_t)FRONT_LIDAR_ADDR);
        // printf("distanceF = %d\n", distanceFront);
        //  errors for left and right
        float speed_local = speedMS;
        float error_speed = SPEED_SETPOINT - speed_local;
        if (break_flag)
        {
            printf("BREAK\n\n\n");
            gpio_set_level(BREAK_LIGHT_GPIO, 1);
            driveBrake();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if (distanceFront < DISTANCE_BRAKEPOINT)
        {
            if (speedMS > BRAKE_SPEED_THRESHOLD)
            {
                // printf("Braking \n");
                driveBrake();
            }
            gpio_set_level(BREAK_LIGHT_GPIO, 1);
        }
        else
        {
            // PID for cruise control
            integral_speed = integral_speed + error_speed * DT;

            derivative_speed = (error_speed - previous_error_speed) / DT;

            output_speed = DRIVE_KP * error_speed + DRIVE_KI * integral_speed + DRIVE_KD * derivative_speed;

            output_pwm = (int)((output_speed / top_speed_Ms) * top_speed_pwm) + 100;

            previous_error_speed = error_speed;
            //printf("distance front: %d \n", distanceFront);
            //printf("output pwm: %d \n", output_pwm);
            driveForward(output_pwm);
            gpio_set_level(BREAK_LIGHT_GPIO, 0);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
///////////////////////////////////////////////////////////////////////////////////
void dispTask()
{
    while (1)
    {
        float speed_local = speedMS;
        uint16_t displaybuffer[4];
        displaybuffer[0] = alphafonttable[(int)speed_local % 10] | alphafonttable[10];
        displaybuffer[1] = alphafonttable[(int)(speed_local * 10) % 10];
        displaybuffer[2] = alphafonttable[((int)(speed_local * 100)) % 10];
        displaybuffer[3] = alphafonttable[((int)(speed_local * 1000)) % 10];
        writeDisplay(DISPLAY_ADDR, displaybuffer);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

///////////////////////////////////////////////////////////////////////////// Main
void init()
{
    initLED();
    initTimer();
    initPWM();
    initI2C();
    initADC();
    uart_init_1();
    uart_init_2();
    wifi_init();
    initUDP();
}
void recive_task()
{
    while (1)
    {
        uint8_t data_b_1[5];
        uint8_t data_b_2[5];
        recv_LIDAR_1(data_b_1);
        D_R = (data_b_1[1] << 8) | data_b_1[2];
        recv_LIDAR_2(data_b_2);
        D_L = (data_b_2[1] << 8) | data_b_2[2];
        // printf("recived from: %x,%x, Distance %d, %d \n",data_b_1[0],data_b_2[0],D_R,D_L);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
void sendData_task()
{
    while (1)
    {
        char sendBuff[6];
        int speedmms = (int)(speedMS * 1000);
        int Edisstance = (int)D_R - (int)D_L;
        int frontDistance = readLidar((uint8_t)FRONT_LIDAR_ADDR);
        uint8_t speedmsb = speedmms >> 8;
        uint8_t speedlsb = speedmms & 0x00FF;
        uint8_t EDmsb = Edisstance >> 8;
        uint8_t EDlsb = Edisstance & 0x00FF;
        uint8_t FDmsb = frontDistance >> 8;
        uint8_t FDlsb = frontDistance & 0x00FF;
        sendBuff[0] = speedmsb;
        sendBuff[1] = speedlsb;
        sendBuff[2] = EDmsb;
        sendBuff[3] = EDlsb;
        sendBuff[4] = FDmsb;
        sendBuff[5] = FDlsb;
        // printf("sending %x%x %x%x %x%x\n", speedmsb, speedlsb, EDmsb, EDlsb, FDmsb, FDlsb);
        sendUDP(sendBuff);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
void recvUDPData_task()
{
    char buff[9];
    while (1)
    {
        buff[0] = 0;
        receiveUDP(buff);
        // printf("got %d\n", (int)buff[0]);
        if (buff[0] - 48 == 1)
            break_flag = 1;
        else if ((buff[0] - 48 == 0))
            break_flag = 0;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // udp_recv_specfic(IPTARTGET,buff);
}
void app_main()
{
    printf("Start\n");
    init();

    xTaskCreate(loopAlarm, "alarm", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(loopPulseCounter, "pulseCounter", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(recive_task, "reciveLidar", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(sendData_task, "sendUDP", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(recvUDPData_task, "recvUDP", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(loopDrive, "Drive", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(loopSteer, "steer", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(dispTask, "disp", 4096, NULL, configMAX_PRIORITIES, NULL);
}
