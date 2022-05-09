#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/semphr.h"
#include "driver/mcpwm.h"
// RTC Timer
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_SEC (1)                       // Sample test interval for the first timer
#define TEST_WITH_RELOAD 1                           // Testing will be done with auto reload
// #define TIMER_DEFAULT_VAL 3655                       // Test value uses all digits in display
#define TIMER_DEFAULT_VAL 20                     // Test value uses all digits in display

// Timer Modes
#define TIMER_MODE_MIN_SEC 0
#define TIMER_MODE_HR_MIN 1


// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF                       // i2c nack value

//SERVO
#define SERVO_MIN_PULSEWIDTH_US (400) // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US (2200) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        (90)   // Maximum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO        (18)   // GPIO connects to the PWM signal line

// Interrupt button(s)
#define BUTTON_MODE_GPIO 14
#define GPIO_INPUT_PIN_SEL_MODE 1ULL << BUTTON_MODE_GPIO
#define NUMBER_FOR_MENU 1

#define BUTTON_UP_GPIO 4
#define GPIO_INPUT_PIN_SEL_UP 1ULL << BUTTON_UP_GPIO

#define BUTTON_DOWN_GPIO 15
#define GPIO_INPUT_PIN_SEL_DOWN 1ULL << BUTTON_DOWN_GPIO

#define ESP_INTR_FLAG_DEFAULT 0
bool buttonFlag = false;
bool b1 = 0;
bool b2 = 0;
bool b3 = 0;
uint8_t feeding = 0;
SemaphoreHandle_t feeding_sem = NULL;
static const uint16_t alphafonttable[] = {

    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
};


// A simple structure to pass "events" to main task
typedef struct
{
    uint8_t flag; // flag for enabling stuff in main code
} timer_event_t;

typedef struct
{
    uint8_t sec; // time object for keeping track of time;
    uint8_t min;
    uint8_t hour;
} times;
times current;
times set;
// Initialize queue handler for timer-based events
xQueueHandle timer_queue;
