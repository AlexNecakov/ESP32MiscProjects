#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"

#define RED_GPIO 12
#define BLUE_GPIO 27
#define GREEN_GPIO 33
#define YELLOW_GPIO 15

#define TOGGLE_MODE 0
#define ECHO_MODE 1
#define CONV_MODE 2

int count = 0;
bool countUp = true;

static uint8_t s_red_state = 0;
static uint8_t s_blue_state = 0;
static uint8_t s_green_state = 0;
static uint8_t s_yellow_state = 0;

static void blink_led(int pin, int s_led_state)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(pin, s_led_state);
}

void app_main(void)
{
    gpio_reset_pin(RED_GPIO);
    gpio_reset_pin(BLUE_GPIO);
    gpio_reset_pin(GREEN_GPIO);
    gpio_reset_pin(YELLOW_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(YELLOW_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        blink_led(RED_GPIO, s_red_state);
        blink_led(BLUE_GPIO, s_blue_state);
        blink_led(GREEN_GPIO, s_green_state);
        blink_led(YELLOW_GPIO, s_yellow_state);
        printf("%d\n", count);

        if (countUp)
        {
            count++;
        }
        else
        {
            count--;
        }

        if (count > 14)
        {
            countUp = false;
        }
        else if (count < 1)
        {
            countUp = true;
        }

        if (count % 2 != 0)
            s_red_state = 1;
        else
            s_red_state = 0;

        if (count / 2 % 2 != 0)
            s_blue_state = 1;
        else
            s_blue_state = 0;

        if (count / 4 % 2 != 0)
            s_green_state = 1;
        else
            s_green_state = 0;

        if (count / 8 > 0)
            s_yellow_state = 1;
        else
            s_yellow_state = 0;

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
