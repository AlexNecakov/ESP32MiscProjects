// note caps + high multisampling leads to a very stable signal

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"

// Determine the pins that can be used for the sensors
//  TEMP = GPIO 32  //works fine
// issues stemming form the way of sampling
//  SONIC = GPIO 33  //works (maybe make it more exact)
//  IR = GPIO 34  //needs wiring

#define NO_OF_SAMPLES 256 // Multisampling
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 256 // Multisampling
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always 25 C)

#define BCOEFFICIENT 3435   // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000 // determined via multimeter using 2x5k resistors but parallel isr from caps//10000 // the value of the 'other' resistor

#define UART_PORT UART_NUM_1

#define ADC_BITWIT ADC_WIDTH_BIT_10

#define SENSOR_TEMPERATURE 0
#define SENSOR_ULTRASONIC 1
#define SENSOR_INFRARED 2

// using single adc unit, switch channel/attenuation per sensor
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channelTemp = ADC_CHANNEL_4;
static const adc_channel_t channelSonic = ADC_CHANNEL_5;
static const adc_channel_t channelIR = ADC_CHANNEL_6;
static const adc_atten_t attenTemp = ADC_ATTEN_DB_11;
static const adc_atten_t attenSonic = ADC_ATTEN_DB_0;
static const adc_atten_t attenIR = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

void initADC()
{
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();
    adc1_config_width(ADC_BITWIT);
    printf("Inited ADC\n");
}

void initUART()
{
    const uart_port_t uart_num = UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 17, 16, 18, 19));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT,
                                        256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_UART));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_PORT);
    printf("Inited UART\n");
}

void init()
{
    initADC();
    initUART();
}

uint32_t ADC_Sample(uint32_t sensor)
{
    uint32_t adc_reading = 0;
    adc_channel_t channel;
    adc_atten_t atten;

    // using single adc (unit 1), must configure based on input param
    switch (sensor)
    {
    case SENSOR_TEMPERATURE:
        channel = channelTemp;
        atten = attenTemp;
        break;
    case SENSOR_ULTRASONIC:
        channel = channelSonic;
        atten = attenSonic;
        break;
    case SENSOR_INFRARED:
        channel = channelIR;
        atten = attenIR;
        break;
    default:
        channel = channelTemp;
        atten = attenTemp;
        break;
    }

    adc1_config_channel_atten(channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_BITWIT, DEFAULT_VREF, adc_chars);
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
    return (adc_reading == 0) ? 1 : adc_reading;
}

float calcTemp(uint32_t adc_reading)
{
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float resistance = SERIESRESISTOR / ((1023.0 / adc_reading) - 1);
    float steinhart;
    float t0 = 1.0 / (TEMPERATURENOMINAL + 273.15);
    float bterm = (1.0 / BCOEFFICIENT) * log(resistance / THERMISTORNOMINAL);
    steinhart = (1.0 / (t0 + bterm)) - 273.15;
    return steinhart;
}

float calcSonicDist(uint32_t adc_reading)
{
    // //with 10-bit width, multiply adc reading by 5. Scale due to lower VCC (/3) (verify this claim)
    float sonicRange = adc_reading * 5.0 / 3.0;
    //convert from mm to m
    sonicRange /= 100;

    // distance in mm is the adc reading in ten bit * 5
    // float sonicRange = adc_reading * 5.0;
    // sonicRange = (sonicRange / 1000.0) * 2.246; // 2.246 scaling factor for the distance , based on measurement
    // seems to be to closest meter
    return sonicRange;
}

// using lookup table right now... is there a better way to characterize? maybe linear?
// we can use a function made in excel, but needs to be done via testing.
float calcIRDist(uint32_t adc_reading)
{
    int32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    // printf("Raw: %d voltage %d\n",adc_reading,voltage);
    int lookup_table_length = 26;
    uint32_t lookuptable[] = {
      2500, // max voltage 20 cm
      2000, // 30 cm
      1500, // 40 cm
      1250, // 50 cm
      1000, // 60 cm
      850, // 70 cm
      770, // 80 cm
      700, // 90 cm
      650, // 100 cm
      600, // 110 cm
      550, // 120 cm
      500, // 130 cm
      450, // 140 cm
      430, // 150 cm
      420, // 160 cm
      410, // 170 cm
      401, // 180 cm
      392, // 190 cm
      383, // 200 cm
      374, // 210 cm
      365, // 220 cm
      356, // 230 cm
      346, // 240 cm
      376, // 250 cm
      330, // 260 cm
      0, // 260 cm
    };
    int lastVal = 700;
    int CurrVal = 0;
    int i;
    for (i = 0; i < lookup_table_length; i++)
    {
        CurrVal = lookuptable[i];
        if (CurrVal < voltage)
        {
            break;
        }
        lastVal = CurrVal;
    }
    if (CurrVal == 0)
    {
        return 0;
    }
    float relDistance = ((float)(voltage - CurrVal) / (float)(lastVal - CurrVal)); // current voltage distance from bottom of range
    // if voltage = currval + 0 if voltage = Last val  = 1
    float val = (((float)(i - 1) + relDistance) * 10.0) + 20;
    // conv to m from cm
    val /= 100;
    return val;
}

void app_main(void)
{
    init();
    while (1)
    {
        uint32_t adc_reading = ADC_Sample(SENSOR_TEMPERATURE);
        float temperature = calcTemp(adc_reading);
        vTaskDelay(50 / portTICK_RATE_MS);

        adc_reading = ADC_Sample(SENSOR_ULTRASONIC);
        float distSonic = calcSonicDist(adc_reading);
        vTaskDelay(50 / portTICK_RATE_MS);

        adc_reading = ADC_Sample(SENSOR_INFRARED);
        float distIR = calcIRDist(adc_reading);
        vTaskDelay(50 / portTICK_RATE_MS);

        printf("[%f,%f,%f]\n", temperature, distSonic, distIR);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
