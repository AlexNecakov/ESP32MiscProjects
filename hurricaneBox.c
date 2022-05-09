//c libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>

//communication protocols
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/mcpwm.h"

//free rtos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

//esp libraries
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_spi_flash.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "protocol_examples_common.h"
#include "./ADXL343.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"

// I2C
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
#define ACK_VAL I2C_MASTER_ACK              // i2c ack value
#define NACK_VAL I2C_MASTER_NACK            // i2c nack value
#define NACK_LAST_VAL I2C_MASTER_LAST_NACK  // i2c last nack value
#define SLAVE_ADDR ADXL343_ADDRESS          // 0x53
#define SLAVE_ADDR_DISPLAY 0x70             // display address
#define OSC 0x21                            // oscillator cmd
#define HT16K33_BLINK_DISPLAYON 0x01        // Display on cmd
#define HT16K33_BLINK_OFF 0                 // Blink off cmd
#define HT16K33_BLINK_CMD 0x80              // Blink cmd
#define HT16K33_CMD_BRIGHTNESS 0xE0         // Brightness cmd
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

// ADC
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 256 // Multisampling
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25 // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT 3435     // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000  // determined via multimeter using 2x5k resistors but parallel isr from caps//10000 // the value of the 'other' resistor
#define ADC_BITWIT ADC_WIDTH_BIT_10
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;
static const adc_channel_t channel2 = ADC_CHANNEL_3;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// PWM
#define SERVO_PULSE_GPIO (18) // GPIO connects to the PWM signal line

// WIFI
#define EXAMPLE_ESP_WIFI_SSID "Group_16"
#define EXAMPLE_ESP_WIFI_PASS "smartsys"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// UDP
#define HOST_IP_ADDR "192.168.0.101"
#define PORT 4444
int sock;
struct sockaddr_in dest_addr;

//////////////////////////////////////////////////////////////////////////////////// I2C
static void i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    conf.clk_flags = 0;
    i2c_param_config(i2c_master_port, &conf); // Configure
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// set oscillation
int alpha_oscillator()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set blink rate to off
int no_blink()
{
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

// Set Brightness
int set_brightness_max()
{
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | 0xF, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

int writeRegister(uint8_t reg, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL343_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint8_t readRegister(uint8_t reg)
{
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL343_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL343_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

int16_t read16(uint8_t reg)
{
    int16_t data;
    uint8_t dataMSB;
    uint8_t dataLSB;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL343_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL343_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, &dataLSB, 1, ACK_VAL);
    i2c_master_read_byte(cmd, &dataMSB, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    data = (dataMSB << 8) | dataLSB;
    return data;
}

void initI2C()
{
    printf("Initializing I2C\n");
    // Routine
    i2c_master_init();
    alpha_oscillator();
    no_blink();
    set_brightness_max();

    // Check for ADXL343
    uint8_t deviceID = readRegister(ADXL343_REG_DEVID);
    if (deviceID == 0xE5)
    {
        printf("Found ADAXL343\n");
    }
    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);

    /* Red the data format register to preserve bits */
    uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);
    /* Update the data rate */
    format &= ~0x0F;
    format |= ADXL343_RANGE_16_G;
    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;
    /* Write the register back to the IC */
    writeRegister(ADXL343_REG_DATA_FORMAT, format);

    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);
}

//write to display, accept array of bitmaps
static void write_buff(uint16_t buffer[4])
{
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR_DISPLAY << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i = 0; i < 4; i++)
    {
        i2c_master_write_byte(cmd4, buffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, buffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd4);
}

// expect count in total seconds, minutes, hours
static void update_display()
{
    uint16_t displaybuffer[4];
    displaybuffer[0] = alphafonttable[6]; //
    displaybuffer[1] = alphafonttable[9]; //
    displaybuffer[2] = alphafonttable[6]; //
    displaybuffer[3] = alphafonttable[9]; //
    write_buff(displaybuffer);
    // Send commands characters to display over I2C
}

void readAccel(float *xp, float *yp, float *zp)
{
    *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}

//////////////////////////////////////////////////////////////////////////////////// ADC
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
    printf("Initializing ADC\n");
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();
    adc1_config_width(ADC_BITWIT);
    adc1_config_channel_atten(channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_BITWIT, DEFAULT_VREF, adc_chars);
}

float readTemp(void)
{
    uint32_t adc_reading = 0;

    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;

    float resistance = SERIESRESISTOR / ((1023.0 / adc_reading) - 1);
    float steinhart;
    float t0 = 1.0 / (TEMPERATURENOMINAL + 273.15);
    float bterm = (1.0 / BCOEFFICIENT) * log(resistance / THERMISTORNOMINAL);
    steinhart = (1.0 / (t0 + bterm)) - 273.15;
    return steinhart;
}

uint32_t readVoltage(void)
{
    uint32_t adc_reading = 0;

    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel2);
    }
    adc_reading /= NO_OF_SAMPLES;

    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage;
}

/////////////////////////////////////////////////////////////////////////////////// PWM
void initPWM(void)
{
    printf("Initializing PWM\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough

    mcpwm_config_t pwm_config = {
        .frequency = 250, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,      // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void controlLED(char brightness)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (brightness - 48) * 10);
    vTaskDelay(pdMS_TO_TICKS(250)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
}

////////////////////////////////////////////////////////////////////////////////// Wifi
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            printf("retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        printf("connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        printf("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void initWifi(void)
{
    printf("Initializing Wifi\n");
    nvs_flash_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &event_handler,
                                        NULL,
                                        &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &event_handler,
                                        NULL,
                                        &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        printf("connected to ap SSID:%s password:%s\n",
               EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        printf("Failed to connect to SSID:%s, password:%s\n",
               EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        printf("UNEXPECTED EVENT\n");
    }

    /* The event will not be processed after unregister */
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    vEventGroupDelete(s_wifi_event_group);
}

////////////////////////////////////////////////////////////////////////////////// UDP
void initUDP(void)
{
    printf("Initializing UDP\n");
    // don't need these with the wifi init happening first
    // nvs_flash_init();
    // esp_netif_init();
    // esp_event_loop_create_default();

    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        printf("Unable to create socket: errno %d", errno);
    }
}

void sendUDP(char *payload)
{
    printf("Sending message\n");
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        printf("Error occurred during sending: errno %d\n", errno);
    }
    printf("Message sent\n");
}

void receiveUDP(char *rx_buffer)
{
    struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
    // Error occurred during receiving
    if (len < 0)
    {
        printf("recvfrom failed: errno %d", errno);
    }
    else
    {
        printf("R-%c\n", *rx_buffer);
    }
}

////////////////////////////////////////////////////////////////////////////////// Main
void init()
{
    initI2C();
    initADC();
    initPWM();
    initWifi();
    initUDP();
}
void senddata()
{
    while (1)
    {
        //celsius
        float temperature = readTemp();
        vTaskDelay(50 / portTICK_RATE_MS);

        //mV
        uint32_t voltage = readVoltage();
        vTaskDelay(50 / portTICK_RATE_MS);

        //m/s^2
        float xAccel, yAccel, zAccel;
        readAccel(&xAccel, &yAccel, &zAccel);
        vTaskDelay(500 / portTICK_RATE_MS);

        //send datagram
        char payload[128];
        sprintf(payload, "%.2f,%d,%.2f,%.2f,%.2f", temperature, voltage, xAccel, yAccel, zAccel);
        printf("%s\n", payload);
        sendUDP(payload);
    }
}
void receivedata()
{
    while (1)
    {

        //receive brightness level
        char receiveLoad;
        receiveUDP(&receiveLoad);
        if ((int)receiveLoad > 48 && (int)receiveLoad < 60)
            printf("recived: %c\n", receiveLoad);
        controlLED(receiveLoad);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void app_main(void)
{
    init();
    update_display();
    int threads = 2;
    xTaskCreate(senddata, "senddata", 4096 / threads, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(receivedata, "receivedata", 4096 / threads, NULL, configMAX_PRIORITIES - 1, NULL);
}
