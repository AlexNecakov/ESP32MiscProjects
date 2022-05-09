#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <sys/random.h>

// WIFI definitions
#define EXAMPLE_ESP_WIFI_SSID "Group_16"
#define EXAMPLE_ESP_WIFI_PASS "smartsys"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define PORT 4444

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_1_PIN_SEL 1ULL << GPIO_INPUT_IO_1
#define GPIO_INPUT_IO_2 36
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_2_PIN_SEL 1ULL << GPIO_INPUT_IO_2

// LED Output pins definitions
#define BLUEPIN 14
#define GREENPIN 32
#define REDPIN 15
#define ONBOARD 13

// Hardware timer definitions
#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_2_SEC (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

// Election values
#define HEART 10
#define NODES 6
#define ELECT 10

#define REDVOTE 1
#define GREENVOTE 2
#define BLUEVOTE 3

// Variables for my ID, minVal and status plus string fragments
char start = 0x1C;
char PID = 0;
int count = 0;
int leader = 0;

int color_vote = REDVOTE;
int color_vote_save = REDVOTE;
int len_out = 6;
int heartbeat_checkout = 0;
char tx_buffer[128];
char rx_buffer[128];

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
SemaphoreHandle_t elections = NULL;
static xQueueHandle gpio_evt_1_queue = NULL;
static xQueueHandle gpio_evt_2_queue = NULL;
static xQueueHandle timer_queue;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// A simple structure to pass "events" to main task
typedef struct
{
    int flag; // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system"; // For debug logs

// Checksum  ///////////////////////////////////////////////////////////////////
char genCheckSum(char *p, int len)
{
    char temp = 0;
    for (int i = 0; i < len; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("%X\n",temp);

    return temp;
}
bool checkCheckSum(uint8_t *p, int len)
{
    char temp = (char)0;
    bool isValid;
    for (int i = 0; i < len - 1; i++)
    {
        temp = temp ^ p[i];
    }
    // printf("Check: %02X ", temp);
    if (temp == p[len - 1])
    {
        isValid = true;
    }
    else
    {
        isValid = false;
    }
    return isValid;
}

// Handlers ////////////////////////////////////////////////////////////////////
static void IRAM_ATTR gpio_1_isr_handler(void *arg)
{
    uint32_t gpio_1_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_1_queue, &gpio_1_num, NULL);
}

static void IRAM_ATTR gpio_2_isr_handler(void *arg)
{
    uint32_t gpio_2_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_2_queue, &gpio_2_num, NULL);
}

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

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
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

// Init Functions //////////////////////////////////////////////////////////////
static void rmt_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

static void uart_init()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void wifi_init()
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

static void led_init()
{
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

static void alarm_init()
{
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
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

static void button_init()
{
    gpio_config_t io_conf1;
    // interrupt of rising edge
    io_conf1.intr_type = GPIO_PIN_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4 here
    io_conf1.pin_bit_mask = GPIO_INPUT_1_PIN_SEL;
    // set as input mode
    io_conf1.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf1.pull_up_en = 1;
    gpio_config(&io_conf1);
    gpio_intr_enable(GPIO_INPUT_IO_1);

    gpio_config_t io_conf2;
    // interrupt of rising edge
    io_conf2.intr_type = GPIO_PIN_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4 here
    io_conf2.pin_bit_mask = GPIO_INPUT_2_PIN_SEL;
    // set as input mode
    io_conf2.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf2.pull_up_en = 1;
    gpio_config(&io_conf2);
    gpio_intr_enable(GPIO_INPUT_IO_2);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_1_isr_handler, (void *)GPIO_INPUT_IO_1);
    // create a queue to handle gpio event from isr
    gpio_evt_1_queue = xQueueCreate(10, sizeof(uint32_t));

    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_2_isr_handler, (void *)GPIO_INPUT_IO_2);
    // create a queue to handle gpio event from isr
    gpio_evt_2_queue = xQueueCreate(10, sizeof(uint32_t));
}

// Tasks ///////////////////////////////////////////////////////////////////////
void UDP_send()
{
    struct sockaddr_in dest_addr1;
    int sock1 = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int bc = 1;
    if (setsockopt(sock1, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc)) < 0)
    {
        printf("Failed to set sock options: errno %d", errno);
        closesocket(sock1);
    }
    struct sockaddr_in *dest_addr_ip41 = (struct sockaddr_in *)&dest_addr1;
    dest_addr_ip41->sin_addr.s_addr = inet_addr("255.255.255.255");
    dest_addr_ip41->sin_family = AF_INET;
    dest_addr_ip41->sin_port = htons(PORT);

    int err = sendto(sock1, tx_buffer, 128 - 1, 0, (struct sockaddr *)&dest_addr1, sizeof(dest_addr1));
    if (err < 0)
    {
        ESP_LOGE(TAG_SYSTEM, "Error occurred during sending: errno %d", errno);
    }
    closesocket(sock1);
    tx_buffer[0] = 0;
    tx_buffer[1] = 0;
    tx_buffer[2] = 0;
    tx_buffer[3] = 0;
    tx_buffer[4] = 0;

    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void button_1_task() // change color of LED
{
    uint32_t io_num1;
    while (1)
    {
        if (xQueueReceive(gpio_evt_1_queue, &io_num1, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);

            if (color_vote == REDVOTE)
                color_vote = GREENVOTE;
            else if (color_vote == GREENVOTE)
                color_vote = BLUEVOTE;
            else if (color_vote == BLUEVOTE)
                color_vote = REDVOTE;

            xSemaphoreGive(mux);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void button_2_task() // send vote to neighbor over uart
{
    uint32_t io_num2;
    while (1)
    {
        if (xQueueReceive(gpio_evt_2_queue, &io_num2, portMAX_DELAY))
        {
            char *data_out = (char *)malloc(len_out);
            xSemaphoreTake(mux, portMAX_DELAY);
            data_out[0] = start;
            data_out[1] = PID;
            data_out[2] = PID;
            data_out[3] = leader;
            data_out[4] = color_vote;

            uart_write_bytes(UART_NUM_1, data_out, len_out);

            xSemaphoreGive(mux);
            free(data_out);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void ir_recv_task() // listen to neighbor votes over uart
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0)
        {
            if (data_in[0] == start)
            {
                xSemaphoreTake(mux, portMAX_DELAY);

                color_vote_save = color_vote;
                color_vote = data_in[4];

                tx_buffer[0] = start;
                tx_buffer[1] = data_in[1];
                tx_buffer[2] = PID;
                tx_buffer[3] = leader;
                tx_buffer[4] = data_in[4];

                //////////////////////////////////////////////////////////////// needs to send the color vote to the leader
                UDP_send();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                color_vote = color_vote_save;
                xSemaphoreGive(mux);
                ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
            }
        }
        else
        {
            //  printf("Nothing received.\n");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

void led_task() // update led to reflect current vote status
{
    while (1)
    {
        switch (color_vote)
        {
        case REDVOTE: // Red
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        case BLUEVOTE: // blue
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 1);
            // printf("Current state: %c\n",status);
            break;
        case GREENVOTE: // Green
            gpio_set_level(GREENPIN, 1);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
          }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void id_task() // LED task to blink onboard LED based on ID
{
    while (1)
    {
        for (int i = 0; i < (int)PID; i++)
        {
            gpio_set_level(ONBOARD, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(ONBOARD, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void election()
{
    // send out election header for some amount of time
    // if no anwer higher self is leader
    // if answer higher drop out
    printf("ELECTION\n");
    //election
    // tx_buffer[0] = start;
    // tx_buffer[1] = PID;
    // tx_buffer[2] = PID;
    // tx_buffer[3] = 0;
    // tx_buffer[4] = 0;
    // UDP_send();
    xSemaphoreTake(mux, portMAX_DELAY);

    for (int i = 0; i < ELECT; i++)
    {
        if (rx_buffer[3] > PID)
        {
            leader = rx_buffer[3];
            printf("Leader is : %d\n", leader);
            xSemaphoreGive(mux);
            return;
        }else if (rx_buffer[3] == PID){
          PID = -1;
          xSemaphoreGive(mux);
          return;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    leader = PID;
    printf("I AM LEADER\n");
    heartbeat_checkout = 0;
    xSemaphoreGive(mux);
}

static void timer_evt_task(void *arg) // Timer task -- R (10 seconds), G (10 seconds), Y (2 seconds)
{
    while (1)
    {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1)
        {
            count = (count + 1) % 70;
            if (leader != PID)
                heartbeat_checkout++;
            if (heartbeat_checkout > HEART)
            {
                heartbeat_checkout = 0;
                leader = 0;
            }
        }
    }
}

void udp_recv_task()
{
    struct sockaddr_in dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG_SYSTEM, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG_SYSTEM, "Socket created");
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG_SYSTEM, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG_SYSTEM, "Socket bound, port %d", PORT);
    ESP_LOGI(TAG_SYSTEM, "Waiting for data");
    while (1)
    {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0)
        {
            ESP_LOGE(TAG_SYSTEM, "recvfrom failed: errno %d", errno);
        }
        if (len > 0)
        {
            printf("Start:%x,Origin:%d,Sender:%d,Leader:%d,vote:%d\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4]);
            if (rx_buffer[0] == start)
            {
                if(rx_buffer[3]==leader)
                    heartbeat_checkout = 0;
                // if (senderID > PID && senderID > leader)
                //     leader = senderID;
                // else if (leaderID == 0)
                //     leader = -2;
                // else if (PID == senderID)
                //     PID = -1;
                // else if (leaderID != 0)
                // {
                //     printf("bum bum\n");
                //     leader = leaderID;
                //     heartbeat_checkout = 0;
                // }
                // else
                // {
                //     printf("Uncoveredstate:(\n");
                // }
                // if (vote != 0)
                //     printf("Vote"); //votes cast
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void start_state() // randomize PID on start to pick random leader
{
    PID = (abs((int)esp_random()) % (254)) + 1;
    printf("NEW PID:%d \n", PID);
}

void state_machine_task()
{
    while (1)
    {
        if (PID == -1)
        {
            start_state();
            continue;
        }
        if (leader < PID || (PID == leader && PID < rx_buffer[3])) // ELECTION
            election();
        else if (leader == PID)
        { // leader heart beat
            tx_buffer[0] = start;
            tx_buffer[1] = PID;
            tx_buffer[2] = PID;
            tx_buffer[3] = PID;
            tx_buffer[4] = 0;
            UDP_send();
            heartbeat_checkout = 0;
        }
        else if (leader > PID)
        { // folower
            leader = rx_buffer[3];
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Initialize all the things
    wifi_init();
    start_state();
    rmt_tx_init();
    uart_init();
    led_init();
    alarm_init();
    button_init();
    // Create tasks for receive, send, set gpio, and button
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(ir_recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "vote_led_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(id_task, "pid_led_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_1_task, "button_1_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_2_task, "button_2_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(state_machine_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(udp_recv_task, "udp_recv", 4096 / 2, NULL, 5, NULL);
}
