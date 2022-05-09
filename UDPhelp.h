#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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
#include <string.h>

// WIFI definitions
#define EXAMPLE_ESP_WIFI_SSID "Group_16"
#define EXAMPLE_ESP_WIFI_PASS "smartsys"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define PORT 4444


void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init();
void receiveUDP(char *rx_buffer);
void sendUDP(char *payload);
void initUDP();
// void udp_recv_specfic(char * ip, char * recvbuff);
// void UDP_send_broad(char * sendbuff,int len);
// void udp_recv_broad(char * recvbuff,int len);
// void send_task(char * buffer);
// void udp_recv_task(char * buffer);
