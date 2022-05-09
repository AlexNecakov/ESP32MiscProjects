#include "UDPhelp.h"
int s_retry_num =  0;

static EventGroupHandle_t s_wifi_event_group;

static const char *TAG_SYSTEM = "system";
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
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
void wifi_init(){
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


#define HOST_IP_ADDR "192.168.1.130"
#define PORT 4444

int sock;
struct sockaddr_in dest_addr;
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
    // printf("Sending message\n");
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        printf("Error occurred during sending: errno %d\n", errno);
    }
    // printf("Message sent\n");
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
    // else
    // {
    //     // printf("R-%x\n", rx_buffer[0]);
    // }
}
// dirty dirty
//
// void UDP_send_broad(char * sendbuff,int len){
//     struct sockaddr_in dest_addr1;
//     int sock1 = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
//     int bc = 1;
//     if (setsockopt(sock1, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc)) < 0)
//     {
//         printf("Failed to set sock options: errno %d", errno);
//         closesocket(sock1);
//     }
//     struct sockaddr_in *dest_addr_ip41 = (struct sockaddr_in *)&dest_addr1;
//     dest_addr_ip41->sin_addr.s_addr = inet_addr("255.255.255.255");
//     dest_addr_ip41->sin_family = AF_INET;
//     dest_addr_ip41->sin_port = htons(PORT);
//
//     int err = sendto(sock1, sendbuff, len - 1, 0, (struct sockaddr *)&dest_addr1, sizeof(dest_addr1));
//     if (err < 0)
//     {
//         ESP_LOGE(TAG_SYSTEM, "Error occurred during sending: errno %d", errno);
//     }
//     closesocket(sock1);
//     vTaskDelay(500 / portTICK_PERIOD_MS);
// }
// void udp_recv_specfic(char * ip, char * recvbuff){
//   printf("Initializing UDP\n");
//   // don't need these with the wifi init happening first
//   // nvs_flash_init();
//   // esp_netif_init();
//   // esp_event_loop_create_default();
//   int sock;
//   struct sockaddr_in dest_addr;
//   dest_addr.sin_addr.s_addr = inet_addr(ip);
//   dest_addr.sin_family = AF_INET;
//   dest_addr.sin_port = htons(PORT);
//   int addr_family = AF_INET;
//   int ip_protocol = IPPROTO_IP;
//   sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//   if (sock < 0)
//   {
//       printf("Unable to create socket: errno %d", errno);
//   }
//   while(1){
//       printf("reciving\n");
//       struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
//       socklen_t socklen = sizeof(source_addr);
//       int len = recvfrom(sock, recvbuff, sizeof(recvbuff) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
//       // Error occurred during receiving
//       if (len < 0)
//       {
//           printf("recvfrom failed: errno %d", errno);
//       }
//       else
//       {
//           printf("R-%c\n", recvbuff[0]);
//       }
//       vTaskDelay(500 / portTICK_PERIOD_MS);
//   }
// }
// char rx_buffer[128];
// void udp_recv_broad(char * recvbuff,int len)
// {
//     struct sockaddr_in dest_addr;
//     struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
//     dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
//     dest_addr_ip4->sin_family = AF_INET;
//     dest_addr_ip4->sin_port = htons(PORT);
//
//     int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
//     if (sock < 0)
//     {
//         ESP_LOGE(TAG_SYSTEM, "Unable to create socket: errno %d", errno);
//     }
//     ESP_LOGI(TAG_SYSTEM, "Socket created");
//     int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//     if (err < 0)
//     {
//         ESP_LOGE(TAG_SYSTEM, "Socket unable to bind: errno %d", errno);
//     }
//     ESP_LOGI(TAG_SYSTEM, "Socket bound, port %d", PORT);
//     ESP_LOGI(TAG_SYSTEM, "Waiting for data");
//     while (1)
//     {
//         struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
//         socklen_t socklen = sizeof(source_addr);
//         int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
//
//         // Error occurred during receiving
//         if (len < 0)
//         {
//             ESP_LOGE(TAG_SYSTEM, "recvfrom failed: errno %d", errno);
//         }
//         if (len > 0)
//         {
//             printf("Start:%x,Origin:%d,Sender:%d,Leader:%d,vote:%d\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4]);
//             // if (rx_buffer[0] == start)
//             // {
//             //     if(rx_buffer[3]==leader)
//             //         heartbeat_checkout = 0;
//                 // if (senderID > PID && senderID > leader)
//                 //     leader = senderID;
//                 // else if (leaderID == 0)
//                 //     leader = -2;
//                 // else if (PID == senderID)
//                 //     PID = -1;
//                 // else if (leaderID != 0)
//                 // {
//                 //     printf("bum bum\n");
//                 //     leader = leaderID;
//                 //     heartbeat_checkout = 0;
//                 // }
//                 // else
//                 // {
//                 //     printf("Uncoveredstate:(\n");
//                 // }
//                 // if (vote != 0)
//                 //     printf("Vote"); //votes cast
//             // }
//         }
//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }
// void send_task(char * buffer){
//   while(1){
//     buffer[0] = 'f';
//     buffer[1] = 'g';
//     buffer[2] = 'o';
//     buffer[3] = 'd';
//     buffer[4] = '\n';
//     UDP_send_broad(buffer,128);
//     vTaskDelay(500 / portTICK_PERIOD_MS);
//   }
// }
// void udp_recv_task(char * buffer){
//   udp_recv_broad(buffer,128);
// }
