#ifndef UARTESP_H
#define UARTESP_H

#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "driver/uart.h"

#include "esp_log.h"


// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

#define UART_TX_GPIO_NUM_1 26 // A0
#define UART_RX_GPIO_NUM_1 34 // A2
#define UART_TX_GPIO_NUM_2 25 // A1
#define UART_RX_GPIO_NUM_2 39 // A3
#define BUF_SIZE (1024)
#define ID 0x01;

#define START 0x1F
#define LEN_OUT 5


char genCheckSum(char *p, int len);
bool checkCheckSum(uint8_t *p, int len);
void uart_init_1();
void uart_init_2();
void send_LIDAR_1(int id,uint8_t msb,uint8_t lsb);
void send_LIDAR_2(int id,uint8_t msb,uint8_t lsb);
void recv_LIDAR_1(uint8_t *data_out);
void recv_LIDAR_2(uint8_t *data_out);
void send_task_1();
void recv_task_1();
void send_task_2();
void recv_task_2();
#endif
