#include "UARTesp.h"
int len_out = LEN_OUT;
char start = START;
SemaphoreHandle_t mux = NULL;
SemaphoreHandle_t mux0 = NULL;
SemaphoreHandle_t mux1 = NULL;

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

void uart_init_1()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 9600, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);

    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM_1, UART_RX_GPIO_NUM_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    mux1 = xSemaphoreCreateMutex();
}
void uart_init_2()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 56000, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_2, &uart_config);

    uart_set_pin(UART_NUM_2, UART_TX_GPIO_NUM_2, UART_RX_GPIO_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    mux0 = xSemaphoreCreateMutex();
}

void send_task_1()
{
    while (1)
    {

        char *data_out = (char *)malloc(len_out);
        xSemaphoreTake(mux, portMAX_DELAY);
        data_out[0] = start;
        data_out[2] = (char)ID;
        data_out[1] = (char)ID;
        data_out[2] = (char)'F';
        data_out[3] = genCheckSum(data_out, len_out - 1);

        uart_write_bytes(UART_NUM_1, data_out, len_out);
        xSemaphoreGive(mux);
        free(data_out);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
void send_LIDAR_1(int id, uint8_t msb, uint8_t lsb)
{
    char *data_out = (char *)malloc(len_out);
    xSemaphoreTake(mux1, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char)id;
    data_out[2] = (char)msb;
    data_out[3] = (char)lsb;
    data_out[4] = genCheckSum(data_out, len_out - 1);

    uart_write_bytes(UART_NUM_1, data_out, len_out);
    xSemaphoreGive(mux1);
    free(data_out);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}
void send_LIDAR_2(int id, uint8_t msb, uint8_t lsb)
{
    char *data_out = (char *)malloc(len_out);
    xSemaphoreTake(mux0, portMAX_DELAY);
    data_out[0] = start;
    data_out[1] = (char)id;
    data_out[2] = (char)msb;
    data_out[3] = (char)lsb;
    data_out[4] = genCheckSum(data_out, len_out - 1);

    uart_write_bytes(UART_NUM_2, data_out, len_out);
    xSemaphoreGive(mux0);
    free(data_out);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}
void recv_LIDAR_1(uint8_t *data_out)
{
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    // uint8_t data_out[3];
    if (len_in > 0)
    {
        if (data_in[0] == start && checkCheckSum(data_in, len_out))
        {
            data_out[0] = *(data_in + 1);
            data_out[1] = *(data_in + 2);
            data_out[2] = *(data_in + 3);
            // printf("\nFrom %X::msb:%x LSB:%X\n",data_out[0],data_out[1],data_out[2]);
        }
    }
    free(data_in);
}
void recv_LIDAR_2(uint8_t *data_out)
{
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    int len_in = uart_read_bytes(UART_NUM_2, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    // uint8_t data_out[3];
    if (len_in > 0)
    {
        if (data_in[0] == start && checkCheckSum(data_in, len_out))
        {
            data_out[0] = *(data_in + 1);
            data_out[1] = *(data_in + 2);
            data_out[2] = *(data_in + 3);
            // printf("\nFrom %X::msb:%x LSB:%X\n",data_out[0],data_out[1],data_out[2]);
        }
    }
    free(data_in);
}
// Receives task -- looks for Start byte then stores received values
void recv_task_1()
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0)
        {
            // printf("Got Data!\n");
            // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
            if (data_in[0] == start)
            {
                // printf("Got w!\n");
                if (checkCheckSum(data_in, len_out))
                {
                    // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                    char *data_out = (char *)malloc(len_out);
                    data_out[0] = *data_in;
                    data_out[1] = *(data_in + 1);
                    data_out[2] = *(data_in + 2);
                    data_out[3] = *(data_in + 3);
                    data_out[4] = *(data_in + 3);
                    printf("\nFrom %X::msb:%x LSB:%X\n", data_out[1], data_out[2], data_out[2]);
                }
            }
        }
        else
        {
            // printf("Nothing received.\n");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

void send_task_0()
{
    while (1)
    {

        char *data_out = (char *)malloc(len_out);
        xSemaphoreTake(mux, portMAX_DELAY);
        data_out[0] = start;
        data_out[1] = (char)ID;
        data_out[2] = (char)'F';
        data_out[3] = genCheckSum(data_out, len_out - 1);

        uart_write_bytes(UART_NUM_2, data_out, len_out);
        xSemaphoreGive(mux);
        free(data_out);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
// // Receives task -- looks for Start byte then stores received values
void recv_task_2()
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len_in = uart_read_bytes(UART_NUM_2, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0)
        {
            // printf("Got Data!\n");
            // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
            if (data_in[0] == start)
            {
                // printf("Got w!\n");
                if (checkCheckSum(data_in, len_out))
                {
                    // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
                    char *data_out = (char *)malloc(len_out);
                    data_out[0] = *data_in;
                    data_out[1] = *(data_in + 1);
                    data_out[2] = *(data_in + 2);
                    data_out[3] = *(data_in + 3);
                    data_out[4] = *(data_in + 3);
                    printf("\nFrom %X::msb:%x LSB:%X\n", data_out[1], data_out[2], data_out[2]);
                }
            }
        }
        else
        {
            // printf("Nothing received.\n");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data_in);
}
