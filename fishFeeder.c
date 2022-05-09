/*
  Quest 1 Main
  Alex Necakov, George Kent-Scheller, Spencer Piligian
*/
#include "config.h"

// ISR handler
TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void decrement_time(times *t)
{
    if (t->sec > 0)
    {
        t->sec--;
        return;
    }
    if (t->min > 0)
    {
        t->min--;
        t->sec = 59;
        return;
    }
    if (t->hour > 0)
    {
        t->hour--;
        t->min = 59;
        t->sec = 59;
        return;
    }
    t->hour = 0;
    t->min = 0;
    t->sec = 0;
    return;
}

static inline uint32_t example_convert_servo_angle_to_duty_us(int angle)
{
    return (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
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
    decrement_time(&current);
}
static void alarm_init()
{
    /* Select and initialize basic parameters of the timer */
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
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}
static void i2c_example_master_init()
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
int set_brightness_max(uint8_t val)
{
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_RATE_MS);
    return ret;
}

//three button gpio handlers
static void IRAM_ATTR gpio_isr_handler_mode(void *arg) // Interrupt handler for your GPIO
{
    b1 = 1;
}
static void IRAM_ATTR gpio_isr_handler_up(void *arg) // Interrupt handler for your GPIO
{
    b2 = 1;
}
static void IRAM_ATTR gpio_isr_handler_down(void *arg) // Interrupt handler for your GPIO
{
    b3 = 1;
}

static void buttonsInit()
{
    // mode button
    gpio_config_t io_conf_mode;
    io_conf_mode.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf_mode.pin_bit_mask = GPIO_INPUT_PIN_SEL_MODE;
    io_conf_mode.mode = GPIO_MODE_INPUT;
    io_conf_mode.pull_up_en = 1;
    io_conf_mode.pull_down_en = 0;
    gpio_config(&io_conf_mode);
    gpio_intr_enable(BUTTON_MODE_GPIO);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(BUTTON_MODE_GPIO, gpio_isr_handler_mode, (void *)BUTTON_MODE_GPIO);

    //up button
    gpio_config_t io_conf_up;
    io_conf_up.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf_up.pin_bit_mask = GPIO_INPUT_PIN_SEL_UP;
    io_conf_up.mode = GPIO_MODE_INPUT;
    io_conf_up.pull_up_en = 1;
    io_conf_up.pull_down_en = 0;
    gpio_config(&io_conf_up);
    gpio_intr_enable(BUTTON_UP_GPIO);
    // gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(BUTTON_UP_GPIO, gpio_isr_handler_up, (void *)BUTTON_UP_GPIO);

    //down button
    gpio_config_t io_conf_down;
    io_conf_down.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf_down.pin_bit_mask = GPIO_INPUT_PIN_SEL_DOWN;
    io_conf_down.mode = GPIO_MODE_INPUT;
    io_conf_down.pull_up_en = 1;
    io_conf_down.pull_down_en = 0;
    gpio_config(&io_conf_down);
    gpio_intr_enable(BUTTON_DOWN_GPIO);
    // gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(BUTTON_DOWN_GPIO, gpio_isr_handler_down, (void *)BUTTON_DOWN_GPIO);
}

//need these to have display on consistently
static void initDisp()
{
    int ret;
    ret = alpha_oscillator();
    if (ret == ESP_OK)
    {
        printf("- oscillator: ok \n");
    }
    ret = no_blink();
    if (ret == ESP_OK)
    {
        printf("- blink: off \n");
    }
    ret = set_brightness_max(0xF);
    if (ret == ESP_OK)
    {
        printf("- brightness: max \n");
    }
}

static void init()
{
    printf("Initialize button:\n");
    buttonsInit();
    printf("Initialize i2c:\n");
    i2c_example_master_init();
    printf("Initialize alarm:\n");
    alarm_init();
    printf("Initialize display:\n");
    initDisp();
    printf("start task:\n");
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

//write to display, accept array of bitmaps
static void write_buff(uint16_t buffer[4])
{
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
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
    uint8_t timerMode = (current.hour > 0) ? TIMER_MODE_HR_MIN : TIMER_MODE_MIN_SEC;
    switch (timerMode)
    {
    case TIMER_MODE_HR_MIN:
        displaybuffer[0] = alphafonttable[current.hour / 10]; // hr 0
        displaybuffer[1] = alphafonttable[current.hour % 10]; // hr 1
        displaybuffer[2] = alphafonttable[current.min / 10];  // min 0
        displaybuffer[3] = alphafonttable[current.min % 10];  // min 1
        break;
    case TIMER_MODE_MIN_SEC:
        displaybuffer[0] = alphafonttable[current.min / 10]; // min 0
        displaybuffer[1] = alphafonttable[current.min % 10]; // min 1
        displaybuffer[2] = alphafonttable[current.sec / 10]; // sec 0
        displaybuffer[3] = alphafonttable[current.sec % 10]; // sec 1
        break;
    default:
        displaybuffer[0] = alphafonttable[0]; //
        displaybuffer[1] = alphafonttable[0]; //
        displaybuffer[2] = alphafonttable[0]; //
        displaybuffer[3] = alphafonttable[0]; //
        break;
    }
    write_buff(displaybuffer);
    // Send commands characters to display over I2C
}

//separate thread to not lose time during servo operation
static void feed()
{
    vTaskDelay(100);

    int dire = -180; //direction pulls angle +- 180deg
    int angle = 90;  //Start position of servo

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(-90));
    vTaskSuspend(TaskHandle_2); // starts off
    while (1)
    {
        //for three cycles, need to loop through this six times
        for (int i = 0; i < 6; i++)
        {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, example_convert_servo_angle_to_duty_us(angle));
            vTaskDelay(pdMS_TO_TICKS(1500)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply
            angle = angle + dire;
            dire = -dire;
        }
        vTaskSuspend(TaskHandle_2); // end of feeding
    }
}

//needs a thread to not pause counter
static void menu()
{
    static uint16_t buff[4] = {
        0b0000010100110110, // M
        0b0000000011111001, // E
        0b0010000100110110, // N
        0b0000000000111110, // U
    };                      // char set for menu
    static const uint16_t charset[6] = {
        0b0000000011110110,     // H
        0b0010000011110011,     // R
        0b0000010100110110,     // M
        0b0010000100110110,     // N
        0b0000000011101101,     // S
        0b0000000011111001,     // E
    };                          // char set for hrs min sec
    vTaskSuspend(TaskHandle_3); // starts off
    while (1)
    { // main task loop
        write_buff(buff);
        printf("Menu Mode!\n");
        vTaskDelay(100);
        uint8_t menuMode = 1;
        uint8_t units = 0;
        b2 = 0;
        b1 = 0;
        b3 = 0;
        while (menuMode < 4)
        { // sub loop for getting progression through task
            if (b1 == 1)
            { // check for select
                b1 = 0;
                menuMode++;
                units = 0;
            }
            if (b2 == 1 && menuMode == 1)
            { // check for up
                b2 = 0;
                units = (units + 1) % 24;
            }
            else if (b2 == 1)
            {
                b2 = 0;
                units = (units + 1) % 60;
            }
            if (b3 == 1 && menuMode == 1)
            { // check for down
                b3 = 0;
                units = (24 + (units - 1)) % 24;
            }
            else if (b3 == 1)
            {
                b3 = 0;
                units = (60 + (units - 1)) % 60;
            }
            // write output
            if (menuMode == 1)
            {
                set.hour = units;
                uint16_t buffs[4];
                int index1 = set.hour / 10;
                int index2 = set.hour % 10;
                buffs[0] = charset[0];
                buffs[1] = charset[1];
                buffs[2] = alphafonttable[index1];
                buffs[3] = alphafonttable[index2];
                write_buff(buffs);
            } // hours
            else if (menuMode == 2)
            {
                set.min = units;
                uint16_t buffs[4];
                buffs[0] = charset[2];
                buffs[1] = charset[3];
                buffs[2] = alphafonttable[(int)set.min / 10];
                buffs[3] = alphafonttable[(int)set.min % 10];
                write_buff(buffs);
            } //minutes
            else if (menuMode == 3)
            {
                set.sec = units;
                uint16_t buffs[4];
                buffs[0] = charset[4];
                buffs[1] = charset[5];
                buffs[2] = alphafonttable[(int)set.sec / 10];
                buffs[3] = alphafonttable[(int)set.sec % 10];
                if (set.min == 0 && set.hour == 0 && set.sec < 10)
                    set.sec = 10; // faustian bargin
                write_buff(buffs);
            } //seconds
            else if (menuMode > 3)
            { //zero the menu mode
                current = set;
            }
        }
        vTaskResume(TaskHandle_1);
        vTaskSuspend(TaskHandle_3);
    }
}

static void timehandleactions()
{
    uint8_t mode_count = 0;
    bool mode_debounce = 0;

    //feed buff
    static uint16_t feed[] = {
        0b0000000001110001, // F
        0b0000000011111001, // E
        0b0000000011111001, // E
        0b0001001000001111, // D
    };
    static uint16_t line[] = {
        0b0000000011000000, // -
        0b0000000011000000, // -
        0b0000000011000000, // -
        0b0000000011000000, // -
    };

    set.sec = TIMER_DEFAULT_VAL % 60;
    set.min = (TIMER_DEFAULT_VAL / 60) % 60;
    set.hour = TIMER_DEFAULT_VAL / (60 * 60);
    timer_event_t evt;
    current = set; //sets the default set to current;
    while (1)
    { // Create dummy structure to store structure from queue

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        if (mode_count >= NUMBER_FOR_MENU)
        {
            vTaskResume(TaskHandle_3); // start menu task
            mode_count = 0;
            vTaskSuspend(TaskHandle_1); // stop this task
        }                               // if menu button hit enough
        // Do something if triggered! on time
        if (evt.flag == 1)
        {
            evt.flag = 0;
            if (mode_debounce == 1)
                mode_debounce = 0; // check for mode count change
            else
                mode_count = 0;
            if (!(current.sec > 0 || current.hour > 0 || current.min > 0))
            {
                feeding = 1;
                current = set;
                vTaskResume(TaskHandle_2);
                printf("Feeding Time\n");
                write_buff(line);
                vTaskDelay(50);
                write_buff(feed);
                vTaskDelay(50);
                write_buff(line);
                vTaskDelay(50);
                write_buff(feed);
                vTaskDelay(50);
                write_buff(line);
                vTaskDelay(50);
                write_buff(feed);
                vTaskDelay(50);
                write_buff(line);
                vTaskDelay(50);
            }                 // end of counter trigger feeding
            update_display(); // update time
        }
        if (b1 == 1)
        { // check for mode switch
            b1 = 0;
            mode_debounce = 1;
            mode_count++;
        }
    }
}

void app_main()
{
    // Create a FIFO queue for timer-based

    int threads = 3;
    init();
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    xTaskCreate(timehandleactions, "time_actions", 4096 / threads, NULL, configMAX_PRIORITIES, &TaskHandle_1);
    xTaskCreate(feed, "feed", 4096 / threads, NULL, configMAX_PRIORITIES - 1, &TaskHandle_2);
    xTaskCreate(menu, "menu", 4096 / threads, NULL, configMAX_PRIORITIES, &TaskHandle_3);
}
