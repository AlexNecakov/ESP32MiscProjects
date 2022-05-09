#ifndef I2CHELP_H
#define I2CHELP_H

#include "driver/i2c.h"
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

void initI2CMaster();
int writeI2C(uint8_t slaveAddr, uint8_t reg, uint8_t *data, uint8_t bytes);
int readI2C(uint8_t slaveAddr, uint8_t reg, uint8_t *buffer, uint8_t bytes);
#endif
