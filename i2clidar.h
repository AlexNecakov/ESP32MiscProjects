#ifndef I2CLIDAR_H
#define I2CLIDAR_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2chelp.h"

#define ASYNC_POWER_MODE 0x00     //off unless distance measurement required
#define SYNC_POWER_MODE 0x01      //uses ANT pin for timing
#define ALWAYS_ON_POWER_MODE 0xFF //default

#define OLD_LIDAR_ADDR 0x62 // factory default
#define NEW_LIDAR_ADDR 0x54 // insert new addr here

void setPowerMode(uint8_t slaveAddress, uint8_t powerMode);
uint16_t readLidar(uint8_t slaveAddr);
void resetLidar(uint8_t slaveAddress);
void setI2CAddr(uint8_t newAddress, uint8_t lidarliteAddress);

#endif
