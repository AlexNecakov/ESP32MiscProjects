#include "i2clidar.h"

void setPowerMode(uint8_t slaveAddress, uint8_t powerMode)
{
    printf("Setting power mode\n");
    uint8_t powerBuffer = powerMode;

    if (writeI2C(slaveAddress, 0xE2, &powerBuffer, 1) == ESP_OK)
    {
        printf("Power mode set\n");
    }
    else
    {
        printf("Power mode set failed\n");
    }

    uint8_t readBuffer;
    readI2C(slaveAddress, 0xE2, &readBuffer, 1);
    printf("Read back %x\n", readBuffer);

    // Wait for the I2C peripheral to be restarted with new device address
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
uint16_t readLidar(uint8_t slaveAddr)
{
    uint8_t primeByte = 0x04;
    writeI2C(slaveAddr, 0x00, &primeByte, 1);

    uint8_t readyCheck = 1;
    while (readyCheck)
    {
        readI2C(slaveAddr, 0x01, &readyCheck, 1);
        readyCheck &= 0x80;
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    uint8_t reading[2];
    readI2C(slaveAddr, 0x10, reading, 2);

    uint16_t reading16 = (reading[1] << 8) | reading[0];
    return reading16;
}
void resetLidar(uint8_t slaveAddress)
{
    uint8_t writeBuffer = 0x01;
    int ret = writeI2C(slaveAddress, 0xE4, &writeBuffer, 1);
    if (ret == ESP_OK)
    {
        printf("Reset success\n");
    }
    else
    {
        printf("Reset failed\n");
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);
}
void setI2CAddr(uint8_t newAddress, uint8_t lidarliteAddress)
{
    printf("Setting I2C Address from %x to %x\n", lidarliteAddress, newAddress);
    uint8_t dataBytes[5];

    // Enable flash storage
    dataBytes[0] = 0x11;
    writeI2C(lidarliteAddress, 0xEA, dataBytes, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Read 4-byte device serial number
    readI2C(lidarliteAddress, 0x16, dataBytes, 4);

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Write the serial number and new address in one 5-byte transaction
    writeI2C(lidarliteAddress, 0x16, dataBytes, 5);

    // Wait for the I2C peripheral to be restarted with new device address
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // If desired, disable default I2C device address (using the new I2C device address)
    // dataBytes[0] = 0x01; // set bit to disable default address
    // writeI2C(newAddress, 0x1b, dataBytes, 1);

    // Wait for the I2C peripheral to be restarted with new device address
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Disable flash storage
    dataBytes[0] = 0;
    if (writeI2C(newAddress, 0xEA, dataBytes, 1) == ESP_OK)
    {
        printf("I2C address set\n");
    }
    else
    {
        printf("Address set failed\n");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
