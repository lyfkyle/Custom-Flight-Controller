#ifndef DRIVER_I2C_H_
#define DRIVER_I2C_H_

#include <stdint.h>

bool I2C_Init();
void I2C_Write(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t *pData, uint16_t size);
void I2C_Read(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t *pData, uint16_t* pSize);

#endif