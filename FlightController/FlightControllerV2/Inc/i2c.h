#ifndef DRIVER_I2C_H_
#define DRIVER_I2C_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool I2C_Init();
void I2C_InterruptHandler();
bool I2C_Write(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t *pData, uint16_t size);
bool I2C_Read(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize, uint8_t *pData, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif