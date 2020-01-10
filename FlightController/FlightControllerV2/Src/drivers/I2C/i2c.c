#include "stm32f1xx_hal.h"

#include "i2c.h"

#include "logging.h"

#define LOG_TAG ("I2C")

#define I2C_TIMEOUT (100)

extern I2C_HandleTypeDef hi2c1;

/*
 * Code
 */

bool I2C_Init()
{
   /*
   hi2c1.Instance = I2C1;
   hi2c1.Init.ClockSpeed = 100000;
   hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
   hi2c1.Init.OwnAddress1 = 0;
   hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
   hi2c1.Init.OwnAddress2 = 0;
   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
   hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
   if (HAL_I2C_Init(&hi2c1) != HAL_OK)
   {
      return false;
   }
   */
   return true;
}

void I2C_InterruptHandler()
{
    // HAL handling
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

bool I2C_Write(uint16_t devAddr, uint16_t memAddr, uint16_t memAddSize, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, memAddSize, pData, size, I2C_TIMEOUT);
    if (status != HAL_OK) {
        LOGE("HAL_I2C_Mem_Write failed, status = %d\r\n", status);
        return false;
    }
    return true;
}

bool I2C_Read(uint16_t devAddr, uint16_t memAddr, uint16_t memAddSize, uint8_t *pData, uint16_t size)
{
   HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, memAddSize, pData, size, I2C_TIMEOUT);
   if (status != HAL_OK) {
       LOGE("HAL_I2C_Mem_Read failed, status = %d, memAddr = %d\r\n", status, memAddr);
       return false;
   }
   return true;
}

