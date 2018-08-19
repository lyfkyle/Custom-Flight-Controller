#include "stm32f4xx_hal.h"

#include <Interrupt.h>

#include "i2c.h"

#define I2C_TIMEOUT (100)

static I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
//static void MX_I2C1_Init(void)
//{
//
//   hi2c1.Instance = I2C1;
//   hi2c1.Init.ClockSpeed = 100000;
//   hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//   hi2c1.Init.OwnAddress1 = 0;
//   hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//   hi2c1.Init.OwnAddress2 = 0;
//   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//   hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//   if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//   {
//      Error_Handler();
//   }
//
//}

static void I2C_ItHandler(void* pParam)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

bool I2C_Init()
{
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

   Interrupt_RegisterISR(INTERRUPT_MODULE_I2C, NULL, I2C_ItHandler);

   return true;
}

void I2C_Write(uint16_t devAddr, uint16_t memAddr, uint16_t memAddSize, uint8_t *pData, uint16_t size)
{
   HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, memAddSize, pData, size, I2C_TIMEOUT);
}

void I2C_Read(uint16_t devAddr, uint16_t memAddr, uint16_t memAddSize, uint8_t *pData, uint16_t* pSize)
{
   // TODO check manual for the size
   HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, memAddSize, pData, *pSize, I2C_TIMEOUT);
}

