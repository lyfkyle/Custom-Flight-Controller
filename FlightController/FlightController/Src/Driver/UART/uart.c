
#include "stm32f4xx_hal.h"

#include "uart.h"

static UART_HandleTypeDef huart3;

bool UART_Init()
{
   huart3.Instance = USART3;
   huart3.Init.BaudRate = 115200;
   huart3.Init.WordLength = UART_WORDLENGTH_8B;
   huart3.Init.StopBits = UART_STOPBITS_1;
   huart3.Init.Parity = UART_PARITY_NONE;
   huart3.Init.Mode = UART_MODE_TX_RX;
   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart3) != HAL_OK)
   {
      return false;
   }
   return true;
}

void UART_Send(const char* pData, const int dataSize)
{
   HAL_UART_Transmit(&huart3, (uint8_t *)pData, dataSize, 0xFFFF);
}