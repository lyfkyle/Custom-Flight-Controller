#include "stm32f1xx_hal.h"

#include "uart.h"

#include "logging.h"

#define LOG_TAG ("UART")

extern UART_HandleTypeDef huart2;

/* USART3 init function */
bool UART_Init()
{
#if 0
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        LOGE("HAL_UART_Init failed\r\n");
        return false;
    }
#endif
    return true;
}

void UART_Send(const char* pData, const int dataSize)
{
   HAL_UART_Transmit(&huart2, (uint8_t *)pData, dataSize, 50);
}