#ifndef DRIVER_SBUS_H_
#define DRIVER_SBUS_H_

#include "stm32f4xx_hal.h"

// empirical
#define SBUS_CHANNEL_MIN 172
#define SBUS_CHANNEL_MAX 1811

typedef struct {
    int channels[16]; // val between SBUS_CHANNEL_MIN and SBUS_CHANNEL_MAX
    bool failsafe;
    bool lostFrame;
} SBUSDataType;

#ifdef __cplusplus
extern "C" {
#endif

bool SBUS_Init();
bool SBUS_Start();
void SBUS_InterruptHandler();
void SBUS_DMAInterruptHandler();
bool SBUS_Read(SBUSDataType* pSBUSData);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif
#endif