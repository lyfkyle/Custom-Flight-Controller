#ifndef DRIVER_SBUS_H_
#define DRIVER_SBUS_H_

#include "stm32f4xx_hal.h"

typedef struct {
    float channels[16];
    bool failsafe;
    bool lostFrame;
} SBUSDataType;

#ifdef __cplusplus
extern "C" {
#endif

bool SBUS_Init();
bool SBUS_Start();
bool SBUS_SetChannelRange(float min, float max);
void SBUS_InterruptHandler();
void SBUS_DMAInterruptHandler();
bool SBUS_Read(SBUSDataType* pSBUSData);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif
#endif