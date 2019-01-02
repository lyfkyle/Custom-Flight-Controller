#ifndef DRIVER_PWM_H_
#define DRIVER_PWM_H_

#include <stdint.h>

typedef enum {
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
} PWMChannelType;

#ifdef __cplusplus
extern "C" {
#endif

bool PWM_Init();
void PWM_Start();
void PWM_Stop();
// bool PWM_SetFrequency();
void PWM_SetDutyCycle(PWMChannelType channel, uint8_t dutyCycle);

#ifdef __cplusplus
}
#endif

#endif