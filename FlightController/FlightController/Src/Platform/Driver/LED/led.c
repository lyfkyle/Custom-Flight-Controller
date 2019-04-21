#include "stm32f4xx_hal.h"

#include "led.h"

#define GREEN_LED_Pin GPIO_PIN_0
#define BLUE_LED_Pin GPIO_PIN_7
#define RED_LED_Pin GPIO_PIN_14

bool LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
    GPIO_InitStruct.Pin = BLUE_LED_Pin|GREEN_LED_Pin|RED_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, RED_LED_Pin|GREEN_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);
    return true;
}

bool LED_SetOn(LEDType led, bool on)
{
    switch(led) {
    case LED_RED:
        HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case LED_GREEN:
        HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case LED_BLUE:
        HAL_GPIO_WritePin(GPIOB, BLUE_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    }

    return true;
}
