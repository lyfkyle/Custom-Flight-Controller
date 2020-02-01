#include "stm32f1xx_hal.h"

#include "led.h"

#define ONBOARD_LED_Pin GPIO_PIN_13

static uint8_t sOnBoardLedStatus = GPIO_PIN_RESET;

bool LED_Init()
{
#if 0
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
    GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
    HAL_GPIO_WritePin(GPIOC, ONBOARD_LED_Pin, GPIO_PIN_SET); // For blue pill, set the pin to low to light up LED
    return true;
}

bool LED_SetOn(LEDType led, bool on)
{
    switch(led) {
    case LED_ONBOARD:
        HAL_GPIO_WritePin(GPIOC, ONBOARD_LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    }

    return true;
}

bool LED_Toggle(LEDType led)
{
    switch(led) {
    case LED_ONBOARD:
        sOnBoardLedStatus ^= 1;
        HAL_GPIO_WritePin(GPIOC, ONBOARD_LED_Pin, sOnBoardLedStatus ? GPIO_PIN_RESET : GPIO_PIN_SET);
        break;
    }
    return true;
}

bool LED_Blink(LEDType led, uint8_t cnt)
{
    // we blink every 200ms
    for (uint8_t i = 0; i < cnt; ++i) {
        LED_Toggle(led);
        HAL_Delay(200);
    }
    return true;
}