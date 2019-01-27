#ifndef _LED_H_
#define _LED_H_

/*
 * Struct
 */

typedef enum {
    LED_RED,
    LED_GREEN,
    LED_BLUE,
} LEDType;

#ifdef __cplusplus
extern "C" {
#endif

bool LED_Init();
bool LED_SetOn(LEDType led, bool on);
// bool LED_SetRGBColour(uint32_t colour);

#ifdef __cplusplus
}
#endif

#endif