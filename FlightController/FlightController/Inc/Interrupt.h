#ifndef _DRIVER_INTERRUPT_
#define _DRIVER_INTERRUPT_

typedef enum {
    INTERRUPT_MODULE_IMU,
    INTERRUPT_MODULE_I2C,
    NUM_OF_INTERRUPT_MODULE,
} InterruptModuleType;

typedef void (*Interrupt_Handler)(void* pParam);

void InterruptInit();
void Interrupt_RegisterISR(int moduleId, void* pParam, Interrupt_Handler handler);

#endif