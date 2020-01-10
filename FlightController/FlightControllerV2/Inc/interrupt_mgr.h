#ifndef _COMPONENT_INTERRUPT_MGR_
#define _COMPONENT_INTERRUPT_MGR_

//typedef enum {
//    INTERRUPT_MODULE_IMU,
//    INTERRUPT_MODULE_I2C,
//    NUM_OF_INTERRUPT_MODULE,
//} InterruptModuleType;

//void InterruptMgr_Init();
typedef void (*InterruptHandler)(void);

void InterruptMgr_RegisterSystickHandler(InterruptHandler handler);

#endif