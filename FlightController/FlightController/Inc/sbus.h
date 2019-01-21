#ifndef DRIVER_SBUS_H_
#define DRIVER_SBUS_H_

typedef struct {
    float channels[16];
    bool failsafe;
    bool lostFrame;
} SBUSDataType;

#ifdef __cplusplus
extern "C" {
#endif

bool SBUS_Init();
bool SBUS_SetChannelRange(float min, float max);
void SBUS_InterruptHandler();
bool SBUS_Read(SBUSDataType* pSBUSData);

#ifdef __cplusplus
}
#endif
#endif