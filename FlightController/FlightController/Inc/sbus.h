#ifndef DRIVER_SBUS_H_
#define DRIVER_SBUS_H_

typedef struct {
    uint16_t channels[16];
    bool failsafe;
    bool lostFrame;
} SBUSDataType;

#ifdef __cplusplus
extern "C" {
#endif

bool SBUS_Init();
void SBUS_InterruptHandler();
bool SBUS_Read(SBUSDataType* pSBUSData);

#ifdef __cplusplus
}
#endif
#endif