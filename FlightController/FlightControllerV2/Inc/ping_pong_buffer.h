#ifndef _LIB_PING_PONG_BUFFER_H_
#define _LIB_PING_PONG_BUFFER_H_

#include <stdint.h>

typedef struct {
    uint32_t size;
    uint8_t* pBuf1;
    uint8_t* pBuf2;
    int writeBufNum;
} PPBufferType;

PPBufferType* PingPongBuffer_Init(uint32_t size);
void PingPongBuffer_DeInit(PPBufferType* pPPBuffer);
bool PingPongBuffer_Write(PPBufferType* pPPBuffer, const uint8_t* pData, const uint32_t size);
bool PingPongBuffer_Read(PPBufferType* pPPBuffer, uint8_t* pData, uint32_t* pDataSize);

#endif