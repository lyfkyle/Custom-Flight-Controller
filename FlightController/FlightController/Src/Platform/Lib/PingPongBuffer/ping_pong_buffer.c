#include <stdlib.h>
#include <string.h>

#include "ping_pong_buffer.h"

#include "logging.h"

#define LOG_TAG ("PPBuffer")

#define BUF1 1
#define BUF2 2

PPBufferType* PingPongBuffer_Init(uint32_t size)
{
    PPBufferType* pPPBuffer = (PPBufferType*) malloc(sizeof(PPBufferType));
    if(pPPBuffer) {
        pPPBuffer->pBuf1 = (uint8_t*) malloc(size);
        pPPBuffer->pBuf2 = (uint8_t*) malloc(size);
        pPPBuffer->writeBufNum = BUF1;
        pPPBuffer->size = size;
        if (pPPBuffer->pBuf1 && pPPBuffer->pBuf2) {
            memset(pPPBuffer->pBuf1, 0, size);
            memset(pPPBuffer->pBuf2, 0, size);
            return pPPBuffer;
        } else {
            PingPongBuffer_DeInit(pPPBuffer);
        }
    }
    return NULL;
}

void PingPongBuffer_DeInit(PPBufferType* pPPBuffer)
{
    if (pPPBuffer) {
        if (pPPBuffer->pBuf1) free(pPPBuffer->pBuf1);
        if (pPPBuffer->pBuf2) free(pPPBuffer->pBuf2);
        free(pPPBuffer);
    }
}

bool PingPongBuffer_Write(PPBufferType* pPPBuffer, const uint8_t* pData, const uint32_t dataSize)
{
    if (!pPPBuffer || !pData || dataSize == 0 || dataSize > pPPBuffer->size ) {
        LOGE("%s, input invalid\r\n", __func__);
        return false;
    }
    
    uint8_t* pBuf = pPPBuffer->pBuf1;
    if (pPPBuffer->writeBufNum == BUF2) {
        pBuf = pPPBuffer->pBuf2;
        pPPBuffer->writeBufNum = BUF1;
    } else {
        pPPBuffer->writeBufNum = BUF2;
    }
    memcpy(pBuf, pData, dataSize);
    
    return true;
}

bool PingPongBuffer_Read(PPBufferType* pPPBuffer, uint8_t* pData, uint32_t* pDataSize)
{
    if (!pPPBuffer || !pData || !pDataSize || *pDataSize < pPPBuffer->size) {
        LOGE("%s, input invalid\r\n", __func__);
        return false;
    }
    
    uint8_t* pBuf = pPPBuffer->pBuf1;
    if (pPPBuffer->writeBufNum == BUF1) {
        pBuf = pPPBuffer->pBuf2;
    }
    memcpy(pData, pBuf, pPPBuffer->size);
    memset(pBuf, 0, pPPBuffer->size); // clear content after read.
    *pDataSize = pPPBuffer->size;
    
    return true;
}

