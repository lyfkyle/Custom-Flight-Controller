#include "stm32f4xx_hal.h"
#include <string.h>

#include "sbus.h"

#include "logging.h"
#include "ping_pong_buffer.h"

/*
 * Defines
 */

#define LOG_TAG ("SBUS")
#define SBUS_DEBUG (0)

#ifdef SBUS_DEBUG 
#define LOG(...) LOGI(__VA_ARGS__)
#else
#define LOG(...)
#endif

#define LOSTFRAME_MASK (0x20)
#define FAILSAFE_MASK (0x10)

/*
 * Constant
 */

#define SBUS_HEADER 0x0F
#define SBUS_ENDBYTE 0x00
#define SBUS_BAUDRATE 100000
#define SBUS_MSG_LENGTH 25

/*
 * Struct
 */

typedef enum {
    SBUS_IDLE,
    SBUS_HEADER_DETECTED,
} SBUSDetectStateType;

/*
 * Static
 */

static UART_HandleTypeDef huart6;

static SBUSDetectStateType sDetectState = SBUS_IDLE;
static uint8_t sRecBuffer[SBUS_MSG_LENGTH];
static uint8_t sRecBytesCnt = 0;
static PPBufferType* spPPBuffer;

/*
 * Prototypes
 */
static bool SBUS_DetectMsg(uint8_t data);

/*
 * Code
 */

/* Detect SBUS data */
static bool SBUS_DetectMsg(uint8_t data)
{
    // find the header
    if (sDetectState == SBUS_IDLE) {
        if (data == SBUS_HEADER) {
            sDetectState = SBUS_HEADER_DETECTED;
        } 
    } 
    else if (sDetectState == SBUS_HEADER_DETECTED) {
        sRecBuffer[sRecBytesCnt++] = data;
        if (sRecBytesCnt == SBUS_MSG_LENGTH) {
            if (sRecBuffer[SBUS_MSG_LENGTH - 1] == SBUS_ENDBYTE) {
                // endbyte valid. Valid data, copy to buffer
                LOG("received SBUS data\r\n");
                // memcpy(sReceivedMsg, sRecBuffer, SBUS_MSG_LENGTH);
                PingPongBuffer_Write(spPPBuffer, sRecBuffer, SBUS_MSG_LENGTH);
            } else {
                LOGE("Invalid SBUS data received\r\n");
            }
            sRecBytesCnt = 0;
            sDetectState = SBUS_IDLE;
        }
    }
    return true;
}

static bool UART_Init()
{
   huart6.Instance = USART6;
   huart6.Init.BaudRate = SBUS_BAUDRATE;
   huart6.Init.WordLength = UART_WORDLENGTH_8B;
   huart6.Init.StopBits = UART_STOPBITS_2;
   huart6.Init.Parity = UART_PARITY_EVEN;
   huart6.Init.Mode = UART_MODE_TX_RX;
   huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart6.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart6) != HAL_OK)
   {
      return false;
   }
   // enable data ready interrupt
   __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
   return true;
}

bool SBUS_Init()
{
    if (UART_Init()) {
        LOGI("SBUS UART init success\n");
    } else {
        LOGE("SBUS UART init failed\n");
        return false;
    }
    
    spPPBuffer = PingPongBuffer_Init(SBUS_MSG_LENGTH);
    if (!spPPBuffer) {
        LOGE("PingPongBuffer_Init failed\r\n");
        return false;
    }
    return true;
}

void SBUS_InterruptHandler()
{    
    // HAL handling
    HAL_UART_IRQHandler(&huart6);
    // Custom handling
    if (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_RXNE)) {
        uint8_t data;
        HAL_UART_Receive(&huart6, &data, 1, 5); // 5ms timeout
        SBUS_DetectMsg(data);
    }
}

bool SBUS_Read(SBUSDataType* pSBUSData)
{
    if (!pSBUSData) {
        LOGE("%s, input invalid\r\n");
        return false;
    }
    
    uint8_t receivedMsg[SBUS_MSG_LENGTH];
    uint32_t msgSize = SBUS_MSG_LENGTH;
    PingPongBuffer_Read(spPPBuffer, receivedMsg, &msgSize);
    // 16 channels of 11 bit data
    pSBUSData->channels[0]  = (uint16_t) ((receivedMsg[0]    | receivedMsg[1] <<8)                       & 0x07FF);
    pSBUSData->channels[1]  = (uint16_t) ((receivedMsg[1]>>3 | receivedMsg[2] <<5)                       & 0x07FF);
    pSBUSData->channels[2]  = (uint16_t) ((receivedMsg[2]>>6 | receivedMsg[3] <<2 | receivedMsg[4]<<10)  & 0x07FF);
    pSBUSData->channels[3]  = (uint16_t) ((receivedMsg[4]>>1 | receivedMsg[5] <<7)                       & 0x07FF);
    pSBUSData->channels[4]  = (uint16_t) ((receivedMsg[5]>>4 | receivedMsg[6] <<4)                       & 0x07FF);
    pSBUSData->channels[5]  = (uint16_t) ((receivedMsg[6]>>7 | receivedMsg[7] <<1 | receivedMsg[8]<<9)   & 0x07FF);
    pSBUSData->channels[6]  = (uint16_t) ((receivedMsg[8]>>2 | receivedMsg[9] <<6)                       & 0x07FF);
    pSBUSData->channels[7]  = (uint16_t) ((receivedMsg[9]>>5 | receivedMsg[10]<<3)                       & 0x07FF);
    pSBUSData->channels[8]  = (uint16_t) ((receivedMsg[11]   | receivedMsg[12]<<8)                       & 0x07FF);
    pSBUSData->channels[9]  = (uint16_t) ((receivedMsg[12]>>3| receivedMsg[13]<<5)                       & 0x07FF);
    pSBUSData->channels[10] = (uint16_t) ((receivedMsg[13]>>6| receivedMsg[14]<<2 | receivedMsg[15]<<10) & 0x07FF);
    pSBUSData->channels[11] = (uint16_t) ((receivedMsg[15]>>1| receivedMsg[16]<<7)                       & 0x07FF);
    pSBUSData->channels[12] = (uint16_t) ((receivedMsg[16]>>4| receivedMsg[17]<<4)                       & 0x07FF);
    pSBUSData->channels[13] = (uint16_t) ((receivedMsg[17]>>7| receivedMsg[18]<<1 | receivedMsg[19]<<9)  & 0x07FF);
    pSBUSData->channels[14] = (uint16_t) ((receivedMsg[19]>>2| receivedMsg[20]<<6)                       & 0x07FF);
    pSBUSData->channels[15] = (uint16_t) ((receivedMsg[20]>>5| receivedMsg[21]<<3)                       & 0x07FF);
    // count lost frames
    if (receivedMsg[22] & LOSTFRAME_MASK) {
        pSBUSData->lostFrame = true;
    } else {
        pSBUSData->lostFrame = false;
    }
    // failsafe state
    if (receivedMsg[22] & FAILSAFE_MASK) {
        pSBUSData->failsafe = true;
    }
    else{
        pSBUSData->failsafe = false;
    }

    // return true on receiving a full packet
    return true;
}
