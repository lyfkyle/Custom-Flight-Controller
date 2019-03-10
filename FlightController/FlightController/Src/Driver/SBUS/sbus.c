#include <string.h>

#include "sbus.h"

#include "led.h"
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

#define SBUS_PRINT_RECEIVED_MSG (0)

#define LOSTFRAME_MASK (0x20)
#define FAILSAFE_MASK (0x10)

/*
 * Constant
 */

#define SBUS_HEADER 0x0f // little endian
#define SBUS_ENDBYTE 0x00
#define SBUS_BAUDRATE 100000
#define SBUS_MSG_LENGTH 25

// empirical
#define SBUS_CHANNEL_MIN 172
#define SBUS_CHANNEL_MAX 1811

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
static DMA_HandleTypeDef hdma_usart6_rx;

static volatile bool sStarted = false;
// static SBUSDetectStateType sDetectState = SBUS_IDLE;
static uint8_t sRecBuffer[SBUS_MSG_LENGTH];
// static uint8_t sRecBytesCnt = 0;
static PPBufferType* spPPBuffer;
static uint8_t sRetryCnt = 0;

static float sChannelOutMin = 0.0f;
static float sChannelOutMax = 100.0f;

/*
 * Prototypes
 */
#if 0
static bool SBUS_DetectMsg(uint8_t data);
#endif

/*
 * Code
 */

#if 0 // not used. As we are not receiving data byte by byte
/* Detect SBUS data */
static bool SBUS_DetectMsg(uint8_t data)
{
    // find the header
    if (sDetectState == SBUS_IDLE) {
        if (data == SBUS_HEADER) {
            sDetectState = SBUS_HEADER_DETECTED;
            LED_SetOn(LED_RED, true);
        }
    }
    else if (sDetectState == SBUS_HEADER_DETECTED) {
        sRecBuffer[sRecBytesCnt++] = data;
        if (sRecBytesCnt == SBUS_MSG_LENGTH) {
            if (sRecBuffer[SBUS_MSG_LENGTH - 1] == SBUS_ENDBYTE) {
                // endbyte valid. Valid data, copy to buffer
                //LOG("received SBUS data\r\n");
                // memcpy(sReceivedMsg, sRecBuffer, SBUS_MSG_LENGTH);
                PingPongBuffer_Write(spPPBuffer, sRecBuffer, SBUS_MSG_LENGTH);
            } else {
                //LOGE("Invalid SBUS data received\r\n");
            }
            sRecBytesCnt = 0;
            sDetectState = SBUS_IDLE;
        }
    }
    return true;
}
#endif

static bool DMA_Init()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    return true;
}

static bool UART_Init()
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = SBUS_BAUDRATE;
    huart6.Init.WordLength = UART_WORDLENGTH_9B;
    huart6.Init.StopBits = UART_STOPBITS_2;
    huart6.Init.Parity = UART_PARITY_EVEN;
    huart6.Init.Mode = UART_MODE_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        return false;
    }

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      return false;
    }
    __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);

    // enable data ready interrupt
    // __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    // enable DMA transfer complete interrupt
    // __HAL_DMA_ENABLE_IT(&hdma_usart6_rx, UART_IT_TC) ;
    return true;
}

/*------------------------------------------*
 * Callbacks/Interrupts
 *------------------------------------------*/

void SBUS_InterruptHandler()
{
    // Custom handling
    if (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE)) {
        HAL_UART_Receive_DMA(&huart6, sRecBuffer, SBUS_MSG_LENGTH); // this is time-critical.
        __HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
    }

    // HAL handling
    HAL_UART_IRQHandler(&huart6);
}

void SBUS_DMAInterruptHandler()
{
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    LOG("SBUS ERROR %d\r\n", huart->ErrorCode);
    ++sRetryCnt;
    LOG("Retry cnt : %d\r\n", sRetryCnt);
    HAL_UART_Receive_DMA(&huart6, sRecBuffer, 5);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    LED_SetOn(LED_GREEN, true);
#if SBUS_PRINT_RECEIVED_MSG
    for (int i = 0; i < 25; ++i) {
        PRINT("0x%x ", sRecBuffer[i]);
    }
    PRINT("\r\n");
#endif
    if (sRecBuffer[0] != SBUS_HEADER) {
        // the first byte is not header. Try again.
        PRINT("SBUS try again %d\r\n", sRecBuffer[0]);
        HAL_UART_DMAStop(&huart6);
        __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
        return;
    }
    sStarted = true;
    PingPongBuffer_Write(spPPBuffer, sRecBuffer, SBUS_MSG_LENGTH);
}

bool SBUS_Init()
{
    if (DMA_Init()) {
        LOG("SBUS DMA init success\r\n");
    }
    if (UART_Init()) {
        LOG("SBUS UART init success\r\n");
    } else {
        LOGE("SBUS UART init failed\r\n");
        return false;
    }

    spPPBuffer = PingPongBuffer_Init(SBUS_MSG_LENGTH);
    if (!spPPBuffer) {
        LOGE("PingPongBuffer_Init failed\r\n");
        return false;
    }
    return true;
}

bool SBUS_Start()
{
    // enable idle line interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // wait for dma to detect header
    int cnt = 5;
    while (!sStarted && --cnt) {
        HAL_Delay(1000); // wait 5 * 1 = 5 sec
    }

    if (!sStarted) {
        LED_SetOn(LED_RED, true);
        LOGE("SBUS failed to start\r\n");
        return false;
    }
    LED_SetOn(LED_GREEN, false);
    LED_SetOn(LED_BLUE, true);
    return true;
}

bool SBUS_SetChannelRange(float min, float max)
{
    sChannelOutMin = min;
    sChannelOutMax = max;
    return true;
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
    uint16_t channels_int[16];
    channels_int[0]  = (uint16_t) ((receivedMsg[1]    | receivedMsg[2] <<8)                       & 0x07FF);
    channels_int[1]  = (uint16_t) ((receivedMsg[2]>>3 | receivedMsg[3] <<5)                       & 0x07FF);
    channels_int[2]  = (uint16_t) ((receivedMsg[3]>>6 | receivedMsg[4] <<2 | receivedMsg[5]<<10)  & 0x07FF);
    channels_int[3]  = (uint16_t) ((receivedMsg[5]>>1 | receivedMsg[6] <<7)                       & 0x07FF);
    channels_int[4]  = (uint16_t) ((receivedMsg[6]>>4 | receivedMsg[7] <<4)                       & 0x07FF);
    channels_int[5]  = (uint16_t) ((receivedMsg[7]>>7 | receivedMsg[8] <<1 | receivedMsg[9]<<9)   & 0x07FF);
    channels_int[6]  = (uint16_t) ((receivedMsg[9]>>2 | receivedMsg[10] <<6)                      & 0x07FF);
    channels_int[7]  = (uint16_t) ((receivedMsg[10]>>5 | receivedMsg[11]<<3)                      & 0x07FF);
    channels_int[8]  = (uint16_t) ((receivedMsg[12]   | receivedMsg[13]<<8)                       & 0x07FF);
    channels_int[9]  = (uint16_t) ((receivedMsg[13]>>3| receivedMsg[14]<<5)                       & 0x07FF);
    channels_int[10] = (uint16_t) ((receivedMsg[14]>>6| receivedMsg[15]<<2 | receivedMsg[16]<<10) & 0x07FF);
    channels_int[11] = (uint16_t) ((receivedMsg[16]>>1| receivedMsg[17]<<7)                       & 0x07FF);
    channels_int[12] = (uint16_t) ((receivedMsg[17]>>4| receivedMsg[18]<<4)                       & 0x07FF);
    channels_int[13] = (uint16_t) ((receivedMsg[18]>>7| receivedMsg[19]<<1 | receivedMsg[20]<<9)  & 0x07FF);
    channels_int[14] = (uint16_t) ((receivedMsg[20]>>2| receivedMsg[21]<<6)                       & 0x07FF);
    channels_int[15] = (uint16_t) ((receivedMsg[21]>>5| receivedMsg[22]<<3)                       & 0x07FF);
    // parse
    for (int i = 0; i < 16; ++i) {
        if (i < 4) LOGI("SBUSData: channel %d : %d\r\n", i, channels_int[i]);
        pSBUSData->channels[i] = ((float) (channels_int[i] - SBUS_CHANNEL_MIN)) / (SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN) * (sChannelOutMax - sChannelOutMin) + sChannelOutMin;
    }

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
