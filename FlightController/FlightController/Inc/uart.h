#ifndef DRIVER_UART_H_
#define DRIVER_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

bool UART_Init();
void UART_Send(const char* pData, const int dataSize);

#ifdef __cplusplus
}
#endif
#endif