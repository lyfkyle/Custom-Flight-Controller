#ifndef DRIVER_UART_H_
#define DRIVER_UART_H_

void UART_Init();
void UART_Send(const char* pData, const int dataSize);

#endif