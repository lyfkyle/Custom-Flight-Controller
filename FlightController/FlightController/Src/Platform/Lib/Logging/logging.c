
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <uart.h>

#include "logging.h"

#define HEADER_LEN (16)
#define MSG_LEN    (128)

static char headerBuf[HEADER_LEN] = {0};
static char msgBuf[MSG_LEN] = {0};

void LogPrint(int level, const char* pTag, const char* pFmt, ...)
{
   va_list args;
   va_start(args, pFmt);
   // vnsprintf(msgBuf, MSG_LEN, pFmt, args); // vnsprintf seems to be unsupported
   vsprintf(msgBuf, pFmt, args);
   va_end(args);

   snprintf(headerBuf, HEADER_LEN, "%c|%-12.12s: ", LEVEL_MAP[level], pTag);
   UART_Send(headerBuf, HEADER_LEN);
   UART_Send(msgBuf, MSG_LEN);
   memset(msgBuf, 0, MSG_LEN);
}

void Print(const char* pFmt, ...)
{
   va_list args;
   va_start(args, pFmt);
   vsprintf(msgBuf, pFmt, args);
   va_end(args);
   UART_Send(msgBuf, MSG_LEN);
   memset(msgBuf, 0, MSG_LEN); // clear buffer
}

