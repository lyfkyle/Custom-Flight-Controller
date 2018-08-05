
#ifndef LIB_LOGGING_H_
#define LIB_LOGGING_H_

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * Defines
 */

#define LOG_VERBOSE (0)
#define LOG_INFO  (1)
#define LOG_ERROR (2)
#define LOG_WARNING (3)

#define LOGE(...) LogPrint(LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...) LogPrint(LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGW(...) LogPrint(LOG_WARNING, LOG_TAG, __VA_ARGS__)
#define LOGV(...) LogPrint(LOG_VERBOSE, LOG_TAG, __VA_ARGS__)

/*
 * Static
 */

static char LEVEL_MAP[] = {
   'V',
   'I',
   'W',
   'E',
};

/*
 * Prototype
 */
void LogPrint(int level, const char* pTag, const char* pFmt, ...);

#endif