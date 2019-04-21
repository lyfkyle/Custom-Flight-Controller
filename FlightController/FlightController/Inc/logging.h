
#ifndef LIB_LOGGING_H_
#define LIB_LOGGING_H_

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

/*
 * Defines
 */

#define LOG_VERBOSE (3)
#define LOG_INFO  (2)
#define LOG_WARNING (1)
#define LOG_ERROR (0)

#define LOG_LEVEL LOG_INFO

#if LOG_LEVEL >= LOG_ERROR
#define LOGE(...) LogPrint(LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGE(...)
#endif
#if LOG_LEVEL >= LOG_INFO
#define LOGI(...) LogPrint(LOG_INFO, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#endif
#if LOG_LEVEL >= LOG_WARNING
#define LOGW(...) LogPrint(LOG_WARNING, LOG_TAG, __VA_ARGS__)
#else
#define LOGW(...)
#endif
#if LOG_LEVEL >= LOG_VERBOSE
#define LOGV(...) LogPrint(LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#else
#define LOGV(...)
#endif
#define PRINT(...) Print(__VA_ARGS__)

/*
 * Static
 */

static char LEVEL_MAP[] = {
   'E',
   'W',
   'I',
   'V',
};

/*
 * Prototype
 */
void LogPrint(int level, const char* pTag, const char* pFmt, ...);
void Print(const char* pFmt, ...);

#endif