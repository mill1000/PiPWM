#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "log.h"

void logPrint(LOG_LEVEL level, const char* format, ...)
{
  va_list list;
  va_start(list, format);
  vprintf(format, list);
  va_end(list);

  if (level == LOG_LEVEL_FATAL)
    exit(EXIT_FAILURE);
}