#ifndef __LOG__
#define __LOG__

typedef enum
{
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARN,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_FATAL,
} LOG_LEVEL;

#define LOG_FORMAT(LETTER, FORMAT)  #LETTER ": %s: " FORMAT "\n"

#define _LOG(LEVEL, FORMAT, ...) do {logPrint(LOG_LEVEL_INFO, FORMAT, ##__VA_ARGS__);} while(0)

#define LOGI(TAG, FORMAT, ...) _LOG(LOG_LEVEL_INFO, LOG_FORMAT(I, FORMAT), TAG, ##__VA_ARGS__)
#define LOGW(TAG, FORMAT, ...) _LOG(LOG_LEVEL_WARN, LOG_FORMAT(W, FORMAT), TAG, ##__VA_ARGS__)
#define LOGE(TAG, FORMAT, ...) _LOG(LOG_LEVEL_ERROR, LOG_FORMAT(E, FORMAT), TAG, ##__VA_ARGS__)
#define LOGF(TAG, FORMAT, ...) _LOG(LOG_LEVEL_FATAL, LOG_FORMAT(F, FORMAT), TAG, ##__VA_ARGS__)

void logPrint(LOG_LEVEL level, const char* format, ...);
#endif