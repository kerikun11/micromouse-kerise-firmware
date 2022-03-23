/**
 * @file app_log.h
 * @brief this provides logging formats
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-12-18
 */
#pragma once

#include "config/config.h"

/* printf format */
#include <esp_timer.h>
#include <cstdio>

/* log config */
#define APP_LOG_MEM 1
/* app log level */
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL 2
#endif

/* utils */
#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)
/* app log base */
#if APP_LOG_MEM
/* app log vars */
#define APP_LOG_BUFFER_SIZE 32768
static int app_log_buffer_ptr = 0;
static char app_log_buffer[APP_LOG_BUFFER_SIZE];
#define APP_LOG_BASE(l, c, f, ...)                                          \
  do {                                                                      \
    const int t = esp_timer_get_time();                                     \
    if (app_log_buffer_ptr > APP_LOG_BUFFER_SIZE - 64)                      \
      app_log_buffer_ptr = 0;                                               \
    app_log_buffer_ptr +=                                                   \
        std::snprintf(app_log_buffer + app_log_buffer_ptr,                  \
                      APP_LOG_BUFFER_SIZE - app_log_buffer_ptr,             \
                      c "[" l "][%d.%06d][" __FILE__                        \
                        ":" APP_TOSTRING(__LINE__) "][%s]\x1b[0m\t" f "\n", \
                      t / 1000000, t % 1000000, __func__, ##__VA_ARGS__);   \
  } while (0)
#define APP_LOG_DUMP()                                      \
  do {                                                      \
    std::printf("%s", app_log_buffer + app_log_buffer_ptr); \
    std::printf("%s", app_log_buffer);                      \
  } while (0)
#else
#define APP_LOG_BASE(l, c, f, ...)                                 \
  do {                                                             \
    const int t = esp_timer_get_time();                            \
    std::fprintf(stdout,                                           \
                 c "[" l "][%d.%06d][" __FILE__                    \
                   ":" APP_TOSTRING(__LINE__) "]\x1b[0m\t" f "\n", \
                 t / 1000000, t % 1000000, ##__VA_ARGS__);         \
  } while (0)
#define APP_LOG_DUMP()
#endif
/* app log */
#if APP_LOG_LEVEL >= 1
#define APP_LOGE(fmt, ...) APP_LOG_BASE("E", "\e[31m", fmt, ##__VA_ARGS__)
#else
#define APP_LOGE(fmt, ...)
#endif
#if APP_LOG_LEVEL >= 2
#define APP_LOGW(fmt, ...) APP_LOG_BASE("W", "\e[33m", fmt, ##__VA_ARGS__)
#else
#define APP_LOGW(fmt, ...)
#endif
#if APP_LOG_LEVEL >= 3
#define APP_LOGI(fmt, ...) APP_LOG_BASE("I", "\e[32m", fmt, ##__VA_ARGS__)
#else
#define APP_LOGI(fmt, ...)
#endif
#if APP_LOG_LEVEL >= 4
#define APP_LOGD(fmt, ...) APP_LOG_BASE("D", "\e[34m", fmt, ##__VA_ARGS__)
#else
#define APP_LOGD(fmt, ...)
#endif

#if 0
/* stream format */
#include <iostream>
#include <sstream>
#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)
#define APP_LOG_OSTREAM_COMMON(s, l, c) \
  (s << c "[" l "][" __FILE__ ":" APP_TOSTRING(__LINE__) "]\x1b[0m\t")

/* select log output (0: std::cout, 1: std::stringstream) */
#if 1
static std::stringstream logs;
#define app_logs_d APP_LOG_OSTREAM_COMMON(logs, "D", "\x1b[34m")
#define app_logs_i APP_LOG_OSTREAM_COMMON(logs, "I", "\x1b[32m")
#define app_logs_w APP_LOG_OSTREAM_COMMON(logs, "W", "\x1b[33m")
#define app_logs_e APP_LOG_OSTREAM_COMMON(logs, "E", "\x1b[31m")
#define app_log_dump()                    \
  do {                                    \
    std::cout << logs.str() << std::endl; \
  } while (0)
#define app_log_reserve(size) \
  do {                        \
    logs.str().reserve(size); \
  } while (0)
#else
#define app_logs_d APP_LOG_OSTREAM_COMMON(std::cout, "D", "\x1b[34m")
#define app_logs_i APP_LOG_OSTREAM_COMMON(std::cout, "I", "\x1b[32m")
#define app_logs_w APP_LOG_OSTREAM_COMMON(std::cout, "W", "\x1b[33m")
#define app_logs_e APP_LOG_OSTREAM_COMMON(std::cout, "E", "\x1b[31m")
#define app_log_dump()
#define app_log_reserve()
#endif

/* select log level */
#define app_log_level 4
#if app_log_level >= 1
#define app_loge(s) (app_logs_e << s << std::endl)
#else
#define app_loge(s)
#endif
#if app_log_level >= 2
#define app_logw(s) (app_logs_w << s << std::endl)
#else
#define app_logw(s)
#endif
#if app_log_level >= 3
#define app_logi(s) (app_logs_i << s << std::endl)
#else
#define app_logi(s)
#endif
#if app_log_level >= 4
#define app_logd(s) (app_logs_d << s << std::endl)
#else
#define app_logd(s)
#endif
#endif
