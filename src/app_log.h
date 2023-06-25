/**
 * @file app_log.h
 * @brief this provides logging formats
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-12-18
 */
#pragma once

#include <esp_timer.h>  //< esp_timer_get_time
#include <stdio.h>      //< for printf

#include "config/config.h"  //< for APP_LOG_LEVEL

/* app log mode */
#ifndef APP_LOG_MEM_MODE
#define APP_LOG_MEM_MODE 0
#endif

/* app log level (0: None, 1: Error, 2: Warn, 3: Info, 4: Debug) */
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL 2
#endif

/* app log utils */
#define APP_LOG_STRINGIFY(n) #n
#define APP_LOG_TO_STRING(n) APP_LOG_STRINGIFY(n)
/* app log base */
#if APP_LOG_MEM_MODE
/* app log mem */
#define APP_LOG_BUFFER_SIZE 32768
static int app_log_buffer_ptr = 0;
static char app_log_buffer[APP_LOG_BUFFER_SIZE];
#define APP_LOG_BASE(l, c, f, ...)                                             \
  do {                                                                         \
    const int us = esp_timer_get_time();                                       \
    if (app_log_buffer_ptr > APP_LOG_BUFFER_SIZE - 64) app_log_buffer_ptr = 0; \
    app_log_buffer_ptr +=                                                      \
        snprintf(app_log_buffer + app_log_buffer_ptr,                          \
                 APP_LOG_BUFFER_SIZE - app_log_buffer_ptr,                     \
                 c "[" l "][%d.%06d][" __FILE__                                \
                   ":" APP_LOG_TO_STRING(__LINE__) "][%s]\e[0m\t" f "\n",      \
                 us / 1'000'000, us % 1'000'000, __func__, ##__VA_ARGS__);     \
  } while (0)
#define APP_LOG_DUMP()                                 \
  do {                                                 \
    printf("%s", app_log_buffer + app_log_buffer_ptr); \
    printf("%s", app_log_buffer);                      \
  } while (0)
#else
#define APP_LOG_BASE(l, c, f, ...)                               \
  do {                                                           \
    const int t = esp_timer_get_time();                          \
    fprintf(stdout,                                              \
            c "[" l "][%d.%06d][" __FILE__                       \
              ":" APP_LOG_TO_STRING(__LINE__) "]\e[0m\t" f "\n", \
            t / 1000000, t % 1000000, ##__VA_ARGS__);            \
  } while (0)
#define APP_LOG_DUMP()
#endif

/* app log definitions for use */
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

/* show warning */
#if APP_LOG_LEVEL > 3
#warning "APP_LOGD Enabled"
#endif
#if APP_LOG_MEM_MODE
#warning "APP_LOG_MEM_MODE is enabled"
#endif
