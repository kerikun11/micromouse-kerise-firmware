/**
 * @file app_log.h
 * @brief this provides logging formats
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-12-18
 */
#pragma once

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
#else
#define app_logs_d APP_LOG_OSTREAM_COMMON(std::cout, "D", "\x1b[34m")
#define app_logs_i APP_LOG_OSTREAM_COMMON(std::cout, "I", "\x1b[32m")
#define app_logs_w APP_LOG_OSTREAM_COMMON(std::cout, "W", "\x1b[33m")
#define app_logs_e APP_LOG_OSTREAM_COMMON(std::cout, "E", "\x1b[31m")
#define app_log_dump()
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

/* printf format */
#include <cstdio>
#define __STRINGIFY__(n) #n
#define __TO_STR__(n) __STRINGIFY__(n)
#define LOG_COMMON(l, c, f, ...)                                             \
  std::fprintf(stdout,                                                       \
               c "[" l "][" __FILE__ ":" __TO_STR__(__LINE__) "]\x1b[0m\t" f \
                                                              "\n",          \
               ##__VA_ARGS__)
#if 1
#define APP_LOGD(fmt, ...) LOG_COMMON("D", "\x1b[34m", fmt, ##__VA_ARGS__)
#define APP_LOGI(fmt, ...) LOG_COMMON("I", "\x1b[32m", fmt, ##__VA_ARGS__)
#define APP_LOGW(fmt, ...) LOG_COMMON("W", "\x1b[33m", fmt, ##__VA_ARGS__)
#define APP_LOGE(fmt, ...) LOG_COMMON("E", "\x1b[31m", fmt, ##__VA_ARGS__)
#else
#define APP_LOGD(fmt, ...)
#define APP_LOGI(fmt, ...)
#define APP_LOGW(fmt, ...)
#define APP_LOGE(fmt, ...)
#endif
