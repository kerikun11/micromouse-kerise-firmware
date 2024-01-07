/**
 * @file timer_semaphore.h
 * @brief Timer Semaphore for ESP32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <esp_timer.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class TimerSemaphore {
 public:
  TimerSemaphore() { semaphore_handle_ = xSemaphoreCreateBinary(); }
  ~TimerSemaphore() {
    end();
    vSemaphoreDelete(semaphore_handle_);
  }
  void start_periodic(uint32_t microseconds) {
    end();
    attach(microseconds, true, callback, this);
  }
  void start_oneshot(uint32_t microseconds) {
    end();
    attach(microseconds, false, callback, this);
  }
  void end() { detach(); }
  portBASE_TYPE take(TickType_t xBlockTime = portMAX_DELAY) {
    return xSemaphoreTake(semaphore_handle_, xBlockTime);
  }

 private:
  SemaphoreHandle_t semaphore_handle_ = nullptr;
  esp_timer_handle_t esp_timer_handle_ = nullptr;

  static void IRAM_ATTR callback(void* arg) {
    static_cast<TimerSemaphore*>(arg)->giveFromISR();
  }
  portBASE_TYPE giveFromISR() {
    return xSemaphoreGiveFromISR(semaphore_handle_, NULL);
  }
  void attach(uint32_t microseconds, bool repeat, esp_timer_cb_t callback,
              void* arg) {
    detach();
    esp_timer_create_args_t esp_timer_create_args = {};
    esp_timer_create_args.arg = arg;
    esp_timer_create_args.callback = callback;
#if CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
    esp_timer_create_args.dispatch_method = ESP_TIMER_ISR;
#else
    esp_timer_create_args.dispatch_method = ESP_TIMER_TASK;
#endif
    esp_timer_create_args.name = "TimerSemaphore";
    esp_timer_create_args.skip_unhandled_events = false;
    ESP_ERROR_CHECK(
        esp_timer_create(&esp_timer_create_args, &esp_timer_handle_));
    if (repeat) {
      ESP_ERROR_CHECK(
          esp_timer_start_periodic(esp_timer_handle_, microseconds));
    } else {
      ESP_ERROR_CHECK(esp_timer_start_once(esp_timer_handle_, microseconds));
    }
  }
  void detach() {
    if (esp_timer_handle_) {
      esp_timer_stop(esp_timer_handle_);
      esp_timer_delete(esp_timer_handle_);
      esp_timer_handle_ = nullptr;
    }
  }
};
