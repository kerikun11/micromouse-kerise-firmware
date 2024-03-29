/**
 * @file mutex.h
 * @brief C++ Wrapper for FreeRTOS in ESP32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-07-09
 */
#pragma once

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace freertospp {

/**
 * @brief C++ Wrapper for Mutex function
 */
class Mutex {
 public:
  Mutex() {
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
      ESP_LOGE(TAG, "xSemaphoreCreateMutex() failed");
    }
  }
  ~Mutex() { vSemaphoreDelete(xSemaphore); }
  bool giveFromISR() {
    return pdTRUE == xSemaphoreGiveFromISR(xSemaphore, NULL);
  }
  bool give() { return pdTRUE == xSemaphoreGive(xSemaphore); }
  bool take(TickType_t xBlockTime = portMAX_DELAY) {
    return pdTRUE == xSemaphoreTake(xSemaphore, xBlockTime);
  }

 private:
  static constexpr const char* TAG = "Mutex";
  SemaphoreHandle_t xSemaphore = NULL;
};

}  // namespace freertospp
