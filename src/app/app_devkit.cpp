#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hardware/hardware.h>
#include <peripheral/esp.h>
#include <peripheral/spiffs.h>
#include <string.h>

#include <sstream>

#include "app_log.h"

void spiffs_test() {
  peripheral::SPIFFS::init();
  peripheral::SPIFFS::show_info();
}

void buzzer_test() {
  auto* bz = new hardware::Buzzer();
  bz->init(BUZZER_PIN, BUZZER_LEDC_TIMER, BUZZER_LEDC_CHANNEL);
  bz->play(hardware::Buzzer::BOOT);
}

void reflector_test() {
  if (!peripheral::ADC::init()) {
    APP_LOGE("ADC init failed:(");
    return;
  }
  auto* rfl = new hardware::Reflector();
  if (!rfl->init(REFLECTOR_TX_PINS, REFLECTOR_RX_CHANNELS)) {
    APP_LOGE("reflector init failed:(");
    return;
  }
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    // rfl->print();
    rfl->csv();
  }
}

void app_devkit() {
  APP_LOGI("This is ESP32 DevKit");
  APP_LOGI("CPU Freq: %d MHz", peripheral::ESP::get_cpu_freq_in_mhz());

  // spiffs_test();
  reflector_test();

  APP_LOG_DUMP();
  vTaskDelay(portMAX_DELAY);
}
