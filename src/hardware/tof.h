/**
 * @file tof.h
 * @brief ToF Sensor Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accumulator.h>
#include <drivers/vl6180x/VL6180X.h>
#include <peripheral/i2c.h>

#include <cstdio>

#include "app_log.h"

namespace hardware {

class ToF {
 public:
  struct Parameter {
    i2c_port_t i2c_port;
    uint8_t max_convergence_time_ms = 49;
    float reference_range_90mm = 90;
    float reference_range_180mm = 180;
  };

 public:
  ToF() {}
  bool init(const Parameter& param) {
    param_ = param;
    vl6180x = new VL6180X(param_.i2c_port);
    vl6180x->setTimeout(20);
    vl6180x->init();
    /* [pre-cal] fixed 3.2ms */
    vl6180x->configureDefault();
    /* [readout average; 1300us + 64.5us * value] default: 48 (4.3ms) */
    // vl6180x->writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD,32);//< 3.2ms
    // vl6180x->writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD,64);//< 5.4ms
    /* [max-convergence; includes readout average time] default: 49ms */
    vl6180x->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME,
                      param_.max_convergence_time_ms);
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "ToF",
        4096, this, TASK_PRIORITY_TOF, NULL, TASK_CORE_ID_TOF);
    vTaskDelay(pdMS_TO_TICKS(40));
    if (vl6180x->last_status != 0) {
      APP_LOGE("ToF init failed :(");
      return false;
    }
    return true;
  }
  void enable() { enabled = true; }
  void disable() { enabled = false; }
  uint16_t getDistance() const { return distance; }
  uint16_t getRangeRaw() const { return range; }
  uint16_t passedTimeMs() const { return passed_ms; }
  bool isValid() const { return passed_ms < 20; }
  const auto& getLog() const { return log; }
  void print() const {
    APP_LOGI("range: %3d [mm] D: %3d [mm] Dur: %3d [ms], Passed: %4d [ms]",
             range, distance, dur, passed_ms);
  }
  void csv() const {
    std::printf("0,45,90,135,180,%d,%d\n", getDistance(), passedTimeMs());
  }

 private:
  VL6180X* vl6180x;
  Parameter param_;
  bool enabled = true;
  uint16_t distance;
  uint16_t range;
  uint16_t dur;
  int passed_ms;
  ctrl::Accumulator<int, 10> log;

  static uint32_t millis() { return esp_timer_get_time() / 1000; }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (!enabled) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        passed_ms++;
        continue;
      }
      /* sampling start */
      vl6180x->writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      vl6180x->writeReg(VL6180X::SYSRANGE__START, 0x01);
      {
        uint32_t startAt = millis();
        while ((vl6180x->readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) &
                0x04) == 0) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
          passed_ms++;
          if (millis() - startAt > 100) break;
        }
        dur = millis() - startAt;
      }
      /* get data from sensor */
      range = vl6180x->readReg(VL6180X::RESULT__RANGE_VAL);
      const auto r90 = param_.reference_range_90mm;
      const auto r180 = param_.reference_range_180mm;
      /* line equation: y-y1 = (y2-y1) / (x2-x1) * (x-x1) */
      /* 2 point: (x1, y1) = (r90, 90), (x2, y2) = (r180, 180) */
      distance = (180.0f - 90.0f) / (r180 - r90) * (range - r90) + 90;
      log.push(distance);
      if (range != 255) passed_ms = 0;
    }
  }
};

};  // namespace hardware
