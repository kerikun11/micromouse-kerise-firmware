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
#include <mutex>

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
    vl6180x_ = new VL6180X(param_.i2c_port);
    vl6180x_->setTimeout(20);
    vl6180x_->init();
    /* [pre-cal] fixed 3.2ms */
    vl6180x_->configureDefault();
    /* [readout average; 1300us + 64.5us * value] default: 48 (4.3ms) */
    // vl6180x_->writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD,32);//< 3.2ms
    // vl6180x_->writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD,64);//< 5.4ms
    /* [max-convergence; includes readout average time] default: 49ms */
    vl6180x_->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME,
                       param_.max_convergence_time_ms);
    const int stack_depth = 4096;
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "ToF",
        stack_depth, this, TASK_PRIORITY_TOF, &handle_, TASK_CORE_ID_TOF);
    vTaskDelay(pdMS_TO_TICKS(40));
    if (vl6180x_->last_status != 0) {
      APP_LOGE("ToF init failed :(");
      return false;
    }
    return true;
  }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; }
  uint16_t getDistance() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return distance_;
  }
  uint16_t getRangeRaw() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return range_;
  }
  uint32_t passedTimeMs() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return passed_ms_;
  }
  bool isValid() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return passed_ms_ < 30;
  }
  // const auto& getLog() const { return log_; }
  void print() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    APP_LOGI("range_: %3d [mm] D: %3d [mm] Dur: %3d [ms], Passed: %4lu [ms]",
             range_, distance_, dur_ms_, passed_ms_);
  }
  void csv() {
    std::printf("0,45,90,135,180,%d,%lu\n", getDistance(), passedTimeMs());
  }

 private:
  TaskHandle_t handle_ = NULL;
  VL6180X* vl6180x_;
  Parameter param_;
  bool enabled_ = true;

  std::mutex mutex_;
  uint16_t distance_;
  uint16_t range_;
  uint16_t dur_ms_;
  uint32_t passed_ms_;
  // ctrl::Accumulator<int, 10> log_;

  static uint32_t millis() { return esp_timer_get_time() / 1000; }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      if (!enabled_) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        passed_ms_++;
        continue;
      }
      /* sampling start */
      vl6180x_->writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
      vl6180x_->writeReg(VL6180X::SYSRANGE__START, 0x01);
      {
        uint32_t startAt = millis();
        while ((vl6180x_->readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO) &
                0x04) == 0) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
          passed_ms_++;
          if (millis() - startAt > 100) break;
        }
        dur_ms_ = millis() - startAt;
      }
      /* get range from sensor */
      uint16_t range_raw = vl6180x_->readReg(VL6180X::RESULT__RANGE_VAL);
      /* update data */
      std::lock_guard<std::mutex> lock_guard(mutex_);
      range_ = range_raw;
      /* calc distance to wall */
      /* equation of line: y-y1 = (y2-y1) / (x2-x1) * (x-x1) */
      /* 2 point: (x1, y1) = (r90, 90), (x2, y2) = (r180, 180) */
      const auto r90 = param_.reference_range_90mm;
      const auto r180 = param_.reference_range_180mm;
      distance_ = (180.0f - 90.0f) / (r180 - r90) * (range_ - r90) + 90;
      // log_.push(distance_);
      if (range_ != 255) passed_ms_ = 0;
    }
  }
};

};  // namespace hardware
