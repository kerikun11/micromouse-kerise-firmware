/**
 * @file encoder.h
 * @brief Encoder Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <drivers/as5048a/as5048a.h>
#include <drivers/ma730/ma730.h>
#include <freertospp/semphr.h>

#include <cmath>  //< for std::sin
#include <mutex>
#include <vector>

#include "app_log.h"
#include "utils/wheel_position.h"

namespace hardware {

class Encoder {
 public:
  enum class SensorType {
    INVALID,
    AS5048A,
    MA730,
  };
  struct Parameter {
    SensorType sensor_type;
    spi_host_device_t spi_host;
    std::vector<gpio_num_t> gpio_nums_spi_cs;
    float gear_ratio;
    float wheel_diameter;
    std::array<float, 2> ec_gain = {0, 0};
    std::array<float, 2> ec_phase = {0, 0};
  };

 private:
  static constexpr float PI = 3.14159265358979323846f;

 public:
  Encoder() {}
  bool init(const Parameter& param) {
    param_ = param;
    switch (param_.sensor_type) {
      case SensorType::AS5048A:
        as_ = new drivers::AS5048A_DUAL();
        if (!as_->init(param.spi_host, param.gpio_nums_spi_cs[0])) {
          APP_LOGE("AS5048A init failed :(");
          return false;
        }
        pulses_size_ = drivers::AS5048A_DUAL::PULSES_SIZE;
        break;
      case SensorType::MA730:
        for (int i = 0; i < 2; ++i) {
          ma_[i] = new drivers::MA730();
          if (!ma_[i]->init(param.spi_host, param.gpio_nums_spi_cs[i])) {
            APP_LOGE("MA730[%d] init failed :(", i);
            return false;
          }
          pulses_size_ = drivers::MA730::PULSES_SIZE;
        }
        break;
      default:
        APP_LOGE("SensorType: %d", static_cast<int>(param_.sensor_type));
        break;
    }
    const uint32_t stack_depth = 2048;
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "Encoder",
        stack_depth, this, TASK_PRIORITY_ENCODER, &handle_,
        TASK_CORE_ID_ENCODER);
    return true;
  }
  int get_pulses(uint8_t ch) {
    /* the reason the type of pulses_ is no problem with type int */
    /* estimated position 1,000 [mm/s] * 10 [min] * 60 [s/min] = 600,000 [mm] */
    /* int32_t 2,147,483,647 / 16384 * 1/3 * 3.1415 * 13 [mm] = 1,784,305 [mm]*/
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return pulses_[ch];
  }
  float get_position(uint8_t ch) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return positions_[ch];
  }
  WheelPosition get_wheel_position() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return {{positions_[0], positions_[1]}};
  }
  void clear_offset() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    pulses_ovf_[0] = pulses_ovf_[1] = 0;
  }
  void csv() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    std::printf("%d,%d\n", -pulses_[0], pulses_[1]);
  }
  void sampling_request() {
    xTaskNotifyGive(handle_);  //
  }
  void sampling_wait(TickType_t xBlockTime = portMAX_DELAY) const {
    sampling_end_semaphore_.take(xBlockTime);
  }

 private:
  TaskHandle_t handle_ = NULL;
  drivers::AS5048A_DUAL* as_;
  drivers::MA730* ma_[2];
  Parameter param_;
  int pulses_size_;
  freertospp::Semaphore sampling_end_semaphore_;

  mutable std::mutex mutex_;
  WheelPosition positions_;
  int pulses_[2] = {};
  int pulses_raw_[2] = {};
  int pulses_prev_[2] = {};
  int pulses_ovf_[2] = {};

  void task() {
    while (1) {
      /* sync */
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      /* fetch */
      update();
      /* notify */
      sampling_end_semaphore_.give();
    }
  }
  void update() {
    /* fetch data from encoder */
    switch (param_.sensor_type) {
      case SensorType::AS5048A: {
        as_->update();
        for (int i = 0; i < 2; i++) pulses_raw_[i] = as_->get(i);
      } break;
      case SensorType::MA730: {
        for (int i = 0; i < 2; i++) {
          ma_[i]->update();
          pulses_raw_[i] = ma_[i]->get();
        }
      } break;
      default:
        APP_LOGE("SensorType: %d", static_cast<int>(param_.sensor_type));
        break;
    }
    /* lock values */
    std::lock_guard<std::mutex> lock_guard(mutex_);
    /* calculate physical value */
    float mm[2];
    for (int i = 0; i < 2; i++) {
      /* count overflow */
      if (pulses_raw_[i] > pulses_prev_[i] + pulses_size_ / 2) {
        pulses_ovf_[i]--;
      } else if (pulses_raw_[i] < pulses_prev_[i] - pulses_size_ / 2) {
        pulses_ovf_[i]++;
      }
      pulses_prev_[i] = pulses_raw_[i];
      /* calculate pulses */
      pulses_[i] = pulses_ovf_[i] * pulses_size_ + pulses_raw_[i];
      /* calculate position */
      float ec_offset =
          param_.ec_gain[i] *
          std::sin(2 * PI *
                   (float(pulses_raw_[i]) / pulses_size_ + param_.ec_phase[i]));
      float SCALE_PULSES_TO_MM = param_.gear_ratio * param_.wheel_diameter * PI;
      mm[i] =
          (pulses_ovf_[i] + float(pulses_raw_[i] + ec_offset) / pulses_size_) *
          SCALE_PULSES_TO_MM;
    }
    /* fix rotation direction */
    switch (param_.sensor_type) {
      case SensorType::AS5048A: {
        positions_[0] = +mm[0];
        positions_[1] = -mm[1];
      } break;
      case SensorType::MA730: {
        positions_[0] = -mm[0];
        positions_[1] = +mm[1];
      } break;
      default:
        APP_LOGE("SensorType: %d", static_cast<int>(param_.sensor_type));
        break;
    }
  }
};

};  // namespace hardware
