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

#include <cmath>
#include <mutex>
#include <vector>

#include "app_log.h"

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
  };

 private:
  static constexpr float PI = 3.14159265358979323846f;

 public:
  Encoder() {}
  bool init(const Parameter& param) {
    param_ = param;
    switch (param_.sensor_type) {
      case SensorType::AS5048A:
        as = new drivers::AS5048A_DUAL();
        if (!as->init(param.spi_host, param.gpio_nums_spi_cs[0])) {
          APP_LOGE("AS5048A init failed :(");
          return false;
        }
        pulses_size_ = drivers::AS5048A_DUAL::PULSES_SIZE;
        break;
      case SensorType::MA730:
        for (int i = 0; i < 2; ++i) {
          ma[i] = new drivers::MA730();
          if (!ma[i]->init(param.spi_host, param.gpio_nums_spi_cs[i])) {
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
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "Encoder",
        2048, this, TASK_PRIORITY_ENCODER, NULL, TASK_CORE_ID_ENCODER);
    return true;
  }
  int get_raw(uint8_t ch) {
    /* the reason the type of pulses is no problem with type int */
    /* estimated position 1,000 [mm/s] * 10 [min] * 60 [s/min] = 600,000 [mm] */
    /* int32_t 2,147,483,647 / 16384 * 1/3 * 3.1415 * 13 [mm] = 1,784,305 [mm]*/
    std::lock_guard<std::mutex> lock_guard(mutex);
    return pulses_raw[ch];
  }
  float get_position(uint8_t ch) {
    std::lock_guard<std::mutex> lock_guard(mutex);
    return positions[ch];
  }
  void clear_offset() {
    std::lock_guard<std::mutex> lock_guard(mutex);
    pulses_ovf[0] = pulses_ovf[1] = 0;
  }
  void csv() {
    std::printf("%d,%d\n", -pulses[0], pulses[1]);  //
  }
  void sampling_sync(TickType_t xBlockTime = portMAX_DELAY) const {
    sampling_end_semaphore.take(xBlockTime);
  }

 private:
  drivers::AS5048A_DUAL* as;
  drivers::MA730* ma[2];
  int pulses_size_;
  int pulses[2] = {};
  int pulses_raw[2] = {};
  int pulses_prev[2] = {};
  int pulses_ovf[2] = {};
  float positions[2] = {};
  freertospp::Semaphore sampling_end_semaphore;
  std::mutex mutex;
  Parameter param_;
  const float ec_gain[2] = {
      9.5e-3f,
      8.0e-3f,
  };
  const float ec_phase[2] = {
      0.5f - 0.1e-1f,
      0.5f - 9.9e-1f,
  };
  const float ec_offset[2] = {
      -9.7732187836986f,
      8.23007897574619f,
  };

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      sampling_end_semaphore.give();
    }
  }

  void update() {
    /* fetch data from encoder */
    switch (param_.sensor_type) {
      case SensorType::AS5048A: {
        as->update();
        for (int i = 0; i < 2; i++) pulses_raw[i] = as->get(i);
      } break;
      case SensorType::MA730: {
        for (int i = 0; i < 2; i++) {
          ma[i]->update();
          pulses_raw[i] = ma[i]->get();
          /* compensate eccentricity */
          pulses_raw[i] += ec_gain[i] * pulses_size_ *
                               std::sin(2 * PI *
                                        (float(pulses_raw[i]) / pulses_size_ +
                                         ec_phase[i])) +
                           ec_offset[i];
        }
      } break;
      default:
        APP_LOGE("SensorType: %d", static_cast<int>(param_.sensor_type));
        break;
    }
    /* lock values */
    std::lock_guard<std::mutex> lock_guard(mutex);
    /* calculate physical value */
    float mm[2];
    for (int i = 0; i < 2; i++) {
      /* count overflow */
      if (pulses_raw[i] > pulses_prev[i] + pulses_size_ / 2) {
        pulses_ovf[i]--;
      } else if (pulses_raw[i] < pulses_prev[i] - pulses_size_ / 2) {
        pulses_ovf[i]++;
      }
      pulses_prev[i] = pulses_raw[i];
      /* calculate position */
      float SCALE_PULSES_TO_MM = param_.gear_ratio * param_.wheel_diameter * PI;
      pulses[i] = pulses_ovf[i] * pulses_size_ + pulses_raw[i];
      mm[i] = (pulses_ovf[i] + float(pulses_raw[i]) / pulses_size_) *
              SCALE_PULSES_TO_MM;
    }
    /* fix rotation direction */
    switch (param_.sensor_type) {
      case SensorType::AS5048A: {
        positions[0] = +mm[0];
        positions[1] = -mm[1];
      } break;
      case SensorType::MA730: {
        positions[0] = -mm[0];
        positions[1] = +mm[1];
      } break;
      default:
        APP_LOGE("SensorType: %d", static_cast<int>(param_.sensor_type));
        break;
    }
  }
};

};  // namespace hardware
