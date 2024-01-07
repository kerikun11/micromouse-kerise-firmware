/**
 * @file wall_detector.h
 * @brief Wall Detector
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <array>
#include <cmath>    //< std::log
#include <fstream>  //< for std::ifstream, std::ofstream
#include <iomanip>
#include <iostream>

#include "hardware/hardware.h"

class WallDetector {
 public:
  static constexpr int average_filter_size = 16;
  static constexpr int wall_threshold_front = 135;
  static constexpr int wall_threshold_side = 25;
  static constexpr auto WALL_DETECTOR_BACKUP_PATH = "/spiffs/WallDetector.txt";

  union WallValue {
    // 意味をもったメンバ
    struct {
      std::array<float, 2> side;
      std::array<float, 2> front;
    };
    // シリアライズされたメンバ
    std::array<float, 4> value = {0, 0, 0, 0};

    WallValue& operator+=(const WallValue& wv) {
      for (int i = 0; i < 4; ++i) value[i] += wv.value[i];
      return *this;
    }
    WallValue& operator-=(const WallValue& wv) {
      for (int i = 0; i < 4; ++i) value[i] -= wv.value[i];
      return *this;
    }
    WallValue operator-(const WallValue& wv) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i) ret.value[i] = value[i] - wv.value[i];
      return ret;
    }
    WallValue operator/(const float div) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i) ret.value[i] = value[i] / div;
      return ret;
    }
  };
  struct Walls {
    union {
      struct {
        bool left;
        bool right;
      };
      bool side[2];
    };
    bool front;
  };

  //  private:
 public:  //< ToDo: make private and lock with mutex
  WallValue distance;
  WallValue distance_average;
  Walls walls;

 public:
  WallDetector(hardware::Hardware* hw) : hw_(hw) {}
  bool init() {
    if (!restore()) return false;
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); },
        "WallDetector", 4096, this, TASK_PRIORITY_WALL_DETECTOR, NULL,
        TASK_CORE_ID_WALL_DETECTOR);
    return true;
  }
  bool backup(const char* filepath = WALL_DETECTOR_BACKUP_PATH) {
    std::ofstream of(filepath);
    if (of.fail()) {
      APP_LOGE("Can't open file. filepath: %s", filepath);
      return false;
    }
    for (const auto& value : wall_ref.value) of << value << std::endl;
    return true;
  }
  bool restore(const char* filepath = WALL_DETECTOR_BACKUP_PATH) {
    std::ifstream f(filepath);
    if (f.fail()) {
      APP_LOGE("Can't open file. filepath: %s", filepath);
      return false;
    }
    for (auto& value : wall_ref.value) {
      if (f.eof()) {
        APP_LOGE("invalid file size. filepath: %s", filepath);
        return false;
      }
      f >> value;
    }
    APP_LOGI("Wall Reference Restored: %10f %10f %10f %10f",
             (double)wall_ref.side[0], (double)wall_ref.front[0],
             (double)wall_ref.front[1], (double)wall_ref.side[1]);
    return true;
  }
  void calibration_side() {
    hw_->tof->disable();
    vTaskDelay(pdMS_TO_TICKS(20));
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++) sum[i] += ref2dist(hw_->rfl->side(i));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++) wall_ref.side[i] = sum[i] / ave_count;
    APP_LOGI("Wall Calibration Side: %10f %10f", (double)wall_ref.side[0],
             (double)wall_ref.side[1]);
    hw_->tof->enable();
  }
  void calibration_front() {
    hw_->tof->disable();
    vTaskDelay(pdMS_TO_TICKS(20));
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++) sum[i] += ref2dist(hw_->rfl->front(i));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++) wall_ref.front[i] = sum[i] / ave_count;
    APP_LOGI("Wall Calibration Front: %10f %10f", (double)wall_ref.front[0],
             (double)wall_ref.front[1]);
    hw_->tof->enable();
  }
  const char* get_info() {
    static char str[128];
    snprintf(str, sizeof(str),
             "Dist:[%5.1f %5.1f %5.1f %5.1f] Wall:[%c %c %c] "
             "ToF:[%3u mm %3lu ms (%3u mm)]",
             (double)distance.side[0], (double)distance.front[0],
             (double)distance.front[1], (double)distance.side[1],
             walls.side[0] ? 'X' : '_', walls.front ? 'X' : '_',
             walls.side[1] ? 'X' : '_', hw_->tof->getDistance(),
             hw_->tof->passedTimeMs(), hw_->tof->getRangeRaw());
    return str;
  }
  void print() { APP_LOGI("%s", get_info()); }
  void csv() {
    std::cout << "0";
    for (int i = 0; i < 4; ++i) std::cout << "," << distance.value[i];
    std::cout << std::endl;
  }

 private:
  hardware::Hardware* hw_;
  WallValue wall_ref;
  ctrl::Accumulator<WallValue, average_filter_size> buffer;
  // ToDo: add mutex

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
    }
  }
  void update() {
    // リフレクタ値の更新
    for (int i = 0; i < 2; i++) {
      distance.side[i] = ref2dist(hw_->rfl->side(i)) - wall_ref.side[i];
      distance.front[i] = ref2dist(hw_->rfl->front(i)) - wall_ref.front[i];
    }
    buffer.push(distance);
    distance_average = buffer.average();

    // 前壁の更新
    int front_mm = hw_->tof->getDistance();
    if (!hw_->tof->isValid())
      walls.front = false;  //< ToFの測距範囲内に壁がない場合はinvalidになる
    else if (front_mm < wall_threshold_front * 0.95f)
      walls.front = true;
    else if (front_mm > wall_threshold_front * 1.05f)
      walls.front = false;

    // 横壁の更新
    for (int i = 0; i < 2; i++) {
      const float value = distance.side[i];
      if (value < wall_threshold_side * 0.97f)
        walls.side[i] = true;
      else if (value > wall_threshold_side * 1.03f)
        walls.side[i] = false;
    }
  }
  float ref2dist(const int16_t value) const {
    return -12.9035f * std::log(float(value)) + 86.7561f;
  }
};
