/**
 * @file wall_detector.h
 * @brief Wall Detector
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "hardware/hardware.h"

#include <cmath>    //< std::log
#include <fstream>  //< for std::ifstream, std::ofstream
#include <iomanip>
#include <iostream>

class WallDetector {
 public:
  static constexpr float Ts = 1e-3f;
  static constexpr int wall_threshold_front = 135;
  static constexpr int wall_threshold_side = 25;
  static constexpr auto WALL_DETECTOR_BACKUP_PATH = "/spiffs/WallDetector.bin";

  union WallValue {
    // 意味をもったメンバ
    struct {
      float side[2];
      float front[2];
    };
    // シリアライズされたメンバ
    float value[4] = {0, 0, 0, 0};

    WallValue& operator+=(const WallValue& wv) {
      for (int i = 0; i < 4; ++i)
        value[i] += wv.value[i];
      return *this;
    }
    WallValue& operator-=(const WallValue& wv) {
      for (int i = 0; i < 4; ++i)
        value[i] -= wv.value[i];
      return *this;
    }
    WallValue operator-(const WallValue& wv) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] - wv.value[i];
      return ret;
    }
    WallValue operator/(const float div) const {
      WallValue ret;
      for (int i = 0; i < 4; ++i)
        ret.value[i] = value[i] / div;
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

 public:
  WallValue distance;
  WallValue distance_average;
  Walls walls;

 private:
  hardware::Hardware* hw;

 public:
  WallDetector(hardware::Hardware* hw) : hw(hw) {}
  bool init() {
    if (!restore())
      return false;
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); },
        "WallDetector", 4096, this, 3, NULL, PRO_CPU_NUM);
    return true;
  }
  bool backup(const char* filepath = WALL_DETECTOR_BACKUP_PATH) {
    std::ofstream of(filepath, std::ios::binary);
    if (of.fail()) {
      APP_LOGE("Can't open file. filepath: %s", filepath);
      return false;
    }
    of.write((const char*)&wall_ref, sizeof(WallDetector::WallValue));
    return true;
  }
  bool restore(const char* filepath = WALL_DETECTOR_BACKUP_PATH) {
    std::ifstream f(filepath, std::ios::binary);
    if (f.fail()) {
      APP_LOGE("Can't open file. filepath: %s", filepath);
      return false;
    }
    if (f.eof()) {
      APP_LOGE("invalid file size. filepath: %s", filepath);
      return false;
    }
    f.read((char*)&wall_ref, sizeof(WallDetector::WallValue));
    APP_LOGI("Wall Reference Restored: %10f %10f %10f %10f",
             (double)wall_ref.side[0], (double)wall_ref.front[0],
             (double)wall_ref.front[1], (double)wall_ref.side[1]);
    return true;
  }
  void calibration_side() {
    hw->tof->disable();
    vTaskDelay(pdMS_TO_TICKS(20));
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(hw->rfl->side(i));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.side[i] = sum[i] / ave_count;
    APP_LOGI("Wall Calibration Side: %10f %10f", (double)wall_ref.side[0],
             (double)wall_ref.side[1]);
    hw->tof->enable();
  }
  void calibration_front() {
    hw->tof->disable();
    vTaskDelay(pdMS_TO_TICKS(20));
    float sum[2] = {0.0f, 0.0f};
    const int ave_count = 500;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (int j = 0; j < ave_count; j++) {
      for (int i = 0; i < 2; i++)
        sum[i] += ref2dist(hw->rfl->front(i));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 2; i++)
      wall_ref.front[i] = sum[i] / ave_count;
    APP_LOGI("Wall Calibration Front: %10f %10f", (double)wall_ref.front[0],
             (double)wall_ref.front[1]);
    hw->tof->enable();
  }
  const char* get_info() {
    static char str[128];
    snprintf(str, sizeof(str),
             "Dist:[%5.1f %5.1f %5.1f %5.1f] Wall:[%c %c %c] "
             "ToF:[%3d mm %3d ms (%3d mm)]",
             (double)distance.side[0], (double)distance.front[0],
             (double)distance.front[1], (double)distance.side[1],
             walls.side[0] ? 'X' : '_', walls.front ? 'X' : '_',
             walls.side[1] ? 'X' : '_', hw->tof->getDistance(),
             hw->tof->passedTimeMs(), hw->tof->getRangeRaw());
    return str;
  }
  void print() { APP_LOGI("%s", get_info()); }
  void csv() {
    std::cout << "0";
    for (int i = 0; i < 4; ++i)
      std::cout << "," << distance.value[i];
    std::cout << std::endl;
  }

 private:
  WallValue wall_ref;
  static const int ave_num = 16;
  ctrl::Accumulator<WallValue, ave_num> buffer;

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
      distance.side[i] = ref2dist(hw->rfl->side(i)) - wall_ref.side[i];
      distance.front[i] = ref2dist(hw->rfl->front(i)) - wall_ref.front[i];
    }
    buffer.push(distance);
    distance_average = buffer.average();

    // 前壁の更新
    int front_mm = hw->tof->getDistance();
    if (!hw->tof->isValid())
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
