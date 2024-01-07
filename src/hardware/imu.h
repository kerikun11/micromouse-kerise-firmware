/**
 * @file imu.h
 * @brief IMU Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <drivers/icm20602/icm20602.h>
#include <freertospp/semphr.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "app_log.h"

namespace hardware {

class IMU {
 public:
  IMU() {}
  bool init(spi_host_device_t spi_host, std::vector<gpio_num_t> gpio_nums_cs,
            float rotation_radius = 0) {
    rotation_radius_ = rotation_radius;
    icm_.resize(gpio_nums_cs.size());
    raw_accel_.resize(gpio_nums_cs.size());
    raw_gyro_.resize(gpio_nums_cs.size());
    APP_LOGI("spi_host: %d size: %d rotation_radius: %f", spi_host,
             gpio_nums_cs.size(), (double)rotation_radius);
    /* check param */
    if (gpio_nums_cs.size() == 2 && rotation_radius_ <= 0) {
      APP_LOGE("IMU param error. rotation_radius: %f",
               (double)rotation_radius_);
      return false;
    }
    /* init icm */
    int ret = 0;
    for (int i = 0; i < icm_.size(); ++i) {
      if (icm_[i].init(spi_host, gpio_nums_cs[i])) {
        APP_LOGE("IMU[%d] init failed :(", i);
        ret += 1 << i;
      }
    }
    if (ret) {
      return false;
    }
    /* create task */
    const uint32_t stack_depth = 4096;
    BaseType_t result = xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "IMU",
        stack_depth, this, TASK_PRIORITY_IMU, &handle_, TASK_CORE_ID_IMU);
    if (result != pdPASS) {
      APP_LOGE("xTaskCreatePinnedToCore failed. result: %d", result);
      return false;
    }
    return true;
  }
  void calibration() {
    calibration_req_ = true;
    /* wait for calibration finished */
    std::unique_lock<std::mutex> unique_lock(calibration_mutex_);
    calibration_cv_.wait(unique_lock, [&] { return !calibration_req_; });
  }
  void sampling_request() {
    xTaskNotifyGive(handle_);  //
  }
  void sampling_wait(TickType_t xBlockTime = portMAX_DELAY) const {
    sampling_end_semaphore_.take(xBlockTime);
  }
  void print() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    APP_LOGI("gy %10f %10f %10f ac: %10f %10f %10f aa: %10f",       //
             (double)gyro_.x, (double)gyro_.y, (double)gyro_.z,     //
             (double)accel_.x, (double)accel_.y, (double)accel_.z,  //
             (double)angular_accel_);
  }
  void csv() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    std::cout << "0";
    std::cout << '\t' << gyro_.x;
    std::cout << '\t' << gyro_.y;
    std::cout << '\t' << gyro_.z;
#if 1
    for (int i = 0; i < icm_.size(); ++i) {
      std::cout << '\t' << raw_gyro_[i].x;
      std::cout << '\t' << raw_gyro_[i].y;
      std::cout << '\t' << raw_gyro_[i].z;
    }
#endif
    std::cout << std::endl;
  }
  float get_accel() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return accel_.y;
  }
  float get_gyro() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return gyro_.z;
  }
  float get_angular_accel() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return angular_accel_;
  }
  const MotionParameter get_gyro3() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return gyro_;
  }
  const MotionParameter get_accel3() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return accel_;
  }

 protected:
  TaskHandle_t handle_ = NULL;
  std::vector<drivers::ICM20602> icm_;
  std::vector<MotionParameter> raw_gyro_, raw_accel_;
  float rotation_radius_ = 0;

  std::mutex mutex_;
  MotionParameter gyro_, accel_;
  float angular_accel_ = 0;
  uint64_t last_timestamp_us_ = 0;

  freertospp::Semaphore sampling_end_semaphore_;
  MotionParameter gyro_offset_, accel_offset_;

  bool calibration_req_ = false;
  std::mutex calibration_mutex_;
  std::condition_variable calibration_cv_;

  void task() {
    while (1) {
      /* sync */
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      /* fetch */
      update();
      /* notify */
      sampling_end_semaphore_.give();
      /* calibration */
      if (calibration_req_) {
        task_calibration();
        std::lock_guard<std::mutex> lock_guard(calibration_mutex_);
        calibration_req_ = false;
        calibration_cv_.notify_all();
      }
    }
  }
  void task_calibration() {
    const int ave_count = 200;
    accel_offset_ = MotionParameter();
    gyro_offset_ = MotionParameter();
    for (int j = 0; j < 2; j++) {
      MotionParameter accel_sum, gyro_sum;
      for (int i = 0; i < ave_count; i++) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        update();
        sampling_end_semaphore_.give();
        accel_sum += accel_;
        gyro_sum += gyro_;
      }
      accel_offset_ += accel_sum / ave_count;
      gyro_offset_ += gyro_sum / ave_count;
    }
  }
  void update() {
    /* sampling */
    for (size_t i = 0; i < icm_.size(); i++) icm_[i].update();

    /* lock local variables */
    std::lock_guard<std::mutex> lock_guard(mutex_);

    /* read sensor data */
    for (size_t i = 0; i < icm_.size(); i++) {
      raw_accel_[i] = icm_[i].accel();
      raw_gyro_[i] = icm_[i].gyro();
    }

    if (icm_.size() == 1) {
      const auto prev_gyro_z = gyro_.z;
      gyro_ = raw_gyro_[0] - gyro_offset_;
      accel_ = raw_accel_[0] - accel_offset_;
      /* calculate angular accel */
      uint64_t us = esp_timer_get_time();
      if (last_timestamp_us_ != 0) {
        float Ts = (us - last_timestamp_us_) / 1e6f;
        angular_accel_ = (gyro_.z - prev_gyro_z) / Ts;
      }
      last_timestamp_us_ = us;
    } else if (icm_.size() == 2) {
      /* fix sensor orientation */
      raw_gyro_[0].x = -raw_gyro_[0].x;
      raw_gyro_[0].y = -raw_gyro_[0].y;
      raw_accel_[0].x = -raw_accel_[0].x;
      raw_accel_[0].y = -raw_accel_[0].y;
      /* average multiple sensor value */
      gyro_ = (raw_gyro_[0] + raw_gyro_[1]) / 2 - gyro_offset_;
      accel_ = (raw_accel_[0] + raw_accel_[1]) / 2 - accel_offset_;
      /* calculate angular accel */
      angular_accel_ =
          (raw_accel_[0].y + raw_accel_[1].y) / 2 / rotation_radius_;
    } else {
      APP_LOGE("IMU size error. icm_.size(): %d", icm_.size());
    }
  }
};

};  // namespace hardware
