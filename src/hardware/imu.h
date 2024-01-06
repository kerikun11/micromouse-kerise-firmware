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

#include <array>
#include <condition_variable>
#include <mutex>

#include "app_log.h"

namespace hardware {

class IMU {
 public:
  static constexpr float Ts = 1e-3f;
  static constexpr int NUM_IMU_MAX = 2;

 public:
  IMU() {}
  int init(spi_host_device_t spi_host, std::vector<gpio_num_t> pins_cs,
           float rotation_radius = 1) {
    int ret = 0;
    num_imu_ = pins_cs.size();
    rotation_radius_ = rotation_radius;
    for (int i = 0; i < num_imu_; ++i) {
      if (icm_[i].init(spi_host, pins_cs[i])) {
        APP_LOGE("IMU[%d] init failed :(", i);
        ret += 1 << i;
      }
    }
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "IMU",
        4096, this, TASK_PRIORITY_IMU, NULL, TASK_CORE_ID_IMU);
    sampling_sync();  //< wait for first sample
    return ret;       //< 0: OK, otherwise: NG
  }
  int deinit() {
    deinit_req_ = true;
    /* wait for task deleted */
    std::unique_lock<std::mutex> unique_lock(deinit_mutex_);
    deinit_cv_.wait(unique_lock, [&] { return !deinit_req_; });
    for (int i = 0; i < num_imu_; ++i) icm_[i].deinit();
    return 0;
  }
  void calibration() {
    calibration_req_ = true;
    /* wait for calibration finished */
    std::unique_lock<std::mutex> unique_lock(calibration_mutex_);
    calibration_cv_.wait(unique_lock, [&] { return !calibration_req_; });
  }
  void sampling_sync(TickType_t xBlockTime = portMAX_DELAY) const {
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
    for (int i = 0; i < num_imu_; ++i) {
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
  std::array<drivers::ICM20602, NUM_IMU_MAX> icm_;
  std::array<MotionParameter, NUM_IMU_MAX> raw_gyro_, raw_accel_;
  int num_imu_ = 1;
  float rotation_radius_ = 1;

  std::mutex mutex_;
  MotionParameter gyro_, accel_;
  float angular_accel_ = 0;

  freertospp::Semaphore sampling_end_semaphore_;
  MotionParameter gyro_offset_, accel_offset_;

  bool deinit_req_ = false;
  std::mutex deinit_mutex_;
  std::condition_variable deinit_cv_;

  bool calibration_req_ = false;
  std::mutex calibration_mutex_;
  std::condition_variable calibration_cv_;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (!deinit_req_) {
      /* sync */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      /* fetch */
      update();
      /* notify */
      sampling_end_semaphore_.give();
      /* calibration */
      if (calibration_req_) {
        task_calibration(xLastWakeTime);
        std::lock_guard<std::mutex> lock_guard(calibration_mutex_);
        calibration_req_ = false;
        calibration_cv_.notify_all();
      }
    }
    /* notify */
    std::lock_guard<std::mutex> lock_guard(deinit_mutex_);
    deinit_req_ = false;
    deinit_cv_.notify_all();
    vTaskDelete(NULL);
  }
  void task_calibration(TickType_t& xLastWakeTime) {
    const int ave_count = 200;
    accel_offset_ = MotionParameter();
    gyro_offset_ = MotionParameter();
    for (int j = 0; j < 2; j++) {
      MotionParameter accel_sum, gyro_sum;
      for (int i = 0; i < ave_count; i++) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
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
    for (size_t i = 0; i < num_imu_; i++) icm_[i].update();

    /* lock local variables */
    std::lock_guard<std::mutex> lock_guard(mutex_);

    /* read sensor data */
    for (size_t i = 0; i < num_imu_; i++) {
      raw_accel_[i] = icm_[i].accel();
      raw_gyro_[i] = icm_[i].gyro();
    }

    if (num_imu_ == 1) {
      const auto prev_gyro_z = gyro_.z;
      gyro_ = raw_gyro_[0] - gyro_offset_;
      accel_ = raw_accel_[0] - accel_offset_;
      /* calculate angular accel */
      angular_accel_ = (gyro_.z - prev_gyro_z) / Ts;
    } else if (num_imu_ == 2) {
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
      APP_LOGE("IMU size error. num_imu_: %d", num_imu_);
    }
  }
};

};  // namespace hardware
