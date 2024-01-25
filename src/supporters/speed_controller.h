/**
 * @file speed_controller.h
 * @brief Speed Controller
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accumulator.h>
#include <ctrl/feedback_controller.h>
#include <ctrl/polar.h>
#include <ctrl/pose.h>
#include <freertospp/semphr.h>

#include "hardware/hardware.h"
#include "utils/timer_semaphore.h"
#include "utils/wheel_position.h"

class SpeedController {
 public:
  static constexpr const int sampling_period_us = 1000;
  static constexpr const float Ts = sampling_period_us / 1e6f;
  static constexpr const int kAccumulateSize = 4;

 public:  // ToDo: make private
  /* 読み取り専用 */
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  ctrl::Polar enc_v;
  ctrl::Pose est_p;
  ctrl::Accumulator<WheelPosition, kAccumulateSize> wheel_position;
  ctrl::Accumulator<ctrl::Polar, kAccumulateSize> accel;

 public:
  SpeedController(hardware::Hardware* hw)
      : hw_(hw), fbc_(model::SpeedControllerModel, model::SpeedControllerGain) {
    reset();
  }
  bool init() {
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); },
        "SpeedCtrl", 4096, this, TASK_PRIORITY_SPEED_CONTROLLER, NULL,
        TASK_CORE_ID_SPEED_CONTROLLER);
    return true;
  }
  void reset() {
    {
      std::lock_guard<std::mutex> lock_guard(mutex_);
      ref_v.clear();
      ref_a.clear();
      est_v.clear();
      est_a.clear();
      est_p.clear();
      enc_v.clear();
      wheel_position.clear(hw_->enc->get_wheel_position());
      accel.clear({hw_->imu->get_accel(), hw_->imu->get_angular_accel()});
      fbc_.reset();
    }
    // vTaskDelay(pdMS_TO_TICKS(50));  //< 緊急ループ防止の delay
  }
  void enable() {
    reset();
    drive_enabled_ = true;
  }
  void disable() {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    drive_enabled_ = false;
    hw_->mt->free();
    hw_->fan->free();
  }
  void set_target(float v_tra, float v_rot, float a_tra = 0, float a_rot = 0) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    ref_v.tra = v_tra, ref_v.rot = v_rot, ref_a.tra = a_tra, ref_a.rot = a_rot;
    drive();
  }
  void update_pose(const ctrl::Pose& new_pose) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    est_p = new_pose;
  }
  void fix_pose(ctrl::Pose fix, bool force = true) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    if (!force) {
      const float max_fix = 1.0f;  //< 補正量の飽和 [mm]
      fix.x = std::max(std::min(fix.x, max_fix), -max_fix);
      fix.y = std::max(std::min(fix.y, max_fix), -max_fix);
    }
    est_p += fix;
  }
  void sampling_wait() const {  //
    data_ready_semaphore_.take();
  }
  const ctrl::FeedbackController<ctrl::Polar>& getFeedbackController() const {
    return fbc_;
  }

 private:
  hardware::Hardware* hw_;
  ctrl::FeedbackController<ctrl::Polar> fbc_;
  bool drive_enabled_ = false;
  freertospp::Semaphore data_ready_semaphore_;
  TimerSemaphore sampling_semaphore_;
  mutable std::mutex mutex_;

  void task() {
    sampling_semaphore_.start_periodic(sampling_period_us);
    while (1) {
      /* wait for sampling trigger */
      sampling_semaphore_.take();
      /* sampling start */
      hw_->sampling_request();
      /* wait for sampling finished */
      hw_->sampling_wait();
      /* lock data */
      std::lock_guard<std::mutex> lock_guard(mutex_);
      /* update data */
      update_estimator();
      update_odometry();
      /* PID control */
      drive();
      /* notify */
      data_ready_semaphore_.give();
    }
  }
  void update_estimator() {
    /* add new samples */
    wheel_position.push(hw_->enc->get_wheel_position());
    accel.push({hw_->imu->get_accel(), hw_->imu->get_angular_accel()});
    /* calculate differential of encoder value */
    WheelPosition wp = (wheel_position[0] - wheel_position[1]) / Ts;
    enc_v = wp.toPolar(model::RotationRadius);
    /* calculate estimated velocity value with complementary filter */
    const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, hw_->imu->get_gyro());
    const ctrl::Polar v_high = est_v + accel[0] * float(Ts);
    const ctrl::Polar alpha = model::velocity_filter_alpha;
    est_v = alpha * v_low + (ctrl::Polar(1, 1) - alpha) * v_high;
    /* estimated acceleration */
    est_a = accel[0];
  }
  void update_odometry() {
    /* estimates slip angle */
    // const float k = 0.01f;
    const float k = 0.0f;
    const float slip_angle = k * ref_v.tra * ref_v.rot / 1000;
    /* calculate odometry value */
    est_p.th += hw_->imu->get_gyro() * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
  }
  void drive() {
    /* calculate pwm value */
    const auto pwm_value = fbc_.update(ref_v, est_v, ref_a, est_a, Ts);
    /* drive the motors */
    if (drive_enabled_) {
      const float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
      const float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
      hw_->mt->drive(pwm_value_L, pwm_value_R);
    }
  }
};
