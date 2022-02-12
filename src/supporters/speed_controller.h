/**
 * @file speed_controller.h
 * @brief Speed Controller
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "config/wheel_parameter.h"

#include "hardware/hardware.h"
#include <ctrl/accumulator.h>
#include <ctrl/feedback_controller.h>
#include <ctrl/polar.h>
#include <ctrl/pose.h>
#include <freertospp/semphr.h>

class SpeedController {
public:
  static constexpr const float Ts = 1e-3f;

public:
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  ctrl::Pose est_p;
  WheelParameter enc_v;
  static constexpr int acc_num = 4;
  ctrl::Accumulator<float, acc_num> wheel_position[2];
  ctrl::Accumulator<ctrl::Polar, acc_num> accel;

public:
  ctrl::FeedbackController<ctrl::Polar> fbc;

private:
  hardware::Hardware *hw;

public:
  SpeedController(const ctrl::FeedbackController<ctrl::Polar>::Model &M,
                  const ctrl::FeedbackController<ctrl::Polar>::Gain &G,
                  hardware::Hardware *hw)
      : fbc(M, G), hw(hw) {
    reset();
  }
  bool init() {
    xTaskCreatePinnedToCore(
        [](void *arg) { static_cast<decltype(this)>(arg)->task(); },
        "SpeedCtrl", 4096, this, 5, NULL, PRO_CPU_NUM);
    return true;
  }
  void enable() {
    reset();
    drive_enabled = true;
  }
  void disable() {
    drive_enabled = false;
    sampling_sync();
    sampling_sync();
    hw->mt->free();
    hw->fan->drive(0);
  }
  void set_target(float v_tra, float v_rot, float a_tra = 0, float a_rot = 0) {
    ref_v.tra = v_tra, ref_v.rot = v_rot, ref_a.tra = a_tra, ref_a.rot = a_rot;
  }
  void fix_pose(ctrl::Pose fix, bool saturate = true) {
    if (saturate) {
      const float max_fix = 1.0f; //< 補正量の飽和 [mm]
      fix.x = std::max(std::min(fix.x, max_fix), -max_fix);
      fix.y = std::max(std::min(fix.y, max_fix), -max_fix);
    }
    est_p += fix; // ToDo: make thread safe
  }
  void sampling_sync() const { //
    data_ready_semaphore.take();
  }

private:
  volatile bool drive_enabled = false;
  freertospp::Semaphore data_ready_semaphore;

  void task() {
    while (1) {
      /* sampling sync */
      hw->imu->sampling_sync();
      hw->enc->sampling_sync();
      /* update data */
      update_samples();
      update_estimator();
      update_odometry();
      /* notify app */
      data_ready_semaphore.give();
      /* ToDo: wait for app here (reference update) */
      /* PID control */
      if (drive_enabled)
        drive();
    }
  }
  void reset() {
    ref_v.clear();
    ref_a.clear();
    est_p.clear();
    est_v.clear();
    est_a.clear();
    enc_v.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(hw->enc->get_position(i));
    accel.clear(ctrl::Polar(hw->imu->accel.y, hw->imu->angular_accel));
    fbc.reset();
  }
  void update_samples() {
    /* add new samples */
    for (int i = 0; i < 2; i++)
      wheel_position[i].push(hw->enc->get_position(i));
    accel.push(ctrl::Polar(hw->imu->accel.y, hw->imu->angular_accel));
  }
  void update_estimator() {
    /* calculate differential of encoder value */
    for (int i = 0; i < 2; i++)
      enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
    enc_v.wheel2pole();
    /* calculate estimated velocity value with complementary filter */
    const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, hw->imu->gyro.z);
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
    est_p.th += hw->imu->gyro.z * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
  }
  void drive() {
    /* calculate pwm value */
    const auto pwm_value = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
    const float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
    const float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
    /* drive the motors */
    hw->mt->drive(pwm_value_L, pwm_value_R);
  }
};
