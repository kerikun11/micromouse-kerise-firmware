#pragma once

#include "accumulator.h"
#include "config/model.h"
#include "global.h"

#include "feedback_controller.h"
#include "polar.h"
#include "pose.h"

#include <atomic>

#define SPEED_CONTROLLER_TASK_PRIORITY 4
#define SPEED_CONTROLLER_STACK_SIZE 4096

struct WheelParameter {
public:
  float tra;      //< translation [mm]
  float rot;      //< rotation [rad]
  float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
public:
  WheelParameter() { clear(); }
  void pole2wheel() {
    wheel[0] = tra - model::RotationRadius * rot;
    wheel[1] = tra + model::RotationRadius * rot;
  }
  void wheel2pole() {
    rot = (wheel[1] - wheel[0]) / 2 / model::RotationRadius;
    tra = (wheel[1] + wheel[0]) / 2;
  }
  void clear() {
    tra = 0;
    rot = 0;
    wheel[0] = 0;
    wheel[1] = 0;
  }
};

class SpeedController {
public:
  static constexpr const float Ts = 1e-3f;

public:
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  WheelParameter enc_v;
  ctrl::Pose est_p;
  ctrl::FeedbackController<ctrl::Polar>::Model M;
  ctrl::FeedbackController<ctrl::Polar>::Gain G;
  ctrl::FeedbackController<ctrl::Polar> fbc;
  static constexpr int acc_num = 8;
  Accumulator<float, acc_num> wheel_position[2];
  Accumulator<ctrl::Polar, acc_num> accel;

public:
  SpeedController(const ctrl::FeedbackController<ctrl::Polar>::Model &M,
                  const ctrl::FeedbackController<ctrl::Polar>::Gain &G)
      : M(M), G(G), fbc(M, G) {
    reset();
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "SpeedController", SPEED_CONTROLLER_STACK_SIZE, this,
                SPEED_CONTROLLER_TASK_PRIORITY, NULL);
  }
  void enable() {
    reset_requested = true;
    enabled = true;
  }
  void disable() { enabled = false; }
  void set_target(const float v_tra, const float v_rot, const float a_tra = 0,
                  const float a_rot = 0) {
    ref_v.tra = v_tra;
    ref_v.rot = v_rot;
    ref_a.tra = a_tra;
    ref_a.rot = a_rot;
  }
  void fix_pose(const ctrl::Pose fix) { this->fix += fix; }

private:
  std::atomic_bool enabled{false};
  std::atomic_bool reset_requested{false};
  ctrl::Pose fix;

  void reset() {
    ref_v.clear();
    ref_a.clear();
    est_p.clear();
    est_v.clear();
    est_a.clear();
    enc_v.clear();
    for (int i = 0; i < 2; i++)
      wheel_position[i].clear(enc.get_position(i));
    accel.clear(ctrl::Polar(imu.accel.y, imu.angular_accel));
    fix.clear();
    fbc.reset();
  }
  void update_samples() {
    /* wait for end sampling */
    imu.samplingSemaphoreTake();
    enc.samplingSemaphoreTake();
    /* add new samples */
    for (int i = 0; i < 2; i++)
      wheel_position[i].push(enc.get_position(i));
    accel.push(ctrl::Polar(imu.accel.y, imu.angular_accel));
  }
  void update_estimator() {
    /* calculate differential of encoder value */
    for (int i = 0; i < 2; i++)
      enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
    enc_v.wheel2pole();
    /* calculate estimated velocity value with complementary filter */
    const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, imu.gyro.z);
    const ctrl::Polar v_high = est_v + accel[0] * float(Ts);
    const ctrl::Polar alpha = model::alpha;
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
    est_p.th += imu.gyro.z * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
  }
  void update_fix() {
    /* Fix Pose */
    const float delta = 0.2;
    if (fix.x > delta) {
      est_p.x += delta;
      fix.x -= delta;
    } else if (fix.x < -delta) {
      est_p.x -= delta;
      fix.x += delta;
    }
    if (fix.y > delta) {
      est_p.y += delta;
      fix.y -= delta;
    } else if (fix.y < -delta) {
      est_p.y -= delta;
      fix.y += delta;
    }
  }
  void drive() {
    /* calculate pwm value */
    const auto pwm_value = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
    const float pwm_value_L = pwm_value.tra - pwm_value.rot / 2;
    const float pwm_value_R = pwm_value.tra + pwm_value.rot / 2;
    /* drive the motors */
    mt.drive(pwm_value_L, pwm_value_R);
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      /* 同期 */
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      /* check if enabled */
      if (!enabled)
        continue;
      /* reset */
      if (reset_requested)
        reset_requested = false, reset();
      /* sampling */
      update_samples();
      /* estimate */
      update_estimator();
      /* drive the motors */
      drive();
      /* calculate odometry value */
      update_odometry();
      /* Fix Pose */
      update_fix();
    }
  }
};
