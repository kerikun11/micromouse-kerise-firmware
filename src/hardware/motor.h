/**
 * @file motor.h
 * @brief Motor Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/mcpwm_prelude.h>
#include <hal/gpio_types.h>

#include <algorithm>  //< for std::max, std::min
#include <cmath>      //< for std::isfinit
#include <iostream>   //< for std::cerr

#include "app_log.h"

namespace hardware {

class OneMotor {
 public:
  OneMotor(int mcpwm_group_id, gpio_num_t gpio_num_1, gpio_num_t gpio_num_2) {
    /* constants */
    const uint32_t pwm_resolution_hz = 80'000'000UL;  // ESP32: max 80MHz
    const uint32_t pwm_freq_hz = 250'000UL;           // DRV8835: max 250kHz
    pwm_ticks_ = pwm_resolution_hz / pwm_freq_hz;     // 80M/250k = 320 ticks
    APP_LOGI("mcpwm_group_id: %d clock: %lu freq: %lu ticks: %lu",
             mcpwm_group_id, pwm_resolution_hz, pwm_freq_hz, pwm_ticks_);
    /* timer */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = mcpwm_group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = pwm_resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = pwm_ticks_,
        .intr_priority = 0,
        .flags{
            .update_period_on_empty = 0,
            .update_period_on_sync = 0,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    /* oper */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = mcpwm_group_id,
        .intr_priority = 0,
        .flags{
            .update_gen_action_on_tez = 0,
            .update_gen_action_on_tep = 0,
            .update_gen_action_on_sync = 0,
            .update_dead_time_on_tez = 0,
            .update_dead_time_on_tep = 0,
            .update_dead_time_on_sync = 0,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    /* connect */
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    /* comparator */
    mcpwm_comparator_config_t comparator_config = {
        .intr_priority = 0,
        .flags{
            .update_cmp_on_tez = 0,
            .update_cmp_on_tep = 0,
            .update_cmp_on_sync = 0,
        },
    };
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(oper, &comparator_config, &cmpr_handle_[0]));
    ESP_ERROR_CHECK(
        mcpwm_new_comparator(oper, &comparator_config, &cmpr_handle_[1]));
    /* generator */
    mcpwm_gen_handle_t generators[2] = {};
    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = gpio_num_1;
    ESP_ERROR_CHECK(
        mcpwm_new_generator(oper, &generator_config, &generators[0]));
    generator_config.gen_gpio_num = gpio_num_2;
    ESP_ERROR_CHECK(
        mcpwm_new_generator(oper, &generator_config, &generators[1]));
    /* comparator */
    for (int i = 0; i < 2; ++i) {
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
          generators[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                      MCPWM_TIMER_EVENT_EMPTY,
                                                      MCPWM_GEN_ACTION_HIGH)));
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
          generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(
                             MCPWM_TIMER_DIRECTION_UP, cmpr_handle_[i],
                             MCPWM_GEN_ACTION_LOW)));
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_handle_[i], 0));
    }
    /* start */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    /* end */
    free();
  }
  void free() {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_handle_[0], 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr_handle_[1], 0));
  }
  void drive(float duty) {
    uint32_t duty_cycles = std::abs(duty) * pwm_ticks_;
    duty_cycles = std::min(duty_cycles, pwm_ticks_);
    if (duty > 0) {
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
          cmpr_handle_[0], pwm_ticks_ - duty_cycles));
      ESP_ERROR_CHECK(
          mcpwm_comparator_set_compare_value(cmpr_handle_[1], pwm_ticks_));
    } else {
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
          cmpr_handle_[1], pwm_ticks_ - duty_cycles));
      ESP_ERROR_CHECK(
          mcpwm_comparator_set_compare_value(cmpr_handle_[0], pwm_ticks_));
    }
  }

 private:
  mcpwm_cmpr_handle_t cmpr_handle_[2] = {};
  uint32_t pwm_ticks_;
};

class Motor {
 private:
  static constexpr float kEmergencyThreshold = 1.3f;

 public:
  Motor(int mcpwm_group_id, gpio_num_t gpio_L1, gpio_num_t gpio_L2,
        gpio_num_t gpio_R1, gpio_num_t gpio_R2)
      : motor_L_(mcpwm_group_id, gpio_L1, gpio_L2),
        motor_R_(mcpwm_group_id, gpio_R1, gpio_R2) {}
  /**
   * @brief set pwm value
   * @param valueL L duty ratio in [-1, 1]
   * @param valueR R duty ratio in [-1, 1]
   */
  void drive(float valueL, float valueR) {
    if (emergency_) return;
    motor_L_.drive(valueL);
    motor_R_.drive(valueR);
    if (std::abs(valueL) > kEmergencyThreshold ||
        std::abs(valueR) > kEmergencyThreshold)
      emergency_stop();
  }
  void free() {
    motor_L_.free();
    motor_R_.free();
  }
  void emergency_stop() {
    emergency_ = true;
    free();
  }
  void emergency_release() {
    emergency_ = false;
    free();
  }
  bool is_emergency() const { return emergency_; }

 private:
  bool emergency_ = false;
  OneMotor motor_L_;
  OneMotor motor_R_;
};

};  // namespace hardware
