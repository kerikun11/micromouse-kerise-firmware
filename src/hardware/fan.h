/**
 * @file fan.h
 * @brief Fan Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <algorithm>  //< for std::min std::max std::clamp

namespace hardware {

class Fan {
 public:
  Fan(gpio_num_t gpio_num, ledc_timer_t timer, ledc_channel_t channel)
      : timer(timer), channel(channel) {
    const uint32_t freq = 250'000;
    const ledc_timer_bit_t duty_resolution = LEDC_TIMER_6_BIT;
    duty_max = (2 << duty_resolution) - 1;
    // LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = mode,
        .duty_resolution = duty_resolution,
        .timer_num = timer,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // LEDC Channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = mode,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .flags{
            .output_invert = 0,
        },
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  }
  /**
   * @brief drive pwm
   * @param duty duty ratio in [0, 1]
   */
  void drive(float duty) {
    uint32_t duty_cycles = duty * duty_max;
    duty_cycles = std::clamp(duty_cycles, static_cast<uint32_t>(0), duty_max);
    ESP_ERROR_CHECK(ledc_set_duty(mode, channel, duty_cycles));
    ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
  }
  void free() { drive(0); }

 private:
  ledc_timer_t timer;
  ledc_channel_t channel;
  ledc_mode_t mode = LEDC_HIGH_SPEED_MODE;
  uint32_t duty_max;
};

};  // namespace hardware
