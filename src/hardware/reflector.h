/**
 * @file reflector.h
 * @brief Reflector Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/gpio.h>

#include <array>

#include "peripheral/adc.h"
#include "utils/timer_semaphore.h"

namespace hardware {

class Reflector {
 public:
  static constexpr int kNumChannels = 4;  //< SL SR FL FR
  static constexpr int kSamplingPeriodMicroSeconds = 250;

 public:
  Reflector() {}
  bool init(const std::array<gpio_num_t, kNumChannels>& gpio_nums_tx,
            const std::array<adc_channel_t, kNumChannels>& rx_channels) {
    gpio_nums_tx_ = gpio_nums_tx;
    rx_channels_ = rx_channels;
    for (int i = 0; i < kNumChannels; i++) {
      // tx
      ESP_ERROR_CHECK(gpio_reset_pin(gpio_nums_tx_[i]));
      ESP_ERROR_CHECK(gpio_set_level(gpio_nums_tx_[i], 0));
      ESP_ERROR_CHECK(gpio_set_direction(gpio_nums_tx_[i], GPIO_MODE_OUTPUT));
      // rx
      int pin;
      ESP_ERROR_CHECK(adc_oneshot_channel_to_io(peripheral::ADC::ADC_UNIT,
                                                rx_channels_[i], &pin));
      ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)pin));
      // value
      value_[i] = 0;
    }
    const int stack_depth = 2048;
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); },
        "Reflector", stack_depth, this, TASK_PRIORITY_REFLECTOR, NULL,
        TASK_CORE_ID_REFLECTOR);
    return true;
  }
  int16_t side(const uint8_t isRight) { return read(isRight ? 1 : 0); }
  int16_t front(const uint8_t isRight) { return read(isRight ? 3 : 2); }
  int16_t read(const int8_t ch) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return value_[ch];
  }
  void csv(std::ostream& out = std::cout) {
    out << "0\t2000\t4000\t";
    for (int8_t i = 0; i < kNumChannels; i++) out << "\t" << read(i);
    out << std::endl;
  }
  void print() {
    APP_LOGI("Reflector: %4d %4d %4d %4d", read(0), read(1), read(2), read(3));
  }

 private:
  std::array<gpio_num_t, kNumChannels> gpio_nums_tx_;  //< 赤外線LEDのピン番号
  std::array<adc_channel_t, kNumChannels>
      rx_channels_;    //< 赤外線センサのADC1_CHANNEL
  TimerSemaphore ts_;  //< インターバル用タイマー

  std::mutex mutex_;                         //< value用のMutex
  std::array<int16_t, kNumChannels> value_;  //< リフレクタの測定値 SL SR FL FR

  void task() {
    ts_.start_periodic(kSamplingPeriodMicroSeconds);
    while (1) {
      for (int i : {2, 1, 0, 3}) {  //< FL SR SL FR
        ts_.take();                 //< 干渉防止のウエイト
        // Sampling
        int offset = peripheral::ADC::read_raw(rx_channels_[i]);  //< ADC取得
        gpio_set_level(gpio_nums_tx_[i], 1);                   //< 放電開始
        int raw = peripheral::ADC::read_raw(rx_channels_[i]);  //< ADC取得
        gpio_set_level(gpio_nums_tx_[i], 0);                   //< 充電開始
        // Calculation
        int diff = raw - offset;  //< オフセットとの差をとる
        if (diff < 1) diff = 1;   //< 0以下にならないように1で飽和
        // Result
        std::lock_guard<std::mutex> lock_guard(mutex_);  //< lock value
        value_[i] = diff;
      }
    }
  }
};

};  // namespace hardware
