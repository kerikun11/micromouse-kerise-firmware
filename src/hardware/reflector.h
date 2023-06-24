/**
 * @file reflector.h
 * @brief Reflector Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accumulator.h>
#include <peripheral/adc.h>
#include <peripheral/timer_semaphore.h>

#include <array>

namespace hardware {

class Reflector {
 public:
  static constexpr int NUM_CHANNELS = 4;
  static constexpr int ave_num = 1;

 public:
  Reflector(const std::array<gpio_num_t, NUM_CHANNELS>& tx_pins,
            const std::array<adc_channel_t, NUM_CHANNELS>& rx_channels)
      : tx_pins(tx_pins), rx_channels(rx_channels) {}
  bool init() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      value[i] = 0;
      // tx
      ESP_ERROR_CHECK(gpio_reset_pin(tx_pins[i]));
      ESP_ERROR_CHECK(gpio_set_level(tx_pins[i], 0));
      ESP_ERROR_CHECK(gpio_set_direction(tx_pins[i], GPIO_MODE_OUTPUT));
      // rx
      int pin;
      ESP_ERROR_CHECK(adc_oneshot_channel_to_io(peripheral::ADC::ADC_UNIT,
                                                rx_channels[i], &pin));
      ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)pin));
    }
    ts.periodic(200);
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); },
        "Reflector", 2048, this, TASK_PRIORITY_REFLECTOR, NULL,
        TASK_CORE_ID_REFLECTOR);
    return true;
  }
  int16_t side(uint8_t isRight) const { return read(isRight ? 1 : 0); }
  int16_t front(uint8_t isRight) const { return read(isRight ? 3 : 2); }
  int16_t read(const int8_t ch) const { return value[ch]; }
  void csv() const {
    std::cout << "0,2000,4000,";
    for (int8_t i = 0; i < NUM_CHANNELS; i++) std::cout << "," << read(i);
    std::cout << std::endl;
  }
  void print() const {
    APP_LOGI("Reflector: %4d %4d %4d %4d", read(0), read(1), read(2), read(3));
  }

 private:
  const std::array<gpio_num_t, NUM_CHANNELS> tx_pins;  //< 赤外線LEDのピン
  const std::array<adc_channel_t, NUM_CHANNELS>
      rx_channels;  //< フォトトランジスタのADC1_CHANNEL
  std::array<int16_t, NUM_CHANNELS> value;  //< リフレクタの測定値
  TimerSemaphore ts;  //< インターバル用タイマー
  ctrl::Accumulator<int, ave_num> buffer[NUM_CHANNELS];  //< 平均計算用

  void update() {
    ts.take();  //< スタートを同期
    for (int i : {2, 1, 0, 3}) {
      ts.take();  //< 干渉防止のウエイト
      // Sampling
      int offset = peripheral::ADC::read_raw(rx_channels[i]);  //< ADC取得
      gpio_set_level(tx_pins[i], 1);                           //< 放電開始
      int raw = peripheral::ADC::read_raw(rx_channels[i]);     //< ADC取得
      gpio_set_level(tx_pins[i], 0);                           //< 充電開始
      // Calculation
      int diff = raw - offset;  //< オフセットとの差をとる
      if (diff < 1) diff = 1;   //< 0以下にならないように1で飽和
      buffer[i].push(diff);     //< 保存
      value[i] = buffer[i].average();
    }
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
    }
  }
};

};  // namespace hardware
