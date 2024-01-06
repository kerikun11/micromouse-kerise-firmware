/**
 * @file led.h
 * @brief LED Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <peripheral/i2c.h>

namespace hardware {

class LED {
 private:
  static constexpr uint8_t PCA9632_DEV_ID = 0x62;  //< 全体制御用のI2Cアドレス

 public:
  LED() { play_list_ = xQueueCreate(/* uxQueueLength = */ 5, sizeof(uint8_t)); }
  bool init(i2c_port_t i2c_port) {
    i2c_port_ = i2c_port;
    writeReg(0x00, 0b10000001);
    xTaskCreatePinnedToCore(
        [](void* arg) { static_cast<decltype(this)>(arg)->task(); }, "LED",
        2048, this, TASK_PRIORITY_LED, NULL, TASK_CORE_ID_LED);
    return true;
  }
  uint8_t set(uint8_t new_value) {
    value_ = new_value;
    xQueueSendToBack(play_list_, &value_, 0);
    return value_;
  }
  uint8_t get() const { return value_; }
  uint8_t operator=(uint8_t new_value) { return set(new_value); }
  operator uint8_t() const { return get(); }

 private:
  i2c_port_t i2c_port_;
  QueueHandle_t play_list_;
  uint8_t value_;

  void task() {
    while (1) {
      uint8_t value_;
      if (xQueueReceive(play_list_, &value_, portMAX_DELAY) == pdTRUE)
        writeValue(value_);
    }
  }
  bool writeValue(uint8_t value_) {
    uint8_t data = 0;
    data |= (value_ & 1) << 0;
    data |= (value_ & 2) << 1;
    data |= (value_ & 4) << 2;
    data |= (value_ & 8) << 3;
    return writeReg(0x08, data);
  }
  bool writeReg(uint8_t reg, uint8_t data) {
    return peripheral::I2C::writeReg8(i2c_port_, PCA9632_DEV_ID, reg, &data, 1,
                                      pdMS_TO_TICKS(10));
  }
};

};  // namespace hardware
