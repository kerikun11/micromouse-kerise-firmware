/**
 * @file i2c.h
 * @brief I2C utility
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-05-23
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/i2c.h>

namespace peripheral {

class I2C {
 public:
  static bool install(i2c_port_t port, gpio_num_t sda, gpio_num_t scl,
                      uint32_t clk_speed = 400'000) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master =
            {
                .clk_speed = clk_speed,
            },
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    esp_err_t ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
      return false;
    }
    return true;
  }

  /* specific functions */
  static bool writeReg8(i2c_port_t port, uint8_t addr7, uint8_t reg8,
                        uint8_t* data, int len, TickType_t ticks_to_wait,
                        bool ack_en = true) {
    return writeReadReg(port, addr7, &reg8, 1, data, len, nullptr, 0,
                        ticks_to_wait, ack_en);
  }
  static bool readReg8(i2c_port_t port, uint8_t addr7, uint8_t reg8,
                       uint8_t* data, int len, TickType_t ticks_to_wait,
                       bool ack_en = true) {
    return writeReadReg(port, addr7, &reg8, 1, nullptr, 0, data, len,
                        ticks_to_wait, ack_en);
  }
  static bool writeReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
                         uint8_t* data, int len, TickType_t ticks_to_wait,
                         bool ack_en = true) {
    const uint8_t reg_buf[2] = {(uint8_t)((reg16 >> 8) & 0xff),
                                (uint8_t)(reg16 & 0xff)};
    return writeReadReg(port, addr7, reg_buf, 2, data, len, nullptr, 0,
                        ticks_to_wait, ack_en);
  }
  static bool readReg16(i2c_port_t port, uint8_t addr7, uint16_t reg16,
                        uint8_t* data, int len, TickType_t ticks_to_wait,
                        bool ack_en = true) {
    const uint8_t reg_buf[2] = {(uint8_t)((reg16 >> 8) & 0xff),
                                (uint8_t)(reg16 & 0xff)};
    return writeReadReg(port, addr7, reg_buf, 2, nullptr, 0, data, len,
                        ticks_to_wait, ack_en);
  }

  /* general function */
  static bool writeReadReg(i2c_port_t port, uint8_t addr7,
                           const uint8_t* reg_data, int reg_len,
                           const uint8_t* tx_data, int tx_len, uint8_t* rx_data,
                           int rx_len, TickType_t ticks_to_wait, bool ack_en) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));
    if (reg_len > 0 || tx_len > 0)
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, ack_en));
    if (reg_len > 0)
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          i2c_master_write(cmd, (uint8_t*)reg_data, reg_len, ack_en));
    if (tx_len > 0)
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          i2c_master_write(cmd, (uint8_t*)tx_data, tx_len, ack_en));
    if (rx_len > 0) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_start(cmd));
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, ack_en));
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          i2c_master_read(cmd, rx_data, rx_len, I2C_MASTER_LAST_NACK));
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, ticks_to_wait);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
      return false;
    }
    return true;
  }
};

};  // namespace peripheral
