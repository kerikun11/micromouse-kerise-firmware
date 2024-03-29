/**
 * @file as5048a.h
 * @brief Driver of Dual Chain-Connected Magnetic Encoder AS5048A
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-05-09
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>

namespace drivers {

class AS5048A_DUAL {
 public:
  static constexpr int PULSES_SIZE = 16384;

 public:
  AS5048A_DUAL() {}
  bool init(const spi_host_device_t spi_host, const int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 1,  //< dummy initial clock
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 1,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 5'000'000,
        .input_delay_ns = 0,
        .spics_io_num = pin_cs,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &encoder_spi_));
    return update();
  }
  bool update() {
    bool res = true;
    /* transaction */
    spi_transaction_t tx{};  //< zero initialization
    tx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    tx.tx_data[0] = tx.tx_data[1] = tx.tx_data[2] = tx.tx_data[3] = 0xFF;
    tx.length = 32;
    ESP_ERROR_CHECK(spi_device_transmit(encoder_spi_, &tx));
    /* data parse */
    uint16_t pkt[2];
    pkt[0] = (tx.rx_data[0] << 8) | tx.rx_data[1];
    pkt[1] = (tx.rx_data[2] << 8) | tx.rx_data[3];
    for (int i = 0; i < 2; i++) {
      if (calc_even_parity(pkt[i])) {
        ESP_LOGW(TAG, "parity error. pkt[%d]: 0x%04X", i, (int)pkt[i]);
        res = false;
        continue;
      }
      pulses_[i] = pkt[i] & 0x3FFF;
    }
    return res;
  }
  int get(int ch) const { return pulses_[ch]; }

 private:
  static constexpr const char* TAG = "AS5048A";
  spi_device_handle_t encoder_spi_ = NULL;
  int pulses_[2] = {0, 0};

  static uint8_t calc_even_parity(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;
  }
};

};  // namespace drivers
