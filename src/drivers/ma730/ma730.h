/**
 * @file ma730.h
 * @brief MA730 Magnetic Encoder Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-05-23
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>

namespace drivers {

class MA730 {
 public:
  static constexpr int PULSES_SIZE = 16384;

 public:
  MA730() {}
  bool init(spi_host_device_t spi_host, gpio_num_t pin_cs) {
    // CS GPIO (SPI dev has only 3 CS Controller)
    pin_cs_ = pin_cs;
    ESP_ERROR_CHECK(gpio_set_direction(pin_cs_, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(pin_cs_, 1));
    // ESP-IDF SPI device initialization
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 3,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 20'000'000,
        .input_delay_ns = 0,
        .spics_io_num = GPIO_NUM_NC,
        .flags = 0,
        .queue_size = 1,
        .pre_cb =
            [](spi_transaction_t* trans) {
              const auto* this_ptr =
                  reinterpret_cast<decltype(this)>(trans->user);
              ESP_ERROR_CHECK(gpio_set_level(this_ptr->pin_cs_, 0));
            },
        .post_cb =
            [](spi_transaction_t* trans) {
              const auto* this_ptr =
                  reinterpret_cast<decltype(this)>(trans->user);
              ESP_ERROR_CHECK(gpio_set_level(this_ptr->pin_cs_, 1));
            },
    };
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &spi_handle_));
    return check() && update();
  }
  bool check() {
    /* transaction */
    spi_transaction_t tx{};  //< zero initialization
    tx.user = this;
    tx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    tx.tx_data[0] = 0b010'00110;  // 0b00110: MGLT(2:0) MGHT(2:0) Reserved (1:0)
    tx.tx_data[1] = 0x00;
    tx.length = 16;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &tx));
    tx.tx_data[0] = tx.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &tx));
    /* data parse */
    return tx.rx_data[0] == 0x1C;
  }
  bool update() {
    /* transaction */
    spi_transaction_t tx{};  //< zero initialization
    tx.user = this;
    tx.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    tx.tx_data[0] = tx.tx_data[1] = 0x00;
    tx.length = 16;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &tx));
    /* data parse */
    pulses_ = uint16_t(tx.rx_data[0] << 6) | (tx.rx_data[1] >> 2);
    return true;
  }
  int get() const { return pulses_; }

 private:
  spi_device_handle_t spi_handle_ = NULL;
  gpio_num_t pin_cs_ = GPIO_NUM_NC;
  int pulses_;
};

};  // namespace drivers
