#pragma once

#include <driver/i2c.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <esp_adc/adc_oneshot.h>

#include <array>
#include <vector>

#include "encoder.h"
#include "reflector.h"

namespace hardware {

struct DeviceParameter {
  /* Battery */
  adc_channel_t adc_channel_battery;
  /* Buzzer */
  gpio_num_t gpio_num_buzzer;
  /* Button */
  gpio_num_t gpio_num_button;

  /* Reflector */
  std::array<gpio_num_t, Reflector::kNumChannels> gpio_nums_reflector_tx;
  std::array<adc_channel_t, Reflector::kNumChannels> adc_channels_reflector_rx;

  /* I2C Bus */
  i2c_port_t i2c_port;
  gpio_num_t gpio_num_i2c_sda;
  gpio_num_t gpio_num_i2c_scl;

  /* SPI Bus */
  spi_host_device_t spi_host_device;
  gpio_num_t gpio_num_spi_miso;
  gpio_num_t gpio_num_spi_mosi;
  gpio_num_t gpio_num_spi_sclk;
  int dma_chain_spi;
  /* Encoder */
  Encoder::SensorType sensor_type_encoder;
  std::vector<gpio_num_t> gpio_nums_spi_cs_encoder;
  /* IMU */
  int num_imu;
  std::vector<gpio_num_t> gpio_nums_spi_cs_imu;
};

}  // namespace hardware
