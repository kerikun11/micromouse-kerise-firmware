/**
 * @file icm20602.h
 * @brief ICM-20602 Driver
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-05-23
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <esp_log.h>
#include <peripheral/spi.h>

struct MotionParameter {
  float x, y, z;
  MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  MotionParameter operator+(const MotionParameter& obj) const {
    return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
  }
  MotionParameter operator-(const MotionParameter& obj) const {
    return MotionParameter(x - obj.x, y - obj.y, z - obj.z);
  }
  MotionParameter operator*(const float mul) const {
    return MotionParameter(x * mul, y * mul, z * mul);
  }
  MotionParameter operator/(const float div) const {
    return MotionParameter(x / div, y / div, z / div);
  }
  MotionParameter& operator+=(const MotionParameter& obj) {
    return x += obj.x, y += obj.y, z += obj.z, *this;
  }
  MotionParameter& operator-=(const MotionParameter& obj) {
    return x -= obj.x, y -= obj.y, z -= obj.z, *this;
  }
};

class ICM20602 {
 private:
  static constexpr float ACCEL_G = 9806.65f;  //< [mm/s/s]
  static constexpr float ACCEL_FACTOR = ACCEL_G / 2048.0f / 4;
  static constexpr float GYRO_FACTOR = PI / 180 / 16.4f;

 public:
  ICM20602() {}
  MotionParameter accel, gyro;  //< ToDo: make private

  int init(spi_host_device_t spi_host, int8_t pin_cs) {
    // ESP-IDF SPI device initialization
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 3,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 20'000'000,
        .input_delay_ns = 0,
        .spics_io_num = pin_cs,
        .flags = 0,
        .queue_size = 2,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, &dev_cfg, &spi_handle));
    return reset();
  }
  void update() {
    uint8_t rx[14];
    readReg(59, rx, 14);
    accel.x = int16_t((rx[0] << 8) | rx[1]) * ACCEL_FACTOR;
    accel.y = int16_t((rx[2] << 8) | rx[3]) * ACCEL_FACTOR;
    accel.z = int16_t((rx[4] << 8) | rx[5]) * ACCEL_FACTOR;
    gyro.x = int16_t((rx[8] << 8) | rx[9]) * GYRO_FACTOR;
    gyro.y = int16_t((rx[10] << 8) | rx[11]) * GYRO_FACTOR;
    gyro.z = int16_t((rx[12] << 8) | rx[13]) * GYRO_FACTOR;
  }

 private:
  static constexpr const char* TAG = "ICM-20602";
  spi_device_handle_t spi_handle = NULL;

  int reset() {
    device_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    device_config();
    return whoami();
  }
  int device_reset() { return writeReg(107, 0x81); /*< PowerManagement 1 */ }
  void device_config() {
    /* Gyro */
    writeReg(26, 0x00); /*< Config; 2:0 DLPF=250[Hz] */
    writeReg(27, 0x18); /*< Gyr Conf; 4:3 FS=2000[dps], 1:0 FCHOICE=8[kHz] */
    /* Acceleration */
    // writeReg(28, 0x18); /*< Acc Conf; 4:3 FS=16[g] */
    writeReg(28, 0x08); /*< Acc Conf; 4:3 FS=4[g] */
    writeReg(29, 0x00); /*< Acc Conf 2; 3 F_CHOICE=1[kHz], DLPF=218.1[Hz] */
    /* Power */
    writeReg(107, 0x01); /*< PowerManagement 1; unset SLEEP */
  }
  int whoami() {
    uint8_t whoami = readReg(117);   /*< WHOAMI */
    uint8_t whoami_icm20600 = 0x11;  //< ICM-20600
    uint8_t whoami_icm20602 = 0x12;  //< ICM-20602
    if (whoami != whoami_icm20600 && whoami != whoami_icm20602) {
      ESP_LOGE(TAG, "whoami failed:( whoami: 0x%X", whoami);
      return whoami;
    }
    return 0;  //< OK
  }
  int writeReg(const uint8_t reg, const uint8_t data) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_TXDATA;
    tx.addr = reg;
    tx.tx_data[0] = data;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    uint8_t read_data = readReg(reg);
    if (read_data != data) {
      ESP_LOGE(TAG, "readReg failed. reg: 0x%02X data: 0x%02X != 0x%02X", reg,
               read_data, data);
      return -1;
    }
    return 0;
  }
  uint8_t readReg(const uint8_t reg) {
    static spi_transaction_t tx;
    tx.flags |= SPI_TRANS_USE_RXDATA;
    tx.addr = 0x80 | reg;
    tx.length = 8;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
    return tx.rx_data[0];
  }
  void readReg(const uint8_t reg, uint8_t* rx_buffer, size_t length) {
    static spi_transaction_t tx;
    tx.addr = 0x80 | reg;
    tx.tx_buffer = NULL;
    tx.rx_buffer = rx_buffer;
    tx.length = 8 * length;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle, &tx));
  }
};
