#pragma once

/* Driver */
#include "hardware/buzzer.h"
#include "hardware/fan.h"
#include "hardware/led.h"
#include "hardware/motor.h"
/* Sensor */
#include "hardware/button.h"
#include "hardware/encoder.h"
#include "hardware/imu.h"
#include "hardware/reflector.h"
#include "hardware/tof.h"
/* Config */
#include "config/io_mapping.h"

namespace hardware {

class Hardware {
 public:
  /* Driver */
  Buzzer* bz;
  LED* led;
  Motor* mt;
  Fan* fan;
  /* Sensor */
  Button* btn;
  IMU* imu;
  Encoder* enc;
  Reflector* rfl;
  ToF* tof;

 private:
  static constexpr float thr_battery = 3.8f;

 public:
  Hardware() {}
  bool init() {
    /* pullup all the pins of the SPI-CS so that the bus is not blocked */
    for (auto p : CONFIG_SPI_CS_PINS) {
      gpio_reset_pin(p);
      gpio_set_direction(p, GPIO_MODE_INPUT);
      gpio_pullup_en(p);
    }

    /* Buzzer (initialize first to notify errors by sound) */
    bz = new Buzzer();
    bz->init(BUZZER_PIN, BUZZER_LEDC_TIMER, BUZZER_LEDC_CHANNEL);
    /* Button */
    btn = new Button();
    btn->init(BUTTON_PIN);
    /* I2C for LED, ToF */
    if (!peripheral::I2C::install(I2C_PORT_NUM, I2C_SDA_PIN, I2C_SCL_PIN))
      bz->play(hardware::Buzzer::ERROR);
    /* LED */
    led = new LED();
    if (!led->init(I2C_PORT_NUM_LED)) bz->play(hardware::Buzzer::ERROR);
    /* ADC for Battery, Reflector */
    if (!peripheral::ADC::init()) bz->play(hardware::Buzzer::ERROR);

    /* Battery Check */
    if (batteryCheck())
      bz->play(hardware::Buzzer::BOOT);
    else
      bz->play(hardware::Buzzer::SHUTDOWN);

    /* SPI for IMU, Encoder */
    if (!peripheral::SPI::install(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN,
                                  CONFIG_SPI_MISO_PIN, CONFIG_SPI_MOSI_PIN,
                                  CONFIG_SPI_DMA_CHAIN))
      bz->play(hardware::Buzzer::ERROR);
    /* IMU */
    imu = new IMU();
    if (!imu->init(ICM20602_SPI_HOST, ICM20602_CS_PINS,
                   model::IMURotationRadius))
      bz->play(hardware::Buzzer::ERROR);
    /* Encoder */
    enc = new Encoder();
    Encoder::Parameter encoder_parameter = {
        .sensor_type = ENCODER_SENSOR_TYPE,
        .spi_host = ENCODER_SPI_HOST,
        .gpio_nums_spi_cs = ENCODER_CS_PINS,
        .gear_ratio = model::GearRatio,
        .wheel_diameter = model::WheelDiameter,
    };
    if (!enc->init(encoder_parameter)) bz->play(hardware::Buzzer::ERROR);
    /* Reflector */
    rfl = new Reflector();
    if (!rfl->init(REFLECTOR_TX_PINS, REFLECTOR_RX_CHANNELS))
      bz->play(hardware::Buzzer::ERROR);
    /* ToF */
    tof = new ToF();
    ToF::Parameter tof_param = {
        .i2c_port = I2C_PORT_NUM_TOF,
        .max_convergence_time_ms = model::vl6180x_max_convergence_time,
        .reference_range_90mm = model::tof_raw_range_90,
        .reference_range_180mm = model::tof_raw_range_180,
    };
    if (!tof->init(tof_param)) bz->play(hardware::Buzzer::ERROR);
    /* Motor */
    mt = new Motor(MOTOR_MCPWM_GROUP_ID, MOTOR_L_CTRL1_PIN, MOTOR_L_CTRL2_PIN,
                   MOTOR_R_CTRL1_PIN, MOTOR_R_CTRL2_PIN);
    /* Fan */
    fan = new Fan(FAN_PIN, FAN_LEDC_TIMER, FAN_LEDC_CHANNEL);

    /* Ending */
    return true;
  }
  void sampling_request() {
    imu->sampling_request();
    enc->sampling_request();
  }
  void sampling_wait() {
    imu->sampling_wait();
    enc->sampling_wait();
  }

  /**
   * @brief Sample the Battery Voltage
   * @return float voltage [V]
   */
  static float getBatteryVoltage() {
    return 1e-3f / model::kBatteryVoltageDividerRatio *
           peripheral::ADC::read_milli_voltage(BAT_VOL_ADC1_CHANNEL, 10);
  }
  /**
   * @brief バッテリー電圧をLEDで表示
   * @param voltage [V]
   */
  void batteryLedIndicate(const float voltage) {
    led->set(0);
    if (voltage < 4.0f)
      led->set(0x01);
    else if (voltage < 4.1f)
      led->set(0x03);
    else if (voltage < 4.2f)
      led->set(0x07);
    else
      led->set(0x0F);
  }
  bool batteryCheck() {
    const float voltage = getBatteryVoltage();
    batteryLedIndicate(voltage);
    APP_LOGI("Battery Voltage: %f [V]", (double)voltage);
    if (voltage < thr_battery) {
      APP_LOGW("Battery Low!");
      return false;
    }
    return true;
  }
};

}  // namespace hardware
