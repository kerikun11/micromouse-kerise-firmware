/**
 * @file adc.h
 * @brief ADC for ESP32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-28
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <esp_adc/adc_oneshot.h>

namespace peripheral {

class ADC {
 public:
  static const adc_unit_t ADC_UNIT = ADC_UNIT_1;

 public:
  static bool init() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    return true;
  }
  static int read_raw(adc_channel_t channel) {
    static adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    int value;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &value));
    return value;
  }
  static int read_milli_voltage(adc_channel_t channel,
                                int num_average_samples = 1) {
    uint32_t adc_reading = 0;
    for (int i = 0; i < num_average_samples; i++)
      adc_reading += read_raw(channel);
    adc_reading /= num_average_samples;
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &chars);
    uint32_t voltage = DEFAULT_VREF * 3.54813389f * adc_reading / 4095;
    return voltage;
  }

 private:
  static const int DEFAULT_VREF = 1100;
  static adc_oneshot_unit_handle_t adc1_handle;
};

};  // namespace peripheral
