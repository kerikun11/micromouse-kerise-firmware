/**
 * @file esp.h
 * @brief ESP32 utility
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-13
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <esp_mac.h>  //< esp_efuse_mac_get_default
#include <soc/rtc.h>  //< for rtc_clk_cpu_freq_get_config

namespace peripheral {

class ESP {
 public:
  static uint64_t get_mac() {
    uint64_t mac = 0;
    esp_efuse_mac_get_default((uint8_t*)&mac);
    return mac;
  }
  static int get_cpu_freq_in_mhz() {
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);
    return conf.freq_mhz;
  }
};

};  // namespace peripheral
