/**
 * @file app_main.cpp
 * @brief MicroMouse KERISE firmware
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-04-02
 */
#include "app_log.h"
#include "machine/machine.h"

void app_mouse();
void app_devkit();
void app_encoder();

extern "C" void app_main() {
  /* for debug */
  // app_encoder(), vTaskDelay(portMAX_DELAY);
  /* Check ID */
  uint64_t mac = peripheral::ESP::get_mac();
  switch (mac) {
    case 0x080C'401D'A0D8:  //< KERISE v4
    case 0x807F'631D'A0D8:  //< KERISE v4 Copy
    case 0xD866'5A1D'A0D8:  //< KERISE v5
      app_mouse();
      vTaskDelay(portMAX_DELAY);
      break;
    default:
      APP_LOGW("unknown ESP32 MAC: 0x%012llX", mac);
      app_devkit();
      vTaskDelay(portMAX_DELAY);
      break;
  }
}
