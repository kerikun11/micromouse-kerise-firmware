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

extern "C" void app_main() {
  /* Check Chip ID */
  uint64_t mac = peripheral::ESP::get_mac();
  switch (mac) {
    case 0x080C'401D'A0D8:  //< KERISE v4
    case 0x807F'631D'A0D8:  //< KERISE v4 Copy
    case 0xD866'5A1D'A0D8:  //< KERISE v5
    case 0xECDC'E8AC'CD98:  //< KERISE v6
      app_mouse();
      break;
    default:
      APP_LOGW("unknown ESP32 MAC: 0x%012llX", mac);
      app_devkit();
      break;
  }
  vTaskDelay(portMAX_DELAY);
}
