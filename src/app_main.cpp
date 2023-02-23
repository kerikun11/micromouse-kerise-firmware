/**
 * @file app_main.cpp
 * @brief MicroMouse KERISE firmware
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-04-02
 */
#include "app_log.h"
#include "machine/machine.h"

void kerise_main();
void devkit_main();

extern "C" void app_main() {
  /* Check ID */
  uint64_t mac = peripheral::ESP::get_mac();
  switch (mac) {
    case 0x080C'401D'A0D8:  //< KERISE v4
    case 0x807F'631D'A0D8:  //< KERISE v4 Copy
    case 0xD866'5A1D'A0D8:  //< KERISE v5
      return kerise_main();
    case 0xD4E3'CAA4'AE30:  //< DevKit
      return devkit_main();
    default:
      APP_LOGW("unknown ESP32 MAC: 0x%012llX", mac);
      return devkit_main();
  }
  vTaskDelay(portMAX_DELAY);
}
