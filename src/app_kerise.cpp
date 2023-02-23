#include "app_log.h"

#include "machine/machine.h"

void kerise_main() {
  /* pull-down motor pin */
  gpio_config_t config = {
      .pin_bit_mask = ((uint64_t)1 << MOTOR_L_CTRL1_PIN) |
                      ((uint64_t)1 << MOTOR_L_CTRL2_PIN) |
                      ((uint64_t)1 << MOTOR_R_CTRL1_PIN) |
                      ((uint64_t)1 << MOTOR_R_CTRL2_PIN) |
                      ((uint64_t)1 << FAN_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&config));
  /* machine */
  auto machine = new machine::Machine();
  machine->init();
  /* infinite loop */
  vTaskDelay(portMAX_DELAY);
}
