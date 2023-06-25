#include "app_log.h"
#include "machine/machine.h"

void app_encoder() {
  /* pull-down motor pin */
  gpio_config_t config = {
      .pin_bit_mask = BIT64(MOTOR_L_CTRL1_PIN) | BIT64(MOTOR_L_CTRL2_PIN) |
                      BIT64(MOTOR_R_CTRL1_PIN) | BIT64(MOTOR_R_CTRL2_PIN) |
                      BIT64(FAN_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&config));
  /* pullup all the pins of the SPI-CS so that the bus is not blocked */
  for (auto p : CONFIG_SPI_CS_PINS) {
    gpio_reset_pin(p);
    gpio_set_direction(p, GPIO_MODE_INPUT);
    gpio_pullup_en(p);
  }
  /* infinite loop */
  vTaskDelay(portMAX_DELAY);
}
