#include "app_log.h"
#include "machine/machine.h"

void app_mouse() {
  /* pull-down motor pin */
  for (gpio_num_t gpio_num : {
           MOTOR_L_CTRL1_PIN,
           MOTOR_L_CTRL2_PIN,
           MOTOR_R_CTRL1_PIN,
           MOTOR_R_CTRL2_PIN,
           FAN_PIN,
       })
    ESP_ERROR_CHECK(gpio_pulldown_en(gpio_num));
  /* machine */
  auto machine = new machine::Machine();
  machine->init();
  /* infinite loop */
  vTaskDelay(portMAX_DELAY);
}
