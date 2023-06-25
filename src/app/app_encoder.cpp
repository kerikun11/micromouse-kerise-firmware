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
  /* SPI for IMU, Encoder */
  if (!peripheral::SPI::install(CONFIG_SPI_HOST, CONFIG_SPI_SCLK_PIN,
                                CONFIG_SPI_MISO_PIN, CONFIG_SPI_MOSI_PIN,
                                CONFIG_SPI_DMA_CHAIN))
    APP_LOGE("install failed");
  /* Encoder */
  auto* enc = new hardware::Encoder();
  if (!enc->init(ENCODER_SPI_HOST, ENCODER_CS_PINS))
    APP_LOGE("enc init failed");
  /* main */
  while (1) {
    APP_LOGI("%d %d", enc->get_raw(0), enc->get_raw(1));
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  /* infinite loop */
  vTaskDelay(portMAX_DELAY);
}
