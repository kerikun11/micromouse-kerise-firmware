#include "app_log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <sstream>

void devkit_main() {
  vTaskDelay(pdMS_TO_TICKS(3000));
  APP_LOGI("This is ESP32 DevKit");

#if 0
  auto *bz = hardware::Buzzer::get_instance();
  bz->init(BUZZER_PIN, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_TIMER);
  bz->play(hardware::Buzzer::BOOT);
#endif

#if 0
  {
    std::stringstream ss;
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
      ss << "Hello world" << std::endl;
      std::cout << i << ":" << ss.str().capacity() << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("t1-t0: %d", t1 - t0);
  }
  {
    std::stringstream ss;
    ss.str().reserve(10 * 12);
    std::cout << "ss.str().capacity():" << ss.str().capacity() << std::endl;
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
      ss << "Hello world" << std::endl;
      std::cout << i << ":" << ss.str().capacity() << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("t1-t0: %d", t1 - t0);
  }
  {
    std::string str;
    str.reserve(10 * 12);
    std::stringstream ss(str);
    std::cout << "str.capacity():" << str.capacity() << std::endl;
    std::cout << "ss.str().capacity():" << ss.str().capacity() << std::endl;
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
      ss << "Hello world" << std::endl;
      std::cout << i << ":" << ss.str().capacity() << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("t1-t0: %d", t1 - t0);
  }
#endif

#if 0
  {
    std::stringstream ss;
    for (int i = 0; i < 10; i++) {
      const int t0 = esp_timer_get_time();
      ss << "Hello world" << std::endl;
      const int t1 = esp_timer_get_time();
      APP_LOGI("i: %d t1-t0: %d", i, t1 - t0);
    }
  }
#endif

#if 0
  {
    const int N = 1000;
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < N; i++) {
      std::stringstream ss;
      ss << 123 << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("time: %d", (t1 - t0) / N);
  }
  {
    const int N = 1000;
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < N; i++) {
      std::stringstream ss;
      ss << 1.23f << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("time: %d", (t1 - t0) / N);
  }
  {
    const int N = 1000
    const int t0 = esp_timer_get_time();
    for (int i = 0; i < N; i++) {
      std::stringstream ss;
      ss << 1.23 << std::endl;
    }
    const int t1 = esp_timer_get_time();
    APP_LOGI("time: %d", (t1 - t0) / N);
  }
#endif

#if 0
  {
    std::string s;
    for (int i = 0; i < 10; i++) {
      const int t0 = esp_timer_get_time();
      s += "Hello world ";
      const int t1 = esp_timer_get_time();
      APP_LOGI("i: %d t1-t0: %d cap: %d", i, t1 - t0, (int)s.capacity());
    }
  }
  {
    std::string s;
    s.reserve(1000);
    for (int i = 0; i < 10; i++) {
      const int t0 = esp_timer_get_time();
      s += "Hello world ";
      const int t1 = esp_timer_get_time();
      APP_LOGI("i: %d t1-t0: %d", i, t1 - t0);
    }
  }
#endif

#if 0
  {
    char str[256];
    int t0, t1;
    const int N = 1000;

    for (int i = 0; i < N; i++) {
      t0 = esp_timer_get_time();
      snprintf(str, sizeof(str), "%f", (double)1.23f);
      t1 = esp_timer_get_time();
    }
    APP_LOGI("time: %d", t1 - t0);

    for (int i = 0; i < N; i++) {
      t0 = esp_timer_get_time();
      snprintf(str, sizeof(str), "%f", 1.23);
      t1 = esp_timer_get_time();
    }
    APP_LOGI("time: %d", t1 - t0);
  }
#endif

#if 0
  {
    char str[256];
    int t0, t1;
    const int N = 1000;

    for (int i = 0; i < N; i++) {
      t0 = esp_timer_get_time();
      snprintf(str, sizeof(str), "%d.%06d", t0 / 1000000, t0 % 1000000);
      t1 = esp_timer_get_time();
    }
    APP_LOGI("time: %d", t1 - t0);

    for (int i = 0; i < N; i++) {
      t0 = esp_timer_get_time();
      const auto res = std::div(t0, 1000000);
      snprintf(str, sizeof(str), "%d.%06d", res.quot, res.rem);
      t1 = esp_timer_get_time();
    }
    APP_LOGI("time: %d", t1 - t0);

    for (int i = 0; i < N; i++) {
      t0 = esp_timer_get_time();
      snprintf(str, sizeof(str), "%f", t0 / 1000000.0);
      t1 = esp_timer_get_time();
    }
    APP_LOGI("time: %d", t1 - t0);
  }
#endif

#if 0
  setlocale(LC_NUMERIC, "");
  printf("%'d\n", 1000000);
#endif

  APP_LOG_DUMP();
  vTaskDelay(portMAX_DELAY);
}
