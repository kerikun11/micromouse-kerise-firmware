/**
 * @file UserInterface.h
 * @brief マイクロマウスに備わっているセンサを用いてUIを構成する
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-02-10
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware/hardware.h"

class UserInterface {
 public:
  /* UI パラメータ */
  static constexpr float PI = 3.14159265358979323846f;
  static constexpr float wait_ms = 200; /**< チャタリング防止時間 */
  static constexpr float thr_accel = 3 * 9807;    /**< 加速度の閾値 */
  static constexpr float thr_gyro = 4 * PI;       /**< 角速度の閾値 */
  static constexpr float enc_interval_mm = 10.0f; /**< エンコーダの間隔 */
  static constexpr int wait_fix_ms = 1000; /**< 静止待機の最小静止時間 */
  static constexpr float thr_fix_gyro = 0.01f * PI; /**< 静止角速度の閾値 */
  static constexpr float thr_gyro_pickup = PI; /**< 回収角速度の閾値 */

 private:
  hardware::Hardware* hw_;

 public:
  UserInterface(hardware::Hardware* hw) : hw_(hw) {}
  /**
   * @brief ユーザーに番号を選択させる
   *
   * @param range 番号の範囲．[1, 16]
   * @return int 0 ~ range-1: 選択された番号
   * @return int -1: キャンセルされた
   */
  int waitForSelect(const int range = 16, const uint8_t init_value = 0) {
    float position_prev = hw_->enc->get_position(0) + hw_->enc->get_position(1);
    uint8_t value = init_value;
    hw_->led->set(value);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      float position_now =
          hw_->enc->get_position(0) + hw_->enc->get_position(1);
      /* SELECT */
      if (hw_->imu->get_gyro3().y > thr_gyro) {
        value += range - 1;
        value %= range;
        hw_->led->set(value);
        hw_->bz->play(hardware::Buzzer::SELECT);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
      }
      if (hw_->imu->get_gyro3().y < -thr_gyro) {
        value += 1;
        value %= range;
        hw_->led->set(value);
        hw_->bz->play(hardware::Buzzer::SELECT);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
      }
#if 1
      if (position_now > position_prev + enc_interval_mm) {
        position_prev += enc_interval_mm;
        value += 1;
        value %= range;
        hw_->led->set(value);
        hw_->bz->play(hardware::Buzzer::SELECT);
      }
      if (position_now < position_prev - enc_interval_mm) {
        position_prev -= enc_interval_mm;
        value += range - 1;
        value %= range;
        hw_->led->set(value);
        hw_->bz->play(hardware::Buzzer::SELECT);
      }
#endif
      /* CONFIRM */
      if (std::abs(hw_->imu->get_accel3().z) > thr_accel) {
        hw_->bz->play(hardware::Buzzer::CONFIRM);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return value;
      }
      if (hw_->btn->pressed) {
        hw_->btn->flags = 0;
        hw_->bz->play(hardware::Buzzer::CONFIRM);
        return value;
      }
      /* CANCEL */
      if (std::abs(hw_->imu->get_accel3().x) > thr_accel) {
        hw_->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return -1;
      }
      if (hw_->btn->long_pressed_1) {
        hw_->btn->flags = 0;
        hw_->bz->play(hardware::Buzzer::CANCEL);
        return -1;
      }
      /* UART */
      int c = getchar();
      if (c != EOF) {
        switch (c) {
          case 'n':
            value += 1;
            value %= range;
            hw_->led->set(value);
            hw_->bz->play(hardware::Buzzer::SELECT);
            APP_LOGI("value: %d", value);
            break;
          case 'p':
            value += range - 1;
            value %= range;
            hw_->led->set(value);
            hw_->bz->play(hardware::Buzzer::SELECT);
            APP_LOGI("value: %d", value);
            break;
          case 'c':
            hw_->bz->play(hardware::Buzzer::CANCEL);
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
            APP_LOGI("cancel");
            return -1;
          case '\n':
            hw_->bz->play(hardware::Buzzer::CONFIRM);
            return value;
        }
      }
    }
  }
  /**
   * @brief 前壁センサが遮られるのを待つ関数
   *
   * @param side センサ位置 true: side, false: front
   * @return true 遮られた
   * @return false キャンセルされた
   */
  bool waitForCover(bool side = false) {
    if (side)
      hw_->led->set(9);
    else
      hw_->led->set(6);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      /* CONFIRM */
      if (!side && hw_->rfl->front(0) > model::ui_thr_ref_front &&
          hw_->rfl->front(1) > model::ui_thr_ref_front) {
        hw_->bz->play(hardware::Buzzer::CONFIRM);
        return true;
      }
      if (side && hw_->rfl->side(0) > model::ui_thr_ref_side &&
          hw_->rfl->side(1) > model::ui_thr_ref_side) {
        hw_->bz->play(hardware::Buzzer::CONFIRM);
        return true;
      }
      /* CANCEL */
      if (std::abs(hw_->imu->get_accel3().x) > thr_accel) {
        hw_->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return false;
      }
      if (hw_->btn->long_pressed_1) {
        hw_->btn->flags = 0;
        hw_->bz->play(hardware::Buzzer::CANCEL);
        return false;
      }
    }
  }
  /**
   * @brief マシン回収まで待つ関数
   *
   * @retval true 回収された
   * @retval false タイムアウト
   */
  bool waitForPickup(const int wait_ms = 1200) {
    hw_->led->set(0xf);
    for (int ms = 0; ms < wait_ms; ms++) {
      vTaskDelay(pdMS_TO_TICKS(1));
      const auto gyro = hw_->imu->get_gyro3();
      if (std::abs(gyro.x) + std::abs(gyro.y) + std::abs(gyro.z) >
          thr_gyro_pickup) {
        hw_->bz->play(hardware::Buzzer::CANCEL);
        hw_->led->set(0x0);
        return true;
      }
    }
    hw_->led->set(0x0);
    return false;
  }
  /**
   * @brief 静止状態になるまで待機する関数
   *
   * @return true 静止状態になった
   * @return false キャンセルされた
   */
  bool waitForFix() {
    int fix_count = 0;
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1));
      /* FIX */
      if (std::abs(hw_->imu->get_gyro3().x) < thr_fix_gyro &&
          std::abs(hw_->imu->get_gyro3().y) < thr_fix_gyro &&
          std::abs(hw_->imu->get_gyro3().z) < thr_fix_gyro) {
        if (fix_count++ > wait_fix_ms) {
          hw_->bz->play(hardware::Buzzer::CONFIRM);
          return true;
        }
      } else {
        fix_count = 0;
      }
      /* CANCEL */
      if (hw_->btn->pressed) {
        hw_->btn->flags = 0;
        hw_->bz->play(hardware::Buzzer::CANCEL);
        return false;
      }
      if (std::abs(hw_->imu->get_accel3().x) > thr_accel) {
        hw_->bz->play(hardware::Buzzer::CANCEL);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return false;
      }
    }
  }
};
