#pragma once

#include <Arduino.h>
#include <TimerSemaphore.h>
#include <accumulator.h>
#include <array>

#define REFLECTOR_TASK_PRIORITY 20
#define REFLECTOR_STACK_SIZE 4096

class Reflector {
public:
  static constexpr int CH_SIZE = 4;

public:
  Reflector(std::array<int8_t, CH_SIZE> tx_pins,
            std::array<int8_t, CH_SIZE> rx_pins)
      : tx_pins(tx_pins), rx_pins(rx_pins) {}
  bool init() {
    for (int8_t i = 0; i < CH_SIZE; i++) {
      value[i] = 0;
      pinMode(tx_pins[i], OUTPUT);
      adcAttachPin(rx_pins[i]);
    }
    ts.periodic(100);
    xTaskCreate([](void *obj) { static_cast<Reflector *>(obj)->task(); },
                "Reflector", REFLECTOR_STACK_SIZE, this,
                REFLECTOR_TASK_PRIORITY, NULL);
    return true;
  }
  int16_t side(uint8_t isRight) const {
    if (isRight == 0)
      return read(0);
    else
      return read(1);
  }
  int16_t front(uint8_t isRight) const {
    if (isRight == 0)
      return read(2);
    else
      return read(3);
  }
  int16_t read(const int8_t ch) const {
    if (ch < 0 || ch >= CH_SIZE) {
      log_e("you refered an invalid channel!");
      return 0;
    }
    return value[ch];
  }
  void csv() const {
    printf("0,1500");
    for (int8_t i = 0; i < CH_SIZE; i++)
      printf(",%d", read(i));
    printf("\n");
  }
  void print() const {
    printf("Reflector: ");
    for (int8_t i = 0; i < CH_SIZE; i++)
      printf("\t%04d", read(i));
    printf("\n");
  }

private:
  const std::array<int8_t, CH_SIZE> tx_pins; //< 赤外線LEDのピン
  const std::array<int8_t, CH_SIZE> rx_pins; //< フォトトランジスタのピン
  std::array<int16_t, CH_SIZE> value;        //< リフレクタの測定値
  TimerSemaphore ts; //< インターバル用タイマー
  static const int ave_num = 16;
  Accumulator<uint16_t, ave_num> buffer[CH_SIZE];

  void update() {
    ts.take(); //< スタートを同期
    for (int i : {2, 1, 0, 3}) {
      // ts.take();                            //< スタートを同期
      adcStart(rx_pins[i]);                 //< オフセットADCスタート
      ts.take();                            //< ADC待ち
      uint16_t offset = adcEnd(rx_pins[i]); //< オフセットを取得
      digitalWrite(tx_pins[i], HIGH);       //< 放電開始
      // delayMicroseconds(20);                //< 調整
      adcStart(rx_pins[i]);              //< ADCスタート
      ts.take();                         //< ADC待ち
      uint16_t raw = adcEnd(rx_pins[i]); //< ADC取得
      digitalWrite(tx_pins[i], LOW);     //< 充電開始

      int temp = (int)raw - offset; //< オフセットとの差をとる
      if (temp < 1)
        temp = 1;           //< 0以下にならないように1で飽和
      buffer[i].push(temp); //< 保存
      value[i] = buffer[i].average();
    }
  }
  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
    }
  }
};
