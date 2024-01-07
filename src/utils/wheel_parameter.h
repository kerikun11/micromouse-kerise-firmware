/**
 * @file wheel_parameter.h
 * @brief ホイールの回転量と並進・回転を相互変換する型を定義
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-13
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/polar.h>

struct WheelParameter {
 public:
  std::array<float, 2> wheel;  //< wheel position, wheel[0]:left, wheel[1]:right

 public:
  WheelParameter() : wheel({0, 0}) {}
  WheelParameter(const ctrl::Polar& polar, const float rotation_radius)
      : wheel({{
            polar.tra - rotation_radius * polar.rot,
            polar.tra + rotation_radius * polar.rot,
        }}) {}
  const ctrl::Polar toPolar(const float rotation_radius) {
    return ctrl::Polar{
        (wheel[1] + wheel[0]) / 2,
        (wheel[1] - wheel[0]) / 2 / rotation_radius,
    };
  }
};
