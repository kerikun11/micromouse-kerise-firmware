/**
 * @file wheel_position.h
 * @brief ホイールの回転量と並進・回転を相互変換する型を定義
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2022-03-13
 * @copyright Copyright 2022 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/polar.h>

#include <array>

struct WheelPosition : public std::array<float, 2> {
 public:
  WheelPosition() : std::array<float, 2>({0, 0}) {}
  WheelPosition(const std::array<float, 2>& o) : std::array<float, 2>(o) {}
  WheelPosition(const ctrl::Polar& polar, const float rotation_radius)
      : std::array<float, 2>({{
            polar.tra - rotation_radius * polar.rot,
            polar.tra + rotation_radius * polar.rot,
        }}) {}
  ctrl::Polar toPolar(const float rotation_radius) const {
    return ctrl::Polar{
        ((*this)[1] + (*this)[0]) / 2,
        ((*this)[1] - (*this)[0]) / 2 / rotation_radius,
    };
  }
  WheelPosition operator+(const WheelPosition& o) const {
    return {{(*this)[0] + o[0], (*this)[1] + o[1]}};
  }
  WheelPosition operator-(const WheelPosition& o) const {
    return {{(*this)[0] - o[0], (*this)[1] - o[1]}};
  }
  WheelPosition operator*(const float k) const {
    return {{k * (*this)[0], k * (*this)[1]}};
  }
  WheelPosition operator/(const float k) const {
    return {{(*this)[0] / k, (*this)[1] / k}};
  }
};
