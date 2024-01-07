/**
 * @file motion_parameter.h
 * @brief Structure of (x, y, z) Motion Parameter
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2024-01-07
 * @copyright Copyright 2024 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

struct MotionParameter {
  float x, y, z;
  MotionParameter(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  MotionParameter operator+(const MotionParameter& obj) const {
    return MotionParameter(x + obj.x, y + obj.y, z + obj.z);
  }
  MotionParameter operator-(const MotionParameter& obj) const {
    return MotionParameter(x - obj.x, y - obj.y, z - obj.z);
  }
  MotionParameter operator*(const float mul) const {
    return MotionParameter(x * mul, y * mul, z * mul);
  }
  MotionParameter operator/(const float div) const {
    return MotionParameter(x / div, y / div, z / div);
  }
  MotionParameter& operator+=(const MotionParameter& obj) {
    return x += obj.x, y += obj.y, z += obj.z, *this;
  }
  MotionParameter& operator-=(const MotionParameter& obj) {
    return x -= obj.x, y -= obj.y, z -= obj.z, *this;
  }
};
