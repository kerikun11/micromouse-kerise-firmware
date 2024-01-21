/**
 * @file model.h
 * @brief マイクロマウスのモデル
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-20
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/feedback_controller.h>
#include <ctrl/trajectory_tracker.h>

#include "config/config.h"
#include "config/field.h"

namespace model {

#if KERISE_SELECT == 6
/* KERISE v6 */
static constexpr uint64_t MAC_ID = 0xECDC'E8AC'CD98;  //< eFuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 33.5f / 2;
static constexpr float GearRatio = 1.0f;
static constexpr float WheelDiameter = 12.72f;  //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 0.0f;
static constexpr float TailLength = 18.0f;
static constexpr float IMURotationRadius = 12.0f;
/* ToF */
static constexpr float tof_raw_range_90 = 75;
static constexpr float tof_raw_range_180 = 160;
static constexpr float wall_fix_offset = -5;  //< 大きく: 前壁に近く
static constexpr uint8_t vl6180x_max_convergence_time = 49;  //< ms
/* Reflector */
static constexpr int ui_thr_ref_front = 1200;
static constexpr int ui_thr_ref_side = 1200;
static constexpr float wall_front_attach_gain = 30.0f;
static constexpr float wall_front_attach_end = 0.4f;
static constexpr float wall_avoid_alpha = 0.05f;
static constexpr float wall_fix_theta_gain = 1e-8f;
static constexpr float wall_comb_threshold = 54;
static constexpr float ref_max_length_mm = 45;  //< リフレクタの最大計測距離
static constexpr float ref_saturation_value = 3100;  //< リフレクタの飽和値
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(4000, 80),
        .T1 = ctrl::Polar(0.14, 0.08),
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0, 0.0),
        .Ki = ctrl::Polar(0.0, 0.0),
        .Kd = ctrl::Polar(0.0, 0.0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(1.0f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 8.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-2f,
};

#elif KERISE_SELECT == 5
/* KERISE v5 */
static constexpr uint64_t MAC_ID = 0xD866'5A1D'A0D8;  //< eFuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 29.0f / 2;
static constexpr float GearRatio = 1.0f;
static constexpr float WheelDiameter = 12.72f;  //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 0.0f;
static constexpr float TailLength = 13.0f;
static constexpr float IMURotationRadius = 0.0f;  //< N/A
/* ToF */
static constexpr float tof_raw_range_90 = 75;
static constexpr float tof_raw_range_180 = 160;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
static constexpr uint8_t vl6180x_max_convergence_time = 49;  //< ms
/* Reflector */
static constexpr int ui_thr_ref_front = 2400;
static constexpr int ui_thr_ref_side = 2400;
static constexpr float wall_front_attach_gain = 30.0f;
static constexpr float wall_front_attach_end = 0.4f;
static constexpr float wall_avoid_alpha = 0.05f;
static constexpr float wall_fix_theta_gain = 1e-8f;
static constexpr float wall_comb_threshold = 54;
static constexpr float ref_max_length_mm = 45;  //< リフレクタの最大計測距離
static constexpr float ref_saturation_value = 3100;  //< リフレクタの飽和値
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(4000, 80),
        .T1 = ctrl::Polar(0.14, 0.08),
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.001, 0.07),
        .Ki = ctrl::Polar(0.04, 4.0),
        .Kd = ctrl::Polar(0.0, 0.0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(1.0f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 8.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-2f,
};

#elif KERISE_SELECT == 4
/* Original KERISE v4 */
static constexpr uint64_t MAC_ID = 0x080C'401D'A0D8;  //< eFuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.67f;  //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 8.0f;
static constexpr float TailLength = 16.4f;
static constexpr float IMURotationRadius = 10.0f;
/* ToF */
static constexpr float tof_raw_range_90 = 69;
static constexpr float tof_raw_range_180 = 154;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
static constexpr uint8_t vl6180x_max_convergence_time = 49;  //< ms
/* Reflector */
static constexpr int ui_thr_ref_front = 2400;
static constexpr int ui_thr_ref_side = 2400;
static constexpr float wall_front_attach_gain = 30.0f;
static constexpr float wall_front_attach_end = 0.1f;
static constexpr float wall_avoid_alpha = 0.05f;
static constexpr float wall_fix_theta_gain = 1e-7f;
static constexpr float wall_comb_threshold = 54;
static constexpr float ref_max_length_mm = 45;  //< リフレクタの最大計測距離
static constexpr float ref_saturation_value = 3100;  //< リフレクタの飽和値
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(5789, 1000),  //< 大きく：FF定常成分小さく
        .T1 = ctrl::Polar(0.12f, 0.48f),  //< 大きく：FF立ち上がり成分大きく
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0008f, 0.15f),
        .Ki = ctrl::Polar(0.1f, 6.0f),
        .Kd = ctrl::Polar(0, 0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 12.0f,
    .low_zeta = 0.8f,
    .low_b = 1e-3f,
};

#elif KERISE_SELECT == 3
/* KERISE v4 Copy */
static constexpr uint64_t MAC_ID = 0x807F'631D'A0D8;  //< eFuse 48 bit MAC
/* Machine Size Parameter */
static constexpr float RotationRadius = 15.0f;
static constexpr float GearRatio = (12.0f / 38.0f);
static constexpr float WheelDiameter = 12.96f;  //< 径を大きく：進行距離を短く
static constexpr float CenterOffsetY = 8.0f;
static constexpr float TailLength = 16.4f;
static constexpr float IMURotationRadius = 10.0f;
/* ToF */
static constexpr float tof_raw_range_90 = 69;
static constexpr float tof_raw_range_180 = 154;
static constexpr float wall_fix_offset = -5; /*< 大きく: 前壁に近く */
static constexpr uint8_t vl6180x_max_convergence_time = 32;  //< ms
/* Reflector */
static constexpr int ui_thr_ref_front = 2400;
static constexpr int ui_thr_ref_side = 2400;
static constexpr float wall_front_attach_gain = 30.0f;
static constexpr float wall_front_attach_end = 0.1f;
static constexpr float wall_avoid_alpha = 0.05f;
static constexpr float wall_fix_theta_gain = 1e-7f;
static constexpr float wall_comb_threshold = 80;
static constexpr float ref_max_length_mm = 45;  //< リフレクタの最大計測距離
static constexpr float ref_saturation_value = 3100;  //< リフレクタの飽和値
/* Model */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(5400, 100),  //< 大きく：FF定常成分小さく
        .T1 = ctrl::Polar(0.18f, 0.12f),  //< 大きく：FF立ち上がり成分大きく
};
static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0004f, 0.03f),
        .Ki = ctrl::Polar(0.1f, 3.0f),
        .Kd = ctrl::Polar(0, 0),
};
static constexpr float turn_back_gain = 10.0f;
/* Velocity Estimation IIR Filter gain */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(0.2f, 1.0f);
/* Trajectory Tracking Gain */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
#if 1
    .zeta = 0.6f,
    .omega_n = 6.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-3f,
#else
    .zeta = 0,
    .omega_n = 0,
    .low_zeta = 0,
    .low_b = 0,
#endif
};

#endif

static constexpr float kBatteryVoltageDividerRatio = 1.0f / 2.0f;

};  // namespace model
