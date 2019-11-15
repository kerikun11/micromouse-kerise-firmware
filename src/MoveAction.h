#pragma once

#include "config/model.h"
#include "global.h"

#include "TrajectoryTracker.h"
#include "slalom_shapes.h"
#include "straight.h"

#include "TaskBase.h"
#include <AccelDesigner.h>
#include <cmath>
#include <queue>
#include <vector>

#define SEARCH_WALL_ATTACH_ENABLED 1
#define SEARCH_WALL_CUT_ENABLED 0
#define SEARCH_WALL_FRONT_ENABLED 1
#define SEARCH_WALL_AVOID_ENABLED 1
#define SEARCH_WALL_THETA_ENABLED 1

#define SEARCH_RUN_TASK_PRIORITY 3
#define SEARCH_RUN_STACK_SIZE 8192

#include <RobotBase.h>
using namespace MazeLib;

class MoveAction : TaskBase {
public:
  struct RunParameter {
  public:
#if KERISE_SELECT == 4
    float search_v = 300;
    float curve_gain = 1.0;
    float max_speed = 720;
    float accel = 3600;
#elif KERISE_SELECT == 3
    float search_v = 300;
    float curve_gain = 1.0;
    float max_speed = 600;
    float accel = 3600;
#elif KERISE_SELECT == 5
    float search_v = 240;
    float curve_gain = 1.0;
    float max_speed = 480;
    float accel = 2400;
#endif
    float fan_duty = 0.2f;
    bool diag_enabled = 1;
    bool front_wall_fix_enabled = 1;
    bool wall_avoid_enabled = 1;
    bool unknown_accel = 1;

  public:
    // [1*1.05**i for i in range(0, 4)]: [1.0, 1.05, 1.1025, 1.1576]
    static constexpr float cg_gain = 1.05f;
    // [int(720*1.2**i) for i in range(0, 4)]: [720, 864, 1036, 1244]
    static constexpr float ms_gain = 1.2f;
    // [int(3600*1.05**i) for i in range(0, 4)]: [3600, 3780, 3969, 4167]
    // [int(3600*1.1**i) for i in range(0, 4)]: [3600, 3960, 4356, 4791]
    static constexpr float ac_gain = 1.05f;
  };
#ifndef M_PI
  static constexpr float M_PI = 3.14159265358979323846f;
#endif

public:
  MoveAction() {}
  ~MoveAction() {}
  void enable() {
    deleteTask();
    isRunningFlag = true;
    createTask("MoveAction", SEARCH_RUN_TASK_PRIORITY, SEARCH_RUN_STACK_SIZE);
  }
  void disable() {
    deleteTask();
    sc.disable();
    while (q.size())
      q.pop();
    path = "";
    isRunningFlag = false;
  }
  void set_action(RobotBase::Action action) {
    q.push(action);
    isRunningFlag = true;
  }
  void set_path(std::string path) { this->path = path; }
  bool isRunning() { return isRunningFlag; }

public:
  RunParameter rp_search;
  RunParameter rp_fast;
  bool continue_straight_if_no_front_wall = false;

  bool positionRecovery() {
    sc.enable();
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 2 * M_PI;
    const float angle = 2 * M_PI;
    constexpr int table_size = 180;
    std::array<float, table_size> table;
    for (auto &t : table)
      t = 255;
    AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float back_gain = 10.0f;
    int index = 0;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      if (ad.x(t) > 2 * PI * index / table_size) {
        index++;
        table[index % table_size] = tof.getDistance();
      }
    }
    /* 最小分散を探す */
    float min_var = 999;
    int min_i = 0;
    for (int i = 0; i < table_size; ++i) {
      const int window = table_size / 6;
      int sum = 0;
      for (int j = -window / 2; j < window / 2; ++j)
        sum += table[(table_size + i + j) % table_size];
      int ave = sum / window;
      int var = 0;
      for (int j = -window / 2; j < window / 2; ++j)
        var = std::pow(table[(table_size + i + j) % table_size] - ave, 2);
      if (min_var > var) {
        min_var = var;
        min_i = i;
      }
    }
    sc.enable(); //< reset
    turn(2 * M_PI * min_i / table_size);
    sc.position.clear();
    /* 前壁補正 */
    wall_attach(true);
    turn(wd.distance.side[0] > wd.distance.side[1] ? M_PI / 2 : -M_PI / 2);
    // wall_attach(true);
    delay(50); //< ToFが有効化するのを待つ
    while (1) {
      if (!wd.wall[2])
        break;
      wall_attach();
      turn(-M_PI / 2);
    }
    sc.disable();
    return true;
  }

private:
  std::queue<RobotBase::Action> q;
  bool isRunningFlag = false;
  ctrl::Position offset;
  std::string path;
  bool prev_wall[2];

  static auto round2(auto value, auto div) {
    return floor((value + div / 2) / div) * div;
  }
  static auto saturate(auto src, auto sat) {
    return std::max(std::min(src, sat), -sat);
  }
  bool isAlong() {
    return (int)(std::abs(offset.th) * 180.0f / PI + 1) % 90 < 2;
  }
  bool isDiag() {
    return (int)(std::abs(offset.th) * 180.0f / PI + 45 + 1) % 90 < 2;
  }

  void wall_attach(bool force = false) {
#if SEARCH_WALL_ATTACH_ENABLED
    if ((force && tof.getDistance() < 210) || tof.getDistance() < 90 ||
        (wd.distance.front[0] > 0 && wd.distance.front[1] > 0)) {
      led = 6;
      bz.play(Buzzer::SHORT6);
      tof.disable();
      delay(10);
      TickType_t xLastWakeTime = xTaskGetTickCount();
      WheelParameter wi;
      for (int i = 0; i < 2000; i++) {
        const float Kp = model::wall_attach_gain_Kp;
        const float Ki = model::wall_attach_gain_Ki;
        const float sat_integral = 60.0f;
        const float end = 0.2f;
        WheelParameter wp;
        for (int j = 0; j < 2; ++j) {
          wp.wheel[j] = -wd.distance.front[j];
          wi.wheel[j] += wp.wheel[j] * 0.001f * Ki;
          wi.wheel[j] = saturate(wi.wheel[j], sat_integral);
          wp.wheel[j] = wp.wheel[j] * Kp + wi.wheel[j];
        }
        if (std::pow(wp.wheel[0], 2) + std::pow(wp.wheel[1], 2) +
                std::pow(wi.wheel[0], 2) + std::pow(wi.wheel[1], 2) <
            end)
          break;
        wp.wheel2pole();
        const float sat_tra = 120.0f;   //< [mm/s]
        const float sat_rot = M_PI / 4; //< [rad/s]
        sc.set_target(saturate(wp.tra, sat_tra), saturate(wp.rot, sat_rot));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
      sc.set_target(0, 0);
      sc.position.x = 0;  //< 直進方向の補正
      sc.position.th = 0; //< 回転方向の補正
      if (force)
        sc.position.y = 0; //< 強制の場合は大きくずれうる
      tof.enable();
      led = 0;
    }
#endif
  }
  void wall_avoid(const float remain, const RunParameter &rp, float &int_y) {
#if SEARCH_WALL_AVOID_ENABLED
    /* 有効 かつ 一定速度より大きい かつ 姿勢が整っているときのみ */
    if (!rp.wall_avoid_enabled || sc.est_v.tra < 150.0f ||
        std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    uint8_t led_flags = 0;
    /* 90 [deg] の倍数 */
    if (isAlong()) {
      const float gain = model::wall_avoid_gain;
      const float wall_diff_thr = 50; //< 吸い込まれ防止
      if (wd.wall[0] && std::abs(wd.diff.side[0]) < wall_diff_thr) {
        sc.position.y += wd.distance.side[0] * gain;
        int_y += wd.distance.side[0];
        led_flags |= 8;
      }
      if (wd.wall[1] && std::abs(wd.diff.side[1]) < wall_diff_thr) {
        sc.position.y -= wd.distance.side[1] * gain;
        int_y -= wd.distance.side[1];
        led_flags |= 1;
      }
#if SEARCH_WALL_THETA_ENABLED
      /* 機体姿勢の補正 */
      sc.position.th += int_y * 0.00000001f;
#endif
    }
    /* 45 [deg] の倍数 */
    if (isDiag() && remain > field::SegWidthFull / 3) {
      const float shift = 0.06f;
      const float threashold = -50;
      if (wd.distance.front[0] > threashold) {
        sc.position.y += shift;
        led_flags |= 4;
      }
      if (wd.distance.front[1] > threashold) {
        sc.position.y -= shift;
        led_flags |= 2;
      }
    }
    led = led_flags;
#endif
  }
  void wall_cut(const float velocity) {
#if SEARCH_WALL_CUT_ENABLED
    if (!wallCutFlag)
      return;
    if (!diag_enabled)
      return;
    if (velocity < 120)
      return;
    /* 曲線なら前半しか使わない */
    if (std::abs(sc.position.th) > M_PI * 0.1f)
      return;
    for (int i = 0; i < 2; i++) {
      if (prev_wall[i] && !wd.wall[i]) {
        /* 90 [deg] の倍数 */
        if (isAlong()) {
          Position prev = sc.position;
          Position fix = sc.position.rotate(-origin.th);
          const float wall_cut_offset = -15; /*< from wall_cut_line */
          fix.x = round2(fix.x, field::SegWidthFull) + wall_cut_offset;
          fix = fix.rotate(origin.th);
          const float diff =
              fix.rotate(-origin.th).x - prev.rotate(-origin.th).x;
          if (-30 < diff && diff < 5) {
            sc.position = fix;
            bz.play(Buzzer::SHORT6);
          }
        }
        /* 45 [deg] + 90 [deg] の倍数 */
        if (isDiag()) {
          Position prev = sc.position;
          Position fix = sc.position.rotate(-origin.th);
          const float extra = 31;
          if (i == 0) {
            fix.x = round2(fix.x - extra - field::SegWidthDiag / 2,
                           field::SegWidthDiag) +
                    field::SegWidthDiag / 2 + extra;
          } else {
            fix.x = round2(fix.x - extra, field::SegWidthDiag) + extra;
          }
          fix = fix.rotate(origin.th);
          if (fabs(prev.rotate(-origin.th).x - fix.rotate(-origin.th).x) <
              10.0f) {
            // sc.position = fix;
            bz.play(Buzzer::SHORT6);
          }
        }
      }
      prev_wall[i] = wd.wall[i];
    }
#endif
  }
  void wall_front_fix(const RunParameter rp, const float dist_to_wall) {
#if SEARCH_WALL_FRONT_ENABLED
    if (rp.front_wall_fix_enabled && tof.isValid()) {
      float value =
          tof.getDistance() - (tof.passedTimeMs() + 5) / 1000.0f * sc.ref_v.tra;
      // value = value * std::cos(sc.position.th); /*< 機体姿勢考慮 */
      float fixed_x = dist_to_wall - value + 6; /*< 大きく:壁に近く */
      if (-30 < fixed_x && fixed_x < 30) {
        // fixed_x = std::max(fixed_x, 5.0f);
        fixed_x /= 2; //*< 補正率 2: 50%
        sc.fix_position(ctrl::Position(fixed_x - sc.position.x, 0, 0));
        bz.play(Buzzer::SHORT7);
      }
    }
#endif
  }
  void turn(const float angle) {
    static constexpr float m_dddth = 4800 * M_PI;
    static constexpr float m_ddth = 48 * M_PI;
    static constexpr float m_dth = 4 * M_PI;
    AccelDesigner ad(m_dddth, m_ddth, 0, m_dth, 0, angle);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float back_gain = 10.0f;
    for (float t = 0; t < ad.t_end(); t += 0.001f) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      sc.set_target(-delta * back_gain, ad.v(t), 0, ad.a(t));
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    /* 確実に目標角度に持っていく処理 */
    float int_error = 0;
    while (1) {
      float delta = sc.position.x * std::cos(-sc.position.th) -
                    sc.position.y * std::sin(-sc.position.th);
      const float Kp = 20.0f;
      const float Ki = 10.0f;
      const float error = angle - sc.position.th;
      int_error += error * 0.001f;
      sc.set_target(-delta * back_gain, Kp * error + Ki * int_error);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.05f * PI)
        break;
    }
    sc.set_target(0, 0);
    /* 移動した量だけ位置を更新 */
    const auto net = ctrl::Position(0, 0, angle);
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void straight_x(const float distance, const float v_max, const float v_end,
                  const RunParameter &rp) {
    if (distance - sc.position.x > 0) {
      const float jerk = 240000;
      const float v_start = sc.ref_v.tra;
      ctrl::TrajectoryTracker tt{model::TrajectoryTrackerGain};
      ctrl::State ref_s;
      ctrl::straight::Trajectory trajectory;
      /* start */
      trajectory.reset(jerk, rp.accel, v_start, v_max, v_end,
                       distance - sc.position.x, sc.position.x);
      tt.reset(v_start);
#if SEARCH_WALL_THETA_ENABLED
      float int_y = 0;
#endif
      TickType_t xLastWakeTime = xTaskGetTickCount();
      for (float t = 0; true; t += 0.001f) {
        /* 終了条件 */
        const float remain = distance - sc.position.x;
        if (remain < 0 || t > trajectory.t_end() + 0.1f)
          break;
        /* 衝突被害軽減ブレーキ(AEBS) */
        if (remain > field::SegWidthFull && tof.isValid() &&
            tof.getDistance() < field::SegWidthFull)
          wall_stop();
        /* 軌道追従 */
        trajectory.update(ref_s, t);
        const auto ref = tt.update(sc.position, sc.est_v, sc.est_a, ref_s);
        sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
        /* 壁制御 */
        wall_avoid(remain, rp, int_y);
        wall_cut(ref.v);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      }
    }
    /* 移動した量だけ位置を更新 */
    sc.position.x -= distance;
    offset += ctrl::Position(distance, 0, 0).rotate(offset.th);
  }
  void trace(slalom::Trajectory &trajectory, const float velocity,
             const RunParameter &rp) {
    const float Ts = 0.001f;
    ctrl::TrajectoryTracker tt{model::TrajectoryTrackerGain};
    ctrl::State s;
    /* start */
    tt.reset(velocity);
    trajectory.reset(velocity);
    TickType_t xLastWakeTime = xTaskGetTickCount();
#if SEARCH_WALL_FRONT_ENABLED
    float front_fix_x = 0;
#endif
    s.q.x = sc.position.x; /*< 既に移動した分を反映 */
    for (float t = 0; t < trajectory.t_end(); t += Ts) {
      /* 打ち切り条件を追加！！！ */
      trajectory.update(s, t, Ts);
      const auto ref = tt.update(sc.position, sc.est_v, sc.est_a, s);
      sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
      float int_y = 0;
      wall_avoid(0, rp, int_y);
      wall_cut(ref.v);
#if SEARCH_WALL_FRONT_ENABLED
      /* V90ターン中の前壁補正 */
      if (rp.front_wall_fix_enabled && tof.isValid() &&
          std::abs(t - trajectory.t_end() / 2) < 0.0005001f &&
          (trajectory.getShape() == SS_FLV90 ||
           trajectory.getShape() == SS_FRV90)) {
        const float tof_value =
            tof.getDistance() - tof.passedTimeMs() / 1000.0f * velocity;
        const float fixed_x =
            field::SegWidthFull - tof_value + 4; /*< 要調整, 大きく:前壁近く*/
        if (-20 < fixed_x && fixed_x < 20) {
          front_fix_x = fixed_x;
          bz.play(Buzzer::SHORT7);
        }
      }
#endif
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
#if SEARCH_WALL_FRONT_ENABLED
    /* V90ターン中の前壁補正 */
    if (rp.front_wall_fix_enabled && front_fix_x != 0)
      sc.fix_position(ctrl::Position(front_fix_x, 0, 0)
                          .rotate(trajectory.get_net_curve().th / 2));
#endif
    sc.set_target(velocity, 0);
    /* 移動した量だけ位置を更新 */
    const auto &net = trajectory.get_net_curve();
    sc.position = (sc.position - net).rotate(-net.th);
    offset += net.rotate(offset.th);
  }
  void SlalomProcess(const slalom::Shape &shape, float &straight,
                     const bool reverse, const RunParameter &rp) {
    slalom::Trajectory st(shape);
    const float velocity = st.get_v_ref() * rp.curve_gain;
    straight += !reverse ? st.get_straight_prev() : st.get_straight_post();
    /* ターン前の直線を消化 */
    if (straight > 1.0f) {
      straight_x(straight, rp.max_speed, velocity, rp);
      straight = 0;
    }
    /* 直線前壁補正 */
    if (isAlong()) {
      if (rp.diag_enabled && reverse == false) {
        wall_front_fix(rp, field::SegWidthFull + field::SegWidthFull / 2 -
                               st.get_straight_prev());
        wall_front_fix(rp, 2 * field::SegWidthFull + field::SegWidthFull / 2 -
                               st.get_straight_prev());
      }
      if (shape == SS_FLS90 || shape == SS_FRS90) {
        wall_front_fix(rp, field::SegWidthFull - st.get_straight_prev());
        wall_front_fix(rp, 2 * field::SegWidthFull - st.get_straight_prev());
      }
    }
    /* スラローム */
    trace(st, velocity, rp);
    straight += reverse ? st.get_straight_prev() : st.get_straight_post();
  }
  void put_back() {
    const int max_v = 150;
    const float th_gain = 100.0f;
    for (int i = 0; i < max_v; i++) {
      sc.set_target(-i, -sc.position.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (int i = 0; i < 100; i++) {
      sc.set_target(-max_v, -sc.position.th * th_gain);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    sc.disable();
    mt.drive(-0.1f, -0.1f);
    vTaskDelay(pdMS_TO_TICKS(200));
    mt.drive(-0.2f, -0.2f);
    vTaskDelay(pdMS_TO_TICKS(200));
    sc.enable(true);
  }
  void uturn() {
    if (wd.distance.side[0] < wd.distance.side[1]) {
      wall_attach();
      turn(-M_PI / 2);
      wall_attach();
      turn(-M_PI / 2);
    } else {
      wall_attach();
      turn(M_PI / 2);
      wall_attach();
      turn(M_PI / 2);
    }
  }
  void wall_stop() {
    bz.play(Buzzer::AEBS);
    float v = sc.est_v.tra;
    while (v > 0) {
      sc.set_target(v, 0);
      v -= 12;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    sc.disable();
    mt.emergencyStop();
    vTaskDelay(portMAX_DELAY);
  }
  void start_init() {
    wall_attach();
    turn(M_PI / 2);
    wall_attach();
    turn(M_PI / 2);
    put_back();
    mt.free();
    isRunningFlag = false;
    vTaskDelay(portMAX_DELAY);
  }
  void queue_wait_decel() {
    /* Actionがキューされるまで直進で待つ */
    float v = sc.ref_v.tra;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (int i = 0; q.empty(); ++i) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      if (v > 0)
        v -= 6;
      xLastWakeTime = xTaskGetTickCount();
      const auto cur = sc.position;
      float th = atan2f(-cur.y, (2 + 20 * v / 240)) - cur.th;
      sc.set_target(v, 40 * th);
      if (i == 60)
        bz.play(Buzzer::MAZE_BACKUP);
    }
    sc.position.y = 0; /**< 横のステップ変化を抑制 */
  }
  void task() override {
    if (q.size())
      search_run_task();
    else
      fast_run_task();
    vTaskDelay(portMAX_DELAY);
  }
  void search_run_known(const RunParameter &rp) {
    std::string path;
    while (1) {
      if (q.empty())
        break;
      const auto action = q.front();
      if (action == MazeLib::RobotBase::Action::ST_HALF ||
          action == MazeLib::RobotBase::Action::ST_FULL ||
          action == MazeLib::RobotBase::Action::TURN_L ||
          action == MazeLib::RobotBase::Action::TURN_R) {
        path += action;
        q.pop();
      } else {
        break;
      }
    }
    if (path.size()) {
      /* 既知区間斜めパターンに変換 */
      path =
          MazeLib::RobotBase::pathConvertSearchToKnown(path, rp.diag_enabled);
      /* 既知区間走行 */
      float straight = 0;
      for (int path_index = 0; path_index < path.length(); path_index++) {
        const auto action =
            static_cast<const MazeLib::RobotBase::FastAction>(path[path_index]);
        fast_run_switch(action, straight, rp);
      }
      /* 最後の直線を消化 */
      if (straight > 0.1f) {
        straight_x(straight, rp.max_speed, rp.search_v, rp);
        straight = 0;
      }
    }
  }
  void search_run_task() {
    const auto &rp = rp_search;
    /* スタート */
    offset = ctrl::Position(field::SegWidthFull / 2 + model::CenterShift,
                            field::SegWidthFull / 2, 0);
    sc.enable();
    while (1) {
      if (q.empty())
        isRunningFlag = false;
      /* Actionがキューされるまで直進で待つ */
      queue_wait_decel();
      /* 既知区間走行 */
      if (q.size() >= 2)
        search_run_known(rp);
      /* 探索走行 */
      if (q.size()) {
        const auto action = q.front();
        q.pop();
        search_run_switch(action, rp);
      }
    }
  }
  void search_run_switch(const RobotBase::Action action,
                         const RunParameter &rp) {
    const float velocity = rp.search_v;
    const bool no_front_wall =
        !tof.isValid() ||
        tof.getDistance() > field::SegWidthFull * 2 + field::SegWidthFull / 3;
    const auto v_end = (rp.unknown_accel &&
                        continue_straight_if_no_front_wall && no_front_wall)
                           ? 600
                           : velocity;
    switch (action) {
    case RobotBase::Action::START_STEP:
      imu.angle = 0;
      sc.position.clear();
      sc.position.x = model::TailLength + field::WallThickness / 2;
      offset = ctrl::Position(field::SegWidthFull / 2, 0, M_PI / 2);
      straight_x(field::SegWidthFull, v_end, v_end, rp);
      break;
    case RobotBase::Action::START_INIT:
      start_init();
      break;
    case RobotBase::Action::ST_FULL:
      if (tof.getDistance() < field::SegWidthFull)
        wall_stop();
#if SEARCH_WALL_FRONT_ENABLED
      wall_front_fix(rp, 2 * field::SegWidthFull);
#endif
      straight_x(field::SegWidthFull, v_end, v_end, rp);
      break;
    case RobotBase::Action::ST_HALF:
      straight_x(field::SegWidthFull / 2 - model::CenterShift, velocity,
                 velocity, rp);
      break;
    case RobotBase::Action::TURN_L: {
      wall_front_fix(rp, field::SegWidthFull);
#if SEARCH_WALL_FRONT_ENABLED
      wall_front_fix(rp, 2 * field::SegWidthFull);
#endif
      slalom::Trajectory st(SS_SL90);
      straight_x(st.get_straight_prev(), velocity, velocity, rp);
      if (wd.wall[0])
        wall_stop();
      trace(st, velocity, rp);
      straight_x(st.get_straight_post(), velocity, velocity, rp);
      break;
    }
    case RobotBase::Action::TURN_R: {
      wall_front_fix(rp, field::SegWidthFull);
#if SEARCH_WALL_FRONT_ENABLED
      wall_front_fix(rp, 2 * field::SegWidthFull);
#endif
      slalom::Trajectory st(SS_SR90);
      straight_x(st.get_straight_prev(), velocity, velocity, rp);
      if (wd.wall[1])
        wall_stop();
      trace(st, velocity, rp);
      straight_x(st.get_straight_post(), velocity, velocity, rp);
      break;
    }
    case RobotBase::Action::ROTATE_180:
      uturn();
      break;
    case RobotBase::Action::ST_HALF_STOP:
      wall_front_fix(rp, field::SegWidthFull);
#if SEARCH_WALL_FRONT_ENABLED
      wall_front_fix(rp, 2 * field::SegWidthFull);
#endif
      straight_x(field::SegWidthFull / 2 + model::CenterShift, velocity, 0, rp);
      turn(0); //*< 姿勢を整える */
      break;
    }
  }
  void fast_run_task() {
    /* パラメータを取得 */
    const auto &rp = rp_fast;
    /* 最短走行用にパターンを置換 */
    path = MazeLib::RobotBase::pathConvertSearchToFast(path, rp.diag_enabled);
    /* キャリブレーション */
    bz.play(Buzzer::CALIBRATION);
    imu.calibration();
    /* 壁に背中を確実につける */
    mt.drive(-0.25f, -0.25f);
    delay(200);
    mt.free();
    /* 走行開始 */
    fan.drive(rp.fan_duty);
    delay(500);  //< ファンの回転数が一定なるのを待つ
    sc.enable(); //< 速度コントローラ始動
    /* 初期位置を設定 */
    offset =
        ctrl::Position(field::SegWidthFull / 2,
                       model::TailLength + field::WallThickness / 2, M_PI / 2);
    sc.position.clear();
    /* 最初の直線を追加 */
    float straight =
        field::SegWidthFull / 2 - model::TailLength - field::WallThickness / 2;
    /* 走行 */
    for (int path_index = 0; path_index < path.length(); path_index++) {
      const auto action =
          static_cast<const MazeLib::RobotBase::FastAction>(path[path_index]);
      fast_run_switch(action, straight, rp);
    }
    /* 最後の直線を消化 */
    if (straight > 0.1f) {
      straight_x(straight, rp.max_speed, 0, rp);
      straight = 0;
    }
    sc.set_target(0, 0);
    fan.drive(0);
    delay(200);
    sc.disable();
    bz.play(Buzzer::COMPLETE);
    path = "";
    isRunningFlag = false;
    vTaskDelay(portMAX_DELAY);
  }
  void fast_run_switch(const MazeLib::RobotBase::FastAction action,
                       float &straight, const RunParameter &rp) {
    switch (action) {
    case MazeLib::RobotBase::FastAction::FL45:
      SlalomProcess(SS_FL45, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45:
      SlalomProcess(SS_FR45, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL45P:
      SlalomProcess(SS_FL45, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR45P:
      SlalomProcess(SS_FR45, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLV90:
      SlalomProcess(SS_FLV90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRV90:
      SlalomProcess(SS_FRV90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FLS90:
      SlalomProcess(SS_FLS90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FRS90:
      SlalomProcess(SS_FRS90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL90:
      SlalomProcess(SS_FL90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR90:
      SlalomProcess(SS_FR90, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135:
      SlalomProcess(SS_FL135, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135:
      SlalomProcess(SS_FR135, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL135P:
      SlalomProcess(SS_FL135, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR135P:
      SlalomProcess(SS_FR135, straight, true, rp);
      break;
    case MazeLib::RobotBase::FastAction::FL180:
      SlalomProcess(SS_FL180, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::FR180:
      SlalomProcess(SS_FR180, straight, false, rp);
      break;
    case MazeLib::RobotBase::FastAction::F_ST_FULL:
      straight += field::SegWidthFull;
      break;
    case MazeLib::RobotBase::FastAction::F_ST_HALF:
      straight += field::SegWidthFull / 2;
      break;
    case MazeLib::RobotBase::FastAction::F_ST_DIAG:
      straight += field::SegWidthDiag / 2;
      break;
    default:
      break;
    }
  }
};