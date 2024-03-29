/**
 * @file maze_robot.h
 * @brief Maze Robot
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <MazeLib/RobotBase.h>
#include <esp_timer.h>

#include "agents/move_action.h"
#include "config/model.h"
#include "hardware/hardware.h"
#include "supporters/supporters.h"

using namespace MazeLib;

/* 大会前には必ず 32 にする */
#define MAZE_ROBOT_TIMEOUT_SELECT 1
#define MAZE_ROBOT_GOAL_SELECT 1

/* ゴール座標 */
#define MAZE_GOAL_32                                          \
  {                                                           \
    MazeLib::Position(16, 16), MazeLib::Position(16, 17),     \
        MazeLib::Position(16, 18), MazeLib::Position(17, 16), \
        MazeLib::Position(17, 17), MazeLib::Position(17, 18), \
        MazeLib::Position(18, 16), MazeLib::Position(18, 17), \
        MazeLib::Position(18, 18),                            \
  }
#define MAZE_GOAL_16 \
  { MazeLib::Position(7, 7), }
#define MAZE_GOAL_TEST \
  { MazeLib::Position(1, 0), }
#if MAZE_ROBOT_GOAL_SELECT == 32
#define MAZE_GOAL MAZE_GOAL_32
#elif MAZE_ROBOT_GOAL_SELECT == 16
#define MAZE_GOAL MAZE_GOAL_16
#elif MAZE_ROBOT_GOAL_SELECT == 1
#define MAZE_GOAL MAZE_GOAL_TEST
#endif

/* ログマクロの定義 (MR_LOG_LEVEL は config.h で定義) */
#if MR_LOG_LEVEL >= 1
#define MR_LOGE APP_LOGE
#else
#define MR_LOGE(s)
#endif
#if MR_LOG_LEVEL >= 2
#define MR_LOGW APP_LOGW
#else
#define MR_LOGW(s)
#endif
#if MR_LOG_LEVEL >= 3
#define MR_LOGI APP_LOGI
#else
#define MR_LOGI(s)
#endif
#if MR_LOG_LEVEL >= 4
#define MR_LOGD APP_LOGD
#else
#define MR_LOGD(s)
#endif

class MazeRobot : public RobotBase {
 private:
  static constexpr int MAZE_ROBOT_TASK_PRIORITY = 2;
  static constexpr int MAZE_ROBOT_STACK_SIZE = 8192;
  static constexpr auto MAZE_SAVE_PATH = "/spiffs/maze_backup.bin";
  static constexpr auto STATE_SAVE_PATH = "/spiffs/maze_state.bin";

 private:
  struct State {
   public:
    static constexpr int max_try_count = 5;

   public:
    State() { set_timeout(MAZE_ROBOT_TIMEOUT_SELECT); }
    void set_timeout(int size) {
      switch (size) {
        case 32:
          competition_limit_time_s = 60 * 10 - 120;
          expected_fast_run_time_s = 60;  //< 探索は4分で打ち切り
          break;
        case 16:
          competition_limit_time_s = 60 * 5 - 60;
          expected_fast_run_time_s = 30;  //< 探索は2分で打ち切り
          break;
        case 1:
        case 8:
        case 9:
          competition_limit_time_s = 60 * 3 - 30;
          expected_fast_run_time_s = 20;  //< 探索は1分で打ち切り
          break;
      }
    }
    bool save(const char* filepath = STATE_SAVE_PATH) {
      backup_time_s = get_elapsed_time_s();
      std::ofstream of(filepath, std::ios::binary);
      if (of.fail()) {
        APP_LOGE("failed to open file: %s", filepath);
        return false;
      }
      of.write((const char*)this, sizeof(*this));
      return true;
    }
    bool restore(const char* filepath = STATE_SAVE_PATH) {
      std::ifstream f(filepath, std::ios::binary);
      if (f.fail()) {
        APP_LOGE("failed to open file: %s", filepath);
        return false;
      }
      f.read((char*)this, sizeof(*this));
      offset_time_s = backup_time_s;
      running_parameter = 0;
      return true;
    }
    /* updaters */
    void start_search_run() {
      try_count++;
      has_reached_goal = false;
      is_fast_run = false;
      save();
    }
    void start_fast_run() {
      try_count++;
      has_reached_goal = false;
      is_fast_run = true;
      save();
    }
    void end_fast_run(bool result) {
      if (result) {
        is_fast_run = false;
        succeeded_parameter = running_parameter;
      }
      save();
    }
    void set_reached_goal() { has_reached_goal = true; }
    /* checkers */
    bool get_fast_run_failed() { return is_fast_run; }
    bool get_has_reached_goal() { return has_reached_goal; }
    bool get_at_least_fast_run_succeeded() {
      return succeeded_parameter > INT_MIN;
    }
    bool no_more_time() {
      int time_limit_s = competition_limit_time_s -
                         get_try_count_remain() * expected_fast_run_time_s;
      return get_elapsed_time_s() > time_limit_s;
    }
    int get_try_count_remain() const { return max_try_count - try_count; }
    int get_try_count() const { return try_count; }
    int running_parameter = 0; /**< 走行パラメータ */

   private:
    int competition_limit_time_s;
    int expected_fast_run_time_s;
    int backup_time_s = 0; /**< 最後にバックアップした時刻 */
    int offset_time_s = 0; /**< 追加時間 */
    int try_count = 0;     /**< 走行回数 */
    int succeeded_parameter = INT_MIN; /**< 成功パラメータ */
    bool has_reached_goal = false; /**< ゴール区画にたどり着いたか */
    bool is_fast_run = false;      /**< 最短走行の途中かどうか */

    int get_elapsed_time_s() const {
      return offset_time_s + esp_timer_get_time() / 1000000;
    }
  };

 private:
  hardware::Hardware* hw;
  supporters::Supporters* sp;
  MoveAction* ma;

 public:
  MazeRobot(hardware::Hardware* hw, supporters::Supporters* sp, MoveAction* ma)
      : hw(hw), sp(sp), ma(ma) {
    replaceGoals(MAZE_GOAL);
  }
  void reset() {
    RobotBase::reset();
    maze.backupWallRecordsToFile(MAZE_SAVE_PATH, true);
    state = State();
    state.save();
  }
  bool backup() {
    state.save();
    return maze.backupWallRecordsToFile(MAZE_SAVE_PATH);
  }
  bool restore() {
    state.restore();
    return maze.restoreWallRecordsFromFile(MAZE_SAVE_PATH);
  }
  void setTimeout(int timeout_select) { state.set_timeout(timeout_select); }
  bool autoRun(const bool isAutoParamSelect = false,
               const bool isPositionIdentificationAtFirst = false) {
    /* 自己位置復帰走行: 任意 -> 復帰 -> ゴール -> スタート */
    MR_LOGD("");
    if (isPositionIdentificationAtFirst) {
      MR_LOGD("");
      if (!auto_pi_run()) return false;  //< 回収された
    }
    /* 探索走行: スタート -> ゴール -> スタート */
    MR_LOGD("");
    if (!calcShortestDirections(true)) {
      MR_LOGD("");
      if (!auto_search_run()) {
        MR_LOGW("");
        return false;  //< 回収された or 探索不能迷路
      }
    }
    /* 最短走行ループ: スタート -> ゴール -> スタート */
    while (1) {
      /* 5走終了 */
      MR_LOGD("");
      if (isAutoParamSelect && state.get_try_count_remain() <= 0) {
        hw->bz->play(hardware::Buzzer::COMPLETE);
        MR_LOGD("");
        if (sp->ui->waitForPickup(2000)) return false;
      }
      /* 回収待ち */
      if (sp->ui->waitForPickup()) return false;
      /* 走行パラメータ選択 */
      if (isAutoParamSelect) auto_parameter_select();
      /* 最短走行 */
      MR_LOGD("");
      if (!auto_fast_run()) {
        if (!hw->mt->is_emergency())
          return false; /*< クラッシュではない場合キャンセル */
        MR_LOGW("");
        /* クラッシュ後の場合、自動復帰 */
        ma->emergency_release();
        // state.save(), esp_restart(); //< 緊急ループ対策
        /* 自動復帰 */
        if (!auto_pi_run()) return false;  //< 回収された
      }
    }
  }
  void print() const {
    for (const auto& wl : maze.getWallRecords()) std::cout << wl << std::endl;
    maze.print();
  }
  void setGoals(const Positions& goal) { replaceGoals(goal); }
  const State& getState() const { return state; }

 private:
  State state;
  bool prevIsForceGoingToGoal = false; /*< ゴール判定用 */

 protected:
  /* override virtual functions */
  void waitForEndAction() override {
    // vTaskDelay(pdMS_TO_TICKS(300));  //< 計算処理に時間がかかる場合を模擬
    ma->waitForEndAction();
    if (hw->mt->is_emergency()) setBreakFlag();
  }
  void queueAction(const RobotBase::SearchAction action) override {
    ma->enqueue_action(action);
  }
  void calibration() override { ma->calibration(); }
  void startDequeue() override { ma->enable(MoveAction::TaskActionSearchRun); }
  void stopDequeue() override { ma->disable(); }
  void backupMazeToFlash() override { backup(); }
  void discrepancyWithKnownWall() override {
    hw->bz->play(hardware::Buzzer::ERROR);
    MR_LOGW("discrepancy! pose: %s", getCurrentPose().toString());
  }
  void senseWalls(bool& left, bool& front, bool& right) override {
    left = ma->getSensedWalls().left;
    right = ma->getSensedWalls().right;
    front = ma->getSensedWalls().front;
  }
  void calcNextDirectionsPreCallback() override {
    /* ゴール判定用フラグ */
    prevIsForceGoingToGoal = calcData.isForceGoingToGoal;
    /* ウルトラマンタイマー */
    if (!calcData.isForceBackToStart && state.no_more_time()) {
      setForceBackToStart();
      hw->bz->play(hardware::Buzzer::TIMEOUT);
      MR_LOGI("timeout");
    }
  }
  void calcNextDirectionsPostCallback(
      SearchAlgorithm::State oldState,
      SearchAlgorithm::State newState) override {
    MR_LOGI("%s %s --> %s", getCurrentPose().toString(),
            SearchAlgorithm::getStateString(oldState),
            SearchAlgorithm::getStateString(newState));
    /* 未知区間加速の設定 */
    ma->set_unknown_accel_flag(getUnknownAccelFlag());
    /* ゴール判定 */
    if (prevIsForceGoingToGoal && !calcData.isForceGoingToGoal) {
      hw->bz->play(hardware::Buzzer::CONFIRM);
    }
    if (!calcData.isForceGoingToGoal) state.set_reached_goal();
    /* 探索情報のお知らせ */
    if (newState == oldState) return;
    if (oldState == SearchAlgorithm::SEARCHING_FOR_GOAL)
      hw->bz->play(hardware::Buzzer::SUCCESSFUL);  //< ゴールについた
    if (oldState == SearchAlgorithm::IDENTIFYING_POSITION &&
        newState != SearchAlgorithm::IMPOSSIBLE)
      hw->bz->play(hardware::Buzzer::COMPLETE);  //< 自己位置同定完了
    if (oldState == SearchAlgorithm::SEARCHING_ADDITIONALLY &&
        newState != SearchAlgorithm::IMPOSSIBLE && !calcData.isForceBackToStart)
      hw->bz->play(hardware::Buzzer::COMPLETE);  //< 追加探索完了
  }
  /* end of override virtual functions */

 private:
  bool auto_maze_check() {
    /* 異常検出 */
    if (!isSolvable()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      MR_LOGE("");
    }
    while (!isSolvable()) {
      maze.resetLastWalls(6);  //< 探索可能になるまで壁を消す
      if (getMaze().getWallRecords().empty()) {
        hw->bz->play(hardware::Buzzer::ERROR);
        MR_LOGE("");
        reset(); /*< reset maze and save */
      }
    }
    return true;
  }
  bool auto_search_run() {
    /* 迷路のチェック */
    MR_LOGD("");
    if (!auto_maze_check()) return false;
    /* 探索走行: スタート -> ゴール -> スタート */
    state.start_search_run();  //< 0 -> 1
    if (searchRun()) {
      hw->bz->play(hardware::Buzzer::COMPLETE);
      MR_LOGD("");
      return true;
    }
    /* エラー処理 */
    if (!isSolvable()) {
      /* 迷路異常の場合 */
      hw->bz->play(hardware::Buzzer::ERROR);
      MR_LOGE("");
      /* ToDo: 迷路を編集して探索を再開 */
      /* なお、ここでは姿勢復帰をせずに自己位置同定走行を開始できる． */
      return false;
    }
    /* クラッシュの場合 */
    if (hw->mt->is_emergency()) {
      ma->emergency_release();
      /* 探索中だった場合はクラッシュ後を想定して直近の壁を削除 */
      hw->bz->play(hardware::Buzzer::MAZE_BACKUP);
      maze.resetLastWalls(6);
      MR_LOGW("");
      /* 自動復帰走行 */
      return auto_pi_run();
    }
    /* タイムアウト */
    return true;
  }
  bool auto_fast_run() {
    /* 迷路のチェック */
    MR_LOGD("");
    if (!auto_maze_check()) return false;
    /* 最短経路の作成 */
    if (!calcShortestDirections(ma->rp_fast.diag_enabled)) {
      hw->bz->play(hardware::Buzzer::ERROR);
      MR_LOGE("");
      return false;
    }
    const auto search_path =
        convertDirectionsToSearchPath(getShortestDirections());
    /* 走行回数インクリメント */
    state.start_fast_run();
    //> FastRun Start
    ma->set_fast_path(search_path);
    ma->enable(MoveAction::TaskActionFastRun);
    ma->waitForEndAction();
    ma->disable();
    //< FastRun End
    if (hw->mt->is_emergency()) {
      state.end_fast_run(false);
      MR_LOGW("");
      return false;  //< クラッシュした
    }
    /* 最短成功 */
    state.end_fast_run(true);
    /* ゴールで回収されるか待つ */
    if (sp->ui->waitForPickup()) return false;  //< 回収された
    /* 帰る */
    MR_LOGD("");
    return endFastRunBackingToStartRun();
  }
  bool auto_pi_run() {
    /* 迷路のチェック */
    MR_LOGD("");
    auto_maze_check();
    /* 念のため */
    ma->emergency_release();
    /* 既知区間斜めを無効化 */
    ma->rp_search.diag_enabled = false;
    /* 自動復帰ループ: 姿勢復帰 -> 自己位置同定 -> ゴール -> スタート */
    MR_LOGD("");
    while (1) {
      /* 姿勢復帰ループ */
      MR_LOGD("");
      while (1) {
        /* 回収待ち */
        MR_LOGD("");
        if (sp->ui->waitForPickup()) return false;
        /* 姿勢復帰走行 */
        ma->enable(MoveAction::TaskActionPositionRecovery);
        ma->waitForEndAction();
        ma->disable();
        /* 失敗したらもう一度 */
        if (hw->mt->is_emergency()) {
          ma->emergency_release();
          MR_LOGW("");
          continue;
        }
        /* 成功 */
        break;
      }
      /* 姿勢復帰完了。回収待ち */
      MR_LOGD("");
      if (sp->ui->waitForPickup()) return false;
      /* ゴール区画の訪問を指定 */
      setForceGoingToGoal(!state.get_has_reached_goal());
      /* 自己位置同定走行 */
      MR_LOGD("");
      if (positionIdentifyRun()) break;
      /* エラー処理 */
      if (hw->mt->is_emergency()) {
        ma->emergency_release();  //< 自己位置同定中にクラッシュ
        MR_LOGW("");
      } else {
        hw->bz->play(hardware::Buzzer::ERROR);  //< 自己位置同定に失敗
        MR_LOGE("");
      }
      /* 失敗した場合は再チャレ */
    }
    /* スタート位置に戻ってきた */
    hw->bz->play(hardware::Buzzer::COMPLETE);
    MR_LOGD("");
    return true;
  }
  bool auto_parameter_select() {
    /* 現在の状態に応じたパラメータ選択 */
    if (state.no_more_time()) {
      MR_LOGD("");
      /* 残り時間が足りない場合 */
      ma->rp_fast.down(state.running_parameter), state.running_parameter = 0;
      ma->rp_fast.up(1), state.running_parameter += 1;
      ma->rp_search.diag_enabled = false;  //< 既知区間斜めは無効化
      /* 最短初回なら斜め有効、それ以外は無効 */
      ma->rp_fast.diag_enabled = (state.get_try_count() == 1);
      hw->bz->play(hardware::Buzzer::TIMEOUT);
    } else if (state.get_fast_run_failed()) {
      MR_LOGD("");
      /* 最短走行中のクラッシュ後の場合 */
      if (state.get_at_least_fast_run_succeeded()) {
        /* 少なくとも1回は最短が成功している: パラメータを1落として再チャレ */
        ma->rp_fast.down(1), state.running_parameter -= 1;
        ma->rp_fast.diag_enabled = true;  //< 斜めあり
      } else {
        /* 最短未成功の状態: パラメータ0の斜めありなしを交互に試す */
        ma->rp_fast.diag_enabled = !ma->rp_fast.diag_enabled;  //< 斜めを交互に
        ma->rp_fast.down(state.running_parameter), state.running_parameter = 0;
      }
      hw->bz->play(hardware::Buzzer::DOWN);
    } else {
      MR_LOGD("");
      /* 初回 or 完走した場合 */
      if (state.get_try_count() == 1)  //< 最短初回だけ特別にパラメータを上げる
        ma->rp_fast.up(3), state.running_parameter += 3;
      if (ma->rp_fast.diag_enabled)  //< 既に斜めあり -> 単にパラメータを上げる
        ma->rp_fast.up(2), state.running_parameter += 2;
      else  //< 斜めなしで成功 -> 同じパラメータで斜めありにする
        ma->rp_fast.diag_enabled = true;
      // ma->rp_search.diag_enabled = true; //< 既知区間斜めを有効化
      hw->bz->play(hardware::Buzzer::UP);
    }
    return true;
  }
};
