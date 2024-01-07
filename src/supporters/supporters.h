#pragma once

#include "supporters/logger.h"
#include "supporters/speed_controller.h"
#include "supporters/user_interface.h"
#include "supporters/wall_detector.h"

namespace supporters {

class Supporters {
 private:
  hardware::Hardware* hw;

 public:
  UserInterface* ui;
  SpeedController* sc;
  WallDetector* wd;

 public:
  Supporters(hardware::Hardware* hw)
      : hw(hw),
        ui(new UserInterface(hw)),
        sc(new SpeedController(hw)),
        wd(new WallDetector(hw)) {}
  bool init() {
    int ret = true;
    if (!sc->init()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      APP_LOGE("SpeedController init failed");
      ret = false;
    }
    if (!wd->init()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      APP_LOGE("WallDetector init failed");
      ret = false;
    }
    return ret;
  }
};

}  // namespace supporters
