/**
 * @file logger.h
 * @brief Logger
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>
#include <string>
#include <vector>

#include "config/config.h"  //< KERISE_SELECT

class Logger {
 public:
  Logger() {}
  void clear() { buf_.clear(); }
  void init(const std::vector<std::string>& labels,
            const std::string& comment) {
    labels_ = labels;
    comment_ = comment;
    clear();
  }
  void push(const std::vector<float>& data) { buf_.push_back(data); }
  void print() const {
    if (buf_.empty()) return;
    /* show header */
    std::printf("# KERISE v%d Build: %s %s Comment: %s\n", KERISE_SELECT,
                __DATE__, __TIME__, comment_.c_str());
    /* show labels */
    for (int i = 0; i < labels_.size(); ++i) {
      std::printf("%s", labels_[i].c_str());
      if (i < labels_.size() - 1) std::printf("\t");
    }
    std::printf("\n");
    /* data */
    for (const auto& data : buf_) {
      for (int i = 0; i < data.size(); ++i) {
        std::printf("%e", (double)data[i]);  //< printf supports only double
        if (i < data.size() - 1) std::printf("\t");
      }
      std::printf("\n");
      taskYIELD();
    }
  }

 private:
  std::vector<std::vector<float>> buf_;
  std::vector<std::string> labels_;
  std::string comment_;
};
