#pragma once

#include <esp_timer.h>

#include <array>
#include <cmath>
#include <iostream>

namespace utils {

template <size_t N_frames, size_t N_items>
class TimeProfiler {
 public:
  using timestamp_t = long;

 public:
  void Reset() {
    frame_index = -1;
    item_index = 0;
  }
  void Start() {
    frame_index++;
    item_index = 0;
    timestamp_table[frame_index][item_index++] = GetCurrentTimestamp();
  }
  void Lap(const char *name) {
    if (item_index >= static_cast<int>(N_items)) return;
    if (frame_index == 0) names[item_index] = name;
    timestamp_table[frame_index][item_index++] = GetCurrentTimestamp();
  }
  void ShowResult(std::ostream &csv, const char sep = '\t') {
    if (frame_index < 1 || item_index < 2) {
      std::cerr << "[" __FILE__ ":" << __LINE__ << "][" __func__ "()] skipped";
      return;
    }
    std::array<timestamp_t, N_items> min;
    std::array<timestamp_t, N_items> max;
    std::array<timestamp_t, N_items> average;
    std::array<timestamp_t, N_items> sigma;
    /* calc min max average */
    for (int i = 1; i < item_index; ++i) {
      min[i] = std::numeric_limits<timestamp_t>::max();
      max[i] = std::numeric_limits<timestamp_t>::min();
      average[i] = 0;
      for (int f = 0; f < frame_index; ++f) {
        auto ts0 = timestamp_table[f][i - 1];
        auto ts1 = timestamp_table[f][i];
        auto diff = ts1 - ts0;
        min[i] = std::min(min[i], diff);
        max[i] = std::max(max[i], diff);
        average[i] += diff;
      }
      average[i] /= N_frames;
    }
    /* calc sigma */
    for (int i = 1; i < item_index; ++i) {
      sigma[i] = 0;
      for (int f = 0; f < frame_index; ++f) {
        auto ts0 = timestamp_table[f][i - 1];
        auto ts1 = timestamp_table[f][i];
        auto diff = ts1 - ts0;
        min[i] = std::min(min[i], diff);
        max[i] = std::max(max[i], diff);
        sigma[i] += diff;
      }
      sigma[i] = std::sqrt(sigma[i] / N_frames);
    }
    /* output result */
    csv << "process"         //
        << sep << "min"      //
        << sep << "max"      //
        << sep << "average"  //
        << sep << "sigma"    //
        << std::endl;
    for (int i = 1; i < item_index; ++i)
      csv << names[i]           //
          << sep << min[i]      //
          << sep << max[i]      //
          << sep << average[i]  //
          << sep << sigma[i]    //
          << std::endl;
  }

 private:
  std::array<const char *, N_items> names;
  std::array<std::array<timestamp_t, N_frames>> timestamp_table;
  int frame_index;
  int item_index;

  timestamp_t GetCurrentTimestamp() { return esp_timer_get_time(); }
};

}  // namespace utils
