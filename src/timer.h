#pragma once

#include "colordef.h"
#include <chrono>
#include <iostream>
#include <string>

class Timer {
public:
  void Tic() {
    start_ticking_ = true;
    start_ = std::chrono::high_resolution_clock::now();
  }

  double Toc(const std::string &desc = "") {
    if (!start_ticking_)
      return 0;
    start_ticking_ = false;
    end_ = std::chrono::high_resolution_clock::now();
    double t = std::chrono::duration<double, std::milli>(end_ - start_).count();
    if (desc == "") {
      std::cout << "Timer: " << kColorYellow << t << kColorReset << " ms"
                << ::std::endl;
    } else {
      std::cout << "Timer: " << kColorYellow << t << " ms in " << desc
                << kColorReset << ::std::endl;
    }
    return t;
  }

private:
  bool start_ticking_ = false;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_;
};
