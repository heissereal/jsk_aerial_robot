#pragma once
#include <cmath>
#include <deque>
#include <utility>

namespace hugmy {
// Observe only quiet, pressure-ready boundaries. Pulse/inflation excursions
// never enter this history. Growth is measured in |target-relative roll|,
// and must agree with the signed Spinal gyro (both roll signs supported).
struct RollTrendGate {
  double window_sec=.5, growth_rad=M_PI/180., outward_rate_rad_s=M_PI/180.;
  bool ready=false, worsening=false;
  std::deque<std::pair<double,double>> samples;
  void reset() { samples.clear(); ready=false; worsening=false; }
  void update(bool enabled,double error,double gyro,double now) {
    if (!enabled || !std::isfinite(error) || !std::isfinite(gyro) ||
        !std::isfinite(now) || (!samples.empty() &&
        (now<samples.back().first || now-samples.back().first>.25))) {
      reset(); return;
    }
    const double magnitude=std::abs(error);
    if (samples.empty() || now>samples.back().first) samples.emplace_back(now,magnitude);
    while(samples.size()>1 && samples[1].first<=now-window_sec) samples.pop_front();
    ready=!samples.empty() && now-samples.front().first>=window_sec-1e-6;
    const double outward=error>=0 ? gyro : -gyro;
    worsening=ready && magnitude-samples.front().second>=growth_rad &&
        outward>=outward_rate_rad_s;
  }
};
} // namespace hugmy
