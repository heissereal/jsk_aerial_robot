#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hugmy {
// Arrival is strict. After arrival, retain readiness through small regulation
// ripple and bounded, brief excursions. This is NOT the raw overpressure or
// sensor-freshness safety check, which must remain independent.
class PressureReadyGate {
public:
  using Four = std::array<double,4>;
  void reset() { ready_=false; have_target_=false; outside_since_=-1; last_=-1; }
  bool update(const Four& measured, const Four& target, const Four& enter,
              double now, bool fresh, double exit_tolerance=5.,
              double grace=.1, double maximum_deviation=8.) {
    if (!fresh || !std::isfinite(now) || (last_>=0 && now<last_) ||
        !std::isfinite(exit_tolerance) || !std::isfinite(grace) ||
        !std::isfinite(maximum_deviation) || exit_tolerance<0 || grace<0 ||
        maximum_deviation<exit_tolerance) { reset(); return false; }
    bool changed=!have_target_, inside=true, outside=false;
    for (size_t i=0;i<4;++i) {
      if (!std::isfinite(measured[i]) || !std::isfinite(target[i]) ||
          !std::isfinite(enter[i]) || enter[i]<0 || enter[i]>maximum_deviation) {
        reset(); return false;
      }
      changed=changed || std::abs(target[i]-target_[i])>1e-6;
      const double error=std::abs(measured[i]-target[i]);
      if (error>maximum_deviation) { reset(); return false; }
      inside=inside && error<=enter[i];
      outside=outside || error>std::max(enter[i],exit_tolerance);
    }
    if (changed) { ready_=false; outside_since_=-1; }
    target_=target; have_target_=true; last_=now;
    if (!ready_) { ready_=inside; outside_since_=-1; return ready_; }
    if (!outside) { outside_since_=-1; return true; }
    if (outside_since_<0) outside_since_=now;
    if (now-outside_since_>=grace) { ready_=false; outside_since_=-1; }
    return ready_;
  }
private:
  Four target_{};
  bool ready_=false, have_target_=false;
  double outside_since_=-1, last_=-1;
};
}  // namespace hugmy
