#pragma once
#include <array>
#include <cmath>

namespace hugmy {
// One-sided release gate: front-arm grasp pressures do not participate.
// Strict acquisition, hysteretic retention, and a bounded excursion timer.
class RearReleaseGate {
public:
  void reset() { ready_=false; have_mask_=false; last_=-1.; outside_since_=-1.; }
  bool update(const std::array<double,4>& pressure,
              const std::array<bool,4>& front, double now, bool fresh,
              double acquire, double retain, double hard, double grace) {
    if (!fresh || !std::isfinite(now) || (last_>=0. && now<last_) ||
        !std::isfinite(acquire) || !std::isfinite(retain) ||
        !std::isfinite(hard) || !std::isfinite(grace) ||
        acquire<0. || retain<acquire || hard<retain || grace<0.) {
      reset(); return false;
    }
    if (!have_mask_ || front!=front_) reset();
    front_=front; have_mask_=true; last_=now;
    bool inside=true, outside=false, have_rear=false;
    for (size_t i=0;i<4;++i) if (!front[i]) {
      have_rear=true;
      if (!std::isfinite(pressure[i]) || pressure[i]>hard) {
        reset(); return false;
      }
      inside=inside && pressure[i]<=acquire;
      outside=outside || pressure[i]>retain;
    }
    if (!have_rear) { reset(); return false; }
    if (!ready_) { ready_=inside; outside_since_=-1.; return ready_; }
    if (!outside) { outside_since_=-1.; return true; }
    if (outside_since_<0.) outside_since_=now;
    if (now-outside_since_>=grace) { reset(); return false; }
    return true;
  }
private:
  std::array<bool,4> front_{};
  bool ready_=false, have_mask_=false;
  double last_=-1., outside_since_=-1.;
};
} // namespace hugmy
