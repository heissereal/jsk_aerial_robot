#ifndef HUGMY_POSTURE_RECOVERY_GATE_H
#define HUGMY_POSTURE_RECOVERY_GATE_H

#include <algorithm>
#include <cmath>

namespace hugmy
{
// A continuous confirmation interval, not an accumulation of short spikes.
// Explicitly reset on sensor faults, disable and task/state changes.
class PostureRecoveryGate
{
public:
  void reset() { active_ = false; }

  bool update(bool condition, double now, double duration)
  {
    if (!condition || !std::isfinite(now))
      {
        reset();
        return false;
      }
    if (!active_ || now < last_)
      {
        since_ = now;
        active_ = true;
      }
    last_ = now;
    return now - since_ >= std::max(0.0, duration);
  }

private:
  bool active_ = false;
  double since_ = 0.0, last_ = 0.0;
};
}  // namespace hugmy

#endif
