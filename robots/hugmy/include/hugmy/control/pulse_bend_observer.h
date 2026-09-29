#pragma once
#include <cmath>

namespace hugmy
{
// Short-horizon relative bend estimate. Seed only at settled pulse start.
// Do not interpret rotor-induced specific force as gravity during the pulse.
struct PulseBendObserver
{
  double bend = 0.0, rate = 0.0, stamp = 0.0;
  bool valid = false;
  void seed(double angle, double velocity, double time)
  {
    bend = angle; rate = velocity; stamp = time;
    valid = std::isfinite(angle) && std::isfinite(velocity) && std::isfinite(time);
  }
  void update(double velocity, double time)
  {
    if (!valid) return;
    const double dt = time - stamp;
    if (!std::isfinite(velocity) || !std::isfinite(time) || dt < 0.0 || dt > 0.1)
      { valid = false; return; }
    if (dt == 0.0) return;
    bend += 0.5 * (rate + velocity) * dt;
    if (!std::isfinite(bend)) { valid = false; return; }
    rate = velocity; stamp = time;
  }
};

// A +Y sensor rotation q expresses gravity as [-sin(q), 0, cos(q)].
// measuredArmAngle() = -q, hence bend = q: positive relative Y gyro
// INCREASES bend (flexion). Do not negate gyro just because bend=-angle.
inline double relativeBendRate(double neuron_y, double body_x, double body_y, double arm_yaw)
{
  const double body_local_y = -std::sin(arm_yaw) * body_x + std::cos(arm_yaw) * body_y;
  return neuron_y - body_local_y;
}
}
