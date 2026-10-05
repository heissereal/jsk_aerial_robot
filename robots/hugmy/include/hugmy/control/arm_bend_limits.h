#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace hugmy
{
// Start readiness is separate from the ON guard. Reserve an excursion plus
// margin for shallow grasps, without crossing the absolute floor. `start`
// must be the frozen ON-edge bend, never the moving current angle.
inline double actionCPulseBendFloor(double start, double nominal_floor,
    double hard_floor, double target_extension, double margin)
{
  if (!std::isfinite(start) || !std::isfinite(nominal_floor) ||
      !std::isfinite(hard_floor) || !std::isfinite(target_extension) ||
      !std::isfinite(margin) || hard_floor < 0.0 ||
      nominal_floor < hard_floor || target_extension < 0.0 || margin <= 0.0)
    return std::numeric_limits<double>::infinity();
  return std::max(hard_floor,
      std::min(nominal_floor, start - target_extension - margin));
}

inline bool actionCStartBendReady(double bend, double prepare_min,
    double hard_floor, double target_extension, double margin)
{
  return std::isfinite(bend) && std::isfinite(prepare_min) &&
      std::isfinite(hard_floor) && std::isfinite(target_extension) &&
      std::isfinite(margin) && prepare_min >= 0.0 && hard_floor >= 0.0 &&
      target_extension >= 0.0 && margin > 0.0 &&
      bend >= prepare_min && bend >= hard_floor + target_extension + margin;
}

} // namespace hugmy
