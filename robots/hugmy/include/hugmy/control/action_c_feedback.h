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

// Pulse-time exception to the ordinary settled-boundary roll supervisor.
// Do not identify every pulse excursion as a recovery: require outward gyro,
// actual roll growth AND impending bend exhaustion. Hard roll/rate limits
// always win. This only requests a thrust-OFF handoff, never extra thrust.
inline bool actionCPulseRollHandoff(double roll, double start_roll, double rate,
    double predicted_bend_margin, double bend_margin, double trigger,
    double hard_roll, double hard_rate, double min_growth, double min_rate)
{
  if (!std::isfinite(roll) || !std::isfinite(start_roll) || !std::isfinite(rate))
    return false;  // Sensor-fault guards own invalid samples.
  if (std::abs(roll) >= hard_roll || std::abs(rate) >= hard_rate) return true;
  const double outward_rate = roll >= 0.0 ? rate : -rate;
  return std::isfinite(predicted_bend_margin) &&
      std::abs(roll) > trigger && outward_rate >= min_rate &&
      std::abs(roll) - std::abs(start_roll) >= min_growth &&
      predicted_bend_margin <= bend_margin;
}

// Pulse-by-pulse amplitude search. A missed angle target must not lock out
// exploration, but incomplete regrip/guards/recovery must never earn a raise.
inline double actionCNextPulseAmplitude(double current, double minimum,
    double ceiling, double step, double retained, double goal,
    bool valid, bool guard, bool recovery, bool pulse_started)
{
  double next = current;
  if (guard || (recovery && pulse_started))
    next -= step;
  else if (valid && !recovery && std::isfinite(retained) &&
           retained < goal)
    next += step;
  return std::max(0.0, std::min(ceiling, std::max(minimum, next)));
}

inline double actionCSearchPulseDuration(double base, double thrust, double ramp)
{
  // Reach the requested peak plus a short plateau. Never turn a slow-ramp
  // misconfiguration into an arbitrarily long ON interval.
  return ramp > 0.0 ? std::min(1.0, std::max(base, thrust / ramp + 0.15)) : base;
}

// Residual positive thrust while PRESSURE supplies the flexion torque.
// Never re-raise thrust within this phase. Taper only near measured rebound,
// and remove assistance if rear release is lost or inputs are invalid.
inline double actionCPressurePullThrust(double previous, double ceiling,
    double rebound, double target, bool rear_released)
{
  if (!rear_released || !std::isfinite(previous) || !std::isfinite(ceiling) ||
      !std::isfinite(rebound) || !std::isfinite(target) || target <= 0.0)
    return 0.0;
  const double scale = std::max(0.0, std::min(1.0,
      (target - rebound) / (0.5 * target)));
  return std::max(0.0, std::min(previous, ceiling * scale));
}

enum ActionCGuard : unsigned int
{
  ACTION_C_OVER_EXTENSION = 1u << 0,
  ACTION_C_LOW_BEND = 1u << 1,
  ACTION_C_EXTENSION_RATE = 1u << 2,
  ACTION_C_PAIR_MISMATCH = 1u << 3,
  ACTION_C_REVERSE_TRAVEL = 1u << 4,
  ACTION_C_BODY_TILT = 1u << 5,
  ACTION_C_ANGLE_UNRELIABLE = 1u << 6
};

// The pulse always stops on ANY guard. Only the optimizer's restriction due
// to pair mismatch is recoverable; physical-limit guards remain latched.
struct ActionCThrustSafety
{
  double hard_max_n = 0.0;
  double pair_max_n = 0.0;
  int safe_pulses = 0;

  void reset(double maximum)
  {
    hard_max_n = pair_max_n = maximum;
    safe_pulses = 0;
  }

  double limit() const { return std::min(hard_max_n, pair_max_n); }

  bool observeTrial(double candidate, double minimum, double step,
                    unsigned int guards, int qualified_pulses, int required)
  {
    // Re-probe increments never exceed 0.25 N, even with a larger search step.
    step = std::max(0.0, std::min(0.25, step));
    if (guards != 0)
      {
        const double reduced = std::max(minimum, candidate - step);
        if ((guards & ~ACTION_C_PAIR_MISMATCH) != 0)
          hard_max_n = std::min(hard_max_n, reduced);
        if ((guards & ACTION_C_PAIR_MISMATCH) != 0)
          pair_max_n = std::min(pair_max_n, reduced);
        safe_pulses = 0;
        return false;
      }
    if (qualified_pulses <= 0 || candidate < limit() - 1.0e-6 ||
        pair_max_n >= hard_max_n - 1.0e-6 || step <= 0.0)
      {
        safe_pulses = 0;
        return false;
      }
    safe_pulses += qualified_pulses;
    if (safe_pulses < std::max(1, required)) return false;
    pair_max_n = std::min(hard_max_n, pair_max_n + step);
    safe_pulses = 0;
    return true;
  }
};

// Compare extension from each arm's OWN pulse-start bend, not absolute IMU
// bend: contact geometry can give the two arms different initial bends.
inline double actionCExtensionSyncReduction(
    double extension, double slowest_extension, double bend_rate,
    double extension_target, double prediction_sec, double deadband,
    double gain, double max_reduction)
{
  if (!std::isfinite(extension) || !std::isfinite(slowest_extension) ||
      !std::isfinite(bend_rate))
    return max_reduction;
  const double predicted = std::max(0.0, extension) +
      std::max(0.0, -bend_rate) * prediction_sec;
  const double reference = std::min(extension_target,
      std::max(0.0, slowest_extension));
  const double lead = std::max(0.0, predicted - reference - deadband);
  return std::max(0.0, std::min(max_reduction, gain * lead));
}

// Positive sub-goal progress is useful optimizer evidence, but pressure
// timeouts, unsafe pulses and incomplete recoil must never earn it.
inline double actionCMeasuredProgress(double dx, bool valid)
{
  return valid ? dx : std::min(0.0, dx);
}

inline double actionCExtensionReference(double elapsed, double duration, double target)
{
  const double u = std::max(0.0, std::min(1.0, elapsed / std::max(0.01, duration)));
  return target * u * u * (3.0 - 2.0 * u);
}

// Independent rotor feedback against one shared relative-extension trajectory.
// Normal synchronization boosts the lagging arm within a small headroom.
// Only predicted target overshoot brakes a leader; hard guards remain active.
inline double actionCIndividualThrust(double nominal, double ceiling, double elapsed, double ramp,
    double reference, double extension, double fastest, double bend_rate,
    double kp, double kd, double prediction, double deadband, double brake_band,
    double extension_target)
{
  if (!std::isfinite(extension) || !std::isfinite(fastest) ||
      !std::isfinite(bend_rate)) return 0.0;
  nominal = std::max(0.0, std::min(ceiling, nominal));
  const double ff = std::min(nominal, std::max(0.0, elapsed) * ramp);
  const double wanted_extension = std::max(reference, std::min(extension_target, fastest));
  const double lag = std::max(0.0, wanted_extension - extension - deadband);
  const double boost = std::max(0.0, std::min(ceiling - nominal,
      kp * lag + kd * std::min(0.0, bend_rate)));
  const double projected = extension + std::max(0.0, -bend_rate) * prediction;
  const double overshoot = std::max(0.0, projected - extension_target - deadband);
  const double protection = std::max(0.0, 1.0 - overshoot / std::max(1.0e-6, brake_band));
  return std::max(0.0, std::min(ceiling, ff + boost)) * protection;
}

inline bool actionCCanRaiseArm(bool valid, bool guard, bool posture_ok,
    double peak_mismatch, double allowed_mismatch, double terminal_extension,
    double peak_extension, double target, double tolerance,
    double peak_thrust, double ceiling)
{
  return valid && !guard && posture_ok && peak_mismatch <= allowed_mismatch &&
      terminal_extension >= -tolerance &&
      peak_extension < target - tolerance &&
      peak_thrust >= ceiling - 0.1;
}
}
