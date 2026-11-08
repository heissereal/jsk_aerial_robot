#ifndef HUGMY_POSTURE_RECOVERY_PHASE_H
#define HUGMY_POSTURE_RECOVERY_PHASE_H

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <array>
#include <limits>

namespace hugmy {

// Logical phases shared by standalone and locomotion-triggered roll recovery.
// The legacy ROS state IDs are preserved: SELECT and RECOVERY execute inside
// GRASP/PREPARE, while CONTACT_RESTORE and VERIFY execute inside RECOVER.
enum class PostureRecoveryPhase : uint8_t {
  HOLD,
  SELECT,
  PREPARE,
  RECOVERY,
  CONTACT_RESTORE,
  VERIFY,
  SAFE_TRANSFER,
  ABORT
};

enum class RecoveryExit : uint8_t {
  CONTINUE,
  CONTACT_RESTORE,
  SAFE_TRANSFER,
  TARGET_CROSSED
};

// The ROS-visible SAFE_TRANSFER state remains id 29.  These subphases make
// its physical completion criteria explicit without changing recorded state
// IDs or allowing the roll PI to run during a failed recovery.
enum class RecoverySafeTransferPhase : uint8_t {
  REGRASP,
  SUPPORT_VERIFY,
  THRUST_RELEASE,
  COMPLETED,
  FAILED
};

enum class RecoveryFaultReason : uint8_t {
  NONE,
  UNCLASSIFIED,
  SENSOR_INVALID,
  WORSENING_RATE,
  RATE_SPIKE,
  STALL,
  ABSOLUTE_WATCHDOG,
  WRONG_WAY,
  UNRESPONSIVE,
  GEOMETRY_INVALID,
  REGRASP_TIMEOUT,
  SUPPORT_UNVERIFIED,
  SUPPORT_LOST
};

inline const char* recoveryFaultReasonName(RecoveryFaultReason reason) {
  switch (reason) {
    case RecoveryFaultReason::NONE: return "none";
    case RecoveryFaultReason::UNCLASSIFIED: return "unclassified";
    case RecoveryFaultReason::SENSOR_INVALID: return "sensor_invalid";
    case RecoveryFaultReason::WORSENING_RATE: return "worsening_rate";
    case RecoveryFaultReason::RATE_SPIKE: return "rate_spike";
    case RecoveryFaultReason::STALL: return "stall";
    case RecoveryFaultReason::ABSOLUTE_WATCHDOG: return "absolute_watchdog";
    case RecoveryFaultReason::WRONG_WAY: return "wrong_way";
    case RecoveryFaultReason::UNRESPONSIVE: return "unresponsive";
    case RecoveryFaultReason::GEOMETRY_INVALID: return "geometry_invalid";
    case RecoveryFaultReason::REGRASP_TIMEOUT: return "regrasp_timeout";
    case RecoveryFaultReason::SUPPORT_UNVERIFIED: return "support_unverified";
    case RecoveryFaultReason::SUPPORT_LOST: return "support_lost";
  }
  return "unknown";
}

// Physical support is estimated only from signals available on hardware.
// MuJoCo contact forces are deliberately excluded and are used later only as
// ground truth when evaluating this estimator.
struct RecoverySupportObservation {
  std::array<double, 4> pressure{{NAN, NAN, NAN, NAN}};
  std::array<double, 4> target_pressure{{NAN, NAN, NAN, NAN}};
  std::array<double, 4> bend{{NAN, NAN, NAN, NAN}};
  double roll = NAN;
  double pitch = NAN;
  double roll_rate = NAN;
  double pitch_rate = NAN;
  double maximum_thrust = NAN;
  bool sensors_valid = false;
};

struct RecoverySupportConfig {
  double pressure_tolerance = 1.5;
  double minimum_pressure = 15.0;
  double maximum_bend_change = 0.035;
  double maximum_rate = 0.05;
  double maximum_attitude_drift = 0.0174533;
  double settle_sec = 0.30;
  size_t minimum_loaded_arms = 3;
};

class RecoverySupportMonitor {
 public:
  void reset() {
    reference_valid_ = false;
    stable_since_ = -1.0;
    reference_roll_ = reference_pitch_ = 0.0;
    reference_bend_.fill(0.0);
  }

  bool update(const RecoverySupportObservation& o, double now,
              const RecoverySupportConfig& c, bool require_thrust_off) {
    if (!valid(o, now, c, require_thrust_off)) {
      reset();
      return false;
    }
    if (!reference_valid_) {
      reference_valid_ = true;
      reference_roll_ = o.roll;
      reference_pitch_ = o.pitch;
      reference_bend_ = o.bend;
      stable_since_ = now;
      return false;
    }
    bool stable = std::abs(o.roll - reference_roll_) <=
                      c.maximum_attitude_drift &&
                  std::abs(o.pitch - reference_pitch_) <=
                      c.maximum_attitude_drift;
    for (size_t arm = 0; arm < o.bend.size(); ++arm)
      stable = stable && std::abs(o.bend[arm] - reference_bend_[arm]) <=
          c.maximum_bend_change;
    if (!stable) {
      reference_roll_ = o.roll;
      reference_pitch_ = o.pitch;
      reference_bend_ = o.bend;
      stable_since_ = now;
      return false;
    }
    return stable_since_ >= 0.0 && now - stable_since_ >= c.settle_sec;
  }

  bool referenceValid() const { return reference_valid_; }
  double stableSince() const { return stable_since_; }

 private:
  bool valid(const RecoverySupportObservation& o, double now,
             const RecoverySupportConfig& c,
             bool require_thrust_off) const {
    if (!o.sensors_valid || !std::isfinite(now) ||
        !std::isfinite(o.roll) || !std::isfinite(o.pitch) ||
        !std::isfinite(o.roll_rate) || !std::isfinite(o.pitch_rate) ||
        !std::isfinite(o.maximum_thrust) ||
        std::abs(o.roll_rate) > c.maximum_rate ||
        std::abs(o.pitch_rate) > c.maximum_rate ||
        (require_thrust_off && o.maximum_thrust > 0.01))
      return false;
    size_t loaded = 0;
    for (size_t arm = 0; arm < o.pressure.size(); ++arm) {
      if (!std::isfinite(o.pressure[arm]) ||
          !std::isfinite(o.target_pressure[arm]) ||
          !std::isfinite(o.bend[arm]))
        return false;
      if (o.pressure[arm] >= c.minimum_pressure &&
          std::abs(o.pressure[arm] - o.target_pressure[arm]) <=
              c.pressure_tolerance)
        ++loaded;
    }
    return loaded >= c.minimum_loaded_arms;
  }

  bool reference_valid_ = false;
  double stable_since_ = -1.0;
  double reference_roll_ = 0.0;
  double reference_pitch_ = 0.0;
  std::array<double, 4> reference_bend_{{0.0, 0.0, 0.0, 0.0}};
};

// A rotor pass is NOT a successful pneumatic handoff just because its PI
// controller stopped. A fault and an unverified capture must not enter the
// full-power handoff PI with a large residual attitude error.
inline RecoveryExit recoveryExitDecision(bool finished, bool fault,
                                          bool target_crossed,
                                          double roll_error_rad,
                                          double capture_limit_rad) {
  // Even a crossed target does not justify contact-restoration PI if the
  // residual has already grown large in the opposite direction.
  if (target_crossed && std::isfinite(roll_error_rad) &&
      std::isfinite(capture_limit_rad) && capture_limit_rad > 0.0 &&
      std::abs(roll_error_rad) <= capture_limit_rad)
    return RecoveryExit::TARGET_CROSSED;
  if (fault) return RecoveryExit::SAFE_TRANSFER;
  if (!finished) return RecoveryExit::CONTINUE;
  if (!std::isfinite(roll_error_rad) ||
      !std::isfinite(capture_limit_rad) ||
      capture_limit_rad <= 0.0 ||
      std::abs(roll_error_rad) > capture_limit_rad)
    return RecoveryExit::SAFE_TRANSFER;
  return RecoveryExit::CONTACT_RESTORE;
}

// Reject unresponsive candidates only after the pressure transition is
// observable AND meaningful rotor authority has actually been applied.
inline bool recoveryResponseCanBeJudged(bool pressure_settled,
                                        bool measured_geometry_settled,
                                        double applied_thrust,
                                        double required_probe_thrust,
                                        double effective_probe_sec,
                                        double required_probe_sec) {
  return pressure_settled && measured_geometry_settled &&
      std::isfinite(applied_thrust) &&
      applied_thrust >= required_probe_thrust &&
      std::isfinite(effective_probe_sec) &&
      effective_probe_sec >= required_probe_sec;
}

// The safety state may *only* restore support and reduce existing rotor
// force; it must never issue a new rotor thrust increase.
inline double recoveryBoundedPressureStep(double previous_kpa,
                                          double target_kpa,
                                          double pressure_rate_kpa_s,
                                          double dt_sec) {
  if (!std::isfinite(previous_kpa) || !std::isfinite(target_kpa) ||
      !std::isfinite(pressure_rate_kpa_s) || !std::isfinite(dt_sec) ||
      pressure_rate_kpa_s <= 0.0 || dt_sec <= 0.0)
    return previous_kpa;
  const double step = pressure_rate_kpa_s * dt_sec;
  return previous_kpa + std::max(-step,
      std::min(step, target_kpa - previous_kpa));
}

inline double recoverySafeThrustRundown(double current_n,
                                        double fall_rate_n_s,
                                        double dt_sec) {
  if (!std::isfinite(current_n) || current_n <= 0.0)
    return 0.0;
  if (!std::isfinite(fall_rate_n_s) || !std::isfinite(dt_sec) ||
      fall_rate_n_s <= 0.0 || dt_sec <= 0.0)
    return current_n;
  return std::max(0.0, current_n - fall_rate_n_s * dt_sec);
}

}  // namespace hugmy

#endif  // HUGMY_POSTURE_RECOVERY_PHASE_H
