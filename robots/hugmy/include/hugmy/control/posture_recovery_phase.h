#ifndef HUGMY_POSTURE_RECOVERY_PHASE_H
#define HUGMY_POSTURE_RECOVERY_PHASE_H

#include <cmath>
#include <cstdint>
#include <algorithm>

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
