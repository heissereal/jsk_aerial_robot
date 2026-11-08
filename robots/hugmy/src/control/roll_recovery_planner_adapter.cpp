#include <hugmy/control/attitude_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>

// The planner itself has no ROS, actuator, or state-machine dependencies.
// This adapter converts Hugmy's measured bends, circle fit and calibrated
// pressure/thrust table into the four candidate geometries.  The caller decides
// whether the returned selection is diagnostic-only or is connected to the
// existing guarded pressure/thrust state machine.
hugmy::recovery::Selection
AttitudePressureController::evaluateUnifiedRecoveryCandidates(
    double roll_error, double desired_roll_moment,
    const hugmy::recovery::Vec3& circle_center_body,
    bool actuating, int previous_candidate_index)
{
  namespace rr = hugmy::recovery;
  rr::Selection result;
  if (!recovery_circle_event_estimate_.valid ||
      !rr::finite(circle_center_body) ||
      !std::isfinite(desired_roll_moment))
    return result;

  const double radius = recovery_circle_event_estimate_.radius;
  if (!std::isfinite(radius) || radius < 1.0e-3) return result;

  const std::array<std::array<size_t, 2>, 2> pairs{{
      {{0, 1}}, {{2, 3}}}};
  std::array<rr::Candidate, 4> candidates;
  std::array<rr::Evaluation, 4> values;
  rr::Config config;
  config.thrust_limit = roll_recovery_config_.limit;
  config.min_signed_lever = recovery_circle_min_signed_lever_m_;
  config.release_penalty = recovery_circle_release_penalty_n_;
  // Transition time is logged independently; no uncalibrated cost weight.
  config.transition_penalty = 0.0;

  const auto rotor_at_bend = [&](size_t arm, double bend) {
    rr::Rotor rotor;
    rotor.position = rotorPositionFromAngle(arm, bend);
    // Match the URDF rotor cant AND the frame used by the existing torque
    // model; rotorDirectionFromAngle() alone omits the local cant.
    const double cant_sign = (arm == 0 || arm == 2) ? 1.0 : -1.0;
    rotor.unit_force = rotateLocalToBody(rotateAboutLocalY(
        {{0.0, cant_sign * std::sin(recovery_rotor_cant_rad_),
          std::cos(recovery_rotor_cant_rad_)}},
        std::max(0.0, std::min(2.966, bend))), arm_yaw_[arm]);
    return rotor;
  };
  const auto pressure_transitioning = [&](size_t arm) {
    return !std::isfinite(measured_pressure_[arm]) ||
        !std::isfinite(last_pressure_[arm]) ||
        std::abs(measured_pressure_[arm] - last_pressure_[arm]) >
            pressure_tolerance_kpa_;
  };

  for (size_t pair_index = 0; pair_index < pairs.size(); ++pair_index)
    {
      // The other pair is hypothesized to remain loaded while the candidate
      // pair actuates.  This is a mode-specific SUPPORT HYPOTHESIS, not a
      // claim that the pressure readings measure normal reaction forces.
      const size_t support_index = 1 - pair_index;
      rr::Vec3 support{{0.0, 0.0, 0.0}};
      bool support_valid = true;
      bool support_pressure_transitioning = false;
      for (const size_t arm : pairs[support_index])
        {
          support_pressure_transitioning = support_pressure_transitioning ||
              pressure_transitioning(arm);
          const double bend = measuredArmBendAngle(arm);
          const rr::Vec3 contact = armContactPointFromAngle(arm, bend);
          if (!rr::finite(contact) || !std::isfinite(measured_pressure_[arm]) ||
              measured_pressure_[arm] < recovery_circle_contact_pressure_kpa_)
            {
              support_valid = false;
              break;
            }
          const double dy = contact[1] - circle_center_body[1];
          const double dz = contact[2] - circle_center_body[2];
          const double radial = std::hypot(dy, dz);
          if (radial < 1.0e-4)
            {
              support_valid = false;
              break;
            }
          support[0] += contact[0] / 2.0;
          support[1] += (circle_center_body[1] + radius * dy / radial) / 2.0;
          support[2] += (circle_center_body[2] + radius * dz / radial) / 2.0;
        }

      for (size_t mode = 0; mode < 2; ++mode)
        {
          const size_t index = 2 * pair_index + mode;
          rr::Candidate& c = candidates[index];
          c.pair = pair_index;
          c.release = (mode == 1);
          c.contact_feasible = support_valid;
          c.support_pressure_transitioning = support_pressure_transitioning;
          c.actuated_pressure_transitioning = false;
          for (const size_t arm : pairs[pair_index])
            c.actuated_pressure_transitioning =
                c.actuated_pressure_transitioning ||
                pressure_transitioning(arm);
          c.support_body = support;
          c.release_fraction = c.release ? 1.0 : 0.0;
          c.transition_seconds = c.release ? recovery_extension_prediction_sec_ : 0.0;
          if (c.release)
            {
              // This is a steady target-shape calculation, not a time-domain
              // prediction of the exhaust transient.  First invert the
              // measured lookup to find the least common rotor command which
              // places BOTH arms inside the configured extension shape.
              c.extension_min_thrust =
                  std::numeric_limits<double>::infinity();
              const int thrust_steps = static_cast<int>(std::ceil(
                  config.thrust_limit / recovery_unified_thrust_step_n_));
              for (int step = 0; step <= thrust_steps; ++step)
                {
                  const double thrust = std::min(config.thrust_limit,
                      step * recovery_unified_thrust_step_n_);
                  bool both_extended = true;
                  for (size_t i = 0; i < 2; ++i)
                    {
                      const double bend = bend_model_.angleRad(
                          recovery_release_pressure_kpa_,
                          thrust + bend_model_thrust_offset_n_,
                          hugmy::PressureThrustBendModel::Trend::DECREASING,
                          hugmy::PressureThrustBendModel::Trend::INCREASING);
                      both_extended = both_extended && std::isfinite(bend) &&
                          bend <= recovery_unified_extension_target_bend_rad_;
                    }
                  if (both_extended)
                    {
                      c.extension_min_thrust = thrust;
                      break;
                    }
                }
              // The same thrust straightens the compliant rotor-carrying arm
              // and produces body roll. Search above the lookup-derived lower
              // bound and recompute the equilibrium rotor pose for each force;
              // do not add a fictitious second copy of extension thrust.
              values[index] = rr::evaluateCoupled(c, desired_roll_moment,
                  config, [&](double thrust) {
                    std::array<rr::Rotor, 2> rotors;
                    for (size_t i = 0; i < 2; ++i)
                      {
                        const size_t arm = pairs[pair_index][i];
                        const double bend = bend_model_.angleRad(
                            recovery_release_pressure_kpa_,
                            thrust + bend_model_thrust_offset_n_,
                            hugmy::PressureThrustBendModel::Trend::DECREASING,
                            hugmy::PressureThrustBendModel::Trend::INCREASING);
                        rotors[i] = rotor_at_bend(arm, bend);
                      }
                    return rotors;
                  }, recovery_unified_thrust_step_n_);
            }
          else
            {
              for (size_t i = 0; i < 2; ++i)
                {
                  const size_t arm = pairs[pair_index][i];
                  c.rotors[i] = rotor_at_bend(arm,
                      measuredArmBendAngle(arm));
                }
              values[index] = rr::evaluate(c, desired_roll_moment, config);
            }
        }
    }

  // Preserve the physical support side across near-tied re-selections.  The
  // release penalty already prices a mode change inside one pair; this term
  // applies only when the selected 0+1 / 2+3 pair itself changes.
  if (previous_candidate_index >= 0 && previous_candidate_index < 4)
    {
      const size_t previous_pair =
          static_cast<size_t>(previous_candidate_index) / 2;
      for (size_t i = 0; i < values.size(); ++i)
        if (values[i].feasible && i / 2 != previous_pair)
          values[i].score += recovery_unified_pair_switch_penalty_n_;
    }

  int best = -1;
  for (size_t i = 0; i < values.size(); ++i)
    if (values[i].feasible &&
        (best < 0 || values[i].score < values[static_cast<size_t>(best)].score))
      best = static_cast<int>(i);
  result.candidates = candidates;
  result.alternatives = values;
  if (best >= 0)
    {
      result.valid = true;
      result.index = static_cast<size_t>(best);
      result.evaluation = values[result.index];
    }

  const double error_rate = -hugmy::recoveryRollRate(
      roll_, pitch_, body_angular_velocity_);
  const bool imminent_fall = rr::imminentOutwardFall(
      roll_error, error_rate, recovery_unsafe_roll_rad_,
      recovery_extension_response_sec_, 0.15);

  ROS_INFO("Unified recovery %s: err %.1f deg, danger=%d, radius %.1f mm, best %d previous %d [0=01 grasp 1=01 extend 2=23 grasp 3=23 extend]; lever [%.3f %.3f %.3f %.3f] m; required N [%.2f %.2f %.2f %.2f], extension minimum [%.2f %.2f] N; score [%.2f %.2f %.2f %.2f]; feasible [%d %d %d %d]",
      actuating ? "ACTIVE (guarded actuation)" : "SHADOW (not actuating)",
      roll_error * 180.0 / M_PI, imminent_fall ? 1 : 0,
      radius * 1000.0, best, previous_candidate_index,
      values[0].signed_lever, values[1].signed_lever,
      values[2].signed_lever, values[3].signed_lever,
      values[0].command_thrust, values[1].command_thrust,
      values[2].command_thrust, values[3].command_thrust,
      candidates[1].extension_min_thrust,
      candidates[3].extension_min_thrust,
      values[0].score, values[1].score,
      values[2].score, values[3].score,
      values[0].feasible ? 1 : 0, values[1].feasible ? 1 : 0,
      values[2].feasible ? 1 : 0, values[3].feasible ? 1 : 0);
  ROS_INFO("Unified recovery candidate reasons [C0=%s C1=%s C2=%s C3=%s]; pressure transient support/actuated [C0=%d/%d C1=%d/%d C2=%d/%d C3=%d/%d]; measured [%.1f %.1f %.1f %.1f] kPa, commanded [%.1f %.1f %.1f %.1f] kPa",
      rr::evaluationFailureReasonName(values[0].failure_reason),
      rr::evaluationFailureReasonName(values[1].failure_reason),
      rr::evaluationFailureReasonName(values[2].failure_reason),
      rr::evaluationFailureReasonName(values[3].failure_reason),
      candidates[0].support_pressure_transitioning ? 1 : 0,
      candidates[0].actuated_pressure_transitioning ? 1 : 0,
      candidates[1].support_pressure_transitioning ? 1 : 0,
      candidates[1].actuated_pressure_transitioning ? 1 : 0,
      candidates[2].support_pressure_transitioning ? 1 : 0,
      candidates[2].actuated_pressure_transitioning ? 1 : 0,
      candidates[3].support_pressure_transitioning ? 1 : 0,
      candidates[3].actuated_pressure_transitioning ? 1 : 0,
      measured_pressure_[0], measured_pressure_[1],
      measured_pressure_[2], measured_pressure_[3],
      last_pressure_[0], last_pressure_[1],
      last_pressure_[2], last_pressure_[3]);
  return result;
}
