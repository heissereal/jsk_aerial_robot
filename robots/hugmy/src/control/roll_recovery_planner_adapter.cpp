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
    bool actuating)
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

  for (size_t pair_index = 0; pair_index < pairs.size(); ++pair_index)
    {
      // The other pair is hypothesized to remain loaded while the candidate
      // pair actuates.  This is a mode-specific SUPPORT HYPOTHESIS, not a
      // claim that the pressure readings measure normal reaction forces.
      const size_t support_index = 1 - pair_index;
      rr::Vec3 support{{0.0, 0.0, 0.0}};
      bool support_valid = true;
      for (const size_t arm : pairs[support_index])
        {
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
          c.support_body = support;
          c.release_fraction = c.release ? 1.0 : 0.0;
          c.transition_seconds = c.release ? recovery_extension_prediction_sec_ : 0.0;
          if (c.release)
            {
              // The same thrust straightens a compliant rotor-carrying arm
              // and produces body roll. Re-evaluate its rotor pose for each
              // sampled thrust instead of assuming instant alpha=0 or adding
              // a fictitious second extension thrust.
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
                  });
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

  ROS_INFO("Unified recovery %s: err %.1f deg, danger=%d, radius %.1f mm, best %d [0=01 grasp 1=01 extend 2=23 grasp 3=23 extend]; lever [%.3f %.3f %.3f %.3f] m; required N [%.2f %.2f %.2f %.2f]; feasible [%d %d %d %d]",
      actuating ? "ACTIVE (guarded actuation)" : "SHADOW (not actuating)",
      roll_error * 180.0 / M_PI, imminent_fall ? 1 : 0,
      radius * 1000.0, best,
      values[0].signed_lever, values[1].signed_lever,
      values[2].signed_lever, values[3].signed_lever,
      values[0].command_thrust, values[1].command_thrust,
      values[2].command_thrust, values[3].command_thrust,
      values[0].feasible ? 1 : 0, values[1].feasible ? 1 : 0,
      values[2].feasible ? 1 : 0, values[3].feasible ? 1 : 0);
  return result;
}
