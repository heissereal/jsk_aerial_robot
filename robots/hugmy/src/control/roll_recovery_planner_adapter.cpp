#include <hugmy/control/attitude_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>

// The planner itself has no ROS, actuator, or state-machine dependencies.
// This adapter converts Hugmy's measured bends, circle fit and calibrated
// pressure/thrust table into the four candidate geometries.  This is the
// active candidate selector; runtime response checks and SAFE_TRANSFER remain
// in the existing guarded pressure/thrust state machine.
hugmy::recovery::Selection
AttitudePressureController::evaluateUnifiedRecoveryCandidates(
    double roll_error, double desired_roll_moment,
    const hugmy::recovery::Vec3& circle_center_body,
    int previous_candidate_index)
{
  namespace rr = hugmy::recovery;
  rr::Selection result;
  if (!recovery_circle_event_estimate_.valid ||
      !rr::finite(circle_center_body) ||
      !std::isfinite(desired_roll_moment))
    return result;

  const double radius = recovery_circle_event_estimate_.radius;
  if (!std::isfinite(radius) || radius < 1.0e-3) return result;
  const double requested_moment_sign = desired_roll_moment >= 0.0 ? 1.0 : -1.0;

  const std::array<std::array<size_t, 2>, 2> pairs{{
      {{0, 1}}, {{2, 3}}}};
  std::array<rr::Candidate, 4> candidates;
  std::array<rr::Evaluation, 4> values;
  rr::Config config;
  config.thrust_limit = roll_recovery_config_.limit;
  config.min_signed_lever = recovery_circle_min_signed_lever_m_;
  config.max_support_rotor_distance =
      recovery_geometry_max_support_rotor_distance_m_;
  config.minimum_command_thrust = roll_recovery_config_.fallback_thrust;
  config.grasp_probe_thrust = recovery_grasp_probe_initial_thrust_n_;
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

  // Contact pressure is used only as a binary plausibility check.  It is not
  // used as a force weight or to move an averaged centre of pressure.
  std::array<rr::Vec3, 4> projected_contacts;
  std::array<bool, 4> projected_contact_valid{{false, false, false, false}};
  for (size_t arm = 0; arm < projected_contacts.size(); ++arm)
    {
      const double bend = measuredArmBendAngle(arm);
      const rr::Vec3 contact = armContactPointFromAngle(arm, bend);
      if (!rr::finite(contact) || !std::isfinite(measured_pressure_[arm]) ||
          measured_pressure_[arm] < recovery_circle_contact_pressure_kpa_)
        continue;
      const double dy = contact[1] - circle_center_body[1];
      const double dz = contact[2] - circle_center_body[2];
      const double radial = std::hypot(dy, dz);
      if (radial < 1.0e-4) continue;
      projected_contacts[arm] = {{contact[0],
          circle_center_body[1] + radius * dy / radial,
          circle_center_body[2] + radius * dz / radial}};
      projected_contact_valid[arm] = rr::finite(projected_contacts[arm]);
    }

  for (size_t pair_index = 0; pair_index < pairs.size(); ++pair_index)
    {
      const size_t support_index = 1 - pair_index;

      for (size_t mode = 0; mode < 2; ++mode)
        {
          const size_t index = 2 * pair_index + mode;
          rr::Candidate& c = candidates[index];
          c.pair = pair_index;
          c.release = (mode == 1);
          c.conditional_grasp_probe = false;
          c.support_body = {{0.0, 0.0, 0.0}};
          c.support_vertex_count = 0;
          c.support_pressure_transitioning = false;
          // A grasped candidate retains all four contacts.  A released pair
          // removes its own two contacts, so only the opposite pair can form
          // its support region.  Evaluate every vertex of that region; the
          // planner takes the least favourable signed lever.
          for (size_t arm = 0; arm < projected_contacts.size(); ++arm)
            {
              const bool belongs_to_opposite_pair =
                  arm == pairs[support_index][0] ||
                  arm == pairs[support_index][1];
              if (c.release && !belongs_to_opposite_pair) continue;
              c.support_pressure_transitioning =
                  c.support_pressure_transitioning ||
                  pressure_transitioning(arm);
              if (!projected_contact_valid[arm]) continue;
              c.support_vertices[c.support_vertex_count++] =
                  projected_contacts[arm];
              for (size_t axis = 0; axis < 3; ++axis)
                c.support_body[axis] += projected_contacts[arm][axis];
            }
          if (c.support_vertex_count > 0)
            for (size_t axis = 0; axis < 3; ++axis)
              c.support_body[axis] /=
                  static_cast<double>(c.support_vertex_count);
          // These are the two discrete contact topologies being compared:
          // four retained contacts or two opposite-pair contacts.
          const size_t required_support_vertices = c.release ? 2 : 4;
          c.contact_feasible =
              c.support_vertex_count == required_support_vertices;
          c.actuated_pressure_transitioning = false;
          for (const size_t arm : pairs[pair_index])
            c.actuated_pressure_transitioning =
                c.actuated_pressure_transitioning ||
                pressure_transitioning(arm);
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
              bool pair_below_object_side = true;
              double signed_center_lever = 0.0;
              for (size_t i = 0; i < 2; ++i)
                {
                  const size_t arm = pairs[pair_index][i];
                  const double bend = measuredArmBendAngle(arm);
                  c.rotors[i] = rotor_at_bend(arm, bend);
                  const double depth = rotorDepthAngleAbout(
                      arm, bend, circle_center_body);
                  pair_below_object_side = pair_below_object_side &&
                      std::isfinite(depth) &&
                      depth >= 0.5 * M_PI +
                          recovery_circle_side_margin_rad_;
                  // This is the exact cross-section test corresponding to
                  // "which way does this rotor push around the object?".
                  // A downward/upward component alone is ambiguous between
                  // the two sides; (r-C)xF retains that side information.
                  signed_center_lever += requested_moment_sign *
                      rr::rollLever(c.rotors[i].position,
                                    c.rotors[i].unit_force,
                                    circle_center_body);
                }
              c.below_object_side = pair_below_object_side;
              c.signed_center_lever = signed_center_lever;
              c.conditional_grasp_probe = pair_below_object_side &&
                  std::isfinite(signed_center_lever) &&
                  signed_center_lever > 0.0;
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

  // A certain grasp is preferred to a conditional grasp probe, and a
  // conditional below-side grasp is preferred to changing the contact
  // topology by releasing the pair.  Emergency extension remains enforced
  // by the guarded state machine which consumes this plan.
  const auto selection_priority = [&](size_t index) {
    if (!values[index].feasible) return 3;
    if (!candidates[index].release &&
        !values[index].response_probe_required) return 0;
    if (values[index].response_probe_required) return 1;
    return 2;
  };
  int best = -1;
  for (size_t i = 0; i < values.size(); ++i)
    if (values[i].feasible &&
        (best < 0 ||
         selection_priority(i) <
             selection_priority(static_cast<size_t>(best)) ||
         (selection_priority(i) ==
              selection_priority(static_cast<size_t>(best)) &&
          values[i].score < values[static_cast<size_t>(best)].score)))
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

  ROS_INFO("Unified robust recovery ACTIVE: err %.1f deg, danger=%d, radius %.1f mm (diagnostic only), best %d previous %d [0=01 grasp 1=01 extend 2=23 grasp 3=23 extend]; robust lever [%.3f %.3f %.3f %.3f] m, range max [%.3f %.3f %.3f %.3f] m, grasp centre lever [%.3f %.3f] m, below-side [%d %d], support vertices [%zu %zu %zu %zu]; required N [%.2f %.2f %.2f %.2f], extension minimum [%.2f %.2f] N; score [%.2f %.2f %.2f %.2f]; feasible [%d %d %d %d], conditional grasp probe [%d %d]",
      roll_error * 180.0 / M_PI, imminent_fall ? 1 : 0,
      radius * 1000.0, best, previous_candidate_index,
      values[0].signed_lever, values[1].signed_lever,
      values[2].signed_lever, values[3].signed_lever,
      values[0].maximum_signed_lever, values[1].maximum_signed_lever,
      values[2].maximum_signed_lever, values[3].maximum_signed_lever,
      candidates[0].signed_center_lever,
      candidates[2].signed_center_lever,
      candidates[0].below_object_side ? 1 : 0,
      candidates[2].below_object_side ? 1 : 0,
      candidates[0].support_vertex_count,
      candidates[1].support_vertex_count,
      candidates[2].support_vertex_count,
      candidates[3].support_vertex_count,
      values[0].command_thrust, values[1].command_thrust,
      values[2].command_thrust, values[3].command_thrust,
      candidates[1].extension_min_thrust,
      candidates[3].extension_min_thrust,
      values[0].score, values[1].score,
      values[2].score, values[3].score,
      values[0].feasible ? 1 : 0, values[1].feasible ? 1 : 0,
      values[2].feasible ? 1 : 0, values[3].feasible ? 1 : 0,
      values[0].response_probe_required ? 1 : 0,
      values[2].response_probe_required ? 1 : 0);
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
