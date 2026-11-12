// Posture-recovery implementation for the shared perching controller.
//
// The motion state machine calls requestActionCPostureRecovery() to transfer
// actuator ownership. updatePostureRecovery() runs the existing recovery and
// measured-contact restoration states. This split does not change the ROS
// node, state IDs, command topics, gains, or fast rotor-thrust rise limits.
#include <hugmy/control/attitude_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

void AttitudePressureController::updatePostureRecovery(
    const ros::Time& now, double dt, double maximum_pressure_step,
    bool body_imu_fresh, bool pressure_valid, bool neurons_fresh)
{
  const double roll_error = wrapAngle(target_roll_ - roll_);
  const double pitch_error = wrapAngle(target_pitch_ - pitch_);
  const double maximum_step = maximum_pressure_step;
  const PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
      // The bottom bladder deliberately changes pitch. Apply the same axis
      // mask to ordinary recovery and Action-C handoff: pitch must neither
      // start recovery nor keep it active after roll reaches the target.
      const bool roll_only_recovery = correctionRollOnly();
      const double attitude_error = roll_only_recovery
          ? std::abs(roll_error) : std::hypot(roll_error, pitch_error);
      const bool model_c_recovery = action_c_continuous_recovery_enabled_ &&
          inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !fixed_arm_test_enabled_ && roll_only_recovery;
      const double recovery_roll_rate = model_c_recovery
          ? hugmy::recoveryRollRate(roll_, pitch_, body_angular_velocity_)
          : body_angular_velocity_[0];
      const double angular_rate = roll_only_recovery
          ? std::abs(recovery_roll_rate)
          : std::hypot(body_angular_velocity_[0], body_angular_velocity_[1]);

      const double recovery_stop_angle = (action_c_resume_pending_ || model_c_recovery)
          ? action_c_posture_resume_rad_ : correction_stop_angle_;
      const bool continuous_c_recovery = action_c_continuous_recovery_enabled_ &&
          inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !fixed_arm_test_enabled_ && roll_only_recovery;
      if (state_ != GRASP)
        {
          recovery_unified_unlock_since_ = ros::Time(0);
          recovery_unified_unlock_ready_ = false;
          recovery_candidate_selection_settled_since_ = ros::Time(0);
        }
      if (action_c_resume_pending_ &&
          std::abs(wrapAngle(roll_ - target_roll_)) >=
              action_c_emergency_stop_roll_rad_)
        {
          stopInchwormToGrasp(now,
              "Action C emergency roll reached configured stop limit");
          return;
        }
      const bool recovery_pressure_fresh =
          !measured_pressure_stamp_.isZero() &&
          (now - measured_pressure_stamp_).toSec() >= 0.0 &&
          (now - measured_pressure_stamp_).toSec() <= 0.25 &&
          !measured_bottom_pressure_stamp_.isZero() &&
          (now - measured_bottom_pressure_stamp_).toSec() >= 0.0 &&
          (now - measured_bottom_pressure_stamp_).toSec() <= 0.25;
      switch (state_)
        {
        case GRASP:
          publishCorrectionThrust(zero_thrust);
          if (rocking_gait_enabled_ || inchworm_action_c_enabled_)
            publishBottomTarget(
                (action_c_resume_pending_ ||
                 recovery_candidate_reselect_pending_) ? 0.0 :
                (hybridLocomotionActive() && hybrid_hold_bottom_ ? hybrid_single_config_.bottom_kpa : rocking_bottom_low_kpa_));
          {
          // Keep the contact-conditioned regrasp pressure through the
          // recovery boundary. Returning to a hard-coded 30 kPa here would
          // recreate the contact moment which the handoff just removed.
          const PressureArray steady_grasp =
              (action_c_resume_pending_ || recovery_candidate_reselect_pending_ ||
               recovery_contact_grasp_active_)
              ? grasp_pressure_
              : ((hybridLocomotionActive() && hybrid_hold_pressure_)
                    ? hybrid_grasp_pressure_
                    : PressureArray{{base_pressure_kpa_,base_pressure_kpa_,
                                     base_pressure_kpa_,base_pressure_kpa_}});
          publishPressureTarget(steady_grasp, maximum_step);
          bool grasp_pressure_ready = pressure_valid;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            grasp_pressure_ready = grasp_pressure_ready &&
                std::abs(measured_pressure_[arm] - steady_grasp[arm]) <=
                pressure_tolerance_kpa_;
          if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
            grasp_pressure_ready = pressure_valid && actionCPressureReady(
                steady_grasp, now);
          if (!grasp_pressure_ready)
            ROS_INFO_THROTTLE(1.0,
                "GRASP waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                base_pressure_kpa_, pressure_tolerance_kpa_);
          if (manual_posture_recovery_requested_ &&
              attitude_error - recovery_stop_angle <= 0.0)
            {
              manual_posture_recovery_requested_ = false;
              ROS_INFO("Operator posture-recovery request already satisfies the %.2f-deg final band",
                       recovery_stop_angle * 180.0 / M_PI);
            }
          // Re-selection is a discrete, supported boundary.  Merely crossing
          // the angle threshold while thrust/pressure are still changing must
          // not unlock the pair and make actuation chatter.  GRASP publishes
          // zero rotor thrust above; require fresh, settled support plus a
          // quiet small-angle dwell before allowing the next PREPARE to
          // compare all four modes again.
          const bool unified_unlock_observation =
              recovery_unified_active_enabled_ &&
              recovery_preferred_pair_locked_ &&
              recovery_pressure_fresh && grasp_pressure_ready &&
              std::abs(roll_error) < recovery_unified_unlock_angle_rad_ &&
              std::abs(recovery_roll_rate) <
                  recovery_unified_unlock_rate_rad_s_;
          if (unified_unlock_observation)
            {
              if (recovery_unified_unlock_since_.isZero())
                recovery_unified_unlock_since_ = now;
              recovery_unified_unlock_ready_ =
                  (now - recovery_unified_unlock_since_).toSec() >=
                      recovery_unified_unlock_hold_sec_;
            }
          else
            {
              recovery_unified_unlock_since_ = ros::Time(0);
              recovery_unified_unlock_ready_ = false;
            }
          const bool unified_unlock_dwell_pending =
              unified_unlock_observation &&
              !recovery_unified_unlock_ready_;
          if (unified_unlock_dwell_pending)
            ROS_INFO_THROTTLE(0.5,
                "Unified recovery holds pair %zu+%zu during small-angle support dwell: error %.2f deg, rate %.2f deg/s, %.2f/%.2f s",
                recovery_preferred_pair_index_ == 0 ? 0ul : 2ul,
                recovery_preferred_pair_index_ == 0 ? 1ul : 3ul,
                roll_error * 180.0 / M_PI,
                recovery_roll_rate * 180.0 / M_PI,
                recovery_unified_unlock_since_.isZero() ? 0.0 :
                    (now - recovery_unified_unlock_since_).toSec(),
                recovery_unified_unlock_hold_sec_);
          // Completion and restart use different boundaries.  Recovery is
          // complete only inside +/-3 deg, but a 3.1-deg contact rebound must
          // not immediately start another pair.  After a completed/handoff
          // pass, require more than 5 deg *and* a persistent worsening trend;
          // the independent hard-roll trigger remains immediate.
          const double completed_recovery_restart_angle = std::max(
              5.0 * M_PI / 180.0,
              action_c_posture_resume_rad_ +
                  2.0 * roll_recovery_config_.stop_margin);
          const double active_correction_trigger =
              manual_posture_recovery_requested_
              ? recovery_stop_angle
              : (recovery_candidate_reselect_pending_ ||
               continuous_recovery_retry_opposite_pending_)
              ? action_c_posture_resume_rad_
              : (action_c_resume_pending_
              ? (continuous_recovery_waiting_for_settle_
                    ? completed_recovery_restart_angle
                    : (std::isfinite(action_c_requested_recovery_trigger_rad_)
                        ? std::min(action_c_posture_recovery_trigger_rad_,
                            action_c_requested_recovery_trigger_rad_)
                        : action_c_posture_recovery_trigger_rad_))
              : correction_trigger_angle_);
          // Recovery thrust starts immediately and does not wait for bottom
          // exhaust. Automatic locomotion resume is different: finish the
          // retained-position sample at the same deflated boundary used at
          // the start of a stride before a new inflation can begin.
          const bool recovery_bottom_ready =
              (!action_c_resume_pending_ &&
               !recovery_candidate_reselect_pending_) ||
              (recovery_pressure_fresh && std::isfinite(bottom_pressure_kpa_) &&
               std::abs(bottom_pressure_kpa_) <= action_c_bottom_deflation_tolerance_kpa_);
          const bool c_trend_mode = inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;
          if (c_trend_mode)
            action_c_roll_trend_.update(body_imu_fresh && pressure_valid && neurons_fresh &&
                (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready)),
                wrapAngle(roll_ - target_roll_), recovery_roll_rate, now.toSec());
          const bool c_hard_roll = c_trend_mode &&
              std::abs(wrapAngle(roll_ - target_roll_)) >= action_c_posture_hard_roll_rad_;
          // Pressure inflation changes the contact moment and may carry a
          // tilted body farther from level.  Apply the same absolute-angle
          // criterion for both roll signs instead of waiting for a signed
          // worsening trend on one side.  Outside this transient, a stable
          // 10--15-deg contact offset remains permitted as before.
          const bool c_preload_roll = continuous_c_recovery &&
              !grasp_pressure_ready &&
              attitude_error >= action_c_preload_recovery_trigger_rad_;
          // Candidate selection has two timing classes.  Ordinary recovery
          // waits for the pneumatic support target and arm motion to settle,
          // so the four steady-state geometries are not evaluated from a
          // transient bend.  A hard/preload roll or a rate prediction which
          // reaches the 15-deg extension boundary bypasses this dwell and
          // preserves the existing fast emergency response.
          const bool c_roll_worsening = std::isfinite(recovery_roll_rate) &&
              roll_error * recovery_roll_rate < 0.0;
          const bool c_roll_materially_worsening = c_roll_worsening &&
              std::abs(recovery_roll_rate) >=
                  recovery_grasp_worsening_rate_rad_s_;
          const double c_predicted_roll_error =
              std::isfinite(recovery_roll_rate)
              ? roll_error - recovery_extension_prediction_sec_ *
                    recovery_roll_rate
              : roll_error;
          const bool c_extension_emergency_raw = continuous_c_recovery &&
              c_roll_materially_worsening &&
              std::abs(c_predicted_roll_error) >=
                  recovery_extension_emergency_error_rad_;
          if (c_extension_emergency_raw)
            {
              if (recovery_extension_emergency_since_.isZero())
                recovery_extension_emergency_since_ = now;
              recovery_extension_emergency_confirmed_ =
                  (now - recovery_extension_emergency_since_).toSec() >=
                      recovery_extension_emergency_confirm_sec_;
            }
          else
            {
              recovery_extension_emergency_since_ = ros::Time(0);
              recovery_extension_emergency_confirmed_ = false;
            }
          const bool c_extension_emergency =
              c_extension_emergency_raw &&
              recovery_extension_emergency_confirmed_;
          const bool c_extension_emergency_confirmation_pending =
              c_extension_emergency_raw &&
              !recovery_extension_emergency_confirmed_;

          bool recovery_candidate_state_quiet = continuous_c_recovery &&
              recovery_pressure_fresh && neurons_fresh;
          double maximum_candidate_pressure_error = 0.0;
          double maximum_candidate_gyro = 0.0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              const double measured_error =
                  std::abs(measured_pressure_[arm] - steady_grasp[arm]);
              const double command_error =
                  std::abs(last_pressure_[arm] - steady_grasp[arm]);
              const double gyro_norm = std::sqrt(
                  neuron_gyro_[arm][0] * neuron_gyro_[arm][0] +
                  neuron_gyro_[arm][1] * neuron_gyro_[arm][1] +
                  neuron_gyro_[arm][2] * neuron_gyro_[arm][2]);
              maximum_candidate_pressure_error =
                  std::isfinite(measured_error) &&
                  std::isfinite(command_error)
                  ? std::max(maximum_candidate_pressure_error,
                        std::max(measured_error, command_error))
                  : std::numeric_limits<double>::infinity();
              maximum_candidate_gyro = std::isfinite(gyro_norm)
                  ? std::max(maximum_candidate_gyro, gyro_norm)
                  : std::numeric_limits<double>::infinity();
              recovery_candidate_state_quiet =
                  recovery_candidate_state_quiet &&
                  std::isfinite(measured_error) &&
                  std::isfinite(command_error) &&
                  std::isfinite(gyro_norm) &&
                  measured_error <= pressure_tolerance_kpa_ &&
                  command_error <= pressure_tolerance_kpa_ &&
                  gyro_norm <= arm_gyro_tolerance_;
            }
          if (recovery_candidate_state_quiet)
            {
              if (recovery_candidate_selection_settled_since_.isZero())
                recovery_candidate_selection_settled_since_ = now;
            }
          else
            recovery_candidate_selection_settled_since_ = ros::Time(0);
          const double recovery_candidate_settle_elapsed =
              recovery_candidate_selection_settled_since_.isZero() ? 0.0 :
              (now - recovery_candidate_selection_settled_since_).toSec();
          const bool recovery_candidate_state_settled =
              recovery_candidate_state_quiet &&
              recovery_candidate_settle_elapsed >=
                  recovery_candidate_selection_settle_sec_;
          const bool recovery_candidate_selection_ready =
              (!c_extension_emergency_confirmation_pending || c_hard_roll) &&
              (!recovery_unified_active_enabled_ ||
               hugmy::recoveryCandidateSelectionAllowed(
                   recovery_candidate_state_settled, c_hard_roll,
                   c_preload_roll, c_extension_emergency));
          const bool stable_c_roll = c_trend_mode && !c_hard_roll &&
              action_c_roll_trend_.ready && !action_c_roll_trend_.worsening;
          const bool c_recovery_requested = !c_trend_mode || c_hard_roll ||
              c_preload_roll || action_c_roll_trend_.worsening ||
              manual_posture_recovery_requested_;
          // A contact-conditioned grasp is a valid recovery result only in
          // the requested final roll band.  A quiet 10--20-deg bias is not an
          // equilibrium to accept: it still leaves little margin for the
          // next locomotion stroke.  In particular, a pending fresh/opposite
          // candidate selected by the handoff must never be hidden by this
          // stationary-contact exception.
          const double retained_equilibrium_limit =
              action_c_posture_resume_rad_;
          const bool retained_contact_equilibrium =
              action_c_resume_pending_ &&
              !recovery_candidate_reselect_pending_ &&
              !continuous_recovery_retry_opposite_pending_ &&
              recovery_handoff_pressure_trimmed_ && stable_c_roll &&
              attitude_error <= retained_equilibrium_limit;
          const bool completed_recovery_restart_allowed =
              !continuous_recovery_waiting_for_settle_ ||
              recovery_candidate_reselect_pending_ ||
              continuous_recovery_retry_opposite_pending_ || c_hard_roll ||
              attitude_error > completed_recovery_restart_angle;
          // A geometry re-estimation changes the contact state by restoring
          // equal four-arm support.  Candidate evidence from the old state is
          // invalid only after the measured chamber pressures have reached
          // that new state.  Until then, hold zero thrust in GRASP instead of
          // immediately selecting again from unchanged geometry.
          const bool geometry_reestimate_ready =
              !recovery_geometry_reestimate_pending_ ||
              (grasp_pressure_ready && recovery_pressure_fresh);
          // A timed-out recovery pass can leave the driven pair partly
          // released and carrying nonzero thrust.  Starting a new pass from
          // that transient contact state releases the same pair a second
          // time and can turn a small residual error into a fall.  Finish
          // the commanded regrasp before retrying a moderate residual.  The
          // independent hard-roll path remains immediate, so this guard
          // never delays an emergency catch.
          const bool recovery_restart_support_ready =
              !continuous_recovery_waiting_for_settle_ ||
              grasp_pressure_ready || c_hard_roll;
          // A handoff already confirmed the recovery request. Do not demand
          // another pressure/settling/trend cycle before correcting it.
          const bool continuous_recovery_requested = continuous_c_recovery &&
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              recovery_pressure_fresh && !continuous_recovery_blocked_ &&
              recovery_candidate_selection_ready &&
              geometry_reestimate_ready && recovery_restart_support_ready &&
              angular_rate < correction_max_angular_rate_ &&
              attitude_error > active_correction_trigger &&
              completed_recovery_restart_allowed &&
              (!manual_posture_recovery_requested_ || grasp_pressure_ready) &&
              (manual_posture_recovery_requested_ ||
               recovery_candidate_reselect_pending_ ||
               continuous_recovery_retry_opposite_pending_ ||
               (action_c_resume_pending_ && !continuous_recovery_started_ &&
                !retained_contact_equilibrium) || c_hard_roll);
          const bool recovery_settled = action_c_boundary_settle_gate_.update(
              body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && recovery_bottom_ready &&
              angular_rate <= action_c_pre_roll_rate_tolerance_rad_s_,
              now.toSec(), action_c_posture_settle_sec_);
          if (action_c_resume_pending_ && !recovery_settled)
            ROS_INFO_THROTTLE(1.0,
                "Action C auto-resume waiting: bottom %.1f kPa (need <= %.1f kPa for retained endpoint), regrasp=%d, roll rate %.3f rad/s, blocked=%d; saved goal retained",
                bottom_pressure_kpa_,action_c_bottom_deflation_tolerance_kpa_,
                grasp_pressure_ready, angular_rate, continuous_recovery_blocked_);
          if (action_c_resume_pending_ && !continuous_c_recovery && !recovery_settled && !continuous_recovery_requested &&
              (now - state_start_).toSec() >= action_c_prepare_timeout_sec_)
            {
              stopInchwormToGrasp(now, "Action C recovery deflation/regrasp settling timeout");
              break;
            }
          // Recovery succeeds only in the requested final band. Stationary
          // contact outside it is still a residual posture error.
          const bool resume_ready = action_c_resume_gate_.update(
              action_c_resume_pending_ && target_received_ &&
              (!continuous_recovery_blocked_ || continuous_recovery_waiting_for_settle_) &&
              body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && recovery_settled &&
              (continuous_recovery_waiting_for_settle_
                  ? attitude_error <= action_c_posture_resume_rad_
                  : (attitude_error <= active_correction_trigger ||
                     retained_contact_equilibrium)) &&
              recovery_pressure_fresh &&
              angular_rate <= (continuous_recovery_waiting_for_settle_
                  ? std::min(0.05, action_c_pre_roll_rate_tolerance_rad_s_)
                  : action_c_pre_roll_rate_tolerance_rad_s_),
              now.toSec(), continuous_recovery_waiting_for_settle_ ? 1.0 : correction_trigger_confirm_sec_);
          // A human-arm trial needs a motion-free PPG interval after posture
          // recovery.  Count only while the ordinary recovery completion
          // conditions remain true. GRASP already commands zero rotor thrust,
          // zero bottom pressure and the steady arm grasp above.
          const bool ppg_resume_ready = action_c_ppg_settle_gate_.update(
              resume_ready, now.toSec(), ppg_settle_sec_);
          if (action_c_resume_pending_ && resume_ready && !ppg_resume_ready)
            ROS_INFO_THROTTLE(1.0,
                "Action C PPG settle after posture recovery: holding bottom 0 kPa and thrust 0 for %.1f s",
                ppg_settle_sec_);
          if (ppg_resume_ready)
            {
              if (resumeActionCAfterRecovery(now)) break;
            }
          const bool correction_confirmed = correction_entry_gate_.update(
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              (!manual_posture_recovery_requested_ || grasp_pressure_ready) &&
              (!continuous_c_recovery || (!continuous_recovery_blocked_ &&
               recovery_pressure_fresh && geometry_reestimate_ready &&
               recovery_candidate_selection_ready &&
               angular_rate < correction_max_angular_rate_)) &&
              (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready &&
               (!action_c_resume_pending_ || recovery_settled))) &&
              attitude_error > active_correction_trigger && c_recovery_requested,
              now.toSec(), correction_trigger_confirm_sec_);
          if (continuous_c_recovery && recovery_unified_active_enabled_ &&
              c_recovery_requested &&
              attitude_error > active_correction_trigger &&
              !recovery_candidate_selection_ready)
            ROS_INFO_THROTTLE(0.5,
                "Normal recovery waits for candidate boundary: pressure error %.2f/%.2f kPa, arm gyro %.3f/%.3f rad/s, quiet dwell %.2f/%.2f s; emergency bypass hard=%d preload=%d extension=%d pending=%d",
                maximum_candidate_pressure_error, pressure_tolerance_kpa_,
                maximum_candidate_gyro, arm_gyro_tolerance_,
                recovery_candidate_settle_elapsed,
                recovery_candidate_selection_settle_sec_,
                c_hard_roll, c_preload_roll, c_extension_emergency,
                c_extension_emergency_confirmation_pending);
          if (!unified_unlock_dwell_pending &&
              (correction_confirmed || continuous_recovery_requested) &&
              (continuous_recovery_requested || last_correction_end_.isZero() ||
               (now - last_correction_end_).toSec() >= cooldown_duration_))
            {
              if (continuous_c_recovery &&
                  recovery_geometry_reestimate_pending_)
                {
                  // The new load-bearing state is now physically present.
                  // A pressure/contact candidate rejected before this
                  // equalisation is not evidence against the same nominal
                  // command under the new geometry, so reopen the candidate
                  // set exactly once at this measured boundary.
                  recovery_geometry_reestimate_pending_ = false;
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  recovery_circle_event_locked_ = false;
                  recovery_event_contact_pivot_locked_ = false;
                  recovery_selected_contact_pivot_enabled_ = false;
                  recovery_preferred_pair_locked_ = false;
                  ROS_INFO("Balanced four-arm support reached; cleared old contact-state candidate rejections and re-estimating recovery geometry");
                }
              ROS_INFO("Posture recovery accepted: error %.2f deg > %.2f deg; confirmation %.2f s, confirmed handoff=%d preload=%d; stop at %.2f deg",
                       attitude_error * 180.0 / M_PI,
                       active_correction_trigger * 180.0 / M_PI,
                       correction_trigger_confirm_sec_,
                       continuous_recovery_requested,
                       c_preload_roll,
                       recovery_stop_angle * 180.0 / M_PI);
              Vector2 desired_moment = desiredThrustMoment();
              if (continuous_c_recovery)
                desired_moment = {{std::copysign(std::max(1e-6, std::abs(desired_moment[0])), roll_error), 0.0}};
              const double desired_moment_norm = std::hypot(
                  desired_moment[0], desired_moment[1]);
              if (desired_moment_norm > 1e-8)
                correction_axis_ = {{desired_moment[0] / desired_moment_norm,
                                     desired_moment[1] / desired_moment_norm}};
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  // Preserve the pressure of the balanced recovery handoff.
                  // Replacing its 15-kPa contact-conditioned support with a
                  // hard-coded 30 kPa here recreated the large roll moment
                  // before every retry. A new independent posture request
                  // still starts from the nominal base pressure.
                  grasp_pressure_[arm] =
                      (action_c_resume_pending_ ||
                       recovery_candidate_reselect_pending_)
                      ? steady_grasp[arm] : base_pressure_kpa_;
                  grasp_imu_angle_[arm] = measuredArmAngle(arm);
                  grasp_model_angle_[arm] = bendAngle(arm, grasp_pressure_[arm]);
                  // Moment-dependent target, bounded by the straight-arm
                  // geometry. Also straighten enough to keep rotor tilt below
                  // 90 deg (80 deg default margin), where nonnegative thrust
                  // retains useful upward and correction authority.
                  const double moment_based = std::max(
                      prepare_min_straighten_angle_,
                      prepare_angle_per_moment_ * desired_moment_norm);
                  const double feasibility_required = std::max(0.0,
                      grasp_model_angle_[arm] - prepare_max_rotor_tilt_);
                  prepare_straighten_angle_by_arm_[arm] =
                      std::min(prepare_max_straighten_angle_,
                          std::min(grasp_model_angle_[arm],
                              std::max(moment_based, feasibility_required)));

                  prepare_target_angle_[arm] = grasp_imu_angle_[arm] -
                      prepare_straighten_angle_by_arm_[arm];
                }
              grasp_baseline_valid_ = true;

              // First choose the correcting pair from the current grasp
              // geometry. During PREPARE, unload only that pair and give it a
              // small shaping thrust so it can become straighter.
              correction_thrust_ = allocateCorrectionThrust(
                  desired_moment, grasp_pressure_);
              prepare_pressure_ = grasp_pressure_;
              prepare_shape_thrust_.fill(0.0);
              prepare_arm_active_.fill(false);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (correction_thrust_[arm] > 0.01)
                  {
                    prepare_arm_active_[arm] = true;
                  prepare_pressure_[arm] = grasp_pressure_[arm];
                  prepare_shape_thrust_[arm] = prepare_shape_thrust_n_;
                  ROS_INFO("Arm %zu shape search: grasp IMU angle %.3f rad, target %.3f rad, pressure starts at %.1f kPa",
                           arm + 1, grasp_imu_angle_[arm], prepare_target_angle_[arm],
                           grasp_pressure_[arm]);
                  }
              best_prepare_angle_ = std::numeric_limits<double>::infinity();
              body_motion_detected_ = false;
              prepare_pair_pressure_ready_latched_ = false;
              correction_motion_candidate_start_ = ros::Time(0);
              correction_motion_candidate_error_ = attitude_error;
              correction_last_progress_ = now;
              correction_best_error_ = attitude_error;
              motion_hold_thrust_.fill(0.0);
              body_motion_last_seen_ = ros::Time(0);
              last_angle_progress_ = now;
              if (fixed_arm_test_enabled_)
                {
                  prepare_pressure_.fill(base_pressure_kpa_);
                  correction_thrust_.fill(0.0);
                  prepare_shape_thrust_.fill(0.0);
                  prepare_arm_active_.fill(false);
                  prepare_pressure_[fixed_test_arm_index_] = fixed_test_pressure_kpa_;
                  correction_thrust_[fixed_test_arm_index_] = fixed_test_thrust_n_;
                  prepare_arm_active_[fixed_test_arm_index_] = true;
                  ROS_WARN("Fixed arm test: arm %d target %.1f kPa, motor %d thrust %.2f N via %s",
                           fixed_test_arm_index_ + 1, fixed_test_pressure_kpa_,
                           fixed_test_arm_index_, fixed_test_thrust_n_,
                           use_pwm_test_for_correction_ ? "pwm_test" : "perching command");
                }
              enterState(PREPARE, now);
              if (continuous_c_recovery)
                {
                  const bool retry_opposite_pair =
                      continuous_recovery_retry_opposite_pending_;
                  const bool contact_candidate_retry =
                      recovery_candidate_reselect_pending_;
                  continuous_recovery_started_ = true;
                  if (!recovery_event_active_ || recovery_event_start_.isZero())
                    {
                      recovery_event_start_ = now;
                      recovery_event_active_ = true;
                      recovery_fault_reason_ =
                          hugmy::RecoveryFaultReason::NONE;
                    }
                  posture_recovery_phase_ = hugmy::PostureRecoveryPhase::SELECT;
                  // A previous timed pass may have been waiting only for a
                  // safe regrasp.  Once a new measured correction actually
                  // starts, return to the normal post-success resume gate;
                  // otherwise a stable 3--10 deg result could wait forever.
                  continuous_recovery_waiting_for_settle_ = false;
                  // Use the verified Spinal pair sign independently of model
                  // magnitude; measured geometry sets magnitude in PREPARE.
                  correction_axis_ = {{roll_error >= 0.0 ? 1.0 : -1.0, 0.0}};
                  // Start with the verified Spinal mapping.  If the previous
                  // pair produced a sustained measured wrong-sign response,
                  // the balanced handoff carries exactly one opposite-pair
                  // retry through GRASP.  Consume it here so it cannot leak
                  // into a later, independent posture event.
                  continuous_recovery_use_opposite_pair_ =
                      retry_opposite_pair && !contact_candidate_retry;
                  continuous_recovery_pair_switches_ =
                      retry_opposite_pair ? 1 : 0;
                  continuous_recovery_retry_opposite_pending_ = false;
                  selectContinuousRecoveryPair(roll_error);
                  recovery_candidate_reselect_pending_ = false;
                  if (!recovery_pair_selection_available_)
                    {
                      if (!recovery_geometry_reestimate_attempted_)
                        {
                          // No pair has a trustworthy instantaneous restoring
                          // lever.  Equalise all four supports once and let the
                          // circle/contact estimator observe that common state;
                          // never apply a speculative rotor command here.
                          recovery_geometry_reestimate_attempted_ = true;
                          recovery_geometry_reestimate_pending_ = true;
                          recovery_candidate_reselect_pending_ = true;
                          continuous_recovery_started_ = false;
                          continuous_recovery_waiting_for_settle_ = true;
                          prepare_arm_active_.fill(false);
                          prepare_shape_thrust_.fill(0.0);
                          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                            grasp_pressure_[arm] = std::min(
                                maximum_pressure_kpa_,
                                std::max(base_pressure_kpa_,
                                         grasp_pressure_[arm]));
                          prepare_pressure_ = grasp_pressure_;
                          publishCorrectionThrust(prepare_shape_thrust_);
                          publishPressureTarget(prepare_pressure_,
                              std::numeric_limits<double>::infinity());
                          enterState(GRASP, now);
                          ROS_WARN("No instantaneous restoring recovery pair; equalising four-arm support once before geometry re-estimation (zero thrust)");
                        }
                      else
                        {
                          // Candidate exhaustion is local to the current
                          // pressure/contact estimate; it is not a locomotion
                          // goal failure.  Rebuild four-arm support, acquire a
                          // fresh circle/contact estimate and try recovery
                          // again while retaining the saved target and hybrid
                          // optimizer trial.  Only the independent emergency
                          // roll limit or an operator stop may discard it.
                          continuous_recovery_blocked_ = false;
                          recovery_geometry_reestimate_pending_ = true;
                          recovery_candidate_reselect_pending_ = true;
                          continuous_recovery_started_ = false;
                          continuous_recovery_waiting_for_settle_ = true;
                          prepare_arm_active_.fill(false);
                          prepare_shape_thrust_.fill(0.0);
                          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                            grasp_pressure_[arm] = std::min(
                                maximum_pressure_kpa_,
                                std::max(base_pressure_kpa_,
                                         grasp_pressure_[arm]));
                          prepare_pressure_ = grasp_pressure_;
                          publishCorrectionThrust(prepare_shape_thrust_);
                          publishPressureTarget(prepare_pressure_,
                              std::numeric_limits<double>::infinity());
                          action_c_resume_gate_.reset();
                          action_c_boundary_settle_gate_.reset();
                          enterState(GRASP, now);
                          ROS_WARN("No restoring recovery pair after balanced-support re-estimation; locomotion goal retained, rebuilding support and retrying with a fresh contact estimate");
                        }
                      break;
                    }
                  manual_posture_recovery_requested_ = false;
                  // Hybrid locomotion may hand recovery ownership over as
                  // soon as the opposite two-arm anchor reaches the measured
                  // support floor.  Recheck that condition after the shared
                  // controller performs its own geometry selection: if its
                  // selected pair differs from the preview, keep all four
                  // chambers filling in GRASP instead of releasing the only
                  // established anchor.  Full 30-kPa grasp is not required.
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      static const std::array<std::array<size_t, 2>, 2>
                          recovery_pairs{{{{0, 1}}, {{2, 3}}}};
                      const size_t anchor_pair =
                          1 - recovery_selected_pair_index_;
                      const bool anchor_ready = std::all_of(
                          recovery_pairs[anchor_pair].begin(),
                          recovery_pairs[anchor_pair].end(),
                          [&](size_t arm) {
                            return std::isfinite(measured_pressure_[arm]) &&
                                measured_pressure_[arm] >=
                                    recovery_handoff_min_pressure_kpa_;
                          });
                      if (!anchor_ready)
                        {
                          continuous_recovery_started_ = false;
                          prepare_arm_active_.fill(false);
                          prepare_shape_thrust_.fill(0.0);
                          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                            prepare_pressure_[arm] = std::min(
                                maximum_pressure_kpa_,
                                std::max(base_pressure_kpa_,
                                         grasp_pressure_[arm]));
                          publishCorrectionThrust(prepare_shape_thrust_);
                          publishPressureTarget(
                              prepare_pressure_,
                              std::numeric_limits<double>::infinity());
                          enterState(GRASP, now);
                          ROS_WARN_THROTTLE(0.5,
                              "Shared recovery waiting only for anchor pair %zu+%zu [%.1f %.1f] / %.1f kPa after geometry re-selection; selected pair %zu+%zu remains free to release",
                              recovery_pairs[anchor_pair][0],
                              recovery_pairs[anchor_pair][1],
                              measured_pressure_[
                                  recovery_pairs[anchor_pair][0]],
                              measured_pressure_[
                                  recovery_pairs[anchor_pair][1]],
                              recovery_handoff_min_pressure_kpa_,
                              recovery_pairs[
                                  recovery_selected_pair_index_][0],
                              recovery_pairs[
                                  recovery_selected_pair_index_][1]);
                          break;
                        }
                    }
                  continuous_roll_recovery_.reset(attitude_error, now.toSec());
                  recovery_response_probe_since_ = ros::Time(0);
                  // A body which is still rolling away from the target needs
                  // correction authority before the slower contact-response
                  // estimator has identified breakaway.  Use the emergency
                  // slew for rate arrest even though extended-arm mode is
                  // already latched; excluding extension here made every
                  // urgent real-arm recovery fall back to the nominal slew.
                  roll_recovery_config_.rise =
                      (action_c_recovery_urgent_thrust_rise_ ||
                       recovery_rate_arrest_active_)
                      ? recovery_preinflate_thrust_rise_n_s_
                      : recovery_nominal_thrust_rise_n_s_;
                  recovery_wrong_geometry_since_ = ros::Time(0);
                  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
                  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
                  recovery_wrong_geometry_reference_error_rad_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_reference_rate_rad_s_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_best_rate_rad_s_ =
                      -std::numeric_limits<double>::infinity();
                  recovery_wrong_geometry_response_observed_ = false;
                  recovery_wrong_geometry_probe_target_valid_ = false;
                  recovery_pressure_brake_started_ = false;
                  // Every contact-conditioned retry initially sees motion
                  // inherited from the old pair (target crossing or measured
                  // sign reversal). Give the new pair enough authority to
                  // brake that motion before attributing its sign to the new
                  // actuator/contact state.
                  const bool braking_inherited_crossing_motion =
                      contact_candidate_retry;
                  resetRecoveryHandoffState();
                  recovery_predictive_.reset();
                  roll_recovery_config_.stop = recovery_stop_angle;
                  // A compliant high-friction contact can have a repeatable
                  // low-rate equilibrium outside the ideal +/-3-deg target.
                  // Keep the configured contact-minimum band here: forcing
                  // the same rotor pair beyond that measured minimum makes
                  // its effective moment reverse as the arm extends.  The
                  // hard roll guard remains independent, and the handoff
                  // below verifies the final grasped equilibrium.
                  roll_recovery_config_.contact_minimum_limit = std::max(
                      recovery_stop_angle,
                      roll_recovery_config_.contact_minimum_limit);
                  // A released/extended pair has a large, unambiguous
                  // free-space lever and should carry the body nearly to the
                  // real +/-3-deg target before pneumatic regrasp. Retain a
                  // half-margin for sensor/contact discretisation: requiring
                  // an exact 3.00-deg sample trapped the 70-mm fixture at
                  // 3.0--3.2 deg indefinitely. A flexed pair hands off up to
                  // two full margins early because its loaded contact lever
                  // can reverse as bend changes.
                  const bool below_side_release =
                      recovery_selected_contact_pivot_enabled_ &&
                      recovery_selected_pair_release_ &&
                      std::isfinite(recovery_selected_pair_mean_depth_rad_) &&
                      recovery_selected_pair_mean_depth_rad_ >=
                          0.5 * M_PI + recovery_circle_side_margin_rad_;
                  roll_recovery_config_.capture_margin_factor =
                      below_side_release
                      ? recovery_below_side_capture_margin_factor_
                      : (recovery_selected_contact_pivot_enabled_ &&
                         recovery_selected_pair_release_ ? 0.5 : 2.0);
                  roll_recovery_config_.horizon = correction_prediction_horizon_sec_;
                  // This ceiling belongs only to continuous Action-C roll
                  // recovery. Do not raise the locomotion/fixed-test ceiling
                  // through the shared correction_max_thrust_n parameter.
                  roll_recovery_config_.limit = std::min(
                      roll_recovery_config_.limit, prepare_max_thrust_n_);
                  if (braking_inherited_crossing_motion)
                    {
                      continuous_roll_recovery_.wrong_way_detection_thrust =
                          std::max(roll_recovery_config_.detect_thrust,
                              0.5 * roll_recovery_config_.limit);
                      continuous_roll_recovery_.wrong_way_detection_until =
                          now.toSec() + roll_recovery_config_.progress_window;
                    }
                  roll_recovery_config_.start_rate = correction_motion_start_rate_;
                  roll_recovery_config_.detect_thrust = correction_motion_detection_min_thrust_n_;
                  roll_recovery_config_.confirm_sec = correction_motion_confirm_sec_;
                  roll_recovery_config_.confirm_angle = correction_motion_min_error_reduction_;
                  roll_recovery_config_.progress_angle = correction_progress_epsilon_;
                  roll_recovery_config_.stall_sec = correction_stall_timeout_sec_;
                  roll_recovery_config_.abort_sec = action_c_recovery_no_progress_timeout_sec_;
                  roll_recovery_config_.max_rate = std::max(
                      correction_max_angular_rate_,
                      recovery_measurement_rate_limit_rad_s_);
                  roll_recovery_config_.max_target_rate = std::min(
                      roll_recovery_config_.max_target_rate, 0.5 * correction_max_angular_rate_);
                  prepare_shape_thrust_.fill(0.0);
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      prepare_pressure_[arm] = recovery_selected_pair_release_
                          ? recoverySelectedPairPressure(arm)
                          : std::min(maximum_pressure_kpa_,
                              std::max(base_pressure_kpa_,
                                       grasp_pressure_[arm]));
                    else
                      // A contact-conditioned pressure from the previous
                      // pair is not the next pair's anchor pressure. Restore
                      // the non-selected support to at least the nominal gait
                      // grasp while the selected pair is exhausted.
                      prepare_pressure_[arm] = std::min(maximum_pressure_kpa_,
                          std::max(base_pressure_kpa_, grasp_pressure_[arm]));
                  publishBottomTarget(0.0);
                  publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
                  ROS_INFO("Action C continuous roll recovery: exhaust bottom; selected pair target pressure %.1f kPa (%s); no zero-pressure wait; limit %.2f N per rotor, rise %.2f N/s%s",
                           recovery_selected_pair_pressure_kpa_,
                           recovery_selected_pair_release_
                               ? "partial/full release" : "nominal grasp",
                           roll_recovery_config_.limit,
                           roll_recovery_config_.rise,
                           recovery_extension_mode_latched_
                               ? " (10/7-style extended-arm slew)"
                               : (action_c_recovery_urgent_thrust_rise_
                                   ? " (bottom-inflation urgent)" : ""));
                }
            }
          }
          break;

        case PREPARE:
          {
          if (continuous_c_recovery)
            {
              posture_recovery_phase_ = hugmy::PostureRecoveryPhase::RECOVERY;
              publishBottomTarget(0.0);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  if (!prepare_arm_active_[arm])
                    prepare_pressure_[arm] = std::min(maximum_pressure_kpa_,
                        std::max(base_pressure_kpa_, grasp_pressure_[arm]));
                  else if (recovery_selected_pair_release_ &&
                           (!recovery_predictive_config_.enabled || recovery_predictive_config_.shadow) &&
                           continuous_roll_recovery_.stalled &&
                           continuous_roll_recovery_.thrust >= 0.8 * roll_recovery_config_.limit)
                    prepare_pressure_[arm] = std::max(
                        recoverySelectedPairPressure(arm),
                        prepare_pressure_[arm] - recovery_pressure_down_rate_ * std::max(0.0, dt));
                }
              const double signed_error = roll_error * correction_axis_[0];
              const double omega = recovery_roll_rate * correction_axis_[0];
              // Absolute recovery-event watchdog.  Unlike state_start_ and
              // the response-identification windows, this timestamp survives
              // pair changes, pneumatic transients and GRASP/RECOVER
              // handoffs.  No sensor noise or target-pressure update may
              // extend it.
              if (recovery_event_active_ &&
                  !recovery_event_start_.isZero() &&
                  (now - recovery_event_start_).toSec() >=
                      action_c_recovery_no_progress_timeout_sec_)
                {
                  beginPostureSafeTransfer(
                      now, "absolute recovery-event watchdog expired", false,
                      hugmy::RecoveryFaultReason::ABSOLUTE_WATCHDOG);
                  break;
                }
              // Phase transitions use the same filtered whole-body rate as
              // the inner controller.  The raw Spinal sample crosses zero on
              // arm/contact impacts; using it produced 34 arrest/return
              // transitions in one 39-s hardware recovery and repeatedly
              // discarded the authority needed to overcome static contact.
              const double phase_rate =
                  continuous_roll_recovery_.rate_initialized
                  ? continuous_roll_recovery_.filtered_rate : omega;
              const double phase_motion_rate = std::max(
                  0.01, 0.5 * roll_recovery_config_.start_rate);
              const bool recovery_decay_window_ready =
                  !continuous_roll_recovery_.progress_history.empty() &&
                  now.toSec() -
                          continuous_roll_recovery_.progress_history.front().time >=
                      std::max(roll_recovery_config_.progress_window,
                          recovery_wrong_geometry_progress_window_sec_);
              const double recovery_error_decay_rate =
                  recovery_decay_window_ready
                  ? continuous_roll_recovery_.recent_progress /
                        std::max(1.e-3,
                            roll_recovery_config_.progress_window)
                  : std::numeric_limits<double>::infinity();
              if (recovery_rate_arrest_active_)
                {
                  if (phase_rate >= phase_motion_rate)
                    {
                      if (recovery_rate_arrest_nonoutward_since_.isZero())
                        {
                          recovery_rate_arrest_nonoutward_since_ = now;
                          recovery_rate_arrest_reference_error_rad_ =
                              std::abs(signed_error);
                        }
                      const double retained_angle_progress =
                          std::isfinite(
                              recovery_rate_arrest_reference_error_rad_)
                          ? recovery_rate_arrest_reference_error_rad_ -
                                std::abs(signed_error)
                          : 0.0;
                      if ((now - recovery_rate_arrest_nonoutward_since_).toSec() >=
                              recovery_rate_arrest_release_sec_ &&
                          retained_angle_progress >=
                              recovery_rate_arrest_release_progress_rad_ &&
                          std::abs(signed_error) <=
                              recovery_circle_flexed_max_error_rad_)
                        {
                          recovery_rate_arrest_active_ = false;
                          recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
                          recovery_rate_arrest_reference_error_rad_ =
                              std::numeric_limits<double>::quiet_NaN();
                          continuous_roll_recovery_.beginAngleReturn();
                          // Retained angle reduction is measured evidence
                          // that the selected contact state is restorative,
                          // even while its instantaneous kinematic lever is
                          // near zero. Preserve the 2--4 N arrest authority
                          // instead of reapplying the 1.5-N geometry-probe
                          // ceiling immediately after this phase handoff.
                          recovery_wrong_geometry_response_observed_ = true;
                          continuous_roll_recovery_.resetResponseWindow(
                              signed_error, now.toSec());
                          ROS_INFO("Emergency extended-arm recovery produced %.2f-deg retained improvement while moving inward; preserving %.2f N and switching from rate arrest to angle return",
                                   retained_angle_progress * 180.0 / M_PI,
                                   continuous_roll_recovery_.thrust);
                        }
                    }
                  else
                    {
                      recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
                      recovery_rate_arrest_reference_error_rad_ =
                          std::numeric_limits<double>::quiet_NaN();
                    }
                }
              // Rate arrest is only the entry phase which supplies a bounded
              // 2-N floor while the body is still moving outward.  After a
              // retained inward response has been measured, the ordinary
              // angle/rate PI already increases authority whenever roll
              // worsens again.  Re-latching arrest from each instantaneous
              // negative gyro sample repeatedly changed the target rate and
              // caused the hardware's discontinuous thrust cycle.
              auto active_roll_recovery_config = roll_recovery_config_;
              const bool conditional_grasp_probe_pending =
                  recovery_conditional_grasp_probe_active_ &&
                  !recovery_conditional_grasp_response_observed_;
              if (recovery_conditional_grasp_probe_active_)
                {
                  // This mode exists only for a geometrically below-side
                  // grasp whose robust minimum lever is uncertain.  Start
                  // from a bounded command with the one existing PI
                  // integrator frozen.  Retained roll improvement promotes
                  // the same contact to the ordinary PI; no second PI loop
                  // is introduced.
                  active_roll_recovery_config.limit = std::min(
                      active_roll_recovery_config.limit,
                      recovery_conditional_grasp_response_observed_
                          ? recovery_grasp_probe_max_thrust_n_
                          : recovery_grasp_probe_initial_thrust_n_);
                  if (conditional_grasp_probe_pending)
                    {
                      active_roll_recovery_config.fallback_thrust =
                          std::min(active_roll_recovery_config.limit,
                              recovery_grasp_probe_initial_thrust_n_);
                      active_roll_recovery_config.minimum_feedforward =
                          active_roll_recovery_config.fallback_thrust;
                      active_roll_recovery_config.rate_ki = 0.0;
                      active_roll_recovery_config.integral_rate = 0.0;
                      active_roll_recovery_config.learn_response = false;
                    }
                  else
                    {
                      const double confirmed_minimum = std::min(
                          active_roll_recovery_config.limit,
                          recovery_grasp_probe_min_thrust_n_);
                      active_roll_recovery_config.fallback_thrust = std::max(
                          active_roll_recovery_config.fallback_thrust,
                          confirmed_minimum);
                      active_roll_recovery_config.minimum_feedforward =
                          std::max(
                              active_roll_recovery_config.minimum_feedforward,
                              confirmed_minimum);
                    }
                }
              // The discrete planner and the continuous actuator loop must
              // use the same force scale.  For a certain (or confirmed)
              // grasped topology, carry the planner's steady-state required
              // thrust into the existing feed-forward floor.  Slew limits,
              // rate PI, predictive braking and the global force limit still
              // own the actual command.  An unconfirmed conditional probe
              // deliberately remains at its bounded 0.8-N test command.
              if (!recovery_selected_pair_release_ &&
                  !conditional_grasp_probe_pending &&
                  std::isfinite(recovery_selected_required_thrust_n_))
                {
                  const double planned_grasp_feedforward =
                      hugmy::recoveryClamp(
                          recovery_selected_required_thrust_n_, 0.0,
                          std::min(active_roll_recovery_config.limit,
                                   recovery_grasp_probe_max_thrust_n_));
                  active_roll_recovery_config.fallback_thrust = std::max(
                      active_roll_recovery_config.fallback_thrust,
                      planned_grasp_feedforward);
                  active_roll_recovery_config.minimum_feedforward = std::max(
                      active_roll_recovery_config.minimum_feedforward,
                      planned_grasp_feedforward);
                }
              // A positive measured roll requires the 2+3-side recovery on
              // the present hardware.  Its ordinary 4-N rate-arrest ceiling
              // can be below static-contact breakaway.  Escalate toward the
              // global 5.5-N recovery limit only after the command has
              // actually reached 4 N and a complete averaged window shows
              // insufficient roll-error decay.  Negative-roll recovery and
              // all moving/braking phases keep their existing authority.
              const bool positive_roll_stalled_at_arrest_cap =
                  recovery_rate_arrest_active_ && roll_error < 0.0 &&
                  continuous_roll_recovery_.thrust + 0.05 >=
                      recovery_rate_arrest_max_thrust_n_ &&
                  recovery_decay_window_ready &&
                  recovery_error_decay_rate <
                      recovery_wrong_geometry_min_error_decay_rate_rad_s_ &&
                  !continuous_roll_recovery_.terminal_brake_latched;
              const double active_rate_arrest_limit =
                  positive_roll_stalled_at_arrest_cap
                  ? roll_recovery_config_.limit
                  : recovery_rate_arrest_max_thrust_n_;
              // During an emergency extension, keep the same zero-centred
              // angle/rate target used by ordinary recovery, but add a
              // bounded force floor and faster slew while outward motion is
              // being arrested.  Aiming at exactly zero rate here made every
              // small inward response trigger braking before the required
              // retained 0.5-deg improvement could be observed.
              if (recovery_rate_arrest_active_)
                {
                  active_roll_recovery_config.rate_arrest_mode = true;
                  active_roll_recovery_config.limit = std::min(
                      active_roll_recovery_config.limit,
                      active_rate_arrest_limit);
                  // Invalid/transitioning bend geometry otherwise reduces
                  // feed-forward to 0.5 N.  During outward-rate arrest only,
                  // retain enough bounded authority to oppose the measured
                  // deterioration.  This is still slew-limited, and the
                  // normal FF+PI law returns as soon as outward roll stops.
                  active_roll_recovery_config.fallback_thrust = std::min(
                      active_roll_recovery_config.limit,
                      std::max(active_roll_recovery_config.fallback_thrust,
                               recovery_rate_arrest_min_thrust_n_));
                  active_roll_recovery_config.minimum_feedforward = std::min(
                      active_roll_recovery_config.limit,
                      recovery_rate_arrest_min_thrust_n_);
                  active_roll_recovery_config.rise = std::max(
                      active_roll_recovery_config.rise,
                      recovery_preinflate_thrust_rise_n_s_);
                  if (positive_roll_stalled_at_arrest_cap)
                    ROS_WARN_THROTTLE(1.0,
                        "Positive-roll recovery remained below %.2f deg/s error decay at %.2f N; extending rate-arrest authority to %.2f N",
                        recovery_wrong_geometry_min_error_decay_rate_rad_s_ *
                            180.0 / M_PI,
                        continuous_roll_recovery_.thrust,
                        active_roll_recovery_config.limit);
                }
              const auto model = measuredRecoveryModel();
              auto& response = roll_recovery_response_[correction_axis_[0] > 0.0 ? 0 : 1];
              const bool recovery_rate_valid = std::isfinite(omega) &&
                  omega > -recovery_measurement_rate_limit_rad_s_ &&
                  omega < 2.0 * recovery_measurement_rate_limit_rad_s_;
              const bool recovery_inputs_valid = recovery_pressure_fresh &&
                  std::isfinite(bottom_pressure_kpa_) && bottom_pressure_kpa_ < pressure_safety_limit_kpa_ &&
                  recovery_rate_valid;
              double predictive_target = std::numeric_limits<double>::quiet_NaN();
              if (recovery_predictive_config_.enabled &&
                  !conditional_grasp_probe_pending &&
                  !recovery_rate_arrest_active_ && recovery_inputs_valid && model.valid &&
                  signed_error > roll_recovery_config_.stop && !continuous_roll_recovery_.fault)
                {
                  std::array<size_t, 2> arms{};
                  size_t count = 0;
                  for (size_t arm=0; arm<ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm] && count<arms.size()) arms[count++]=arm;
                  hugmy::RecoveryPredictiveState observation;
                  observation.now=now.toSec(); observation.error=signed_error; observation.rate=omega;
                  observation.thrust=continuous_roll_recovery_.thrust; observation.bottom=bottom_pressure_kpa_;
                  for(size_t j=0;j<2;++j) {
                    observation.pressure[j]=measured_pressure_[arms[j]];
                    observation.pressure_command[j]=prepare_pressure_[arms[j]];
                    observation.bend[j]=measuredArmBendAngle(arms[j]);
                  }
                  auto config=recovery_predictive_config_;
                  config.pressure_min=recovery_selected_pair_release_
                      ? recoverySelectedPairPressure(arms[0])
                      : minimum_prepare_pressure_kpa_;
                  config.pressure_max=std::min(maximum_pressure_kpa_,
                      std::min(grasp_pressure_[arms[0]],grasp_pressure_[arms[1]]));
                  config.pressure_slew=pressure_rate_limit_kpa_s_;
                  const auto candidate=recovery_predictive_.update(observation,config,active_roll_recovery_config,
                      [&](size_t, double pressure, double force) {
                        const auto range=bend_model_.ranges();
                        const double coordinate=force+bend_model_thrust_offset_n_;
                        if(pressure<range.first.first-5. || pressure>range.first.second ||
                           coordinate<range.second.first || coordinate>range.second.second)
                          return std::numeric_limits<double>::quiet_NaN();
                        // Permit small sub-atmospheric sensor offsets; no
                        // negative-pressure actuation or table extrapolation.
                        return bend_model_.angleRad(std::max(range.first.first,pressure),coordinate);
                      }, [&](size_t j, double angle) { return recoveryRotorLever(arms[j],angle); });
                  if(!config.shadow) {
                    if(!candidate.feasible || count!=2) {
                      // A single infeasible horizon is common while chamber
                      // pressure and bend pass between two contact modes.  It
                      // does not prove that the selected physical contact has
                      // the wrong moment sign.  Brake for this control cycle
                      // and re-plan from the next measurement; the measured
                      // wrong-way/no-progress logic below remains responsible
                      // for rejecting and re-selecting a contact candidate.
                      predictive_target = 0.0;
                      ROS_WARN_THROTTLE(1., "Action C predictive recovery has no feasible horizon; braking this cycle and replanning from measured contact state");
                    } else {
                      predictive_target=candidate.thrust;
                      for(size_t j=0;j<2;++j)
                        prepare_pressure_[arms[j]]+=hugmy::recoveryClamp(
                            candidate.pressure[j]-prepare_pressure_[arms[j]],
                            -config.pressure_slew*dt,config.pressure_slew*dt);
                    }
                  }
                  ROS_INFO_THROTTLE(.5,
                      "Action C predictive recovery: shadow=%d feasible=%d target %.2f -> %.2f N, pressure [%.1f %.1f] kPa, worst terminal roll %.2f deg, score %.3f, candidates %u rejected %u budget_exhausted=%d, confidence %.2f (heuristic), prediction errors rate %.3f rad/s pressure %.2f kPa bend %.2f deg, observations %u",
                      config.shadow,candidate.feasible,candidate.thrust,candidate.later_thrust,
                      candidate.pressure[0],candidate.pressure[1],candidate.worst_terminal_error*180./M_PI,
                      candidate.score,candidate.evaluated,candidate.rejected,candidate.budget_exhausted,recovery_predictive_.confidence,
                      recovery_predictive_.prediction_error,recovery_predictive_.pressure_prediction_error,
                      recovery_predictive_.bend_prediction_error*180./M_PI,recovery_predictive_.accepted_observations);
                }
              else if(recovery_predictive_config_.enabled &&
                      !conditional_grasp_probe_pending &&
                      !recovery_rate_arrest_active_ &&
                      !recovery_predictive_config_.shadow &&
                      (!recovery_inputs_valid || !model.valid))
                continuous_roll_recovery_.fault=true;
              // Preserve the extended-arm recovery geometry throughout the
              // active correction.  Rotor speed/prediction braking can begin
              // far from level (31 deg in walk_c_3_after_build); it is not a
              // pneumatic regrasp event.  A non-shadow predictor may still
              // request pressure, but it cannot bend the selected arms above
              // the configured release pressure before a real terminal
              // handoff.
              if (recovery_selected_pair_release_ &&
                  !continuous_roll_recovery_.pressureHandoffReady() &&
                  !recovery_pressure_brake_started_)
                {
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        const double active_pressure_ceiling =
                            recoverySelectedPairPressure(arm);
                        prepare_pressure_[arm] = std::min(
                            prepare_pressure_[arm], active_pressure_ceiling);
                      }
                }
              publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
              bool recovery_response_faults_enabled = true;
              if (recovery_selected_pair_release_)
                {
                  // Venting the lower pair transfers support before the rotor
                  // moment has built.  Do not label that pneumatic transient
                  // as a wrong-sign rotor response.  The inner law also waits
                  // for its configured meaningful-thrust threshold after this
                  // measured-pressure gate opens.
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        // "Pressure settled" refers to the active pneumatic
                        // contact state, not permanently to the original
                        // release target.  Terminal capture deliberately
                        // changes the selected pair from 0 to about 15 kPa;
                        // continuing to compare against 0 kPa kept the
                        // response gate closed and capped a useful retry at
                        // 1.5 N for the rest of the recovery event.
                        const double active_pressure_target =
                            prepare_pressure_[arm];
                        recovery_response_faults_enabled =
                            recovery_response_faults_enabled &&
                            std::isfinite(measured_pressure_[arm]) &&
                            std::isfinite(active_pressure_target) &&
                            std::abs(measured_pressure_[arm] -
                                     active_pressure_target) <=
                                roll_recovery_config_.stable_pressure;
                      }
                }
              if (recovery_response_faults_enabled &&
                  continuous_roll_recovery_.thrust + 0.05 >=
                      roll_recovery_config_.response_fault_min_thrust)
                {
                  if (recovery_response_probe_since_.isZero())
                    recovery_response_probe_since_ = now;
                }
              else
                recovery_response_probe_since_ = ros::Time(0);
              const bool recovery_response_evaluation_ready =
                  recovery_response_faults_enabled &&
                  !recovery_response_probe_since_.isZero() &&
                  (now - recovery_response_probe_since_).toSec() >=
                      recovery_response_min_probe_sec_;
              // The circle planner evaluates the geometry expected after a
              // released pair has extended.  On the positive-roll hardware
              // trial, its current measured lever was still negative while
              // that extension was taking place; applying the ordinary PI
              // force during this interval increased roll from 19 to 39 deg.
              // Start with a bounded, measurable probe force until either
              // the measured lever has a restoring sign or the actual body
              // response confirms that the selected discrete pair is useful.
              const bool measured_restoring_geometry = model.valid &&
                  std::isfinite(model.lever) &&
                  model.lever >= roll_recovery_config_.min_lever;
              const bool wrong_geometry_candidate =
                  recovery_selected_pair_release_ &&
                  !measured_restoring_geometry;
              if (wrong_geometry_candidate)
                {
                  bool probe_pressure_target_changed =
                      !recovery_wrong_geometry_probe_target_valid_;
                  if (!probe_pressure_target_changed)
                    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                      if (prepare_arm_active_[arm] &&
                          std::abs(prepare_pressure_[arm] -
                                   recovery_wrong_geometry_probe_pressure_target_[arm]) >
                              roll_recovery_config_.stable_pressure)
                        probe_pressure_target_changed = true;
                  if (probe_pressure_target_changed)
                    {
                      recovery_wrong_geometry_probe_pressure_target_ =
                          prepare_pressure_;
                      recovery_wrong_geometry_probe_target_valid_ = true;
                      recovery_wrong_geometry_pressure_settled_since_ =
                          ros::Time(0);
                      recovery_wrong_geometry_reference_error_rad_ =
                          std::numeric_limits<double>::quiet_NaN();
                      recovery_wrong_geometry_reference_rate_rad_s_ =
                          std::numeric_limits<double>::quiet_NaN();
                      recovery_wrong_geometry_best_rate_rad_s_ =
                          -std::numeric_limits<double>::infinity();
                      // Keep an already measured restorative response for
                      // this selected pair. A pressure-target transition may
                      // reset the observation window, but it must not put the
                      // same verified pair back behind the 1.5-N probe cap.
                      ROS_INFO("Recovery response probe follows active pair pressure target [%.1f %.1f %.1f %.1f] kPa",
                               prepare_pressure_[0], prepare_pressure_[1],
                               prepare_pressure_[2], prepare_pressure_[3]);
                    }
                  // The kinematic lever is evaluated while the chamber and
                  // compliant contact are still moving toward the candidate
                  // state.  An outward angular velocity may therefore remain
                  // for a short time even when the new moment is already
                  // decelerating it.  Start the conclusive observation only
                  // after measured pressure has settled, and use improvement
                  // of the signed angular rate as empirical restoring
                  // evidence.  Angle growth alone before that point is not a
                  // wrong-sign result.
                  if (recovery_wrong_geometry_since_.isZero())
                    {
                      recovery_wrong_geometry_since_ = now;
                    }
                  // Exhausting the selected pair moves the contact pivot and
                  // can increase roll before its rotor moment has any useful
                  // authority.  That pressure-transfer motion is not an
                  // actuator-sign experiment.  In particular, the positive-
                  // roll hardware trial selected the physically correct 2+3
                  // pair but rejected it here while its command was still
                  // zero.  Keep the pair and defer every wrong-way decision
                  // until pressure has settled and the rotors have delivered
                  // the configured meaningful probe force.
                  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
                  const bool meaningful_probe_authority =
                      continuous_roll_recovery_.thrust + 0.05 >=
                          roll_recovery_config_.response_fault_min_thrust;
                  if (recovery_response_faults_enabled &&
                      meaningful_probe_authority &&
                      recovery_wrong_geometry_pressure_settled_since_.isZero())
                    {
                      recovery_wrong_geometry_pressure_settled_since_ = now;
                      recovery_wrong_geometry_reference_error_rad_ =
                          std::abs(signed_error);
                      recovery_wrong_geometry_reference_rate_rad_s_ = omega;
                      recovery_wrong_geometry_best_rate_rad_s_ = omega;
                      // Start the decay-rate window at the first
                      // pressure-settled, meaningful-force sample.  Motion
                      // accumulated during exhaust belongs to pneumatic
                      // contact transfer and must not bias this candidate's
                      // measured rotor response.
                      continuous_roll_recovery_.resetResponseWindow(
                          signed_error, now.toSec());
                    }
                  if (!recovery_wrong_geometry_pressure_settled_since_.isZero())
                    recovery_wrong_geometry_best_rate_rad_s_ = std::max(
                        recovery_wrong_geometry_best_rate_rad_s_, omega);
                  const double pressure_settled_age =
                      recovery_wrong_geometry_pressure_settled_since_.isZero()
                      ? 0.0
                      : (now - recovery_wrong_geometry_pressure_settled_since_).toSec();
                  // Probe validity is judged after a *measured-pressure*
                  // and meaningful-thrust dwell, not 0.6 s after the
                  // release command while the contact lever is moving.
                  // This gate does not delay the fast RECOVERY thrust ramp.
                  const bool response_probe_complete =
                      hugmy::recoveryResponseCanBeJudged(
                          recovery_response_faults_enabled,
                          recovery_wrong_geometry_probe_target_valid_,
                          continuous_roll_recovery_.thrust,
                          roll_recovery_config_.response_fault_min_thrust,
                          pressure_settled_age,
                          recovery_response_min_probe_sec_);
                  const bool rate_response_confirmed =
                      pressure_settled_age >=
                          recovery_wrong_geometry_response_confirm_sec_ &&
                      std::isfinite(recovery_wrong_geometry_reference_rate_rad_s_) &&
                      recovery_wrong_geometry_best_rate_rad_s_ >=
                          recovery_wrong_geometry_reference_rate_rad_s_ +
                          recovery_wrong_geometry_rate_improvement_rad_s_;
                  // A bounded probe may first cancel the continuing fall
                  // without immediately reversing angular velocity.  Once
                  // the probe has been held for the confirmation interval,
                  // a rate which has not become materially more outward is
                  // sufficient to continue increasing the same selected
                  // pair.  Requiring a full 2-deg/s improvement deadlocked a
                  // correct 2+3 pair at 0.6 N.
                  const bool probe_did_not_worsen_rate =
                      pressure_settled_age >=
                          recovery_wrong_geometry_response_confirm_sec_ &&
                      std::isfinite(recovery_wrong_geometry_reference_rate_rad_s_) &&
                      omega >= recovery_wrong_geometry_reference_rate_rad_s_ -
                          0.25 * recovery_wrong_geometry_rate_improvement_rad_s_;
                  // Candidate quality is based on the averaged roll-error
                  // decay rate maintained by ContinuousRollRecovery.  Do not
                  // compare a single displacement with the former fixed
                  // 0.5-deg/0.8-s gate: that rejected the correct positive-
                  // roll pair before static contact had broken away.
                  const bool useful_probe_decay =
                      recovery_decay_window_ready &&
                      recovery_error_decay_rate >=
                          recovery_wrong_geometry_min_error_decay_rate_rad_s_;
                  const bool helpful_probe_motion =
                      omega >= roll_recovery_config_.start_rate;
                  const bool response_was_observed =
                      recovery_wrong_geometry_response_observed_;
                  recovery_wrong_geometry_response_observed_ =
                      response_was_observed || rate_response_confirmed ||
                      probe_did_not_worsen_rate;
                  if (!response_was_observed &&
                      recovery_wrong_geometry_response_observed_)
                    {
                      // The old progress window contains only the pneumatic
                      // release transient.  Begin rotor wrong-way detection
                      // from the empirically confirmed deceleration instead
                      // of immediately rejecting the candidate for motion
                      // inherited before pressure settled.
                      continuous_roll_recovery_.resetResponseWindow(
                          signed_error, now.toSec());
                      ROS_INFO("Recovery pair %zu+%zu retained a model-wrong lever %.4f m, but its pressure-settled %.2f-N probe did not worsen signed rate %.2f -> %.2f deg/s; enabling measured-response authority",
                               recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                               recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                               model.lever,
                               continuous_roll_recovery_.thrust,
                               recovery_wrong_geometry_reference_rate_rad_s_ *
                                   180.0 / M_PI,
                               recovery_wrong_geometry_best_rate_rad_s_ *
                                   180.0 / M_PI);
                    }
                  const bool geometry_worsened =
                      response_probe_complete &&
                      recovery_decay_window_ready &&
                      !recovery_wrong_geometry_response_observed_ &&
                      recovery_error_decay_rate <=
                          -recovery_wrong_geometry_min_error_decay_rate_rad_s_ &&
                      omega <=
                          -recovery_wrong_geometry_transition_outward_rate_rad_s_;
                  const bool pressure_settled_insufficient_decay =
                      response_probe_complete &&
                      pressure_settled_age >=
                          recovery_wrong_geometry_progress_window_sec_ &&
                      recovery_decay_window_ready &&
                      !recovery_wrong_geometry_response_observed_ &&
                      !useful_probe_decay && !helpful_probe_motion;
                  if (pressure_settled_insufficient_decay ||
                      geometry_worsened)
                    {
                      // This decision is made only after the pressure-settled
                      // meaningful-force probe above.  A changing kinematic
                      // lever or motion inherited from bottom/pressure
                      // transfer alone can no longer reject the selected
                      // pair.
                      continuous_roll_recovery_.fault = true;
                      continuous_roll_recovery_.wrong_way = geometry_worsened;
                      continuous_roll_recovery_.unresponsive =
                          !geometry_worsened;
                      ROS_ERROR("Recovery pair %zu+%zu retained a non-restoring measured lever %.4f m for %.2f s after pressure settled with roll-error decay %.2f deg/s below %.2f deg/s (roll error %.2f -> %.2f deg, signed rate %.2f -> best %.2f deg/s); rejecting this contact state",
                                recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                                recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                                model.lever, pressure_settled_age,
                                recovery_error_decay_rate * 180.0 / M_PI,
                                recovery_wrong_geometry_min_error_decay_rate_rad_s_ *
                                    180.0 / M_PI,
                                recovery_wrong_geometry_reference_error_rad_ *
                                    180.0 / M_PI,
                                std::abs(signed_error) * 180.0 / M_PI,
                                recovery_wrong_geometry_reference_rate_rad_s_ *
                                    180.0 / M_PI,
                                recovery_wrong_geometry_best_rate_rad_s_ *
                                    180.0 / M_PI);
                    }
                }
              else
                {
                  recovery_wrong_geometry_since_ = ros::Time(0);
                  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
                  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
                  recovery_wrong_geometry_reference_error_rad_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_reference_rate_rad_s_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_best_rate_rad_s_ =
                      -std::numeric_limits<double>::infinity();
                  // Do not erase a pressure-settled measured response merely
                  // because the instantaneous bend lever has crossed back
                  // into its valid region. Pair selection resets this flag
                  // for a new event; retained roll worsening still rejects a
                  // genuinely reversed contact state.
                }
              // A released candidate is selected from its terminal,
              // pressure/thrust-dependent arm shape. The former fixed
              // 1.5-N geometry cap required that terminal lever to exist
              // before allowing the lookup-table force needed to create it.
              // Use measured decompression as the independent acquisition
              // coordinate instead. Both selected arms must vent: the
              // slower arm owns the common synchronous authority.
              const double extension_initial_thrust = std::max(
                  recovery_wrong_sign_shaping_thrust_n_,
                  roll_recovery_config_.response_fault_min_thrust);
              double extension_pressure_progress = 0.0;
              double extension_shape_authority = extension_initial_thrust;
              if (recovery_selected_pair_release_ &&
                  wrong_geometry_candidate &&
                  std::isfinite(recovery_selected_required_thrust_n_))
                {
                  bool have_selected_arm = false;
                  extension_pressure_progress = 1.0;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        have_selected_arm = true;
                        const double loaded_pressure = hugmy::recoveryClamp(
                            std::max(base_pressure_kpa_, grasp_pressure_[arm]),
                            minimum_prepare_pressure_kpa_,
                            maximum_pressure_kpa_);
                        const double released_pressure =
                            recoverySelectedPairPressure(arm);
                        extension_pressure_progress = std::min(
                            extension_pressure_progress,
                            hugmy::recoveryPressureReleaseProgress(
                                measured_pressure_[arm], loaded_pressure,
                                released_pressure));
                      }
                  if (!have_selected_arm)
                    extension_pressure_progress = 0.0;
                  extension_shape_authority =
                      hugmy::recoveryScheduledExtensionAuthority(
                          extension_initial_thrust,
                          recovery_selected_required_thrust_n_,
                          extension_pressure_progress,
                          active_roll_recovery_config.limit);
                  // This is a shape-acquisition floor, not a permanent roll
                  // command. Overspeed and terminal braking below retain
                  // priority and may reduce the actuator immediately. Once
                  // the measured lever becomes restorative, ordinary moment
                  // FF + rate PI owns the command again.
                  active_roll_recovery_config.fallback_thrust = std::max(
                      active_roll_recovery_config.fallback_thrust,
                      extension_shape_authority);
                  active_roll_recovery_config.minimum_feedforward = std::max(
                      active_roll_recovery_config.minimum_feedforward,
                      extension_shape_authority);
                  active_roll_recovery_config.rise = std::max(
                      active_roll_recovery_config.rise,
                      recovery_preinflate_thrust_rise_n_s_);
                }
              const double geometry_authority_limit =
                  recovery_selected_pair_release_ &&
                  !measured_restoring_geometry &&
                  !recovery_wrong_geometry_response_observed_
                  ? std::max(
                        extension_shape_authority,
                        recovery_rate_arrest_active_
                            ? std::min(roll_recovery_config_.limit,
                                       active_rate_arrest_limit)
                            : 0.0)
                  : std::numeric_limits<double>::infinity();
              if (recovery_conditional_grasp_probe_active_ &&
                  recovery_conditional_grasp_probe_start_.isZero())
                {
                  recovery_conditional_grasp_probe_start_ = now;
                  recovery_conditional_grasp_probe_start_error_rad_ =
                      std::abs(signed_error);
                }
              if (conditional_grasp_probe_pending &&
                  !recovery_conditional_grasp_probe_start_.isZero())
                {
                  const double probe_elapsed =
                      (now - recovery_conditional_grasp_probe_start_).toSec();
                  const double retained_probe_progress =
                      std::isfinite(
                          recovery_conditional_grasp_probe_start_error_rad_)
                      ? recovery_conditional_grasp_probe_start_error_rad_ -
                            std::abs(signed_error)
                      : 0.0;
                  if (probe_elapsed >= recovery_grasp_probe_confirm_sec_ &&
                      retained_probe_progress >=
                          recovery_grasp_probe_min_progress_rad_)
                    {
                      recovery_conditional_grasp_response_observed_ = true;
                      continuous_roll_recovery_.boost = 0.0;
                      continuous_roll_recovery_.resetResponseWindow(
                          signed_error, now.toSec());
                      ROS_INFO("Below-side grasp probe confirmed %.2f-deg retained roll improvement in %.2f s; enabling existing rate PI up to %.2f N",
                               retained_probe_progress * 180.0 / M_PI,
                               probe_elapsed,
                               recovery_grasp_probe_max_thrust_n_);
                    }
                  else if (probe_elapsed >=
                               recovery_grasp_probe_timeout_sec_)
                    {
                      // Let ContinuousRollRecovery perform its ordinary
                      // slew-limited rundown.  Its existing contact-candidate
                      // rejection then marks only this grasp topology and
                      // reselects an extended candidate after SAFE_TRANSFER.
                      continuous_roll_recovery_.unresponsive = true;
                      continuous_roll_recovery_.timed_out = true;
                      ROS_ERROR_THROTTLE(0.5,
                          "Below-side grasp probe produced only %.2f-deg retained improvement in %.2f s; rejecting this grasp candidate before extension re-selection",
                          retained_probe_progress * 180.0 / M_PI,
                          probe_elapsed);
                    }
                }
              double thrust = continuous_roll_recovery_.update(
                  recovery_inputs_valid ? signed_error : std::numeric_limits<double>::quiet_NaN(),
                  omega, now.toSec(), dt, model, active_roll_recovery_config, response,
                  predictive_target,
                  recovery_response_evaluation_ready &&
                      !conditional_grasp_probe_pending &&
                      (!wrong_geometry_candidate ||
                       recovery_wrong_geometry_response_observed_),
                  geometry_authority_limit);
              const bool retained_decay_window_ready =
                  !continuous_roll_recovery_.progress_history.empty() &&
                  now.toSec() -
                          continuous_roll_recovery_.progress_history.front().time >=
                      std::max(roll_recovery_config_.progress_window,
                          recovery_wrong_geometry_progress_window_sec_);
              const double retained_error_decay_rate =
                  retained_decay_window_ready
                  ? continuous_roll_recovery_.recent_progress /
                        std::max(1.e-3,
                            roll_recovery_config_.progress_window)
                  : std::numeric_limits<double>::infinity();
              // The inner response window is reset while bend/contact
              // geometry changes. That is necessary for model learning, but
              // it can hide a slower failure mode: a candidate first makes
              // real progress, then a low-authority retry steadily gives all
              // of it back. Detect retained error loss directly from body
              // roll after pressure has settled. Do not reject a quiet
              // static-friction plateau, and do not blame a candidate while
              // predictive braking is intentionally reducing force.
              const bool retained_candidate_worsening =
                  recovery_response_evaluation_ready &&
                  continuous_roll_recovery_.progress_observed &&
                  !continuous_roll_recovery_.terminal_brake_latched &&
                  continuous_roll_recovery_.recent_thrust_change >= -0.1 &&
                  thrust >= roll_recovery_config_.detect_thrust &&
                  retained_decay_window_ready &&
                  retained_error_decay_rate <=
                      -recovery_wrong_geometry_min_error_decay_rate_rad_s_;
              if (retained_candidate_worsening)
                {
                  continuous_roll_recovery_.wrong_way = true;
                  continuous_roll_recovery_.timed_out = true;
                  ROS_ERROR_THROTTLE(0.5,
                      "Recovery candidate lost retained progress at %.2f deg/s under %.2f N (best %.2f deg, current %.2f deg); braking and re-selecting contact geometry",
                      retained_error_decay_rate * 180.0 / M_PI,
                      thrust,
                      continuous_roll_recovery_.best_error *
                          180.0 / M_PI,
                      std::abs(signed_error) * 180.0 / M_PI);
                }
              // At the captured attitude the selected pair is no longer used
              // as a long moment arm.  Build only the minimum load-bearing
              // pressure while rotor thrust is already near zero, so contact
              // Pneumatic damping must start before the body enters the final
              // angle band.  In both MuJoCo and hardware the pressure path is
              // much slower than rotor rundown: waiting for +/-3 deg let a
              // correctly moving body coast through zero and start another
              // recovery on the opposite pair.  Once a full-window decrease
              // in roll error has been measured and the rotor controller is
              // braking, request the minimum load-bearing pressure.  This is
              // only the first support stage; full grasp remains owned by
              // RECOVER.  The independent pressure controller/pneumatic
              // dynamics still limit the actual pressure slew.
              const double pressure_brake_horizon = std::max(
                  correction_prediction_horizon_sec_,
                  recovery_predictive_config_.pressure_tau);
              const double pressure_brake_predicted_error =
                  std::abs(signed_error) -
                  std::max(0.0, omega) * pressure_brake_horizon;
              const bool pneumatic_capture_imminent =
                  omega > roll_recovery_config_.settle_rate &&
                  pressure_brake_predicted_error <=
                      roll_recovery_config_.stop +
                      2.0 * roll_recovery_config_.stop_margin;
              const bool predictive_pressure_brake =
                  (continuous_roll_recovery_.speed_braking ||
                   continuous_roll_recovery_.predicted_braking) &&
                  (continuous_roll_recovery_.progress_observed ||
                   pneumatic_capture_imminent);
              // Stopping-distance prediction can become true on a short
              // stick-slip rate impulse while the body is still tens of
              // degrees from level. Rotor braking is appropriate there,
              // but pneumatic regrasp is not: changing 0 -> support pressure
              // bends the long arm, resets the measured-response probe and
              // restores the 1.5-N wrong-geometry ceiling. Keep the released
              // geometry until the measured angle itself reaches the same
              // terminal capture neighbourhood used by the inner law.
              const double pressure_capture_limit =
                  roll_recovery_config_.stop +
                  roll_recovery_config_.capture_margin_factor *
                      roll_recovery_config_.stop_margin;
              const bool pressure_capture_angle_ready =
                  std::abs(signed_error) <= pressure_capture_limit;
              const double pressure_brake_thrust_gate = std::max(
                  0.35, 0.25 * roll_recovery_config_.limit);
              if ((continuous_roll_recovery_.capture_active ||
                   (predictive_pressure_brake &&
                    pressure_capture_angle_ready)) &&
                  thrust <= pressure_brake_thrust_gate)
                recovery_pressure_brake_started_ = true;
              if (recovery_pressure_brake_started_)
                {
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      prepare_pressure_[arm] = std::max(
                          prepare_pressure_[arm],
                          std::min(grasp_pressure_[arm],
                                   recoverySupportPressure()));
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                }
              // Keep the extended-arm geometry through the completed angular
              // correction. RECOVER will close the pneumatic loop gradually;
              // a 0 -> 30 kPa step here re-bent the pair and recreated the
              // original roll error after an otherwise successful recovery.
              // The pair sign is fixed by the verified Spinal/MuJoCo mapping
              // for the whole recovery.  A temporary lack of progress is a
              // contact/load observation; it must not swap to the opposite
              // pair.  The rate PI may adapt magnitude, and the bounded
              // no-progress timeout below performs a smooth regrasp handoff.
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                prepare_shape_thrust_[arm] = prepare_arm_active_[arm] ? thrust : 0.0;
              publishCorrectionThrust(prepare_shape_thrust_);
              ROS_INFO_THROTTLE(0.25,
                  "Action C model-rate recovery: roll error %.2f deg, rate %.3f/%.3f rad/s, pair thrust %.2f/%.2f N (command/target), FF %.2f N (plan %.2f), I %.2f N, breakaway %.2f N, moment %.3f Nm, lever %.3f m, model=%d learned=%d gain %.3f samples=%u, moving=%d stalled=%d wrong_way=%d response_gate=%d extension_latched=%d rate_arrest=%d terminal_brake=%d stored_brake=%d reapproach=%d rebound_retry=%d authority_hold=%d cap=%.2f N geometry_cap=%.2f N shape_pressure=%.2f shape_target=%.2f N speed_brake=%d predicted_brake=%d horizon=%.3f s, gravity_ff=0, bottom %.1f kPa (exhaust, no wait), recent_progress %.2f deg / %.2f s, progress_age %.2f s",
                  signed_error * 180.0 / M_PI, omega, continuous_roll_recovery_.target_rate, thrust,
                  continuous_roll_recovery_.target_thrust,
                  continuous_roll_recovery_.feedforward,
                  std::isfinite(recovery_selected_required_thrust_n_)
                      ? recovery_selected_required_thrust_n_ : -1.0,
                  continuous_roll_recovery_.boost,
                  continuous_roll_recovery_.hold, model.moment, model.lever,
                  continuous_roll_recovery_.model_valid, continuous_roll_recovery_.learned_response,
                  response.gain, response.samples,
                  continuous_roll_recovery_.moving, continuous_roll_recovery_.stalled,
                  continuous_roll_recovery_.wrong_way,
                  recovery_response_faults_enabled,
                  recovery_extension_mode_latched_,
                  recovery_rate_arrest_active_,
                  continuous_roll_recovery_.terminal_brake_latched,
                  continuous_roll_recovery_.stored_authority_braking,
                  continuous_roll_recovery_.low_authority_reapproach,
                  continuous_roll_recovery_.terminal_rebound_retry,
                  continuous_roll_recovery_.pre_response_authority_hold,
                  continuous_roll_recovery_.reapproach_authority,
                  std::isfinite(geometry_authority_limit)
                      ? geometry_authority_limit : -1.0,
                  extension_pressure_progress,
                  extension_shape_authority,
                  continuous_roll_recovery_.speed_braking, continuous_roll_recovery_.predicted_braking,
                  continuous_roll_recovery_.braking_horizon,
                  bottom_pressure_kpa_,
                  continuous_roll_recovery_.recent_progress * 180.0 / M_PI,
                  roll_recovery_config_.progress_window,
                  now.toSec() - continuous_roll_recovery_.last_progress);
              if (!continuous_roll_recovery_.model_valid &&
                  (!recovery_predictive_config_.enabled || recovery_predictive_config_.shadow))
                ROS_WARN_THROTTLE(1.0, "Recovery geometry unavailable/wrong-sign lever: bounded rate-PI fallback, no absolute-value inversion");
                  if (continuous_roll_recovery_.fault || continuous_roll_recovery_.finished ||
                  (now - state_start_).toSec() >= prepare_timeout_)
                {
                  // A wrong-way flag is latched until the old pair's thrust
                  // has slewed to zero, so this is the first safe instant to
                  // change actuators. Transfer pneumatic support and retry
                  // with the other pair directly; a several-second full
                  // regrasp before the switch let a 17-deg body fall past 30
                  // deg in MuJoCo. Permit one measured opposite-pair retry per
                  // posture event. Switching back to the original pair after
                  // the retry had already improved roll recreated the first
                  // wrong-way response and caused a fall; the measured minimum
                  // is instead preserved by the ordinary handoff/regrasp.
                  // The effective contact pivot can move as the selected
                  // arms extend.  A pair may initially reduce the error and
                  // later reverse its roll moment.  A measured wrong-way
                  // response must therefore override an earlier progress
                  // observation.  Lack of response alone remains guarded by
                  // the no-progress condition, since static friction is not
                  // evidence that the pair sign is wrong.
                  const bool measured_pair_sign_reversal =
                      continuous_roll_recovery_.wrong_way;
                  const bool never_effective_pair =
                      continuous_roll_recovery_.unresponsive &&
                      !continuous_roll_recovery_.progress_observed;
                  // A pair that first produced real improvement and only
                  // later reversed has reached a contact-lever minimum.  Do
                  // not throw that measured progress away by opening the
                  // opposite support pair.  Regrasp the same pair under the
                  // closed-loop handoff, then make a fresh decision from the
                  // new contact state.  The alternate pair is reserved for a
                  // pair which was never effective at all.
                  const bool pair_never_corrected = never_effective_pair ||
                      (measured_pair_sign_reversal &&
                       !continuous_roll_recovery_.progress_observed);
                  const double contact_capture_limit = recovery_stop_angle +
                      1.5 * roll_recovery_config_.stop_margin;
                  // Crossing the target is evidence that this candidate did
                  // produce the requested moment; it is not a bad candidate.
                  // The rotor pair is one-directional, however, so first
                  // restore a full four-arm grasp and then select again for
                  // the newly measured error sign. This path is required in
                  // recovery-only validation as well as during a paused gait.
                  if (continuous_roll_recovery_.fault &&
                      continuous_roll_recovery_.target_crossed)
                    {
                      if (hugmy::recoveryExitDecision(false, true, true,
                              roll_error, contact_capture_limit) !=
                          hugmy::RecoveryExit::TARGET_CROSSED)
                        {
                          beginPostureSafeTransfer(now,
                              "target crossed but residual outside contact capture",
                              false,
                              hugmy::RecoveryFaultReason::SUPPORT_UNVERIFIED);
                          break;
                        }
                      recovery_rejected_candidate_mask_ = 0;
                      recovery_rejected_direction_sign_ = 0;
                      recovery_candidate_reselect_pending_ = false;
                      continuous_recovery_blocked_ = false;
                      continuous_recovery_waiting_for_settle_ = true;
                      continuous_recovery_started_ = true;
                      continuous_roll_recovery_.finished = true;
                      resetRecoveryHandoffState();
                      // Crossing the requested attitude is not itself a
                      // failed contact state.  Freeze the actual chamber
                      // pressures and first try to catch the body there.  If
                      // it subsequently leaves the +/-stop band, RECOVER
                      // performs an immediate opposite-sign re-selection;
                      // if it remains inside, the same path completes the
                      // monitored pneumatic regrasp without starting an
                      // unnecessary second rotor pair.
                      recovery_handoff_target_pressure_ =
                          recoveryMeasuredContactPressure();
                      prepare_pressure_ = recovery_handoff_target_pressure_;
                      grasp_pressure_ = recovery_handoff_target_pressure_;
                      if (hybridLocomotionActive() && action_c_resume_pending_)
                        {
                          hybrid_grasp_pressure_ = grasp_pressure_;
                          hybrid_hold_pressure_ = true;
                        }
                      prepare_shape_thrust_.fill(0.0);
                      publishCorrectionThrust(prepare_shape_thrust_);
                      publishPressureTarget(prepare_pressure_,
                          std::numeric_limits<double>::infinity());
                      action_c_resume_gate_.reset();
                      action_c_boundary_settle_gate_.reset();
                      ROS_WARN("Continuous recovery crossed the target with residual %.2f deg; froze measured contact pressure [%.1f %.1f %.1f %.1f] kPa for monitored regrasp (opposite-sign selection only if the residual leaves the stop band)",
                               roll_error * 180.0 / M_PI,
                               grasp_pressure_[0], grasp_pressure_[1],
                               grasp_pressure_[2], grasp_pressure_[3]);
                      enterState(RECOVER, now);
                      break;
                    }
                  // The circular-contact model proposes a *contact state*,
                  // not merely a rotor pair: pair 0+1 / 2+3 can each be used
                  // either while flexed against the object or after release.
                  // A measured reversal (including one after useful initial
                  // progress) disproves only that exact state.  Likewise, a
                  // candidate which remains motionless after reaching the
                  // high-thrust retry gate is locally ineffective.  Remove
                  // that candidate, equalise pneumatic support, then infer
                  // the circle again and choose from the unused candidates.
                  const bool reject_contact_candidate =
                      recovery_selected_contact_pivot_enabled_ &&
                      continuous_roll_recovery_.fault &&
                      (measured_pair_sign_reversal ||
                       continuous_roll_recovery_.unresponsive);
                  if (reject_contact_candidate)
                    {
                      const uint8_t candidate_bit = static_cast<uint8_t>(
                          1u << (4u * recovery_selected_pair_index_ +
                                 recovery_selected_pressure_candidate_index_));
                      recovery_rejected_candidate_mask_ |= candidate_bit;
                      recovery_rejected_direction_sign_ =
                          roll_error >= 0.0 ? 1 : -1;
                      const bool measured_retry_safe =
                          std::abs(roll_error) <
                          action_c_posture_hard_roll_rad_;
                      ROS_WARN("Circular recovery rejected candidate pair %zu+%zu at %.1f kPa after measured %s response; mask=0x%02x, safe retry=%d (requires restored measured support)",
                          recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                          recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                          recovery_selected_pair_pressure_kpa_,
                          measured_pair_sign_reversal
                              ? "reversed" : "no-progress",
                          recovery_rejected_candidate_mask_,
                          measured_retry_safe);
                      beginPostureSafeTransfer(now,
                          "measured contact candidate ineffective",
                          measured_retry_safe,
                          measured_pair_sign_reversal
                              ? hugmy::RecoveryFaultReason::WRONG_WAY
                              : hugmy::RecoveryFaultReason::UNRESPONSIVE);
                      break;
                    }
                  if (continuous_roll_recovery_.fault &&
                      pair_never_corrected &&
                      action_c_resume_pending_ &&
                      continuous_recovery_pair_switches_ < 1)
                    {
                      const bool measured_retry_safe =
                          std::abs(roll_error) <
                          action_c_posture_hard_roll_rad_;
                      // Preserve the one-shot alternate-pair request, but
                      // never release it from an unconfirmed pressure state.
                      continuous_recovery_retry_opposite_pending_ =
                          measured_retry_safe;
                      beginPostureSafeTransfer(now,
                          "fallback pair had no measured restoring response",
                          measured_retry_safe,
                          hugmy::RecoveryFaultReason::UNRESPONSIVE);
                      break;
                    }
                  ROS_WARN("Action C rotor phase ended: captured=%d fault=%d elapsed=%.2f s; entering monitored contact restoration (recovery is not complete yet)",
                           continuous_roll_recovery_.finished, continuous_roll_recovery_.fault,
                           (now - state_start_).toSec());
                  // Only a verified near-target completion is allowed to
                  // enter contact restoration.  In particular, a model/rate
                  // fault while pressure is still releasing must not be
                  // converted into `finished=true`: that previously ran the
                  // regrasp PI for ~20 s and drove thrust to its 5.5-N cap.
                  // The existing rapid rotor slew during PREPARE/RECOVERY is
                  // untouched; this gate applies only to exit arbitration.
                  const auto recovery_exit = hugmy::recoveryExitDecision(
                      continuous_roll_recovery_.finished,
                      continuous_roll_recovery_.fault ||
                          continuous_roll_recovery_.timed_out ||
                          (!continuous_roll_recovery_.finished &&
                           (now - state_start_).toSec() >= prepare_timeout_),
                      continuous_roll_recovery_.target_crossed,
                      roll_error, contact_capture_limit);
                  if (recovery_exit != hugmy::RecoveryExit::CONTACT_RESTORE)
                    {
                      const char* fault_reason =
                          !recovery_inputs_valid
                              ? "IMU/pressure freshness or roll-rate envelope"
                              : (!model.valid
                                  ? "invalid or wrong-sign measured moment arm"
                                  : (continuous_roll_recovery_.timed_out
                                      ? "measured response timed out"
                                      : "rotor recovery interrupted"));
                      const auto classified_fault = !recovery_inputs_valid
                          ? hugmy::RecoveryFaultReason::SENSOR_INVALID
                          : (!model.valid
                              ? hugmy::RecoveryFaultReason::GEOMETRY_INVALID
                              : (continuous_roll_recovery_.wrong_way
                                  ? hugmy::RecoveryFaultReason::WRONG_WAY
                                  : (continuous_roll_recovery_.unresponsive
                                      ? hugmy::RecoveryFaultReason::UNRESPONSIVE
                                      : (continuous_roll_recovery_.stalled
                                          ? hugmy::RecoveryFaultReason::STALL
                                          : hugmy::RecoveryFaultReason::UNCLASSIFIED))));
                      beginPostureSafeTransfer(now,
                          continuous_roll_recovery_.fault
                              ? fault_reason
                              : (continuous_roll_recovery_.finished
                                  ? "completed outside contact capture"
                                  : "continuous rotor recovery timeout"),
                          false, classified_fault);
                      break;
                    }
                  // All other exits must be a genuinely finished, captured
                  // rotor correction: interrupted passes have already gone
                  // to SAFE_TRANSFER above.
                  resetRecoveryHandoffState();
                  enterState(RECOVER, now);
                }
              break;
            }
          publishCorrectionThrust(fixed_arm_test_enabled_ ? zero_thrust
                                                          : prepare_shape_thrust_);
          publishPressureTarget(prepare_pressure_, maximum_step);
          const double correction_omega =
              body_angular_velocity_[0] * correction_axis_[0] +
              body_angular_velocity_[1] * correction_axis_[1];
          // Brake against the posture predicted from the Spinal gyro instead
          // of waiting for the measured angle to arrive at the target. This
          // removes rotor/rig inertia before it can carry roll through zero.
          const double predicted_attitude_error = std::max(0.0,
              attitude_error - std::max(0.0, correction_omega) *
                                   correction_prediction_horizon_sec_);
          const double angle_scale = std::max(0.0, std::min(1.0,
              (predicted_attitude_error - recovery_stop_angle) /
              std::max(1e-6, correction_slowdown_angle_ -
                                recovery_stop_angle)));
          // Once body motion starts, keep the captured breakaway thrust
          // constant until the attitude enters the slowdown band.  The old
          // rate_scale reduced thrust as soon as angular velocity rose, so
          // static friction stopped the body before the attitude target and
          // the controller repeatedly searched for breakaway again.
          const double feedback_scale = angle_scale;
          // A correction pair is one actuator for posture purposes. Do not
          // start one rotor merely because its pneumatic chamber exhausted
          // faster than its partner (the exact failure observed in c7).
          bool active_pair_pressure_ready_now = true;
          size_t selected_arm_count = 0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (prepare_arm_active_[arm])
              {
                ++selected_arm_count;
                active_pair_pressure_ready_now = active_pair_pressure_ready_now &&
                    prepare_pressure_[arm] <=
                        minimum_prepare_pressure_kpa_ + 1.0e-6 &&
                    measured_pressure_[arm] <=
                        minimum_prepare_pressure_kpa_ + pressure_tolerance_kpa_;
              }
          if (selected_arm_count > 0 && active_pair_pressure_ready_now)
            prepare_pair_pressure_ready_latched_ = true;
          // Once both chambers have reached the floor, keep the thrust phase
          // enabled while the controller continues commanding that floor.
          // c10 repeatedly crossed the 2 kPa measurement boundary and the old
          // instantaneous gate switched both rotors off and on each time.
          const bool active_pair_pressure_ready =
              prepare_pair_pressure_ready_latched_;
          bool geometry_ready = fixed_arm_test_enabled_;
          bool target_angle_reached = !fixed_arm_test_enabled_;
          double active_angle_sum = 0.0;
          size_t active_count = 0;
          PressureArray relative_angle{{0.0, 0.0, 0.0, 0.0}};
          PressureArray achieved_bend{{0.0, 0.0, 0.0, 0.0}};
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              // PREPARE may alter only the selected correcting pair. Reassert
              // the captured GRASP command on the other two arms every cycle
              // so pressure search and shaping thrust cannot leak into them.
              if (!prepare_arm_active_[arm])
                {
                  prepare_pressure_[arm] = grasp_pressure_[arm];
                  prepare_shape_thrust_[arm] = 0.0;
                }
              const double imu_angle = measuredArmAngle(arm);
              const double relative_change = wrapAngle(imu_angle - grasp_imu_angle_[arm]);
              relative_angle[arm] = relative_change;
              achieved_bend[arm] = std::max(0.0,
                  std::min(2.966, grasp_model_angle_[arm] + relative_change));
              if (prepare_arm_active_[arm])
                {
                  target_angle_reached = target_angle_reached &&
                      imu_angle <= prepare_target_angle_[arm] + prepare_angle_tolerance_;
                  const double target_error =
                      imu_angle - prepare_target_angle_[arm];
                  if (body_motion_detected_)
                    {
                      // Neuron arm angle is useful while preparing the low-
                      // pressure contact, but after the Spinal IMU detects
                      // whole-body motion it must not interrupt posture
                      // correction.  Hold the breakaway thrust outside the
                      // slowdown band and taper only as the body attitude
                      // approaches the target.
                      const double running_floor = std::min(
                          correction_hold_min_thrust_n_,
                          motion_hold_thrust_[arm]);
                      const double feedback_target = running_floor +
                          (motion_hold_thrust_[arm] - running_floor) *
                          feedback_scale;
                      prepare_shape_thrust_[arm] = std::max(feedback_target,
                          prepare_shape_thrust_[arm] -
                              correction_thrust_down_rate_ * dt);
                    }
                  else if (target_error > prepare_angle_tolerance_)
                    {
                      if (prepare_pressure_[arm] > minimum_prepare_pressure_kpa_ + 1e-6)
                        {
                          prepare_pressure_[arm] = std::max(minimum_prepare_pressure_kpa_,
                              prepare_pressure_[arm] - prepare_pressure_ramp_rate_ * dt);
                          prepare_shape_thrust_[arm] = std::max(0.0,
                              prepare_shape_thrust_[arm] - prepare_thrust_ramp_rate_ * dt);
                        }
                      else if (active_pair_pressure_ready &&
                               correction_motion_candidate_start_.isZero())
                        prepare_shape_thrust_[arm] = std::min(
                            std::min(prepare_max_thrust_n_,
                                     correction_max_thrust_n_),
                            prepare_shape_thrust_[arm] + prepare_thrust_ramp_rate_ * dt);
                    }
                  else if (target_error < -prepare_angle_tolerance_)
                    {
                      if (prepare_shape_thrust_[arm] > 1e-6)
                        prepare_shape_thrust_[arm] = std::max(0.0,
                            prepare_shape_thrust_[arm] - prepare_thrust_ramp_rate_ * dt);
                      else
                        prepare_pressure_[arm] = std::min(grasp_pressure_[arm],
                            prepare_pressure_[arm] + prepare_pressure_ramp_rate_ * dt);
                    }
                  active_angle_sum += imu_angle;
                  ++active_count;
                }
            }
          if (!fixed_arm_test_enabled_ && selected_arm_count > 1)
            {
              // Preserve the verified two-rotor roll/pitch moment. If either
              // member is not ready or its angle feedback asks to stop, stop
              // both instead of creating an uncommanded yaw/roll impulse.
              double common_prepare_thrust =
                  std::numeric_limits<double>::infinity();
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  common_prepare_thrust = std::min(
                      common_prepare_thrust, prepare_shape_thrust_[arm]);
              if (!active_pair_pressure_ready ||
                  !std::isfinite(common_prepare_thrust))
                common_prepare_thrust = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  prepare_shape_thrust_[arm] = common_prepare_thrust;
              if (!active_pair_pressure_ready)
                ROS_INFO_THROTTLE(1.0,
                    "PREPARE holds both correction rotors at zero until both selected arm pressures reach %.1f +/- %.1f kPa",
                    minimum_prepare_pressure_kpa_, pressure_tolerance_kpa_);
            }
          publishArmShapeDebug(relative_angle);

          double active_prepare_thrust = 0.0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (prepare_arm_active_[arm])
              active_prepare_thrust = std::max(
                  active_prepare_thrust, prepare_shape_thrust_[arm]);
          if (!body_motion_detected_)
            {
              const bool candidate_motion = active_pair_pressure_ready &&
                  active_prepare_thrust >=
                      correction_motion_detection_min_thrust_n_ &&
                  correction_omega >= correction_motion_start_rate_;
              if (!candidate_motion)
                correction_motion_candidate_start_ = ros::Time(0);
              else
                {
                  if (correction_motion_candidate_start_.isZero())
                    {
                      correction_motion_candidate_start_ = now;
                      correction_motion_candidate_error_ = attitude_error;
                    }
                  const double candidate_duration =
                      (now - correction_motion_candidate_start_).toSec();
                  const double error_reduction =
                      correction_motion_candidate_error_ - attitude_error;
                  ROS_INFO_THROTTLE(0.5,
                      "Spinal correction-motion candidate: omega %.3f rad/s for %.2f/%.2f s, posture improvement %.2f/%.2f deg",
                      correction_omega, candidate_duration,
                      correction_motion_confirm_sec_,
                      error_reduction * 180.0 / M_PI,
                      correction_motion_min_error_reduction_ * 180.0 / M_PI);
                  if (candidate_duration >= correction_motion_confirm_sec_ &&
                      error_reduction >=
                          correction_motion_min_error_reduction_)
                    {
                      body_motion_detected_ = true;
                      motion_hold_thrust_ = prepare_shape_thrust_;
                      body_motion_last_seen_ = now;
                      correction_last_progress_ = now;
                      correction_best_error_ = attitude_error;
                      ROS_INFO("Spinal confirmed whole-body correction at %.3f rad/s after %.2f s and %.2f-deg posture improvement; holding %.2f N",
                               correction_omega, candidate_duration,
                               error_reduction * 180.0 / M_PI,
                               active_prepare_thrust);
                    }
                }
            }
          else if (body_motion_detected_)
            {
              if (correction_omega >= correction_motion_stop_rate_)
                body_motion_last_seen_ = now;
              else if (!body_motion_last_seen_.isZero() &&
                       (now - body_motion_last_seen_).toSec() >=
                           correction_motion_stop_timeout_ &&
                       attitude_error > correction_slowdown_angle_)
                {
                  // A short zero-rate interval is expected around stiction.
                  // Keep the motion latch and captured thrust instead of
                  // recovering to GRASP and starting another search pulse.
                  // The thrust is lowered only by attitude proximity above.
                  ROS_INFO_THROTTLE(1.0,
                      "Correction-direction body motion paused; holding captured thrust until the attitude reaches the slowdown band");
                  body_motion_last_seen_ = now;
                }
            }
          if (body_motion_detected_)
            {
              if (attitude_error <= correction_best_error_ -
                                        correction_progress_epsilon_)
                {
                  correction_best_error_ = attitude_error;
                  correction_last_progress_ = now;
                }

              const double stalled_for = correction_last_progress_.isZero()
                  ? 0.0 : (now - correction_last_progress_).toSec();
              const double hold_limit = std::min(
                  prepare_max_thrust_n_, correction_max_thrust_n_);

              // If the captured breakaway thrust settles into a new static
              // equilibrium, resume only a slow bounded search. Near the
              // target, scale the search down with the same braking schedule
              // to retain the overshoot protection.
              if (stalled_for >= correction_stall_timeout_sec_ &&
                  std::abs(correction_omega) <=
                      correction_motion_stop_rate_ &&
                  attitude_error > recovery_stop_angle)
                {
                  const double stall_ramp =
                      correction_stall_thrust_ramp_n_s_ *
                      std::max(0.20, angle_scale);
                  double adapted_hold = 0.0;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        motion_hold_thrust_[arm] = std::min(
                            hold_limit, motion_hold_thrust_[arm] +
                                            stall_ramp * dt);
                        adapted_hold = std::max(
                            adapted_hold, motion_hold_thrust_[arm]);
                      }
                  ROS_WARN_THROTTLE(0.5,
                      "Posture correction stalled for %.2f s; adapting hold thrust %.2f -> %.2f N (limit %.2f N)",
                      stalled_for, active_prepare_thrust,
                      adapted_hold, hold_limit);
                }

              if (stalled_for >= correction_stall_abort_timeout_sec_)
                {
                  ROS_WARN("Posture did not improve for %.1f s despite adaptive thrust (limit %.2f N); restoring grasp",
                           stalled_for, hold_limit);
                  enterState(RECOVER, now);
                  break;
                }
            }
          if (body_motion_detected_)
            ROS_INFO_THROTTLE(0.25,
                "Continuous posture correction: error %.2f deg, predicted %.2f deg, omega %.3f rad/s, thrust %.2f N",
                attitude_error * 180.0 / M_PI,
                predicted_attitude_error * 180.0 / M_PI,
                correction_omega, active_prepare_thrust);

          // PREPARE thrust may already have corrected the body attitude.
          // Stop increasing it and begin regripping as soon as the attitude
          // enters the stop band; RECOVER tapers the thrust instead of
          // dropping it in one cycle.
          if (!fixed_arm_test_enabled_ &&
              attitude_error <= recovery_stop_angle)
            {
              ROS_INFO("Attitude corrected during PREPARE; tapering thrust while restoring grasp pressure");
              enterState(RECOVER, now);
              break;
            }
          if (angular_rate >= correction_max_angular_rate_)
            {
              prepare_shape_thrust_.fill(0.0);
              ROS_WARN("Angular-rate limit reached during PREPARE; stopping thrust immediately");
              enterState(RECOVER, now);
              break;
            }

          if (!fixed_arm_test_enabled_ && body_motion_detected_)
            {
              // Do not fall through to the arm-geometry completion path: it
              // would enter THRUST_CORRECTION and turn this continuous hold
              // back into another pulse.  A long timeout remains as a final
              // pneumatic/mechanical fault escape.
              if ((now - state_start_).toSec() >= prepare_timeout_)
                {
                  ROS_WARN("Continuous posture correction timed out after %.1f s; restoring grasp",
                           prepare_timeout_);
                  enterState(RECOVER, now);
                }
              break;
            }

          if (!fixed_arm_test_enabled_ && active_count > 0)
            {
              const double average_angle = active_angle_sum / active_count;
              double mean_prepare_thrust = 0.0;
              double mean_prepare_pressure = 0.0;
              double mean_target_straighten = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  {
                    mean_prepare_thrust += prepare_shape_thrust_[arm];
                    mean_prepare_pressure += prepare_pressure_[arm];
                    mean_target_straighten += prepare_straighten_angle_by_arm_[arm];
                  }
              mean_prepare_thrust /= active_count;
              mean_prepare_pressure /= active_count;
              mean_target_straighten /= active_count;
              ROS_INFO_THROTTLE(0.5,
                  "PREPARE pair: relative angle %.2f deg (target %.2f +/- %.2f), target pressure %.2f kPa, search thrust %.2f N",
                  (average_angle - ([&]() {
                    double baseline = 0.0;
                    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                      if (prepare_arm_active_[arm]) baseline += grasp_imu_angle_[arm];
                    return baseline / active_count;
                  })()) * 180.0 / M_PI,
                  -mean_target_straighten * 180.0 / M_PI,
                  prepare_angle_tolerance_ * 180.0 / M_PI,
                  mean_prepare_pressure,
                  mean_prepare_thrust);
              ROS_INFO_THROTTLE(1.0,
                  "PREPARE pressure targets [%.1f, %.1f, %.1f, %.1f] kPa; active arms [%d, %d, %d, %d]",
                  prepare_pressure_[0], prepare_pressure_[1],
                  prepare_pressure_[2], prepare_pressure_[3],
                  prepare_arm_active_[0], prepare_arm_active_[1],
                  prepare_arm_active_[2], prepare_arm_active_[3]);
              if (!std::isfinite(best_prepare_angle_))
                best_prepare_angle_ = average_angle;
              else if (average_angle < best_prepare_angle_ -
                                       prepare_progress_epsilon_)
                {
                  best_prepare_angle_ = average_angle;
                  last_angle_progress_ = now;
                }
              bool output_limited = true;
              const double paired_prepare_thrust_limit = std::min(
                  prepare_max_thrust_n_, correction_max_thrust_n_);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  output_limited = output_limited &&
                      prepare_shape_thrust_[arm] >=
                          paired_prepare_thrust_limit - 0.01;
              const bool shape_limited = output_limited &&
                  (now - last_angle_progress_).toSec() >= prepare_progress_timeout_;
              const bool timed_out = (now - state_start_).toSec() >= prepare_timeout_;
              if (timed_out && !active_pair_pressure_ready)
                {
                  prepare_shape_thrust_.fill(0.0);
                  ROS_WARN("PREPARE timed out before both selected arm pressures were ready; aborting paired thrust and restoring grasp");
                  enterState(RECOVER, now);
                  break;
                }
              geometry_ready = active_pair_pressure_ready &&
                  (target_angle_reached || shape_limited || timed_out);
              if (geometry_ready)
                {
                  correction_thrust_ = allocateCorrectionThrustFromAngles(
                      desiredThrustMoment(), achieved_bend);
                  if (target_angle_reached)
                    ROS_INFO_THROTTLE(1.0,
                        "PREPARE relative-angle target reached; reallocating correction thrust from measured geometry");
                  else
                    ROS_WARN_THROTTLE(1.0,
                        "PREPARE accepted object-dependent reachable limit; reallocating correction thrust from measured geometry");
                }
            }
          else if (fixed_arm_test_enabled_ && pressure_valid)
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              geometry_ready = geometry_ready &&
                  std::abs(measured_pressure_[arm] - prepare_pressure_[arm]) <=
                  pressure_tolerance_kpa_;

          if (geometry_ready)
            {
              if (ready_start_.isZero()) ready_start_ = now;
              if ((now - ready_start_).toSec() >= prepare_stable_duration_)
                enterState(THRUST_CORRECTION, now);
            }
          else
            ready_start_ = ros::Time(0);
          break;
          }

        case THRUST_CORRECTION:
          publishPressureTarget(prepare_pressure_, maximum_step);
          {
          const double correction_omega =
              body_angular_velocity_[0] * correction_axis_[0] +
              body_angular_velocity_[1] * correction_axis_[1];
          if (correction_omega >= correction_motion_start_rate_)
            {
              const double angle_scale = std::max(0.0, std::min(1.0,
                  (attitude_error - recovery_stop_angle) /
                  std::max(1e-6, correction_slowdown_angle_ -
                                    recovery_stop_angle)));
              for (double& thrust : correction_thrust_)
                if (thrust > 0.0)
                  thrust = std::max(correction_hold_min_thrust_n_,
                      thrust - correction_thrust_down_rate_ *
                                   (1.0 - angle_scale) * dt);
            }
          publishCorrectionThrust(correction_thrust_);
          if (attitude_error <= recovery_stop_angle ||
              angular_rate >= correction_max_angular_rate_ ||
              (now - state_start_).toSec() >= correction_thrust_duration_)
            {
              if (angular_rate >= correction_max_angular_rate_)
                prepare_shape_thrust_.fill(0.0);
              else
                prepare_shape_thrust_ = correction_thrust_;
              enterState(RECOVER, now);
            }
          }
          break;

        case POSTURE_SAFE_TRANSFER:
          {
            // Fault-safe *support transfer*, NOT another roll correction.
            // No angle PI or stalled-error integrator is active here.  The
            // same ROS state id contains explicit mechanical subphases:
            // REGRASP -> SUPPORT_VERIFY -> THRUST_RELEASE -> COMPLETED.
            // FAILED retains pneumatic support and can only remove existing
            // rotor force; it never restarts the recovery autonomously.
            publishBottomTarget(0.0);
            // A rapid but finite rotation is expected immediately after an
            // emergency handoff.  It must delay SUPPORT_VERIFY, but must not
            // be confused with stale/invalid sensing and skip REGRASP.  The
            // support monitor below applies the tighter settled-rate limits.
            const bool transfer_sensor_valid = body_imu_fresh &&
                pressure_valid && neurons_fresh && recovery_pressure_fresh &&
                std::isfinite(recovery_roll_rate) &&
                std::isfinite(body_angular_velocity_[1]);

            const double transfer_dt = std::max(0.0, dt);
            bool measured_support_ready = true;
            bool commanded_support_ready = true;
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              {
                const double target = recovery_safe_transfer_target_pressure_[arm];
                prepare_pressure_[arm] = hugmy::recoveryBoundedPressureStep(
                    prepare_pressure_[arm], target,
                    recovery_handoff_pressure_rate_kpa_s_, transfer_dt);
                commanded_support_ready = commanded_support_ready &&
                    std::abs(prepare_pressure_[arm] - target) <= 0.05;
                measured_support_ready = measured_support_ready &&
                    std::isfinite(measured_pressure_[arm]) &&
                    std::abs(measured_pressure_[arm] - target) <=
                        recovery_safe_transfer_pressure_tolerance_kpa_;
              }

            if (!transfer_sensor_valid)
              {
                if (!recovery_safe_transfer_failed_)
                  ROS_ERROR("Posture SAFE_TRANSFER lost valid IMU/pressure or exceeded the plausibility envelope; entering FAILED with bounded thrust rundown");
                recovery_safe_transfer_failed_ = true;
                recovery_safe_transfer_retry_allowed_ = false;
                continuous_recovery_blocked_ = true;
                recovery_candidate_reselect_pending_ = false;
                continuous_recovery_retry_opposite_pending_ = false;
                recovery_safe_transfer_phase_ =
                    hugmy::RecoverySafeTransferPhase::FAILED;
                posture_recovery_phase_ = hugmy::PostureRecoveryPhase::ABORT;
                recovery_fault_reason_ =
                    hugmy::RecoveryFaultReason::SENSOR_INVALID;
                recovery_support_monitor_.reset();
              }

            hugmy::RecoverySupportObservation support_observation;
            support_observation.pressure = measured_pressure_;
            support_observation.target_pressure =
                recovery_safe_transfer_target_pressure_;
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              support_observation.bend[arm] = measuredArmBendAngle(arm);
            support_observation.roll = roll_;
            support_observation.pitch = pitch_;
            support_observation.roll_rate = recovery_roll_rate;
            support_observation.pitch_rate = body_angular_velocity_[1];
            support_observation.maximum_thrust = *std::max_element(
                prepare_shape_thrust_.begin(),
                prepare_shape_thrust_.end());
            support_observation.sensors_valid = transfer_sensor_valid;
            hugmy::RecoverySupportConfig support_config;
            support_config.pressure_tolerance =
                recovery_safe_transfer_pressure_tolerance_kpa_;
            support_config.minimum_pressure =
                recovery_handoff_min_pressure_kpa_;
            support_config.maximum_bend_change =
                roll_recovery_config_.stable_bend;
            support_config.maximum_rate = roll_recovery_config_.settle_rate;
            support_config.maximum_attitude_drift =
                std::max(roll_recovery_config_.stop_margin,
                         0.5 * M_PI / 180.0);
            support_config.settle_sec = recovery_handoff_settle_sec_;
            support_config.minimum_loaded_arms = 3;

            double safe_fall_rate = recovery_handoff_thrust_fall_n_s_;
            if (recovery_safe_transfer_phase_ ==
                    hugmy::RecoverySafeTransferPhase::REGRASP &&
                commanded_support_ready && measured_support_ready &&
                transfer_sensor_valid)
              {
                recovery_safe_transfer_phase_ =
                    hugmy::RecoverySafeTransferPhase::SUPPORT_VERIFY;
                recovery_support_monitor_.reset();
                ROS_WARN("Posture SAFE_TRANSFER entered SUPPORT_VERIFY at roll %.1f deg",
                         roll_error * 180.0 / M_PI);
              }

            if (recovery_safe_transfer_phase_ ==
                    hugmy::RecoverySafeTransferPhase::SUPPORT_VERIFY)
              {
                const bool support_verified = recovery_support_monitor_.update(
                    support_observation, now.toSec(), support_config, false);
                if (support_verified)
                  {
                    recovery_safe_transfer_phase_ =
                        hugmy::RecoverySafeTransferPhase::THRUST_RELEASE;
                    recovery_support_monitor_.reset();
                    ROS_WARN("Posture SAFE_TRANSFER verified pressure/bend/attitude support; releasing residual rotor thrust");
                  }
              }

            if (recovery_safe_transfer_phase_ ==
                    hugmy::RecoverySafeTransferPhase::THRUST_RELEASE)
              safe_fall_rate = recovery_safe_transfer_rate_n_s_;
            else if (recovery_safe_transfer_phase_ ==
                         hugmy::RecoverySafeTransferPhase::FAILED)
              safe_fall_rate = recovery_handoff_thrust_fall_n_s_;

            for (double& thrust : prepare_shape_thrust_)
              thrust = hugmy::recoverySafeThrustRundown(
                  thrust, safe_fall_rate, transfer_dt);
            publishPressureTarget(prepare_pressure_,
                std::numeric_limits<double>::infinity());
            publishCorrectionThrust(prepare_shape_thrust_);
            const bool thrust_off = std::all_of(
                prepare_shape_thrust_.begin(), prepare_shape_thrust_.end(),
                [](double value) { return value <= 0.01; });

            support_observation.maximum_thrust = *std::max_element(
                prepare_shape_thrust_.begin(),
                prepare_shape_thrust_.end());
            if (recovery_safe_transfer_phase_ ==
                    hugmy::RecoverySafeTransferPhase::THRUST_RELEASE &&
                !measured_support_ready)
              {
                recovery_safe_transfer_failed_ = true;
                recovery_safe_transfer_retry_allowed_ = false;
                continuous_recovery_blocked_ = true;
                recovery_safe_transfer_phase_ =
                    hugmy::RecoverySafeTransferPhase::FAILED;
                posture_recovery_phase_ = hugmy::PostureRecoveryPhase::ABORT;
                recovery_fault_reason_ =
                    hugmy::RecoveryFaultReason::SUPPORT_LOST;
                recovery_support_monitor_.reset();
                ROS_ERROR("Posture SAFE_TRANSFER lost measured pneumatic support during thrust release; holding full support pressure and continuing bounded rundown");
              }
            const bool stable_after_thrust_off =
                recovery_safe_transfer_phase_ ==
                    hugmy::RecoverySafeTransferPhase::THRUST_RELEASE &&
                thrust_off && recovery_support_monitor_.update(
                    support_observation, now.toSec(), support_config, true);
            if (stable_after_thrust_off)
              {
                recovery_safe_transfer_phase_ =
                    hugmy::RecoverySafeTransferPhase::COMPLETED;
                grasp_pressure_ = recovery_safe_transfer_target_pressure_;
                recovery_contact_grasp_active_ = true;
                continuous_recovery_started_ = false;
                continuous_recovery_waiting_for_settle_ =
                    recovery_safe_transfer_retry_allowed_;
                continuous_recovery_blocked_ =
                    !recovery_safe_transfer_retry_allowed_;
                if (hybridLocomotionActive() && action_c_resume_pending_)
                  {
                    hybrid_grasp_pressure_ = grasp_pressure_;
                    hybrid_hold_pressure_ = true;
                  }
                if (!recovery_safe_transfer_retry_allowed_)
                  recovery_event_active_ = false;
                enterState(GRASP, now);
                ROS_WARN("Posture support transfer physically verified at roll %.1f deg; rotor off, measured grasp [%.1f %.1f %.1f %.1f] kPa, automatic correction %s",
                    roll_error * 180.0 / M_PI,
                    measured_pressure_[0], measured_pressure_[1],
                    measured_pressure_[2], measured_pressure_[3],
                    continuous_recovery_blocked_ ? "blocked" : "enabled");
                break;
              }
            if (!recovery_safe_transfer_failed_ &&
                (now - state_start_).toSec() >=
                    recovery_safe_transfer_timeout_sec_)
              {
                recovery_safe_transfer_failed_ = true;
                posture_recovery_phase_ = hugmy::PostureRecoveryPhase::ABORT;
                continuous_recovery_blocked_ = true;
                recovery_safe_transfer_retry_allowed_ = false;
                recovery_candidate_reselect_pending_ = false;
                continuous_recovery_retry_opposite_pending_ = false;
                recovery_safe_transfer_phase_ =
                    hugmy::RecoverySafeTransferPhase::FAILED;
                recovery_fault_reason_ = thrust_off
                    ? hugmy::RecoveryFaultReason::SUPPORT_UNVERIFIED
                    : hugmy::RecoveryFaultReason::REGRASP_TIMEOUT;
                recovery_support_monitor_.reset();
                ROS_ERROR("Posture SAFE_TRANSFER timed out after %.1f s; holding pneumatic support, rotor command continues to zero; automatic retry disabled",
                    recovery_safe_transfer_timeout_sec_);
              }
            ROS_WARN_THROTTLE(1.0,
                "Posture SAFE_TRANSFER: phase=%u fault=%s roll %.1f deg, target pressure [%.1f %.1f %.1f %.1f] kPa, measured_ready=%d command_ready=%d, rotor %.2f/%.2f N",
                static_cast<unsigned>(recovery_safe_transfer_phase_),
                hugmy::recoveryFaultReasonName(recovery_fault_reason_),
                roll_error * 180.0 / M_PI,
                recovery_safe_transfer_target_pressure_[0],
                recovery_safe_transfer_target_pressure_[1],
                recovery_safe_transfer_target_pressure_[2],
                recovery_safe_transfer_target_pressure_[3],
                measured_support_ready, commanded_support_ready,
                prepare_shape_thrust_[0], prepare_shape_thrust_[2]);
          }
          break;

        case RECOVER:
        default:
          if (continuous_c_recovery && continuous_recovery_started_ &&
              continuous_roll_recovery_.finished)
            {
              posture_recovery_phase_ = hugmy::PostureRecoveryPhase::CONTACT_RESTORE;
              // Repressurizing an extended arm changes both its moment arm and
              // its contact reaction.  Treat that as a measured disturbance:
              // advance pressure only inside the roll capture region, retain
              // the minimum rotor force needed to oppose rebound, and finish
              // only after pressure, roll and rotor force have all settled.
              publishBottomTarget(0.0);
              const bool handoff_sensors_fresh = body_imu_fresh &&
                  pressure_valid && neurons_fresh &&
                  recovery_pressure_fresh &&
                  std::isfinite(recovery_roll_rate);
              const bool handoff_rate_valid = handoff_sensors_fresh &&
                  std::abs(recovery_roll_rate) <
                      recovery_measurement_rate_limit_rad_s_;
              const bool handoff_inputs_valid = handoff_sensors_fresh &&
                  handoff_rate_valid;
              if (!handoff_inputs_valid)
                {
                  prepare_shape_thrust_.fill(0.0);
                  prepare_pressure_ = grasp_pressure_;
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  ready_start_ = ros::Time(0);
                  if (!handoff_sensors_fresh)
                    ROS_ERROR_THROTTLE(1.0,
                        "Action C recovery handoff lost fresh pressure/IMU data; removing rotor force and commanding safe grasp");
                  else
                    ROS_ERROR_THROTTLE(1.0,
                        "Action C recovery handoff roll rate %.1f deg/s exceeds the %.1f-deg/s measurement envelope; removing rotor force and commanding safe grasp",
                        recovery_roll_rate * 180.0 / M_PI,
                        recovery_measurement_rate_limit_rad_s_ *
                            180.0 / M_PI);
                  break;
                }

              const double signed_error = roll_error * correction_axis_[0];
              const double signed_rate = recovery_roll_rate * correction_axis_[0];
              const double abs_error = std::abs(roll_error);
              // CONTACT_RESTORE is a near-target pressure/contact operation,
              // not a fallback full-angle recovery PI.  If re-bending loses
              // capture, transfer physical support and request an explicit
              // new recovery rather than accumulating thrust for 20 seconds.
              const double restore_capture_limit = std::max(
                  15.0 * M_PI / 180.0,
                  2.0 * (recovery_stop_angle +
                         1.5 * roll_recovery_config_.stop_margin));
              if (abs_error > restore_capture_limit)
                {
                  beginPostureSafeTransfer(now,
                      "contact restoration lost near-target capture", false,
                      hugmy::RecoveryFaultReason::SUPPORT_LOST);
                  break;
                }
              // grasp_pressure_ may contain the contact-conditioned pressure
              // left by an earlier handoff.  It is useful as the present
              // command, but it must not become a ratchet which lowers the
              // mechanical anchor on every automatic recovery retry.  The
              // nominal locomotion grasp pressure is therefore a lower bound
              // for the final regrasp target.  A larger operator-selected
              // grasp pressure is still preserved.
              const auto nominal_regrasp_pressure = [&](size_t arm) {
                return hugmy::recoveryClamp(
                    std::max(base_pressure_kpa_, grasp_pressure_[arm]),
                    recovery_handoff_min_pressure_kpa_,
                    maximum_pressure_kpa_);
              };
              const auto support_floor_for = [&](size_t arm) {
                // Regrasp without surrendering the established anchor.  For
                // a released correction pair, stage A raises only that pair
                // to the load-bearing floor while the opposite pair remains
                // at nominal grasp. Stage B then bends the correction pair
                // back to nominal under measured-roll feedback. Lowering the
                // anchor pair to the same 15-kPa floor made 70/100-mm bodies
                // coast by 20--30 deg after rotor thrust had already stopped.
                if (recovery_selected_pair_release_ &&
                    prepare_arm_active_[arm])
                  return std::min(recoverySupportPressure(),
                                  nominal_regrasp_pressure(arm));
                return nominal_regrasp_pressure(arm);
              };
              const auto pressure_search_floor_for = [&](size_t arm) {
                // The opposite pair remains the nominal-pressure anchor. A
                // selected flexed pair may be relaxed gradually only during
                // the monitored terminal search; this creates a partial
                // contact state without jumping directly to the released
                // candidate.
                if (!recovery_selected_pair_release_ &&
                    prepare_arm_active_[arm])
                  return std::min(recovery_handoff_min_pressure_kpa_,
                                  nominal_regrasp_pressure(arm));
                return support_floor_for(arm);
              };
              // Use a small hysteresis outside the final +/-3-deg target.
              // This is wide enough to start a pneumatic handoff without
              // asking the rotor loop to chatter on the target boundary,
              // but remains much smaller than the locomotion recovery
              // trigger.
              const double capture_error = recovery_stop_angle +
                  1.5 * roll_recovery_config_.stop_margin;
              // A 20-deg hard limit is not a valid regrasp target. Search for
              // a contact equilibrium only inside the ordinary recovery
              // envelope, while retaining the independent hard limit solely
              // as a safety boundary.
              // Regrasp is part of recovery, not permission to settle again
              // at the 10-deg locomotion trigger.  Pressure may advance only
              // while the body remains in the terminal capture band.  If
              // flexion pushes roll back out, pause pressure and let the
              // continuous thrust loop restore the attitude first.
              const double handoff_acceptance_error = std::min(
                  action_c_posture_hard_roll_rad_, capture_error);
              // Braking may start slightly outside the requested band to
              // account for rotor rundown, but completion itself is strict:
              // nominal/contact-conditioned regrasp must retain +/-3 deg.
              const double handoff_measurement_margin = std::min(
                  0.25 * M_PI / 180.0,
                  0.25 * roll_recovery_config_.stop_margin);
              const double handoff_settle_error = recovery_stop_angle;
              const bool strict_target_observed_before_regrasp =
                  recovery_handoff_pressure_trimmed_ &&
                  std::isfinite(recovery_handoff_best_error_rad_) &&
                  recovery_handoff_best_error_rad_ <= handoff_settle_error;
              const double active_handoff_settle_error =
                  handoff_settle_error;
              // The requested recovery result is the configured +/- stop
              // band itself.  Driving all the way to one degree (the former
              // stop-minus-two-margins value) kept 0.6--2 N applied while
              // the measured body was already oscillating across +/-3 deg,
              // so the handoff could never satisfy its own settle test. Stop
              // adding torque on entry to the requested band and let the
              // bounded fall rate/prediction remove the residual command.
              // Start rundown just before the strict +/-3-deg boundary.
              // Waiting for a sampled value at or below exactly 3.000 deg
              // left about 2 N applied at the 3.01-deg static-friction
              // plateau.  The quarter-degree lead is smaller than the
              // completion/IMU hysteresis and lets the finite slew-down
              // carry the body into, rather than back out of, the target.
              const double handoff_drive_stop = recovery_stop_angle +
                  handoff_measurement_margin;
              const double handoff_brake_entry = handoff_drive_stop;
              const double pressure_advance_rate_limit = std::min(
                  0.08, roll_recovery_config_.settle_rate);
              const bool pressure_not_worsening =
                  roll_error * recovery_roll_rate >= -0.002;
              // Regrasp is deliberately split into two stages.  The pair not
              // used for correction is the mechanical anchor and must remain
              // at its grasp pressure.  Stage A confirms that anchor while
              // the released correction pair reaches only the minimum
              // load-bearing pressure.  Stage B raises the correction pair
              // toward grasp under the measured roll loop.  Lowering the
              // anchor pair to 15 kPa here removed the only established
              // support and reproduced the roll error after an otherwise
              // successful rotor correction.
              bool support_commands_equalized = true;
              bool support_measurements_equalized = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double support_floor = support_floor_for(arm);
                  support_commands_equalized = support_commands_equalized &&
                    std::abs(prepare_pressure_[arm] -
                        support_floor) <= 0.05;
                  support_measurements_equalized =
                      support_measurements_equalized &&
                      std::isfinite(measured_pressure_[arm]) &&
                      std::abs(measured_pressure_[arm] -
                          support_floor) <= 1.0;
                }
              if (!recovery_handoff_support_equalized_ &&
                  support_commands_equalized &&
                  support_measurements_equalized)
                recovery_handoff_support_equalized_ = true;
              const bool support_transfer_equalized =
                  recovery_handoff_support_equalized_;
              // The pre-transfer [full grasp, released pair] pressure is not
              // a candidate equilibrium: restoring it after a wrong-way
              // response recreates exactly the pneumatic roll disturbance
              // that stage A removed.  Start the best-pressure search only
              // after all four chambers actually share the support load.
              if (support_transfer_equalized &&
                  !std::isfinite(recovery_handoff_best_error_rad_))
                {
                  recovery_handoff_best_error_rad_ = abs_error;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor =
                          pressure_search_floor_for(arm);
                      recovery_handoff_best_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                }
              if (!recovery_handoff_pressure_trimmed_ &&
                  !recovery_handoff_support_equalized_ &&
                  !support_commands_equalized)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  {
                    const double support_floor = support_floor_for(arm);
                    prepare_pressure_[arm] += hugmy::recoveryClamp(
                      support_floor - prepare_pressure_[arm],
                      -recovery_handoff_pressure_rate_kpa_s_ *
                          std::max(0.0, dt),
                      recovery_handoff_pressure_rate_kpa_s_ *
                          std::max(0.0, dt));
                  }

              const bool pressure_advance_allowed =
                  recovery_selected_pair_release_ &&
                  !recovery_handoff_pressure_trimmed_ &&
                  !recovery_handoff_rollback_before_reselect_ &&
                  support_transfer_equalized &&
                  abs_error <= handoff_acceptance_error &&
                  std::abs(recovery_roll_rate) <= pressure_advance_rate_limit &&
                  pressure_not_worsening;
              if (pressure_advance_allowed)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  {
                    prepare_pressure_[arm] = std::min(nominal_regrasp_pressure(arm),
                        prepare_pressure_[arm] +
                            recovery_handoff_pressure_rate_kpa_s_ *
                                std::max(0.0, dt));
                  }

              // A flexed pair may correct most of the roll but retain a
              // 3--4-deg zero-thrust bias at full pressure. Rather than
              // rejecting that measured-good contact and jumping to the
              // fully released state, relax only the selected pair slowly
              // while the opposite pair remains the 30-kPa anchor. Search
              // starts only after a quiet retained-progress plateau near the
              // target. The best measured pressure is recorded below and is
              // restored if the contact moment changes sign.
              const bool terminal_pressure_search =
                  !recovery_selected_pair_release_ &&
                  !recovery_handoff_pressure_trimmed_ &&
                  support_transfer_equalized &&
                  signed_error > handoff_drive_stop &&
                  abs_error <= capture_error &&
                  std::abs(recovery_roll_rate) <=
                      pressure_advance_rate_limit &&
                  ((recovery_handoff_braking_ &&
                    abs_error > recovery_stop_angle) ||
                   (recovery_handoff_last_progress_time_ >= 0.0 &&
                    now.toSec() - recovery_handoff_last_progress_time_ >=
                        roll_recovery_config_.stall_sec));
              if (terminal_pressure_search)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  if (prepare_arm_active_[arm])
                    {
                      const double floor = pressure_search_floor_for(arm);
                      // Keep the command outside the approximately 1-kPa
                      // pressure-controller deadband. The chamber still
                      // vents at its physical/rate-limited speed; this is a
                      // target lead, not an instantaneous pressure change.
                      const double venting_target =
                          std::isfinite(measured_pressure_[arm])
                          ? measured_pressure_[arm] - 2.0
                          : prepare_pressure_[arm];
                      prepare_pressure_[arm] = std::max(floor,
                          std::min(venting_target,
                              prepare_pressure_[arm] -
                                  0.25 *
                                      recovery_handoff_pressure_rate_kpa_s_ *
                                      std::max(0.0, dt)));
                      recovery_handoff_target_pressure_[arm] =
                          prepare_pressure_[arm];
                    }

              // Record the actual chamber pressure at the best measured roll.
              // The command leads the pneumatic state, so retaining the
              // command value would overshoot the equilibrium by several kPa.
              double minimum_handoff_pressure = std::numeric_limits<double>::infinity();
              bool handoff_pressure_valid = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double support_floor =
                      pressure_search_floor_for(arm);
                  handoff_pressure_valid = handoff_pressure_valid &&
                      std::isfinite(measured_pressure_[arm]);
                  minimum_handoff_pressure = std::min(
                      minimum_handoff_pressure,
                      measured_pressure_[arm] - support_floor);
                }
              const bool pressure_candidate = support_transfer_equalized &&
                  handoff_pressure_valid &&
                  minimum_handoff_pressure >= 0.0 &&
                  std::abs(recovery_roll_rate) <= pressure_advance_rate_limit;
              if (!recovery_handoff_pressure_trimmed_ && pressure_candidate &&
                  abs_error < recovery_handoff_best_error_rad_)
                {
                  recovery_handoff_best_error_rad_ = abs_error;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor =
                          pressure_search_floor_for(arm);
                      recovery_handoff_best_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                }
              double pressure_distance_from_best = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                pressure_distance_from_best = std::max(
                    pressure_distance_from_best,
                    std::abs(measured_pressure_[arm] -
                             recovery_handoff_best_pressure_[arm]));
              const bool pressure_worsening = !recovery_handoff_pressure_trimmed_ &&
                  support_transfer_equalized &&
                  std::isfinite(recovery_handoff_best_error_rad_) &&
                  abs_error >= recovery_handoff_best_error_rad_ +
                      recovery_handoff_trim_margin_rad_ &&
                  pressure_distance_from_best >= 0.5;
              // A clear outward body rate after pressure has moved away from
              // the best measured point is not chamber noise.  Roll back in
              // the same cycle instead of waiting through the ordinary
              // pressure confirmation dwell while the soft contact keeps
              // rotating the body.
              const bool fast_pressure_worsening = pressure_worsening &&
                  roll_error * recovery_roll_rate < -0.01 &&
                  std::abs(recovery_roll_rate) >=
                      0.5 * roll_recovery_config_.start_rate;
              if (pressure_worsening)
                {
                  if (recovery_handoff_worsening_since_ < 0.0)
                    recovery_handoff_worsening_since_ = now.toSec();
                  if (fast_pressure_worsening ||
                      now.toSec() - recovery_handoff_worsening_since_ >=
                          recovery_handoff_trim_confirm_sec_)
                    {
                      recovery_handoff_pressure_trimmed_ = true;
                      // Regrasp means restoring a load-bearing contact, not
                      // forcing every chamber to the same nominal pressure.
                      // On a curved object, the selected pair's moment can
                      // reverse while it is re-bent: in the 70-mm fixture the
                      // measured minimum occurred near [15,15,30,30] kPa and
                      // forcing the released side to 30 kPa recreated a
                      // persistent 3--6-deg oscillation. Retain the measured
                      // per-arm pressure at the roll minimum. The opposite
                      // anchor remains at nominal grasp because its support
                      // floor and best pressure are both nominal.
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        recovery_handoff_target_pressure_[arm] =
                            hugmy::recoveryClamp(
                                recovery_handoff_best_pressure_[arm],
                                pressure_search_floor_for(arm),
                                nominal_regrasp_pressure(arm));
                      ROS_WARN("Action C recovery handoff selected contact-conditioned grasp [%.1f %.1f %.1f %.1f] kPa after roll minimum %.2f deg; nominal repressurization worsened posture",
                               recovery_handoff_target_pressure_[0],
                               recovery_handoff_target_pressure_[1],
                               recovery_handoff_target_pressure_[2],
                               recovery_handoff_target_pressure_[3],
                               recovery_handoff_best_error_rad_ * 180.0 / M_PI);
                    }
                }
              else
                recovery_handoff_worsening_since_ = -1.0;
              if (recovery_handoff_pressure_trimmed_)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  prepare_pressure_[arm] += hugmy::recoveryClamp(
                      recovery_handoff_target_pressure_[arm] - prepare_pressure_[arm],
                      -recovery_handoff_pressure_rate_kpa_s_ * std::max(0.0, dt),
                      recovery_handoff_pressure_rate_kpa_s_ * std::max(0.0, dt));

              // Arm flexion during pneumatic handoff changes the contact
              // pivot.  Consequently a pair which corrected roll while the
              // arm was extended can reverse its effective moment after the
              // chamber becomes load bearing.  Detect that reversal from a
              // sustained measured loss relative to the best handoff pose;
              // never continue integrating the same pair to the thrust cap.
              double selected_handoff_thrust = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  selected_handoff_thrust = std::max(
                      selected_handoff_thrust, prepare_shape_thrust_[arm]);
              const bool thrust_wrong_way_candidate =
                  !recovery_handoff_thrust_wrong_way_ &&
                  selected_handoff_thrust >=
                      roll_recovery_config_.detect_thrust &&
                  ((support_transfer_equalized &&
                    std::isfinite(recovery_handoff_best_error_rad_) &&
                    abs_error >= recovery_handoff_best_error_rad_ +
                        recovery_handoff_trim_margin_rad_ &&
                    roll_error * recovery_roll_rate < -0.002) ||
                   (!support_transfer_equalized &&
                    signed_error > recovery_stop_angle &&
                    signed_rate < -roll_recovery_config_.start_rate));
              if (thrust_wrong_way_candidate)
                {
                  if (recovery_handoff_thrust_worsening_since_ < 0.0)
                    recovery_handoff_thrust_worsening_since_ = now.toSec();
                  if (now.toSec() - recovery_handoff_thrust_worsening_since_ >=
                      recovery_handoff_trim_confirm_sec_)
                    {
                      recovery_handoff_thrust_wrong_way_ = true;
                      recovery_handoff_pressure_trimmed_ = true;
                      if (recovery_selected_contact_pivot_enabled_)
                        {
                          const uint8_t candidate_bit = static_cast<uint8_t>(
                              1u << (4u * recovery_selected_pair_index_ +
                                     recovery_selected_pressure_candidate_index_));
                          recovery_rejected_candidate_mask_ |= candidate_bit;
                          recovery_rejected_direction_sign_ =
                              roll_error >= 0.0 ? 1 : -1;
                          recovery_candidate_reselect_pending_ = true;
                          // The rejection mask identifies the exact
                          // pair/contact state.  Do not additionally force
                          // the opposite pair through the legacy mapping.
                          recovery_handoff_pair_was_wrong_ = false;
                        }
                      if (recovery_selected_contact_pivot_enabled_)
                        recovery_handoff_target_pressure_ =
                            recoveryCandidateSwitchPressure();
                      else
                        for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                          recovery_handoff_target_pressure_[arm] =
                              support_floor_for(arm);
                      recovery_handoff_braking_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      ready_start_ = ros::Time(0);
                      ROS_WARN("Action C recovery handoff measured a reversed rotor response after re-bending; braking pair at %.2f N and retaining pressure [%.1f %.1f %.1f %.1f] kPa before a fresh recovery decision",
                               selected_handoff_thrust,
                               recovery_handoff_target_pressure_[0],
                               recovery_handoff_target_pressure_[1],
                               recovery_handoff_target_pressure_[2],
                               recovery_handoff_target_pressure_[3]);
                    }
                }
              else if (!recovery_handoff_thrust_wrong_way_)
                recovery_handoff_thrust_worsening_since_ = -1.0;

              // The selected pair can create moment only in its original
              // correction direction. Model FF is used outside the terminal
              // band; rate PI supplies only the residual needed while arm
              // pressure and contact reaction are changing.
              const auto model = measuredRecoveryModel();
              // The requested +/-3-deg band is the final zero-thrust
              // tolerance, not the best point at which to turn the rotor
              // off. On small curved fixtures the contact spring returned
              // roll by about two degrees after rundown, producing a 3--6
              // deg limit cycle. During the pneumatic handoff drive to an
              // inner target, then use the outer band only for completion.
              const double requested_rate = signed_error > handoff_drive_stop
                  ? std::min(roll_recovery_config_.max_target_rate,
                        roll_recovery_config_.angle_gain *
                            (signed_error - handoff_drive_stop))
                  : 0.0;
              const double rate_error = requested_rate - signed_rate;
              double feedforward = 0.0;
              if (signed_error > handoff_drive_stop && model.valid &&
                  std::isfinite(model.lever) && model.lever >=
                      roll_recovery_config_.min_lever)
                feedforward = hugmy::recoveryClamp(
                    model.moment / model.lever, 0.0,
                    roll_recovery_config_.limit);
              // The handoff controller must be weaker than the actual
              // RECOVERY loop and can only integrate in settled contact.
              const double contact_restore_limit = std::min(
                  roll_recovery_config_.limit,
                  recovery_contact_restore_thrust_limit_n_);
              bool pressure_geometry_settled = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                pressure_geometry_settled = pressure_geometry_settled &&
                    std::isfinite(measured_pressure_[arm]) &&
                    std::abs(measured_pressure_[arm] - prepare_pressure_[arm]) <=
                        recovery_safe_transfer_pressure_tolerance_kpa_;
              double integral_delta = hugmy::recoveryClamp(
                  roll_recovery_config_.rate_ki * rate_error,
                  -roll_recovery_config_.integral_rate,
                  roll_recovery_config_.integral_rate) * std::max(0.0, dt);
              // A quiet full-grasp plateau must not *force* a positive I
              // increment: that was the 20-second, 5.5-N pressure-restoration
              // runaway on the human forearm. Ordinary PI is still used
              // when measured motion is responsive, but it may not wind up
              // solely because the error has stalled at fixed contact.
              const bool terminal_contact_plateau =
                  support_transfer_equalized &&
                  pressure_geometry_settled &&
                  !recovery_handoff_braking_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  signed_error > handoff_brake_entry &&
                  recovery_handoff_last_progress_time_ >= 0.0 &&
                  now.toSec() - recovery_handoff_last_progress_time_ >=
                      roll_recovery_config_.stall_sec &&
                  std::abs(recovery_roll_rate) <
                      roll_recovery_config_.start_rate;
              if (terminal_contact_plateau)
                integral_delta = std::min(0.0, integral_delta);
              if (!pressure_geometry_settled || !support_transfer_equalized)
                {
                  integral_delta = 0.0;
                  recovery_handoff_integral_n_ = 0.0;
                }
              const double raw_target = feedforward +
                  roll_recovery_config_.rate_kp * rate_error +
                  recovery_handoff_integral_n_;
              if ((integral_delta > 0.0 && raw_target >= contact_restore_limit) ||
                  (integral_delta < 0.0 && raw_target <= 0.0))
                integral_delta = 0.0;
              recovery_handoff_integral_n_ = hugmy::recoveryClamp(
                  recovery_handoff_integral_n_ + integral_delta,
                  0.0, contact_restore_limit);
              double handoff_target = hugmy::recoveryClamp(feedforward +
                  roll_recovery_config_.rate_kp * rate_error +
                  recovery_handoff_integral_n_, 0.0,
                  contact_restore_limit);
              // Keep measured roll feedback active below the nominal contact
              // floor as well.  The same released geometry has just produced
              // the successful correction; disabling it until 15 kPa leaves
              // pneumatic rebound completely open loop. Wrong-way detection
              // below still removes thrust if the effective sign really
              // changes during flexion.
              // During stage A retain only a conservative amount of the
              // already verified correction force.  The pneumatic transfer
              // can otherwise move roll by more than ten degrees before the
              // four chambers reach equal support.  Do not integrate during
              // this changing geometry, and never use the retained force on
              // the far side of the target.  Once support is equalized, the
              // full measured model/rate loop resumes.
              if (!support_transfer_equalized)
                {
                  recovery_handoff_integral_n_ = 0.0;
                  if (signed_error > recovery_stop_angle &&
                      !recovery_handoff_thrust_wrong_way_)
                    handoff_target = std::min(handoff_target,
                        recovery_handoff_stage_a_thrust_max_n_);
                  else
                    handoff_target = 0.0;
                  // Repressurising an exhausted pneumatic arm creates a
                  // repeatable roll moment before the attitude error has had
                  // time to grow.  Pure error feedback therefore reacted one
                  // to two seconds late: pressure reached 15 kPa while the
                  // body had already rebounded from 1 to 6--8 deg.  Schedule
                  // a bounded model feed-forward with the measured support
                  // build-up, then leave all residual correction to the same
                  // rate loop.  If the target has actually been crossed, the
                  // one-direction pair is kept off as before.
                  // Do not add pressure feed-forward while inertia is
                  // already carrying the body toward the target faster than
                  // requested.  The former condition kept 1.5 N on until
                  // the body had crossed the target by the full stop angle;
                  // on the 130-mm fixture this converted a +0.95-deg,
                  // otherwise successful handoff into a -10-deg overshoot.
                  const double pressure_feedforward_rate_limit = std::max(
                      requested_rate,
                      0.5 * roll_recovery_config_.settle_rate);
                  if (signed_error > 0.0 &&
                      signed_rate <= pressure_feedforward_rate_limit &&
                      !recovery_handoff_thrust_wrong_way_)
                    {
                      double support_fraction = 0.0;
                      size_t support_count = 0;
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        if (prepare_arm_active_[arm])
                          {
                            support_fraction += hugmy::recoveryClamp(
                                prepare_pressure_[arm] /
                                    std::max(1.0,
                                      recovery_handoff_min_pressure_kpa_),
                                0.0, 1.0);
                            ++support_count;
                          }
                      if (support_count > 0)
                        support_fraction /= support_count;
                      const double pressure_feedforward =
                          roll_recovery_config_.fallback_thrust +
                          support_fraction * std::max(0.0,
                              recovery_handoff_stage_a_thrust_max_n_ -
                                  roll_recovery_config_.fallback_thrust);
                      handoff_target = std::max(handoff_target,
                          std::min(recovery_handoff_stage_a_thrust_max_n_,
                                   pressure_feedforward));
                    }
                }
              if (recovery_handoff_thrust_wrong_way_)
                {
                  handoff_target = 0.0;
                  recovery_handoff_integral_n_ = 0.0;
                }
              bool command_at_grasp = true;
              handoff_target = std::min(handoff_target,
                  contact_restore_limit);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                command_at_grasp = command_at_grasp &&
                    std::abs(prepare_pressure_[arm] -
                        recovery_handoff_target_pressure_[arm]) <= 0.05;
              if (!recovery_handoff_braking_ && command_at_grasp &&
                  abs_error <= (strict_target_observed_before_regrasp
                      ? active_handoff_settle_error : handoff_brake_entry) &&
                  std::abs(recovery_roll_rate) <=
                      0.5 * roll_recovery_config_.settle_rate)
                {
                  recovery_handoff_braking_ = true;
                  recovery_handoff_brake_error_rad_ = abs_error;
                }
              // Terminal braking is a one-way transition for this measured
              // contact pass. Re-accelerating the same pair after pneumatic
              // rebound caused the observed 0 -> 5.5-N command and a large
              // overshoot. If zero-thrust contact settles outside the target,
              // the bounded logic below requests a fresh measured pass.
              // While the released pair is still taking load, being briefly
              // inside the stop band must not erase the pressure-scheduled
              // feed-forward above.  Otherwise the changing contact force
              // rebounds the body before attitude feedback can react.  The
              // ordinary terminal brake is enabled once support has been
              // equalised; target crossing and verified wrong-way response
              // still stop thrust immediately.
              if (recovery_handoff_braking_ ||
                  // A selected pair is one-directional.  Once the measured
                  // attitude crosses the target, any remaining command can
                  // only increase overshoot.  Brake at the crossing; if the
                  // zero-thrust body later leaves the stop band, the code
                  // below freezes contact pressure and selects the opposite
                  // signed candidate.
                  signed_error <= 0.0 ||
                  // Entering the requested +/-3-deg band is already the
                  // signal to remove torque. Requiring a small angular rate
                  // here kept applying 0.6--1.2 N while the body crossed the
                  // target, which excited a persistent handoff limit cycle.
                  // Rate is still required by handoff_settled below, after
                  // the command has actually fallen to zero.
                  (support_transfer_equalized &&
                   abs_error <= handoff_drive_stop))
                {
                  handoff_target = 0.0;
                  recovery_handoff_integral_n_ = std::max(0.0,
                      recovery_handoff_integral_n_ -
                          roll_recovery_config_.integral_rate * std::max(0.0, dt));
                }
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double target = prepare_arm_active_[arm]
                      ? handoff_target : 0.0;
                  const double handoff_fall_rate =
                      recovery_handoff_thrust_wrong_way_
                      ? std::max(3.0, recovery_handoff_thrust_fall_n_s_)
                      : (recovery_handoff_braking_
                          ? std::max(1.5,
                              recovery_handoff_thrust_fall_n_s_)
                          : recovery_handoff_thrust_fall_n_s_);
                  prepare_shape_thrust_[arm] += hugmy::recoveryClamp(
                      target - prepare_shape_thrust_[arm],
                      -handoff_fall_rate *
                          std::max(0.0, dt),
                      roll_recovery_config_.rise * std::max(0.0, dt));
                }
              publishPressureTarget(prepare_pressure_,
                  std::numeric_limits<double>::infinity());
              publishCorrectionThrust(prepare_shape_thrust_);

              if (!command_at_grasp) action_c_pressure_ready_gate_.reset();
              const bool pressure_ready = command_at_grasp &&
                  actionCPressureReady(recovery_handoff_target_pressure_, now);
              const bool thrust_stopped = std::all_of(
                  prepare_shape_thrust_.begin(), prepare_shape_thrust_.end(),
                  [](double thrust) { return thrust <= 0.01; });
              // Crossing zero during regrasp is often caused by the pressure
              // transfer itself, not by a wrong rotor pair.  First return to
              // the pressure measured at the best roll.  Start a new rotor
              // pass only if that contact-conditioned pressure has settled
              // and the residual is still outside the accepted roll band.
              const bool rollback_pressure_settled =
                  recovery_handoff_rollback_before_reselect_ &&
                  pressure_ready && thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate;
              if (rollback_pressure_settled)
                {
                  recovery_handoff_rollback_before_reselect_ = false;
                  if (abs_error > active_handoff_settle_error)
                    {
                      recovery_candidate_reselect_pending_ = true;
                      recovery_handoff_thrust_wrong_way_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      ready_start_ = ros::Time(0);
                      ROS_WARN("Action C best-pressure rollback settled at roll %.2f deg outside the %.2f-deg target; requesting a fresh measured-contact pass",
                               roll_error * 180.0 / M_PI,
                               active_handoff_settle_error * 180.0 / M_PI);
                    }
                  else
                    {
                      recovery_candidate_reselect_pending_ = false;
                      recovery_handoff_thrust_wrong_way_ = false;
                      ready_start_ = ros::Time(0);
                      ROS_INFO("Action C best-pressure rollback retained roll %.2f deg inside the target; completing handoff without another rotor pass",
                               roll_error * 180.0 / M_PI);
                    }
                }
              // A useful rotor correction may disappear after the selected
              // pair bends back around the object.  In that case the moment
              // does not necessarily reverse: it can simply vanish at a
              // contact equilibrium outside the requested +/-3-deg band.
              // The existing wrong-way detector cannot observe that case.
              // Once full pneumatic grasp is measured, therefore retain a
              // progress reference and judge the candidate from the averaged
              // roll-error decay rate over a bounded observation window.
              // Low thrust alone never rejects a candidate, and ongoing
              // angular motion is allowed to finish before this decision.
              const bool full_grasp_tracking =
                  recovery_selected_contact_pivot_enabled_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  command_at_grasp && pressure_ready &&
                  abs_error > active_handoff_settle_error;
              // The rate-sign detector above catches a clear reverse
              // rotation, but not the slow contact creep observed after a
              // flexed pair has reached full pressure.  Establish a fresh
              // reference only after pressure is settled and meaningful
              // thrust is present.  A quiet plateau remains eligible for
              // the normal high-authority retry below; only retained error
              // growth rejects the candidate at low authority.
              const bool flexed_handoff_response_active =
                  recovery_selected_contact_pivot_enabled_ &&
                  !recovery_selected_pair_release_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  !recovery_handoff_braking_ &&
                  command_at_grasp && pressure_ready &&
                  signed_error > recovery_stop_angle &&
                  selected_handoff_thrust >=
                      roll_recovery_config_.detect_thrust;
              bool handoff_candidate_worsening = false;
              if (!flexed_handoff_response_active)
                {
                  recovery_handoff_candidate_reference_rad_ =
                      std::numeric_limits<double>::infinity();
                  recovery_handoff_candidate_reference_time_ = -1.0;
                }
              else if (!std::isfinite(
                           recovery_handoff_candidate_reference_rad_))
                {
                  recovery_handoff_candidate_reference_rad_ = abs_error;
                  recovery_handoff_candidate_reference_time_ = now.toSec();
                }
              else
                {
                  const double candidate_age = now.toSec() -
                      recovery_handoff_candidate_reference_time_;
                  if (candidate_age >=
                      recovery_wrong_geometry_progress_window_sec_)
                    {
                      const double candidate_decay_rate =
                          (recovery_handoff_candidate_reference_rad_ -
                           abs_error) / std::max(1.e-3, candidate_age);
                      handoff_candidate_worsening =
                          candidate_decay_rate <=
                              -recovery_wrong_geometry_min_error_decay_rate_rad_s_;
                      // Start the next averaged observation window from the
                      // current measured attitude.  This avoids allowing an
                      // old large improvement to hide a later moment-sign
                      // reversal, without relying on a fixed angle quantum.
                      recovery_handoff_candidate_reference_rad_ = abs_error;
                      recovery_handoff_candidate_reference_time_ = now.toSec();
                    }
                }
              if (!full_grasp_tracking)
                {
                  recovery_handoff_progress_reference_rad_ =
                      std::numeric_limits<double>::infinity();
                  recovery_handoff_last_progress_time_ = -1.0;
                }
              else
                {
                  if (!std::isfinite(
                          recovery_handoff_progress_reference_rad_) ||
                      recovery_handoff_last_progress_time_ < 0.0)
                    {
                      recovery_handoff_progress_reference_rad_ = abs_error;
                      recovery_handoff_last_progress_time_ = now.toSec();
                    }
                  else if (now.toSec() -
                               recovery_handoff_last_progress_time_ >=
                           recovery_wrong_geometry_progress_window_sec_ &&
                           (recovery_handoff_progress_reference_rad_ -
                                abs_error) /
                                   std::max(1.e-3,
                                       now.toSec() -
                                           recovery_handoff_last_progress_time_) >=
                               recovery_wrong_geometry_min_error_decay_rate_rad_s_)
                    {
                      recovery_handoff_progress_reference_rad_ = abs_error;
                      recovery_handoff_last_progress_time_ = now.toSec();
                    }

                  const double quiet_rate_limit = std::max(
                      2.0 * roll_recovery_config_.settle_rate,
                      roll_recovery_config_.start_rate);
                  // A loaded flexed contact also has static breakaway
                  // friction.  The 70-mm fixture repeatedly improved from
                  // 12 to about 6 deg, then was rejected after only 0.9 N
                  // because the ordinary detection threshold was mistaken
                  // for proof that its moment had vanished.  Releasing that
                  // useful pair then produced a much larger roll excursion.
                  // For either contact state, declare no response only after
                  // the same bounded high-authority gate used by the rotor
                  // phase.  A measured wrong-sign response remains an
                  // immediate independent rejection path above.
                  const double handoff_retry_thrust = std::min(
                      contact_restore_limit,
                      std::max(roll_recovery_config_.detect_thrust,
                          contact_restore_limit *
                              roll_recovery_config_.pair_retry_thrust_fraction));
                  const double handoff_retry_dwell =
                      recovery_selected_pair_release_
                      ? 2.0 * roll_recovery_config_.pair_retry_sec
                      : roll_recovery_config_.pair_retry_sec;
                  const bool handoff_candidate_stalled =
                      abs_error > capture_error &&
                      selected_handoff_thrust >= handoff_retry_thrust &&
                      std::abs(recovery_roll_rate) <= quiet_rate_limit &&
                      now.toSec() - recovery_handoff_last_progress_time_ >=
                          handoff_retry_dwell &&
                      (recovery_handoff_progress_reference_rad_ - abs_error) /
                              std::max(1.e-3,
                                  now.toSec() -
                                      recovery_handoff_last_progress_time_) <
                          recovery_wrong_geometry_min_error_decay_rate_rad_s_;
                  if (handoff_candidate_stalled ||
                      handoff_candidate_worsening)
                    {
                      const uint8_t candidate_bit = static_cast<uint8_t>(
                          1u << (4u * recovery_selected_pair_index_ +
                                 recovery_selected_pressure_candidate_index_));
                      recovery_rejected_candidate_mask_ |= candidate_bit;
                      recovery_rejected_direction_sign_ =
                          roll_error >= 0.0 ? 1 : -1;
                      // A quiet no-progress contact can be replaced by the
                      // other pressure/topology candidate of the SAME rotor
                      // pair without first removing useful rotor authority.
                      // This is deliberately forbidden for a measured
                      // wrong-way response, a direction change, or the final
                      // target envelope: those cases still use the ordinary
                      // monotonic rundown and supported re-selection.
                      bool same_pair_continuation = false;
                      if (handoff_candidate_stalled &&
                          !handoff_candidate_worsening &&
                          !recovery_handoff_braking_ &&
                          recovery_preferred_pair_locked_ &&
                          abs_error > handoff_drive_stop +
                              roll_recovery_config_.stop_margin &&
                          signed_rate >= -roll_recovery_config_.start_rate)
                        {
                          const size_t old_pair = recovery_selected_pair_index_;
                          const auto old_active = prepare_arm_active_;
                          const bool old_release = recovery_selected_pair_release_;
                          const double old_pressure = recovery_selected_pair_pressure_kpa_;
                          const size_t old_pressure_index =
                              recovery_selected_pressure_candidate_index_;
                          const bool old_pivot_enabled =
                              recovery_selected_contact_pivot_enabled_;
                          const Vector3 old_pivot = recovery_selected_contact_pivot_;
                          const double old_depth = recovery_selected_pair_mean_depth_rad_;
                          const double old_support_loss =
                              recovery_selected_support_loss_moment_nm_;
                          const PressureArray old_release_start =
                              recovery_selected_release_start_pressure_;
                          const bool old_path_zero =
                              recovery_selected_path_requires_zero_thrust_;
                          const double old_path_lever =
                              recovery_selected_path_min_lever_m_;
                          const bool old_extension_latch =
                              recovery_extension_mode_latched_;
                          const int old_extension_direction =
                              recovery_extension_direction_sign_;
                          const int old_unified_candidate =
                              recovery_unified_selected_candidate_index_;
                          const bool old_selection_available =
                              recovery_pair_selection_available_;

                          const auto measured_model_before_reselection =
                              measuredRecoveryModel();
                          selectContinuousRecoveryPair(roll_error);
                          same_pair_continuation =
                              recovery_pair_selection_available_ &&
                              recovery_selected_pair_index_ == old_pair &&
                              measured_model_before_reselection.valid &&
                              std::isfinite(measured_model_before_reselection.lever) &&
                              measured_model_before_reselection.lever >=
                                  roll_recovery_config_.min_lever;
                          if (same_pair_continuation)
                            {
                              continuous_roll_recovery_.retargetKeepingThrust(
                                  attitude_error, now.toSec(),
                                  selected_handoff_thrust);
                              resetRecoveryHandoffState();
                              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                                {
                                  const bool selected_arm =
                                      prepare_arm_active_[arm];
                                  prepare_pressure_[arm] = selected_arm &&
                                      recovery_selected_pair_release_
                                      ? recoverySelectedPairPressure(arm)
                                      : nominal_regrasp_pressure(arm);
                                }
                              recovery_handoff_target_pressure_ =
                                  prepare_pressure_;
                              recovery_candidate_reselect_pending_ = false;
                              continuous_recovery_started_ = true;
                              continuous_recovery_waiting_for_settle_ = false;
                              posture_recovery_phase_ =
                                  hugmy::PostureRecoveryPhase::RECOVERY;
                              recovery_response_probe_since_ = ros::Time(0);
                              recovery_wrong_geometry_probe_target_valid_ = false;
                              publishPressureTarget(prepare_pressure_,
                                  std::numeric_limits<double>::infinity());
                              publishCorrectionThrust(prepare_shape_thrust_);
                              enterState(PREPARE, now);
                              ROS_WARN("Recovery re-selection retained pair %zu+%zu: carrying %.2f N without a zero-thrust gap; terminal taper remains active inside %.2f deg",
                                  old_pair == 0 ? 0ul : 2ul,
                                  old_pair == 0 ? 1ul : 3ul,
                                  selected_handoff_thrust,
                                  handoff_drive_stop * 180.0 / M_PI);
                              break;
                            }

                          // The alternative is not the same safe physical
                          // pair. Restore the running candidate state and use
                          // the existing zero-thrust support transfer before
                          // any different pair can be energised.
                          prepare_arm_active_ = old_active;
                          recovery_selected_pair_index_ = old_pair;
                          recovery_selected_pair_release_ = old_release;
                          recovery_selected_pair_pressure_kpa_ = old_pressure;
                          recovery_selected_pressure_candidate_index_ =
                              old_pressure_index;
                          recovery_selected_contact_pivot_enabled_ =
                              old_pivot_enabled;
                          recovery_selected_contact_pivot_ = old_pivot;
                          recovery_selected_pair_mean_depth_rad_ = old_depth;
                          recovery_selected_support_loss_moment_nm_ =
                              old_support_loss;
                          recovery_selected_release_start_pressure_ =
                              old_release_start;
                          recovery_selected_path_requires_zero_thrust_ =
                              old_path_zero;
                          recovery_selected_path_min_lever_m_ = old_path_lever;
                          recovery_extension_mode_latched_ =
                              old_extension_latch;
                          recovery_extension_direction_sign_ =
                              old_extension_direction;
                          recovery_unified_selected_candidate_index_ =
                              old_unified_candidate;
                          recovery_pair_selection_available_ =
                              old_selection_available;
                        }
                      recovery_candidate_reselect_pending_ = true;
                      recovery_handoff_thrust_wrong_way_ = true;
                      recovery_handoff_pair_was_wrong_ = false;
                      recovery_handoff_pressure_trimmed_ = true;
                      recovery_handoff_braking_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      recovery_handoff_progress_reference_rad_ =
                          std::numeric_limits<double>::infinity();
                      recovery_handoff_last_progress_time_ = -1.0;
                      recovery_handoff_candidate_reference_rad_ =
                          std::numeric_limits<double>::infinity();
                      recovery_handoff_candidate_reference_time_ = -1.0;
                      ready_start_ = ros::Time(0);
                      recovery_handoff_target_pressure_ =
                          recoveryCandidateSwitchPressure();
                      ROS_WARN("Circular recovery rejected pair %zu+%zu at %.1f kPa: full-grasp roll %s at %.2f deg under %.2f N; mask=0x%02x. Regrasping before candidate re-selection",
                          recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                          recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                          recovery_selected_pair_pressure_kpa_,
                          handoff_candidate_worsening
                              ? "worsened from its measured reference"
                              : "made no retained progress",
                          roll_error * 180.0 / M_PI,
                          selected_handoff_thrust,
                          recovery_rejected_candidate_mask_);
                    }
                }
              // A quiet residual outside the target after terminal rundown
              // needs another measured pass, not same-pass re-acceleration.
              // Preserve the grasp and ask GRASP to re-evaluate geometry.
              const double handoff_brake_deadlock_sec = std::max(
                  1.0, recovery_handoff_settle_sec_);
              const bool handoff_brake_deadlock = full_grasp_tracking &&
                  recovery_handoff_braking_ && thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate &&
                  recovery_handoff_last_progress_time_ >= 0.0 &&
                  now.toSec() - recovery_handoff_last_progress_time_ >=
                      handoff_brake_deadlock_sec;
              if (handoff_brake_deadlock)
                {
                  recovery_candidate_reselect_pending_ = true;
                  recovery_handoff_pressure_trimmed_ = true;
                  recovery_handoff_integral_n_ = 0.0;
                  ready_start_ = ros::Time(0);
                  ROS_WARN("Action C recovery terminal brake settled at %.2f deg after %.1f s without retained progress; preserving zero thrust and requesting a fresh measured-contact pass",
                           roll_error * 180.0 / M_PI,
                           handoff_brake_deadlock_sec);
                }
              // A contact candidate rejected during pneumatic handoff has
              // already demonstrated either a reversed moment or a loaded
              // no-response.  Once its rotor thrust reaches zero, prescribed
              // pressure equalisation no longer adds safety; it only leaves
              // the body in the disproved geometry. Freeze the measured
              // pressures and immediately re-run signed candidate selection.
              const bool rejected_handoff_candidate_stopped =
                  recovery_candidate_reselect_pending_ &&
                  recovery_handoff_thrust_wrong_way_ && thrust_stopped &&
                  !recovery_handoff_finalizing_regrasp_;
              if (rejected_handoff_candidate_stopped)
                {
                  recovery_handoff_target_pressure_ =
                      recoveryCandidateSwitchPressure();
                  prepare_pressure_ = recovery_handoff_target_pressure_;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  prepare_shape_thrust_.fill(0.0);
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  action_c_resume_gate_.reset();
                  action_c_boundary_settle_gate_.reset();
                  ROS_WARN("Action C rejected handoff candidate reached zero thrust; froze measured pressure [%.1f %.1f %.1f %.1f] kPa for immediate contact-geometry re-selection (mask 0x%02x)",
                           grasp_pressure_[0], grasp_pressure_[1],
                           grasp_pressure_[2], grasp_pressure_[3],
                           recovery_rejected_candidate_mask_);
                  enterState(GRASP, now);
                  break;
                }
              // If the first correction coasted through zero, the selected
              // pair cannot correct the residual error on the other side.
              // Once pneumatic support is symmetric, retain that safe
              // intermediate grasp and immediately make a fresh pair
              // decision.  Previously this condition waited for the 60-s
              // handoff timeout because signed_error made the old pair's
              // target thrust identically zero.
              const double opposite_selection_boundary =
                  recovery_stop_angle +
                  2.0 * roll_recovery_config_.stop_margin;
              // Re-select only after the measured residual has left the
              // stop hysteresis. A rate-only look-ahead was tested here,
              // but contact-model sign errors could then select an
              // opposite pair while the measured posture was still safe.
              const bool crossed_target_after_rundown =
                  !recovery_candidate_reselect_pending_ &&
                  !recovery_handoff_rollback_before_reselect_ &&
                  thrust_stopped &&
                  signed_error <= -opposite_selection_boundary;
              if (crossed_target_after_rundown)
                {
                  const bool measured_best_pressure_available =
                      std::isfinite(recovery_handoff_best_error_rad_);
                  if (measured_best_pressure_available)
                    {
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        recovery_handoff_target_pressure_[arm] =
                            hugmy::recoveryClamp(
                                recovery_handoff_best_pressure_[arm],
                                pressure_search_floor_for(arm),
                                nominal_regrasp_pressure(arm));
                      recovery_handoff_pressure_trimmed_ = true;
                      recovery_handoff_rollback_before_reselect_ = true;
                      recovery_handoff_braking_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      recovery_handoff_brake_error_rad_ = 0.0;
                      ready_start_ = ros::Time(0);
                      ROS_WARN("Action C regrasp crossed the roll target to %.2f deg; restoring best measured pressure [%.1f %.1f %.1f %.1f] kPa before considering another rotor pass",
                               roll_error * 180.0 / M_PI,
                               recovery_handoff_target_pressure_[0],
                               recovery_handoff_target_pressure_[1],
                               recovery_handoff_target_pressure_[2],
                               recovery_handoff_target_pressure_[3]);
                      break;
                    }
                  recovery_handoff_target_pressure_ =
                      recoveryMeasuredContactPressure();
                  // The old pair is already at zero thrust and can no longer
                  // brake motion on the opposite side of the target.  Do not
                  // wait for a prescribed support pressure: on a small
                  // cylinder that delay alone can double the roll error.
                  // Freeze the pressure which is carrying load now, preserve
                  // the opposite anchor, and select a new signed lever from
                  // the current circle/contact geometry immediately.
                  recovery_handoff_pressure_trimmed_ = true;
                  recovery_handoff_thrust_wrong_way_ = true;
                  recovery_handoff_braking_ = true;
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  // The correction direction changed after crossing the
                  // target. Candidate evidence is directional, so begin the
                  // opposite-sign recovery with a fresh candidate set.
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  recovery_candidate_reselect_pending_ = true;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  ready_start_ = ros::Time(0);
                  prepare_pressure_ = recovery_handoff_target_pressure_;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  prepare_shape_thrust_.fill(0.0);
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  action_c_resume_gate_.reset();
                  action_c_boundary_settle_gate_.reset();
                  ROS_WARN("Action C correction crossed the roll target and left the %.2f-deg stop hysteresis (residual %.2f deg); froze measured pressure [%.1f %.1f %.1f %.1f] kPa and requested immediate opposite-sign geometry selection",
                           opposite_selection_boundary * 180.0 / M_PI,
                           roll_error * 180.0 / M_PI,
                           grasp_pressure_[0], grasp_pressure_[1],
                           grasp_pressure_[2], grasp_pressure_[3]);
                  enterState(GRASP, now);
                  break;
                }
              // Re-selection first establishes only a load-bearing contact
              // on the old active pair. If that state is already inside the
              // terminal roll band, do not start another rotor candidate.
              // Instead, restore nominal grasp as a separate monitored
              // pressure phase. A posture loss during this last pressure
              // ramp freezes the measured per-arm pressure and returns to
              // geometry selection; it is never allowed to become an
              // unobserved 15 -> 30 kPa roll impulse.
              const bool retry_contact_settled =
                  recovery_candidate_reselect_pending_ && pressure_ready &&
                  thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate;
              if (retry_contact_settled &&
                  !recovery_handoff_finalizing_regrasp_ &&
                  abs_error <= active_handoff_settle_error)
                {
                  recovery_handoff_finalizing_regrasp_ = true;
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    recovery_handoff_target_pressure_[arm] =
                        nominal_regrasp_pressure(arm);
                  ready_start_ = ros::Time(0);
                  ROS_INFO("Action C load-bearing recovery is inside the roll band at %.2f deg; monitoring final nominal regrasp",
                           roll_error * 180.0 / M_PI);
                  break;
                }
              if (recovery_handoff_finalizing_regrasp_ &&
                  abs_error > active_handoff_settle_error +
                      recovery_handoff_reopen_angle_rad_)
                {
                  recovery_handoff_finalizing_regrasp_ = false;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor = support_floor_for(arm);
                      recovery_handoff_target_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                  ready_start_ = ros::Time(0);
                  ROS_WARN("Action C final regrasp changed roll to %.2f deg; freezing contact-conditioned pressure [%.1f %.1f %.1f %.1f] kPa before re-selection",
                           roll_error * 180.0 / M_PI,
                           recovery_handoff_target_pressure_[0],
                           recovery_handoff_target_pressure_[1],
                           recovery_handoff_target_pressure_[2],
                           recovery_handoff_target_pressure_[3]);
                  break;
                }
              // A measured sign reversal means this contact geometry needs a
              // fresh pair decision.  Finish only the balanced support
              // transfer here; do not label a large roll error as recovered.
              // GRASP will immediately start a new measured-roll pass while
              // retaining the locomotion goal and optimizer trial.
              const bool handoff_retry_ready =
                  (recovery_candidate_reselect_pending_ ||
                   recovery_handoff_thrust_wrong_way_) &&
                  pressure_ready && thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate;
              if (handoff_retry_ready)
                {
                  last_correction_end_ = now;
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  continuous_recovery_retry_opposite_pending_ =
                      !recovery_candidate_reselect_pending_ &&
                      recovery_handoff_pair_was_wrong_;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  recovery_handoff_braking_ = false;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  ROS_WARN("Action C balanced support transfer complete at roll %.2f deg; requesting %s recovery selection (rejected mask 0x%02x)",
                           roll_error * 180.0 / M_PI,
                           recovery_candidate_reselect_pending_
                               ? "contact-conditioned" : "fresh measured-roll",
                           recovery_rejected_candidate_mask_);
                  enterState(GRASP, now);
                  break;
                }
              hugmy::RecoverySupportObservation handoff_support;
              handoff_support.pressure = measured_pressure_;
              handoff_support.target_pressure =
                  recovery_handoff_target_pressure_;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                handoff_support.bend[arm] = measuredArmBendAngle(arm);
              handoff_support.roll = roll_;
              handoff_support.pitch = pitch_;
              handoff_support.roll_rate = recovery_roll_rate;
              handoff_support.pitch_rate = body_angular_velocity_[1];
              handoff_support.maximum_thrust = *std::max_element(
                  prepare_shape_thrust_.begin(),
                  prepare_shape_thrust_.end());
              handoff_support.sensors_valid = handoff_inputs_valid;
              hugmy::RecoverySupportConfig handoff_support_config;
              handoff_support_config.pressure_tolerance =
                  recovery_safe_transfer_pressure_tolerance_kpa_;
              handoff_support_config.minimum_pressure =
                  recovery_handoff_min_pressure_kpa_;
              handoff_support_config.maximum_bend_change =
                  roll_recovery_config_.stable_bend;
              handoff_support_config.maximum_rate =
                  roll_recovery_config_.settle_rate;
              handoff_support_config.maximum_attitude_drift =
                  std::max(roll_recovery_config_.stop_margin,
                           0.5 * M_PI / 180.0);
              handoff_support_config.settle_sec =
                  recovery_handoff_settle_sec_;
              handoff_support_config.minimum_loaded_arms = 3;
              const bool physical_support_verified =
                  recovery_support_monitor_.update(
                      handoff_support, now.toSec(), handoff_support_config,
                      true);
              const bool handoff_settled = pressure_ready && thrust_stopped &&
                  std::abs(recovery_roll_rate) <= roll_recovery_config_.settle_rate &&
                  abs_error <= active_handoff_settle_error &&
                  physical_support_verified;
              if (handoff_settled)
                {
                  posture_recovery_phase_ = hugmy::PostureRecoveryPhase::VERIFY;
                  if (ready_start_.isZero()) ready_start_ = now;
                  if ((now - ready_start_).toSec() >=
                      recovery_handoff_settle_sec_)
                    {
                      last_correction_end_ = now;
                      continuous_recovery_started_ = false;
                      recovery_rejected_candidate_mask_ = 0;
                      recovery_rejected_direction_sign_ = 0;
                      recovery_candidate_reselect_pending_ = false;
                      recovery_geometry_reestimate_attempted_ = false;
                      recovery_geometry_reestimate_pending_ = false;
                      recovery_circle_event_locked_ = false;
                      recovery_event_contact_pivot_locked_ = false;
                      recovery_selected_contact_pivot_enabled_ = false;
                      recovery_preferred_pair_locked_ = false;
                      recovery_extension_mode_latched_ = false;
                      recovery_extension_direction_sign_ = 0;
                      recovery_unified_selected_candidate_index_ = -1;
                      recovery_unified_unlock_since_ = ros::Time(0);
                      recovery_unified_unlock_ready_ = false;
                      recovery_rate_arrest_active_ = false;
                      recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
                      recovery_rate_arrest_reference_error_rad_ =
                          std::numeric_limits<double>::quiet_NaN();
                      recovery_event_active_ = false;
                      recovery_fault_reason_ =
                          hugmy::RecoveryFaultReason::NONE;
                      // This branch is reached only after the final pressure,
                      // zero-thrust, low-rate and attitude dwell have all
                      // succeeded.  Do not carry the stricter trigger which
                      // originally requested recovery back into GRASP: the
                      // 2026-10-29 run completed at 4.66 deg, then immediately
                      // restarted because the stale 3-deg trigger remained
                      // armed.  GRASP may still start a new independent
                      // recovery at the ordinary 10-deg/hard-roll boundary.
                      continuous_recovery_waiting_for_settle_ = false;
                      action_c_requested_recovery_trigger_rad_ =
                          std::numeric_limits<double>::quiet_NaN();
                      recovery_handoff_integral_n_ = 0.0;
                      recovery_handoff_brake_error_rad_ = 0.0;
                      recovery_handoff_braking_ = false;
                      // Preserve the measured contact equilibrium through the
                      // deflated endpoint and the automatically resumed gait.
                      // A new operator command resets it to nominal pressure.
                      grasp_pressure_ = recovery_handoff_target_pressure_;
                      recovery_contact_grasp_active_ = true;
                      if (hybridLocomotionActive() && action_c_resume_pending_)
                        {
                          hybrid_grasp_pressure_ = grasp_pressure_;
                          hybrid_hold_pressure_ = true;
                        }
                      ROS_INFO("Action C recovery handoff complete: contact-conditioned grasp [%.1f %.1f %.1f %.1f] kPa with roll %.2f deg and rate %.3f rad/s",
                               grasp_pressure_[0], grasp_pressure_[1],
                               grasp_pressure_[2], grasp_pressure_[3],
                               roll_error * 180.0 / M_PI,
                               recovery_roll_rate);
                      enterState(GRASP, now);
                      break;
                    }
                }
              else
                ready_start_ = ros::Time(0);

              // Do not reopen a partly regrasped pair merely because the
              // normal 10-deg recovery threshold is crossed.  In MuJoCo this
              // 26 -> 0 kPa discontinuity, rather than the residual roll,
              // caused the physical fall.  Pressure is already frozen above
              // the acceptance band; retain the current contact geometry and
              // let the bounded rate PI bring roll back before continuing.
              if ((now - state_start_).toSec() >=
                  recovery_handoff_timeout_sec_)
                {
                  continuous_recovery_waiting_for_settle_ = true;
                  continuous_recovery_started_ = false;
                  resetRecoveryHandoffState();
                  prepare_shape_thrust_.fill(0.0);
                  prepare_pressure_ = grasp_pressure_;
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  enterState(GRASP, now);
                  ROS_WARN("Action C recovery handoff timed out after %.1f s; safe grasp commanded and automatic measured-roll retry retained",
                           recovery_handoff_timeout_sec_);
                  break;
                }
              ROS_INFO_THROTTLE(0.5,
                  "Action C closed-loop regrasp: roll %.2f deg rate %.3f rad/s, pressure [%.1f %.1f %.1f %.1f] kPa, pair thrust %.2f N, support_equalized=%d pressure_advance=%d ready=%d",
                  roll_error * 180.0 / M_PI, recovery_roll_rate,
                  prepare_pressure_[0], prepare_pressure_[1],
                  prepare_pressure_[2], prepare_pressure_[3],
                  handoff_target, support_transfer_equalized,
                  pressure_advance_allowed, pressure_ready);
              break;
            }

          // Restore contact first while removing rotor support gradually.
          // Sensor/IMU faults and angular-rate violations remain hard stops.
          if (!body_imu_fresh || !pressure_valid || !neurons_fresh ||
              (continuous_c_recovery && (!std::isfinite(recovery_roll_rate) ||
               std::abs(body_angular_velocity_[0]) >= correction_max_angular_rate_)) ||
              angular_rate >= correction_max_angular_rate_)
            prepare_shape_thrust_.fill(0.0);
          else
            for (double& thrust : prepare_shape_thrust_)
              thrust = std::max(0.0,
                  thrust - correction_thrust_down_rate_ * dt);
          publishCorrectionThrust(prepare_shape_thrust_);
          // Recovery must command the grasp pressure immediately. Applying
          // the PREPARE slew limit here can leave the old high-pressure
          // target active for several seconds even though thrust has stopped.
          publishPressureTarget(grasp_pressure_,
                                std::numeric_limits<double>::infinity());
          {
            bool recovered = pressure_valid;
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              recovered = recovered && std::abs(measured_pressure_[arm] - grasp_pressure_[arm]) <=
                                       pressure_tolerance_kpa_;
            if (continuous_c_recovery)
              recovered = pressure_valid && actionCPressureReady(grasp_pressure_, now);
            const bool thrust_stopped = std::all_of(
                prepare_shape_thrust_.begin(), prepare_shape_thrust_.end(),
                [](double thrust) { return thrust <= 0.01; });
            if (recovered && thrust_stopped)
              {
                last_correction_end_ = now;
                enterState(GRASP, now);
              }
            else if ((now - state_start_).toSec() >= recover_timeout_)
              ROS_WARN_THROTTLE(1.0,
                  "RECOVER waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                  grasp_pressure_[0], pressure_tolerance_kpa_);
          }
          break;
        }
      return;
}

void AttitudePressureController::selectContinuousRecoveryPair(
    double roll_error)
{
  using Pair = std::array<size_t, 2>;
  const std::array<Pair, 2> pairs{{Pair{{0, 1}}, Pair{{2, 3}}}};
  struct Candidate {
    bool valid = false;
    bool rejected = false;
    size_t pair = 0;
    size_t pressure_index = 0;
    bool release = true;
    double pressure = 0.0;
    double release_fraction = 1.0;
    double lever = 0.0;
    double support_loss_moment = 0.0;
    double required_thrust = std::numeric_limits<double>::infinity();
    double depth = std::numeric_limits<double>::quiet_NaN();
    double score = std::numeric_limits<double>::infinity();
    Vector3 pivot{{0.0, 0.0, 0.0}};
  };

  recovery_selected_path_requires_zero_thrust_ = false;
  recovery_selected_path_min_lever_m_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_since_ = ros::Time(0);
  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
  recovery_wrong_geometry_reference_error_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_reference_rate_rad_s_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_best_rate_rad_s_ =
      -std::numeric_limits<double>::infinity();
  recovery_wrong_geometry_response_observed_ = false;
  recovery_wrong_geometry_probe_target_valid_ = false;
  recovery_conditional_grasp_probe_active_ = false;
  recovery_conditional_grasp_response_observed_ = false;
  recovery_conditional_grasp_probe_start_ = ros::Time(0);
  recovery_conditional_grasp_probe_start_error_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_selected_required_thrust_n_ =
      std::numeric_limits<double>::quiet_NaN();

  // A new event is the only time a world-fixed cylinder centre can be
  // latched.  Never reuse the previous event's absolute centre.
  if (!recovery_event_contact_pivot_locked_)
    recovery_circle_center_world_locked_ = false;

  const bool circle_model_available = recovery_circle_estimate_.valid &&
      recovery_circle_estimate_.confidence >= recovery_circle_min_confidence_ &&
      !recovery_circle_stamp_.isZero();
  const bool circle_model_fresh = circle_model_available &&
      (ros::Time::now() - recovery_circle_stamp_).toSec() <=
          recovery_circle_max_age_sec_;
  if (circle_model_available && !recovery_circle_event_locked_)
    {
      recovery_circle_event_estimate_ = recovery_circle_estimate_;
      recovery_circle_event_locked_ = true;
      // The recovery may have started before four reliable contact samples
      // existed and therefore locked the fixed fallback pivot.  When the
      // first trusted circle becomes available at a monitored regrasp
      // boundary, promote the model and its centre atomically.  Keeping the
      // old fallback centre while using the new circle radius reverses the
      // predicted small-angle moment sign (observed in rolling_5 at +6 deg).
      // This is a one-shot upgrade: later bend/contact transients cannot move
      // the pivot because recovery_circle_event_locked_ is now true.
      if (recovery_event_contact_pivot_locked_ &&
          std::isfinite(recovery_circle_event_estimate_.center_y) &&
          std::isfinite(recovery_circle_event_estimate_.center_z))
        {
          const double previous_y = recovery_event_contact_pivot_[1];
          const double previous_z = recovery_event_contact_pivot_[2];
          recovery_event_contact_pivot_[1] =
              recovery_circle_event_estimate_.center_y;
          recovery_event_contact_pivot_[2] =
              recovery_circle_event_estimate_.center_z;
          ROS_INFO("Recovery geometry pivot upgraded with first trusted circle: yz=[%.1f %.1f] -> [%.1f %.1f] mm; freezing the new centre for this event",
                   1000.0 * previous_y, 1000.0 * previous_z,
                   1000.0 * recovery_event_contact_pivot_[1],
                   1000.0 * recovery_event_contact_pivot_[2]);
        }
      if (!circle_model_fresh)
        ROS_INFO("Recovery reuses last trusted circle model: radius %.1f mm, confidence %.2f, age %.1f s; the event keeps this circle while rotor positions follow measured bend",
                 1000.0 * recovery_circle_event_estimate_.radius,
                 recovery_circle_event_estimate_.confidence,
                 (ros::Time::now() - recovery_circle_stamp_).toSec());
    }
  // Freeze the representative object centre before changing any chamber
  // pressure.  A fitted circular centre is a geometric quantity; using the
  // commanded pressure as a contact-force weight made the centre move merely
  // because the controller requested exhaust.  If no reliable fit exists at
  // event entry, retain the configured fixed pivot until the first trusted
  // circle is acquired at a monitored contact-reselection boundary.
  if (!recovery_event_contact_pivot_locked_)
    {
      recovery_event_contact_pivot_ = contact_position_;
      if (recovery_circle_event_locked_ &&
          std::isfinite(recovery_circle_event_estimate_.center_y) &&
          std::isfinite(recovery_circle_event_estimate_.center_z))
        {
          recovery_event_contact_pivot_[1] =
              recovery_circle_event_estimate_.center_y;
          recovery_event_contact_pivot_[2] =
              recovery_circle_event_estimate_.center_z;
        }
      recovery_event_contact_pivot_locked_ = true;
      ROS_INFO("Recovery geometry pivot locked at yz=[%.1f %.1f] mm (%s); pressure changes cannot move it during this event",
               1000.0 * recovery_event_contact_pivot_[1],
               1000.0 * recovery_event_contact_pivot_[2],
               recovery_circle_event_locked_ ? "stable circle fit" : "fixed fallback");
    }
  // The object geometry does not change inside one recovery event.  A
  // partial regrasp may make a live four-contact fit ill-conditioned, so use
  // the stable entry snapshot until a balanced regrasp explicitly starts a
  // new event.
  const bool circle_fresh = recovery_circle_event_locked_ &&
      recovery_circle_event_estimate_.valid &&
      recovery_circle_event_estimate_.confidence >=
          recovery_circle_min_confidence_;
  const hugmy::CircularGraspEstimate& event_circle =
      recovery_circle_event_estimate_;
  static_cast<void>(event_circle);  // ROS logging is compiled out in unit stubs.
  // Object-relative arm position for recovery means the rotor force
  // application point, not an extrapolated distal-arm contact.  The rotor is
  // mounted between joints 3 and 4, so its position is fully determined by
  // the measured proximal three-joint bend and the URDF rotor offset.
  std::array<double, ARM_COUNT> recovery_depth_angle{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  if (circle_fresh)
    {
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        recovery_depth_angle[arm] = rotorDepthAngleAbout(
            arm, measuredArmBendAngle(arm), recovery_event_contact_pivot_);
    }
  // The physical target is zero roll.  The +/-stop band is an acceptance
  // tolerance, not an offset in the desired moment.  Subtracting it here
  // made both candidate selection and model feed-forward collapse before the
  // body had actually returned to level.
  const double fixed_mapping_sign = correction_pair_reverse_spinal_mapping_
      ? -1.0 : 1.0;
  const double correction_sign = roll_error >= 0.0 ? 1.0 : -1.0;
  const int correction_direction = roll_error >= 0.0 ? 1 : -1;
  const double measured_roll_rate = hugmy::recoveryRollRate(
      roll_, pitch_, body_angular_velocity_);
  // Preserve the attitude-error direction and use angular rate only to
  // modify the required magnitude. A correcting rate reduces the request;
  // an outward rate increases it without reversing the selected rotor pair.
  const double correcting_roll_rate = std::isfinite(measured_roll_rate)
      ? correction_sign * measured_roll_rate : 0.0;
  const double required_moment = std::max(
      0.0, roll_moment_kp_ * std::abs(roll_error) -
               roll_moment_kd_ * correcting_roll_rate);
  const double predicted_roll_error = std::isfinite(measured_roll_rate)
      ? roll_error - recovery_extension_prediction_sec_ * measured_roll_rate
      : roll_error;
  const bool outward_roll_rate = std::isfinite(measured_roll_rate) &&
      roll_error * measured_roll_rate < 0.0;
  const bool materially_worsening_roll = outward_roll_rate &&
      std::abs(measured_roll_rate) >=
          recovery_grasp_worsening_rate_rad_s_;
  const bool predicted_emergency =
      recovery_extension_emergency_confirmed_ &&
      materially_worsening_roll &&
      std::abs(predicted_roll_error) >=
          recovery_extension_emergency_error_rad_;
  // Pair/mode selection is discrete: retain it throughout one measured
  // correction pass rather than recomputing from every transient bend sample.
  // At a new PREPARE boundary, the unified selector may clear both locks only
  // after GRASP has held fresh pressure, zero command and a quiet small roll
  // for the configured dwell.  It then compares all four target modes again.
  // The legacy selector retains its original extension-only unlock rule.
  const bool stable_small_angle_reselection =
      std::isfinite(measured_roll_rate) &&
      (recovery_unified_active_enabled_
          ? (recovery_preferred_pair_locked_ &&
             recovery_unified_unlock_ready_ &&
             std::abs(roll_error) < recovery_unified_unlock_angle_rad_ &&
             std::abs(predicted_roll_error) <
                 recovery_unified_unlock_angle_rad_ &&
             std::abs(measured_roll_rate) <
                 recovery_unified_unlock_rate_rad_s_)
          : (recovery_extension_mode_latched_ &&
             std::abs(roll_error) <
                 recovery_circle_flexed_max_error_rad_ &&
             std::abs(predicted_roll_error) <
                 recovery_circle_flexed_max_error_rad_ &&
             (!outward_roll_rate ||
              std::abs(measured_roll_rate) <
                  correction_motion_start_rate_)));
  if (stable_small_angle_reselection)
    {
      if (recovery_unified_active_enabled_)
        ROS_INFO("Stable small-angle roll recovery: clearing mode direction-%+d and physical-pair locks after %.2f-s supported dwell; re-evaluating all four target modes",
                 recovery_extension_direction_sign_,
                 recovery_unified_unlock_hold_sec_);
      else
        ROS_INFO("Stable small-angle roll recovery: clearing the old extended-arm direction-%+d latch and physical-pair lock; re-evaluating legacy current-bend candidates",
                 recovery_extension_direction_sign_);
      recovery_extension_mode_latched_ = false;
      recovery_extension_direction_sign_ = 0;
      recovery_preferred_pair_locked_ = false;
      recovery_unified_unlock_since_ = ros::Time(0);
      recovery_unified_unlock_ready_ = false;
      action_c_recovery_urgent_thrust_rise_ = false;
      recovery_rate_arrest_active_ = false;
      recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
      recovery_rate_arrest_reference_error_rad_ =
          std::numeric_limits<double>::quiet_NaN();
      continuous_recovery_use_opposite_pair_ = false;
    }
  // A large correction may legitimately latch the straight-arm topology.
  // If the body crosses zero while it is still moving outward too quickly to
  // satisfy the stable-small-angle test above, the old one-directional pair
  // cannot arrest the new sign.  Only that dynamic case requires the other
  // physical pair immediately; a quiet residual is handled by the all-pair,
  // current-bend re-evaluation above.
  const bool crossed_from_latched_extension =
      recovery_extension_mode_latched_ &&
      recovery_extension_direction_sign_ != 0 &&
      recovery_extension_direction_sign_ != correction_direction &&
      std::abs(roll_error) < recovery_circle_flexed_max_error_rad_;
  if (crossed_from_latched_extension)
    {
      const size_t previous_pair = recovery_preferred_pair_index_;
      ROS_INFO("Roll recovery crossed target with an unstable opposite residual: clearing old direction-%+d extension latch and requiring the opposite physical pair %zu+%zu",
               recovery_extension_direction_sign_,
               previous_pair == 0 ? 2ul : 0ul,
               previous_pair == 0 ? 3ul : 1ul);
      recovery_extension_mode_latched_ = false;
      recovery_extension_direction_sign_ = 0;
      action_c_recovery_urgent_thrust_rise_ = false;
      recovery_rate_arrest_active_ = false;
      recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
      recovery_rate_arrest_reference_error_rad_ =
          std::numeric_limits<double>::quiet_NaN();
      // A flexed version of the same pair may have a modeled opposite lever,
      // but hardware must first unload/reload that contact before the sign is
      // physically observable. Reusing it immediately produced no visible
      // reversal after a target crossing. Select the other physical pair;
      // its grasped or straight topology is still chosen by the same signed
      // moment calculation below.
      recovery_preferred_pair_index_ = 1 - previous_pair;
      recovery_preferred_pair_locked_ = true;
      continuous_recovery_use_opposite_pair_ = false;
    }
  // Object radius is not a contact-topology classifier.  A large but quiet
  // error may retain a grasp at any radius when its measured rotor position
  // and thrust vector produce a restoring signed moment.  Only a materially
  // outward rate may force the released/straight topology before the normal
  // four-candidate comparison.
  const bool emergency_extension_requested =
      recovery_extension_mode_latched_ ||
      predicted_emergency;
  if (emergency_extension_requested)
    {
      if (!recovery_extension_mode_latched_)
        ROS_WARN("Roll recovery latching extended-arm mode: error %.1f deg, rate %.1f deg/s, %.2f-s prediction %.1f deg",
                 roll_error * 180.0 / M_PI,
                 std::isfinite(measured_roll_rate)
                     ? measured_roll_rate * 180.0 / M_PI : 0.0,
                 recovery_extension_prediction_sec_,
                 predicted_roll_error * 180.0 / M_PI);
      // Urgent thrust slew and contact topology are independent decisions.
      // Do not latch the straight-arm topology until the candidate search
      // has actually selected a released pair.  In particular, a small roll
      // error whose outward rate has already stopped may retain a valid,
      // load-bearing current-bend pair even if locomotion requested a fast
      // initial thrust rise.
      // A large residual needs the bounded 2--4 N breakaway search even when
      // the entry gyro sample happens to point inward. Contact rebound made
      // that sample alternate in the real bag and otherwise left a 24--46
      // deg correction behind the 1.5-N geometry probe ceiling. Release
      // rate-arrest only after sustained inward rate plus retained angle
      // improvement, as checked by the PREPARE state.
      if (predicted_emergency)
        {
          recovery_rate_arrest_active_ = true;
          recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
          recovery_rate_arrest_reference_error_rad_ =
              std::numeric_limits<double>::quiet_NaN();
        }
    }
  // For a small residual, do not impose the emergency 2-N floor, but allow
  // the ordinary PI request to reach the actuator quickly when the body is
  // already rolling away. Its magnitude remains governed by the model and
  // rate feedback rather than by this slew-only flag.
  if (outward_roll_rate && std::isfinite(measured_roll_rate) &&
      std::abs(measured_roll_rate) >=
          0.5 * correction_motion_start_rate_)
    action_c_recovery_urgent_thrust_rise_ = true;
  // Candidate rejection is evidence for one requested moment direction.
  // Regrasp can move the contacts across the cylinder side and reverse both
  // the roll error and the effective rotor moment. Carrying the old mask into
  // that new geometry can reject its only restoring candidate. Clear it only
  // after the required moment sign changes; same-direction retries still
  // cannot repeat a disproved pair/contact state.
  if (recovery_rejected_candidate_mask_ != 0 &&
      recovery_rejected_direction_sign_ != 0 &&
      recovery_rejected_direction_sign_ != correction_direction)
    {
      ROS_INFO("Circular recovery required-moment sign changed after regrasp; clearing directional rejection mask 0x%02x",
               recovery_rejected_candidate_mask_);
      recovery_rejected_candidate_mask_ = 0;
      recovery_rejected_direction_sign_ = 0;
    }
  const auto nominal_pressure = [&](size_t arm) {
    return hugmy::recoveryClamp(
        std::max(base_pressure_kpa_, grasp_pressure_[arm]),
        minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
  };
  const auto selected_pressure = [&](size_t arm, double requested) {
    return hugmy::recoveryClamp(requested,
        minimum_prepare_pressure_kpa_, nominal_pressure(arm));
  };
  const size_t grasp_pressure_index =
      recovery_circle_pressure_candidates_kpa_.size() - 1;
  auto evaluate = [&](size_t pair_index, size_t pressure_index) {
    Candidate candidate;
    candidate.pair = pair_index;
    candidate.pressure_index = pressure_index;
    const bool grasped_candidate = pressure_index == grasp_pressure_index;
    candidate.pressure = recovery_circle_pressure_candidates_kpa_[pressure_index];
    double release_fraction_sum = 0.0;
    double selected_pressure_sum = 0.0;
    for (const size_t arm : pairs[pair_index])
      {
        const double nominal = nominal_pressure(arm);
        const double target = grasped_candidate
            ? nominal : selected_pressure(arm, candidate.pressure);
        selected_pressure_sum += target;
        release_fraction_sum += (nominal - target) /
            std::max(1.0e-6, nominal - minimum_prepare_pressure_kpa_);
      }
    candidate.release_fraction = hugmy::recoveryClamp(
        release_fraction_sum / static_cast<double>(pairs[pair_index].size()),
        0.0, 1.0);
    // The last entry is a topology, not merely a 30-kPa numerical sample:
    // it means keep the currently load-bearing bend at each arm's actual
    // nominal grasp pressure.  Otherwise a 40-kPa grasp made the nominal
    // "grasped" candidate look partially released and evaluate alpha=0.
    candidate.release = !grasped_candidate &&
        candidate.release_fraction > 1.0e-3;
    if (grasped_candidate)
      candidate.pressure = selected_pressure_sum /
          static_cast<double>(pairs[pair_index].size());
    const uint8_t candidate_bit = static_cast<uint8_t>(
        1u << (4u * pair_index + pressure_index));
    candidate.rejected =
        (recovery_rejected_candidate_mask_ & candidate_bit) != 0;
    std::array<bool, ARM_COUNT> selected_pair{{false, false, false, false}};
    for (const size_t arm : pairs[pair_index]) selected_pair[arm] = true;
    // The two modes use explicit, interpretable geometry. A grasped
    // candidate keeps the current measured bend. A released candidate is
    // evaluated at the straight-arm endpoint (alpha=0); measured bend closes
    // the loop while the pneumatic system travels toward that endpoint.
    PressureArray candidate_pressure = measured_pressure_;
    PressureArray candidate_bend{};
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      candidate_bend[arm] = measuredArmBendAngle(arm);
    for (const size_t arm : pairs[pair_index])
      {
        candidate_pressure[arm] = grasped_candidate
            ? nominal_pressure(arm)
            : selected_pressure(arm, candidate.pressure);
        if (candidate.release) candidate_bend[arm] = 0.0;
      }
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      if (!selected_pair[arm]) candidate_pressure[arm] = nominal_pressure(arm);
    candidate.pivot = recoveryContactPivot(
        selected_pair, candidate.release, &candidate_pressure, &candidate_bend);
    double minimum_selected_pressure = std::numeric_limits<double>::infinity();
    for (const size_t arm : pairs[pair_index])
      minimum_selected_pressure = std::min(
          minimum_selected_pressure, candidate_pressure[arm]);
    const bool selected_contact_retained = minimum_selected_pressure >=
        recovery_circle_contact_pressure_kpa_;
    // Releasing the geometrically lower pair also removes part of the
    // load-bearing contact reaction.  The rotor cross-product alone therefore
    // over-estimates the useful restoring moment and selected ~0.1-N commands
    // in the 2026-10-28 hardware trial.  Pressure is not a force sensor, but
    // its normalized removed share provides a bounded first-order estimate of
    // this adverse support-transfer moment.  Only the component opposing the
    // requested correction is charged to the candidate.
    if (candidate.release && recovery_circle_release_support_scale_ > 0.0)
      {
        double pressure_sum = 0.0;
        for (size_t arm = 0; arm < ARM_COUNT; ++arm)
          if (std::isfinite(measured_pressure_[arm]))
            pressure_sum += std::max(0.0, measured_pressure_[arm]);
        if (pressure_sum > 1.0e-6)
          {
            double projected_removed_support_moment = 0.0;
            for (const size_t arm : pairs[pair_index])
              {
                const double bend = measuredArmBendAngle(arm);
                if (!std::isfinite(bend) ||
                    !std::isfinite(measured_pressure_[arm]))
                  continue;
                const double release_target =
                    selected_pressure(arm, candidate.pressure);
                const double removed_share = std::max(
                    0.0, measured_pressure_[arm] - release_target) /
                    pressure_sum;
                // The exact distal contact is unobserved.  Use the known
                // rotor application point as a bounded support-transfer
                // proxy rather than extending the measured three-joint bend
                // through unmeasured joints 4--5.  The term is deliberately
                // small; the explicit release penalty carries the remaining
                // uncertainty.
                const Vector3 contact = rotorPositionFromAngle(arm, bend);
                Vector3 lever, removed_force;
                for (size_t axis = 0; axis < 3; ++axis)
                  {
                    lever[axis] = contact[axis] - candidate.pivot[axis];
                    // Loss of an upward support reaction is equivalent to a
                    // bounded added force in the measured gravity direction.
                    removed_force[axis] = mass_kg_ * gravity_body_[axis] *
                        removed_share * recovery_circle_release_support_scale_;
                  }
                projected_removed_support_moment += correction_sign *
                    cross(lever, removed_force)[0];
              }
            candidate.support_loss_moment = std::max(
                0.0, -projected_removed_support_moment);
          }
      }
    double depth_sum = 0.0;
    size_t depth_count = 0;
    for (const size_t arm : pairs[pair_index])
      {
        const double measured_bend = measuredArmBendAngle(arm);
        if (!std::isfinite(measured_bend)) return candidate;
        const double evaluated_bend = candidate_bend[arm];
        // A released arm uses the empirically verified actuator-axis mapping:
        // once straightened, it no longer has a loaded contact reaction that
        // can redirect its rotor force.  A flexed/grasped arm is different;
        // its contact location on the inferred circle determines the moment
        // sign, so applying the fixed mapping again inverted deep (>90-deg)
        // contacts.  Keep these two physical cases separate.
        // A loaded arm can use its physical contact-dependent sign only when
        // a reliable object centre exists.  With no circle estimate, both
        // grasped and released candidates must use the verified Spinal/URDF
        // actuator mapping; otherwise the fixed-pivot planner can disagree
        // with the measured-angle allocator and select the opposite pair.
        const double actuator_sign =
            circle_fresh && selected_contact_retained
                ? 1.0 : fixed_mapping_sign;
        candidate.lever += actuator_sign * correction_sign *
            recoveryRotorRawLeverAbout(
                arm, evaluated_bend, candidate.pivot);
        if (circle_fresh && std::isfinite(recovery_depth_angle[arm]))
          {
            depth_sum += recovery_depth_angle[arm];
            ++depth_count;
          }
      }
    candidate.depth = depth_count > 0
        ? depth_sum / static_cast<double>(depth_count)
        : std::numeric_limits<double>::quiet_NaN();
    // Depth is retained for diagnostics. Feasibility is determined by the
    // signed moment itself, which covers deep human-arm grasps and shallow
    // rigid fixtures without separate object-specific pair rules.
    const double usable_lever = std::max(
        recovery_circle_min_signed_lever_m_, roll_recovery_config_.min_lever);
    if (!std::isfinite(candidate.lever) ||
        candidate.lever < usable_lever) return candidate;
    candidate.required_thrust =
        (required_moment + candidate.support_loss_moment) /
        std::max(candidate.lever, usable_lever);
    candidate.score = candidate.required_thrust +
        recovery_circle_release_penalty_n_ * candidate.release_fraction;
    candidate.valid = !candidate.rejected &&
        std::isfinite(candidate.score);
    return candidate;
  };

  // The standalone planner evaluates the same four physical modes and is used
  // only by the guarded active path. No parallel contact-model shadow path is
  // run. Pressure settling, response rejection and SAFE_TRANSFER remain in
  // the existing state machine below.
  hugmy::recovery::Selection unified_plan;
  if (recovery_unified_active_enabled_ && circle_fresh)
    {
      Vector3 body_center = recovery_event_contact_pivot_;
      const bool mocap_fresh = recovery_mocap_pose_valid_ &&
          !mocap_stamp_.isZero() &&
          (ros::Time::now() - mocap_stamp_).toSec() <= 0.25;
      bool geometry_pressure_stable = true;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        geometry_pressure_stable = geometry_pressure_stable &&
            std::isfinite(measured_pressure_[arm]) &&
            std::isfinite(last_pressure_[arm]) &&
            std::abs(measured_pressure_[arm] - last_pressure_[arm]) <=
                pressure_tolerance_kpa_;
      const bool geometry_rate_stable = std::isfinite(measured_roll_rate) &&
          std::abs(measured_roll_rate) <=
              recovery_geometry_update_max_roll_rate_rad_s_;
      const bool geometry_update_allowed = mocap_fresh &&
          geometry_pressure_stable && geometry_rate_stable;
      if (geometry_update_allowed)
        {
          if (!recovery_circle_center_world_locked_)
            {
              recovery_circle_center_world_ =
                  hugmy::recovery::objectCenterWorld(body_center,
                      recovery_mocap_pose_);
              recovery_circle_center_world_locked_ = true;
            }
          const Vector3 transformed_center =
              hugmy::recovery::objectCenterBody(
              recovery_circle_center_world_, recovery_mocap_pose_);
          const double centre_shift = std::hypot(
              transformed_center[1] - recovery_event_contact_pivot_[1],
              transformed_center[2] - recovery_event_contact_pivot_[2]);
          if (hugmy::recovery::finite(transformed_center) &&
              centre_shift <=
                  recovery_geometry_max_support_rotor_distance_m_)
            body_center = transformed_center;
          else
            ROS_WARN("Unified recovery rejected implausible transformed cylinder centre (shift %.3f m); using the stable event-entry geometry",
                     centre_shift);
        }
      else
        ROS_WARN("Unified recovery holds event-entry cylinder geometry: mocap=%d pressure_stable=%d roll_rate=%.1f/%.1f deg/s",
                 mocap_fresh ? 1 : 0, geometry_pressure_stable ? 1 : 0,
                 std::isfinite(measured_roll_rate)
                     ? measured_roll_rate * 180.0 / M_PI : 0.0,
                 recovery_geometry_update_max_roll_rate_rad_s_ *
                     180.0 / M_PI);
      unified_plan = evaluateUnifiedRecoveryCandidates(roll_error,
          required_moment * correction_sign, body_center,
          recovery_unified_selected_candidate_index_);
    }
  bool unified_support_available = false;
  if (circle_fresh)
    for (const auto& candidate : unified_plan.candidates)
      unified_support_available = unified_support_available ||
          (candidate.contact_feasible &&
           hugmy::recovery::finite(candidate.support_body));

  recovery_pair_selection_available_ = false;
  Candidate selected;
  if (recovery_unified_active_enabled_ && circle_fresh)
    {
      const size_t extension_pressure_index = 0;
      const auto allowed = [&](size_t index, int forced_pair) {
        const size_t pair = index / 2;
        const bool release = index % 2 == 1;
        const size_t pressure_index = release
            ? extension_pressure_index : grasp_pressure_index;
        const uint8_t candidate_bit = static_cast<uint8_t>(
            1u << (4u * pair + pressure_index));
        return index < unified_plan.alternatives.size() &&
            unified_plan.alternatives[index].feasible &&
            (!emergency_extension_requested || release) &&
            (!recovery_preferred_pair_locked_ ||
             pair == recovery_preferred_pair_index_) &&
            (forced_pair < 0 || pair == static_cast<size_t>(forced_pair)) &&
            (recovery_rejected_candidate_mask_ & candidate_bit) == 0;
      };
      const auto best_index = [&](int forced_pair) {
        size_t best = unified_plan.alternatives.size();
        const auto priority = [&](size_t index) {
          if (!unified_plan.alternatives[index].feasible) return 3;
          if (index % 2 == 0 &&
              !unified_plan.alternatives[index].response_probe_required)
            return 0;
          if (unified_plan.alternatives[index].response_probe_required)
            return 1;
          return 2;
        };
        for (size_t index = 0; index < unified_plan.alternatives.size(); ++index)
          if (allowed(index, forced_pair) &&
              (best == unified_plan.alternatives.size() ||
               priority(index) < priority(best) ||
               (priority(index) == priority(best) &&
                unified_plan.alternatives[index].score <
                    unified_plan.alternatives[best].score)))
            best = index;
        return best;
      };

      size_t chosen = best_index(-1);
      if (chosen < unified_plan.alternatives.size() &&
          continuous_recovery_use_opposite_pair_ &&
          !recovery_preferred_pair_locked_)
        {
          const int opposite = 1 - static_cast<int>(chosen / 2);
          const size_t alternate = best_index(opposite);
          if (alternate < unified_plan.alternatives.size())
            {
              chosen = alternate;
              ROS_WARN("Unified active recovery uses the opposite physical pair after a measured wrong-way response");
            }
        }

      if (chosen < unified_plan.alternatives.size())
        {
          const size_t pair = chosen / 2;
          const bool release = chosen % 2 == 1;
          const size_t pressure_index = release
              ? extension_pressure_index : grasp_pressure_index;
          // Reuse the legacy candidate container so every downstream pressure,
          // response and SAFE_TRANSFER guard remains unchanged.  Replace only
          // the quantities owned by the unified support-point/arm-shape model.
          selected = evaluate(pair, pressure_index);
          selected.valid = true;
          selected.rejected = false;
          selected.pair = pair;
          selected.pressure_index = pressure_index;
          selected.release = release;
          selected.release_fraction = release ? 1.0 : 0.0;
          selected.pressure = release
              ? recovery_release_pressure_kpa_
              : 0.5 * (nominal_pressure(pairs[pair][0]) +
                       nominal_pressure(pairs[pair][1]));
          selected.pivot =
              unified_plan.alternatives[chosen].worst_support_body;
          selected.lever = unified_plan.alternatives[chosen].signed_lever;
          selected.support_loss_moment = 0.0;
          selected.required_thrust =
              unified_plan.alternatives[chosen].command_thrust;
          selected.score = unified_plan.alternatives[chosen].score;
          recovery_conditional_grasp_probe_active_ =
              unified_plan.alternatives[chosen].response_probe_required;
          recovery_conditional_grasp_response_observed_ = false;
          recovery_conditional_grasp_probe_start_ = ros::Time(0);
          recovery_conditional_grasp_probe_start_error_rad_ =
              std::numeric_limits<double>::quiet_NaN();
          recovery_unified_selected_candidate_index_ =
              static_cast<int>(chosen);
          ROS_WARN("Unified robust recovery ACTIVE selected candidate %zu: pair %zu+%zu, mode=%s%s, worst support yz=[%.1f %.1f] mm, robust signed lever %.4f m (max %.4f over %zu vertices), required %.2f N/rotor",
                   chosen, pairs[pair][0], pairs[pair][1],
                   release ? "straight-arm" : "grasped-current-bend",
                   recovery_conditional_grasp_probe_active_
                       ? " (below-side response probe)" : "",
                   1000.0 * selected.pivot[1],
                   1000.0 * selected.pivot[2], selected.lever,
                   unified_plan.alternatives[chosen].maximum_signed_lever,
                   unified_plan.alternatives[chosen].support_vertex_count,
                   selected.required_thrust);
        }
    }
  else if (recovery_circle_selection_enabled_ && circle_fresh)
    {
      const size_t extension_pressure_index = 0;
      Candidate best_grasped;
      Candidate best_extended;
      for (size_t pair = 0; pair < pairs.size(); ++pair)
        {
          if (recovery_preferred_pair_locked_ &&
              pair != recovery_preferred_pair_index_)
            continue;
          Candidate extended = evaluate(pair, extension_pressure_index);
          const Candidate grasped = evaluate(pair, grasp_pressure_index);
          const double pair_depth = std::isfinite(grasped.depth)
              ? grasped.depth : extended.depth;
          ROS_INFO("Recovery candidates pair %zu+%zu depth %.1f deg: grasped lever %.4f m, FF %.2f N, valid=%d; straight-arm lever %.4f m, FF %.2f N, valid=%d; emergency=%d",
                   pairs[pair][0], pairs[pair][1],
                   std::isfinite(pair_depth)
                       ? pair_depth * 180.0 / M_PI : -1.0,
                   grasped.lever, grasped.required_thrust,
                   grasped.valid ? 1 : 0,
                   extended.lever, extended.required_thrust,
                   extended.valid ? 1 : 0,
                   emergency_extension_requested ? 1 : 0);
          static_cast<void>(pair_depth);  // ROS logging is compiled out in unit stubs.
          if (grasped.valid && grasped.score < best_grasped.score)
            best_grasped = grasped;
          if (extended.valid && extended.score < best_extended.score)
            best_extended = extended;
        }
      // Compare every pair/contact topology with the *same* objective:
      // model-required rotor force plus the estimated cost of removing
      // contact support. The previous unconditional flexed preference could
      // choose a 30-kPa arm despite a much better released candidate. For a
      // fast-growing large angle, straight-arm geometry is a feasibility
      // constraint rather than a soft preference: never silently return to
      // the deep-grasped mode at large roll when release has been requested.
      selected = emergency_extension_requested
          ? best_extended
          : (best_extended.valid &&
             (!best_grasped.valid ||
              best_extended.score + 1.0e-6 < best_grasped.score)
                 ? best_extended : best_grasped);
    }
  else if (!recovery_unified_active_enabled_ &&
           recovery_circle_selection_enabled_)
    {
      // The object-centre estimate is unavailable or stale.  Do not revert
      // immediately to a roll-sign/lower-side lookup: the URDF rotor pose and
      // the fixed bottom contact still provide a directly measured signed
      // moment test.  This is the same three-joint geometry used by the
      // general attitude allocator and avoids selecting 2+3 solely because
      // body roll is positive.
      const size_t extension_pressure_index = 0;
      Candidate best_grasped;
      Candidate best_extended;
      for (size_t pair = 0; pair < pairs.size(); ++pair)
        {
          const Candidate grasped = evaluate(pair, grasp_pressure_index);
          const Candidate extended = evaluate(pair, extension_pressure_index);
          if (grasped.valid && grasped.score < best_grasped.score)
            best_grasped = grasped;
          if (extended.valid && extended.score < best_extended.score)
            best_extended = extended;
        }
      selected = emergency_extension_requested
          ? best_extended
          : (best_extended.valid &&
             (!best_grasped.valid ||
              best_extended.score + 1.0e-6 < best_grasped.score)
                 ? best_extended : best_grasped);
    }

  if (selected.valid)
    {
      if (!recovery_unified_active_enabled_ &&
          continuous_recovery_use_opposite_pair_ &&
          !recovery_preferred_pair_locked_)
        {
          const size_t opposite = 1 - selected.pair;
          Candidate alternate;
          for (const size_t pressure_index :
               {size_t(0),
                recovery_circle_pressure_candidates_kpa_.size() - 1})
            {
              const Candidate option = evaluate(opposite, pressure_index);
              if (option.valid && option.score < alternate.score)
                alternate = option;
            }
          if (alternate.valid) selected = alternate;
          else
            {
              selected.pair = opposite;
              selected.release = true;
              selected.pressure_index = 0;
              selected.pressure = recovery_circle_pressure_candidates_kpa_[0];
              selected.depth = std::numeric_limits<double>::quiet_NaN();
            }
          ROS_WARN("Continuous recovery uses the opposite circle-model pair after a measured wrong-way response");
        }
      // Intermediate 10/20-kPa states are now transitions only.  Their
      // instantaneous bend-dependent lever may flip while roll is changing,
      // so it must not stop a topology-locked 30 -> 0-kPa extension.
      recovery_selected_path_min_lever_m_ = selected.lever;
      recovery_selected_path_requires_zero_thrust_ = false;
      prepare_arm_active_.fill(false);
      for (const size_t arm : pairs[selected.pair]) prepare_arm_active_[arm] = true;
      recovery_selected_pair_index_ = selected.pair;
      if (!recovery_preferred_pair_locked_)
        {
          recovery_preferred_pair_index_ = selected.pair;
          recovery_preferred_pair_used_fallback_ = false;
          recovery_preferred_pair_locked_ = true;
        }
      recovery_selected_pair_release_ = selected.release;
      recovery_selected_required_thrust_n_ = selected.required_thrust;
      if (selected.release)
        {
          recovery_extension_mode_latched_ = true;
          recovery_extension_direction_sign_ = correction_direction;
        }
      recovery_selected_pair_pressure_kpa_ = selected.pressure;
      recovery_selected_pressure_candidate_index_ = selected.pressure_index;
      recovery_selected_contact_pivot_enabled_ = true;
      recovery_selected_contact_pivot_ = selected.pivot;
      recovery_pair_selection_available_ = true;
      recovery_selected_pair_mean_depth_rad_ = selected.depth;
      recovery_selected_support_loss_moment_nm_ =
          selected.support_loss_moment;
      recovery_selected_release_start_pressure_ = measured_pressure_;
      ROS_INFO("Recovery rotor-geometry selection: object model=%s radius %.1f mm (rms %.1f mm, confidence %.2f), pair %zu+%zu, mode=%s, required thrust %.2f N/rotor, rotor depth %.1f deg (%s side), target pressure %.1f kPa, fixed pivot yz=[%.1f %.1f] mm, signed lever %.3f m, support-loss %.3f Nm",
               circle_fresh ? "circle" : "fixed-contact fallback",
               circle_fresh ? 1000.0 * event_circle.radius : -1.0,
               circle_fresh ? 1000.0 * event_circle.rms : -1.0,
               circle_fresh ? event_circle.confidence : 0.0,
               pairs[selected.pair][0], pairs[selected.pair][1],
               selected.release ? "straight-arm" : "grasped-current-bend",
               selected.required_thrust,
               std::isfinite(selected.depth) ?
                   selected.depth * 180.0 / M_PI : -1.0,
               std::isfinite(selected.depth) && selected.depth >= 0.5 * M_PI
                   ? "below" : "above",
               selected.pressure,
               1000.0 * selected.pivot[1], 1000.0 * selected.pivot[2],
               selected.lever, selected.support_loss_moment);
      return;
    }

  // Neither pair has a model-valid restoring contact state.  Do not apply a
  // rotor command with the wrong signed lever; keep support and wait for a
  // fresh contact estimate instead.
  if (recovery_circle_selection_enabled_ && circle_fresh &&
      (!recovery_unified_active_enabled_ || unified_support_available) &&
      recovery_preferred_pair_locked_)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      ROS_ERROR("No restoring moment candidate remains in either pair after preferred pair %zu+%zu was exhausted",
                recovery_preferred_pair_index_ == 0 ? 0ul : 2ul,
                recovery_preferred_pair_index_ == 0 ? 1ul : 3ul);
      return;
    }

  // A fresh circular estimate plus a non-empty rejection set means the
  // measured-response supervisor has already disproved at least one model
  // candidate in this posture event.  If none of the remaining candidates
  // has a restoring signed lever, do not silently fall back to the fixed pair
  // and repeat an actuator state which has already failed.
  if (recovery_circle_selection_enabled_ && circle_fresh &&
      (!recovery_unified_active_enabled_ || unified_support_available) &&
      recovery_rejected_candidate_mask_ != 0)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      ROS_ERROR("No unused circular recovery candidate remains for rejected mask 0x%02x",
                recovery_rejected_candidate_mask_);
      return;
    }

  // With a valid support hypothesis, active unified mode remains fail-closed:
  // a model-valid but infeasible/rejected candidate must not be replaced by a
  // speculative command.  Missing circle/support geometry is different.  In
  // that case use the long-standing fixed bottom contact and lower-side pair
  // below so a temporary estimator dropout cannot cancel posture recovery.
  if (recovery_unified_active_enabled_ && circle_fresh &&
      unified_support_available)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      ROS_ERROR("Unified recovery ACTIVE found no unused feasible candidate; refusing legacy lower-side actuation");
      return;
    }
  if (recovery_unified_active_enabled_)
    ROS_WARN("Unified recovery geometry/support unavailable; using the fixed-contact lower-side fallback without stopping recovery");

  // If circle fitting is temporarily unavailable, preserve the same
  // lower-side topology used by the hardware convention instead of choosing
  // from an instantaneous bend-dependent lever.  The latter can change sign
  // simply because the body keeps rolling during exhaust and caused the
  // controller to move actuation to the upper pair.  The physical lower side
  // is determined from body roll itself: negative roll puts arms 0+1 lower,
  // positive roll puts arms 2+3 lower.
  const size_t lower_pair = recovery_preferred_pair_locked_
      ? recovery_preferred_pair_index_ : (roll_ < 0.0 ? 0 : 1);
  prepare_arm_active_ = lower_pair == 0
      ? std::array<bool, ARM_COUNT>{{true, true, false, false}}
      : std::array<bool, ARM_COUNT>{{false, false, true, true}};
  recovery_selected_pair_release_ = true;
  recovery_selected_required_thrust_n_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_extension_mode_latched_ = true;
  recovery_extension_direction_sign_ = correction_direction;
  recovery_selected_pair_pressure_kpa_ = recovery_release_pressure_kpa_;
  recovery_selected_pressure_candidate_index_ = 0;
  recovery_selected_contact_pivot_enabled_ = false;
  recovery_selected_pair_index_ = lower_pair;
  recovery_preferred_pair_index_ = lower_pair;
  recovery_preferred_pair_locked_ = true;
  recovery_preferred_pair_used_fallback_ = true;
  recovery_pair_selection_available_ = true;
  recovery_selected_pair_mean_depth_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_selected_support_loss_moment_nm_ = 0.0;
  recovery_selected_release_start_pressure_ = measured_pressure_;
  ROS_WARN("Circle recovery estimate unavailable; latched topology lower-side pair %zu+%zu in fully extended mode for roll error %.1f deg",
           pairs[lower_pair][0], pairs[lower_pair][1],
           roll_error * 180.0 / M_PI);
}

void AttitudePressureController::beginPostureSafeTransfer(
    const ros::Time& now, const char* reason, bool retry_after_settle,
    hugmy::RecoveryFaultReason fault_reason)
{
  // This branch does not mean that the attitude has been corrected.  It
  // establishes a measured, load-bearing contact boundary after a failed
  // rotor pass; another posture attempt needs explicit new authorization.
  // A failed rotor pass needs a true four-arm support boundary, not the
  // minimum pressure used for an ordinary candidate re-selection.  Restore
  // the operator/locomotion grasp pressure on every arm; this changes no
  // global grasp default and is local to SAFE_TRANSFER.
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    recovery_safe_transfer_target_pressure_[arm] = hugmy::recoveryClamp(
        std::max(base_pressure_kpa_, grasp_pressure_[arm]),
        recovery_handoff_min_pressure_kpa_, maximum_pressure_kpa_);
  recovery_safe_transfer_retry_allowed_ = retry_after_settle;
  recovery_safe_transfer_failed_ = false;
  recovery_safe_transfer_settled_since_ = ros::Time(0);
  recovery_safe_transfer_phase_ =
      hugmy::RecoverySafeTransferPhase::REGRASP;
  recovery_support_monitor_.reset();
  recovery_fault_reason_ = fault_reason;
  continuous_recovery_blocked_ = !retry_after_settle;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  recovery_candidate_reselect_pending_ = retry_after_settle &&
      recovery_rejected_candidate_mask_ != 0;
  recovery_handoff_integral_n_ = 0.0;
  recovery_handoff_braking_ = false;
  action_c_resume_gate_.reset();
  action_c_boundary_settle_gate_.reset();
  enterState(POSTURE_SAFE_TRANSFER, now);
  ROS_ERROR("Posture recovery interrupted (%s, fault=%s): SAFE_TRANSFER without roll PI, roll %.1f deg, measured pressure [%.1f %.1f %.1f %.1f] kPa -> safe target [%.1f %.1f %.1f %.1f] kPa; retaining existing rotor thrust only for bounded rundown",
      reason, hugmy::recoveryFaultReasonName(recovery_fault_reason_),
      wrapAngle(roll_ - target_roll_) * 180.0 / M_PI,
      measured_pressure_[0], measured_pressure_[1],
      measured_pressure_[2], measured_pressure_[3],
      recovery_safe_transfer_target_pressure_[0],
      recovery_safe_transfer_target_pressure_[1],
      recovery_safe_transfer_target_pressure_[2],
      recovery_safe_transfer_target_pressure_[3]);
}

AttitudePressureController::PressureArray
AttitudePressureController::recoveryReselectionPressure() const
{
  PressureArray target{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double nominal = hugmy::recoveryClamp(
          std::max(base_pressure_kpa_, grasp_pressure_[arm]),
          recovery_handoff_min_pressure_kpa_, maximum_pressure_kpa_);
      // A flexed/grasped candidate is already load bearing: lowering it to
      // the generic support floor would release the only useful small-radius
      // contact merely because its rotor moment was ineffective. A released
      // candidate, in contrast, must be brought back only to the minimum
      // contact pressure before geometry is selected again. The opposite
      // pair remains at nominal pressure as the mechanical anchor.
      target[arm] = prepare_arm_active_[arm] &&
                    recovery_selected_pair_release_
          ? std::min(std::max(recoverySupportPressure(),
                              recoverySelectedPairPressure(arm)), nominal)
          : nominal;
    }
  return target;
}

AttitudePressureController::PressureArray
AttitudePressureController::recoveryMeasuredContactPressure() const
{
  PressureArray target{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      // At a measured target crossing, the current chamber pressure is the
      // only four-arm pressure state known to support the attitude reached
      // by the measured response.  Freeze *all* chambers there before
      // recomputing the contact circle.  Restoring the nominal 30-kPa anchor
      // on the non-selected pair changed the contact geometry during the
      // handoff; on the 130-mm fixture it kicked a 1.7-deg target crossing
      // back to a 5--6-deg residual before the new pair could be selected.
      // Keep only the ordinary load-bearing floor as a safety bound.
      target[arm] = hugmy::recoveryClamp(
          measured_pressure_[arm], recovery_handoff_min_pressure_kpa_,
          maximum_pressure_kpa_);
    }
  return target;
}

AttitudePressureController::PressureArray
AttitudePressureController::recoveryCandidateSwitchPressure() const
{
  PressureArray target{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      // Candidate rejection is not a successful target capture. Keep the
      // opposite pair load bearing, but leave a deliberately released pair
      // at its measured low pressure while thrust runs down and geometry is
      // selected again. Raising that pair immediately to the generic 15-kPa
      // handoff floor changes its bend/contact pivot; in the 2026-11-02
      // trial that transition increased roll from about 15 to 25 deg before
      // the next candidate could generate restoring force.
      const bool released_selected_arm =
          prepare_arm_active_[arm] && recovery_selected_pair_release_;
      const double minimum = released_selected_arm
          ? 0.0 : recovery_handoff_min_pressure_kpa_;
      target[arm] = hugmy::recoveryClamp(
          measured_pressure_[arm], minimum, maximum_pressure_kpa_);
    }
  return target;
}

double AttitudePressureController::recoverySupportPressure() const
{
  const bool below_side_release =
      recovery_selected_contact_pivot_enabled_ &&
      recovery_selected_pair_release_ &&
      std::isfinite(recovery_selected_pair_mean_depth_rad_) &&
      recovery_selected_pair_mean_depth_rad_ >=
          0.5 * M_PI + recovery_circle_side_margin_rad_;
  return hugmy::recoveryClamp(
      below_side_release
          ? std::max(recovery_handoff_min_pressure_kpa_,
                     recovery_below_side_support_pressure_kpa_)
          : recovery_handoff_min_pressure_kpa_,
      minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
}

double AttitudePressureController::recoverySelectedPairPressure(
    size_t arm) const
{
  if (arm >= ARM_COUNT) return minimum_prepare_pressure_kpa_;
  const double nominal = hugmy::recoveryClamp(
      std::max(base_pressure_kpa_, grasp_pressure_[arm]),
      minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
  if (!recovery_selected_pair_release_) return nominal;
  return hugmy::recoveryClamp(recovery_selected_pair_pressure_kpa_,
      minimum_prepare_pressure_kpa_, nominal);
}

void AttitudePressureController::updateCircularGraspGeometry(
    const ros::Time& now)
{
  std::array<std::array<double, 2>, ARM_COUNT> contact_yz{};
  std::array<bool, ARM_COUNT> contact_available{{false, false, false, false}};
  const bool simulation_contact_fresh = grip_state_valid_ &&
      !grip_state_stamp_.isZero() &&
      (now - grip_state_stamp_).toSec() <= 0.25;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bend = measuredArmBendAngle(arm);
      const Vector3 contact = armContactPointFromAngle(arm, bend);
      contact_yz[arm] = {{contact[1], contact[2]}};
      contact_available[arm] = std::isfinite(bend) &&
          std::isfinite(measured_pressure_[arm]) &&
          measured_pressure_[arm] >= recovery_circle_contact_pressure_kpa_ &&
          (!simulation_contact_fresh || grip_active_[arm]);
    }
  const auto estimate = hugmy::estimateCircularGrasp(
      contact_yz, contact_available,
      {{contact_position_[1], contact_position_[2]}},
      {{gravity_body_[1], gravity_body_[2]}},
      recovery_circle_min_radius_m_, recovery_circle_max_radius_m_,
      recovery_circle_max_rms_m_);
  if (estimate.valid && estimate.confidence >=
      recovery_circle_min_confidence_)
    {
      recovery_circle_estimate_ = estimate;
      recovery_circle_stamp_ = now;
    }

  const double age = recovery_circle_stamp_.isZero()
      ? std::numeric_limits<double>::infinity()
      : (now - recovery_circle_stamp_).toSec();
  // A circle fitted from reliable contact samples remains the object model
  // until a newer reliable fit replaces it.  Its radius and center do not
  // expire merely because the recovery event starts later; only the current
  // rotor locations are recomputed from the live bend measurements.
  const bool usable = recovery_circle_estimate_.valid &&
      recovery_circle_estimate_.confidence >=
          recovery_circle_min_confidence_ &&
      !recovery_circle_stamp_.isZero();
  // Publish the validity of the retained trusted circle, rather than the
  // success of only this control-cycle's fit.  Once a reliable estimate is
  // acquired, a temporary pressure/contact transition must not report false
  // while recovery continues to use that retained object model.
  std_msgs::Bool valid_msg;
  valid_msg.data = usable;
  recovery_circle_estimate_valid_pub_.publish(valid_msg);
  // Layout: usable, radius, center y/z, RMS, confidence, sample count, age,
  // four CURRENT ROTOR depth angles (0=top, pi/2=side, pi=underside),
  // selected pair (0=01, 1=23, -1=none), release flag, and selected
  // correcting-pair pressure [kPa].  Do not publish the distal-contact
  // depths captured when the circle was fitted: they can be tens of seconds
  // old and are not the rotor force application points used by recovery.
  std_msgs::Float32MultiArray msg;
  msg.data.resize(15, std::numeric_limits<float>::quiet_NaN());
  msg.data[0] = usable ? 1.0f : 0.0f;
  msg.data[1] = static_cast<float>(recovery_circle_estimate_.radius);
  msg.data[2] = static_cast<float>(recovery_circle_estimate_.center_y);
  msg.data[3] = static_cast<float>(recovery_circle_estimate_.center_z);
  msg.data[4] = static_cast<float>(recovery_circle_estimate_.rms);
  msg.data[5] = static_cast<float>(recovery_circle_estimate_.confidence);
  msg.data[6] = static_cast<float>(recovery_circle_estimate_.samples);
  msg.data[7] = static_cast<float>(age);
  if (usable)
    {
      const Vector3 object_center{{
          0.0, recovery_circle_estimate_.center_y,
          recovery_circle_estimate_.center_z}};
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        msg.data[8 + arm] = static_cast<float>(rotorDepthAngleAbout(
            arm, measuredArmBendAngle(arm), object_center));
    }
  int selected_pair = -1;
  if (prepare_arm_active_[0] && prepare_arm_active_[1]) selected_pair = 0;
  if (prepare_arm_active_[2] && prepare_arm_active_[3]) selected_pair = 1;
  msg.data[12] = static_cast<float>(selected_pair);
  msg.data[13] = recovery_selected_pair_release_ ? 1.0f : 0.0f;
  msg.data[14] = static_cast<float>(recovery_selected_pair_pressure_kpa_);
  recovery_circle_geometry_pub_.publish(msg);
}


hugmy::RollRecoveryModel AttitudePressureController::measuredRecoveryModel() const
{
  hugmy::RollRecoveryModel result;
  result.valid = true;
  // Perched recovery asks for ADDITIONAL restoring moment, not the full
  // gravity moment: contacts already support the body, and their reaction
  // moments are not known. Do not inherit gravity_compensation_ from the
  // legacy/other-mode allocator. Rate PI handles residual contact/friction.
  // The old PD derivative is not stacked with the inner velocity PI either.
  // Aim the model at zero roll; roll_recovery_config_.stop is used only by
  // the completion/capture logic.  The correction-axis projection keeps the
  // requested moment positive for the already selected restoring pair.
  result.moment = roll_moment_kp_ * std::max(0.0,
      wrapAngle(target_roll_ - roll_) * correction_axis_[0]);
  if (recovery_selected_pair_release_ &&
      recovery_selected_support_loss_moment_nm_ > 0.0)
    {
      double release_progress = 0.0;
      size_t release_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (!prepare_arm_active_[arm] ||
              !std::isfinite(measured_pressure_[arm]) ||
              !std::isfinite(recovery_selected_release_start_pressure_[arm]))
            continue;
          const double release_target = recoverySelectedPairPressure(arm);
          const double release_span =
              recovery_selected_release_start_pressure_[arm] - release_target;
          if (release_span <= 1.0e-6) continue;
          release_progress += hugmy::recoveryClamp(
              (recovery_selected_release_start_pressure_[arm] -
               measured_pressure_[arm]) / release_span, 0.0, 1.0);
          ++release_count;
        }
      if (release_count > 0)
        result.moment += recovery_selected_support_loss_moment_nm_ *
            release_progress / static_cast<double>(release_count);
    }
  result.context[4] = bottom_pressure_kpa_;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bend = measuredArmBendAngle(arm);
      result.context[arm] = measured_pressure_[arm];
      result.context[5 + arm] = bend;
      result.valid = result.valid && std::isfinite(bend) && std::isfinite(measured_pressure_[arm]);
      if (!prepare_arm_active_[arm]) continue;
      result.lever += recoveryRotorLever(arm,bend);
    }
  result.valid = result.valid && std::isfinite(result.context[4]);
  // selectContinuousRecoveryPair() accepts only a candidate whose predicted
  // signed lever points toward the target.  Keep that sign here: taking the
  // absolute value allowed a pair whose moment reversed after flexion to
  // receive an increasing feed-forward command.
  return result;
}

double AttitudePressureController::recoveryRotorRawLever(
    size_t arm, double bend) const
{
  return recoveryRotorRawLeverAbout(arm, bend, contact_position_);
}

AttitudePressureController::Vector3
AttitudePressureController::recoveryContactPivot(
    const std::array<bool, ARM_COUNT>& selected_pair,
    bool selected_pair_release,
    const PressureArray* pressure_override,
    const PressureArray* bend_override) const
{
  (void)selected_pair;
  (void)selected_pair_release;
  (void)pressure_override;
  (void)bend_override;
  return recovery_event_contact_pivot_locked_
      ? recovery_event_contact_pivot_ : contact_position_;
}

double AttitudePressureController::recoveryRotorRawLeverAbout(
    size_t arm, double bend, const Vector3& pivot) const
{
      const Vector3 position = rotorPositionFromAngle(arm, bend);
      // Same local-X cant and rotor signs as hugmy2.urdf.xacro. Neuron bend
      // is measured BELOW the rotor, so add cant here, not to the sensor angle.
      const double direction = (arm == 0 || arm == 2) ? 1.0 : -1.0;
      const Vector3 unit_force = rotateLocalToBody(rotateAboutLocalY(
          {{0.0, direction * std::sin(recovery_rotor_cant_rad_),
            std::cos(recovery_rotor_cant_rad_)}}, std::max(0.0, std::min(2.966, bend))), arm_yaw_[arm]);
      Vector3 lever;
      for (size_t axis = 0; axis < 3; ++axis)
        lever[axis] = position[axis] - pivot[axis];
      return cross(lever, unit_force)[0];
}

double AttitudePressureController::recoveryRotorLever(
    size_t arm, double bend) const
{
  // Loaded circle contacts use the physical contact-dependent moment.
  // Released arms, like the no-circle fallback, use the empirically verified
  // actuator-axis mapping after they have straightened away from contact.
  const bool flexed_circle_contact =
      recovery_selected_contact_pivot_enabled_ &&
      recovery_selected_pair_pressure_kpa_ >=
          recovery_circle_contact_pressure_kpa_;
  const double mapping_sign = flexed_circle_contact
      ? 1.0 : (correction_pair_reverse_spinal_mapping_ ? -1.0 : 1.0);
  const Vector3 pivot = recovery_selected_contact_pivot_enabled_
      ? recovery_selected_contact_pivot_ : contact_position_;
  return recoveryRotorRawLeverAbout(arm, bend, pivot) *
      correction_axis_[0] * mapping_sign;
}


void AttitudePressureController::requestActionCPostureRecovery(
    const ros::Time& now, const char* reason,
    double confirmed_trigger_rad, bool urgent_thrust_rise)
{
  pauseHybridLocomotion(now);
  if (!target_received_)
    {
      stopInchwormToGrasp(now, "Action C cannot recover without a target attitude");
      return;
    }
  ++action_c_posture_recovery_count_;
  recovery_event_start_ = now;
  recovery_event_active_ = true;
  recovery_fault_reason_ = hugmy::RecoveryFaultReason::NONE;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_use_opposite_pair_ = false;
  continuous_recovery_retry_opposite_pending_ = false;
  continuous_recovery_pair_switches_ = 0;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_event_contact_pivot_locked_ = false;
  recovery_selected_contact_pivot_enabled_ = false;
  recovery_preferred_pair_locked_ = false;
  // `urgent_thrust_rise` requests only the fast actuator slew.  Candidate
  // geometry below independently decides whether the selected pair must be
  // released/extended; coupling these two decisions forced a straight-arm
  // pair for small roll errors after the outward motion had already stopped.
  recovery_extension_mode_latched_ = false;
  recovery_extension_direction_sign_ = 0;
  recovery_unified_selected_candidate_index_ = -1;
  recovery_selected_required_thrust_n_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_unified_unlock_since_ = ros::Time(0);
  recovery_unified_unlock_ready_ = false;
  recovery_rate_arrest_active_ = false;
  recovery_rate_arrest_nonoutward_since_ = ros::Time(0);
  recovery_rate_arrest_reference_error_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  action_c_recovery_gate_.reset();
  action_c_resume_pending_ = true;
  action_c_requested_recovery_trigger_rad_ =
      std::isfinite(confirmed_trigger_rad) && confirmed_trigger_rad > 0.0
          ? confirmed_trigger_rad
          : action_c_posture_recovery_trigger_rad_;
  action_c_recovery_urgent_thrust_rise_ = urgent_thrust_rise;
  action_c_resume_direction_ = inchworm_direction_body_;
  inchworm_thrust_.fill(0.0);
  inchworm_pressure_.fill(base_pressure_kpa_);
  // Zero the gait output before handing ownership to shared roll recovery.
  publishCorrectionThrust(inchworm_thrust_);
  publishPressureTarget(inchworm_pressure_, std::numeric_limits<double>::infinity());
  publishBottomTarget(0.0);
  inchworm_active_ = false;
  enterState(GRASP, now);
  ROS_WARN("Action C paused for shared attitude recovery (attempt %d, no count stop): %s; saved goal x %.4f m, current x %.4f m",
      action_c_posture_recovery_count_, reason,
      action_c_absolute_target_x_m_, locomotionPosition()[0]);
}
