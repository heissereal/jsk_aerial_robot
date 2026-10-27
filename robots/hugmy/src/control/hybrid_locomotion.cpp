#include <hugmy/control/attitude_pressure_controller.h>

void AttitudePressureController::observeHybridThrust(const ros::Time& now,const PressureArray& thrust)
{
  if(!hybridLocomotionActive() || !hybrid_trial_active_) return;
  const double dt=now.toSec()-hybrid_effort_stamp_;
  if(hybrid_effort_stamp_>=0. && dt>=0. && std::isfinite(dt))
    for(double t:hybrid_sent_thrust_) hybrid_result_.effort+=t*t*dt;
  hybrid_effort_stamp_=now.toSec();hybrid_sent_thrust_=thrust;
}

void AttitudePressureController::pauseHybridLocomotion(const ros::Time& now)
{
  if (!hybridLocomotionActive() || !hybrid_trial_active_ || hybrid_recovery_start_>=0.) return;
  const double relative_x=locomotionPosition()[0];
  hybrid_recovery_start_=now.toSec();hybrid_recovery_x_=relative_x;
  hybrid_pre_recovery_dx_=hybrid_run_start_>=0. ?
      action_c_direction_sign_x_*(relative_x-hybrid_run_x_) : 0.;
}

bool AttitudePressureController::hybridResumeAllowed(const ros::Time& now)
{
  // Hybrid C is an anytime experiment: an operator stop, the absolute target,
  // a sensor/pressure fault, or the 90-deg posture limit terminates it.  A
  // wall-clock deadline must not turn a recoverable trial into a demand for a
  // new command.
  (void)now;
  return true;
}

void AttitudePressureController::finishHybridRecovery(const ros::Time& now)
{
  // Recovery is not itself a failed motion: a completed powered segment may
  // retain credit after successful recovery, with its full time/energy cost.
  if (hybridLocomotionActive()) {
    if(hybrid_start_mean_valid_ &&
       hugmy::hybrid::bottomPressureReady(bottom_pressure_kpa_,0.,
           action_c_bottom_deflation_tolerance_kpa_) &&
       hybrid_endpoint_mocap_window_.stable(now.toSec())) {
      hybrid_end_mean_x_=hybrid_endpoint_mocap_window_.mean();
      hybrid_end_mean_valid_=true;
      ROS_INFO("Hybrid interrupted endpoint measured after recovery/bottom deflation: x %.6f",
          hybrid_end_mean_x_);
    }
    finishHybridLocomotion(now,hybrid_phase_==2 && !hybrid_terminal_settle_,true);
  }
}

hugmy::hybrid::Force AttitudePressureController::hybridRotorForce(
    size_t arm,double bend,double roll_error,double pitch) const
{
  const double cant=recovery_rotor_cant_rad_;
  const double sign=(arm==0 || arm==2)?1.:-1.;
  const auto direction=rotateLocalToBody(rotateAboutLocalY(
      {{0.,sign*std::sin(cant),std::cos(cant)}},bend),arm_yaw_[arm]);
  auto lever=rotorPositionFromAngle(arm,bend);
  for(size_t axis=0;axis<3;++axis) lever[axis]-=hybrid_contact_position_[axis];
  const double roll=roll_error+target_roll_;
  const double z=std::sin(roll)*direction[1]+std::cos(roll)*direction[2];
  hugmy::hybrid::Force force;
  // Fixture signed mocap x must be aligned with body +/-x at setup.
  force.forward=action_c_direction_sign_x_*(std::cos(pitch)*direction[0]+std::sin(pitch)*z);
  force.up=-std::sin(pitch)*direction[0]+std::cos(pitch)*z;
  force.side=std::cos(roll)*direction[1]-std::sin(roll)*direction[2];
  force.roll_moment=cross(lever,direction)[0];
  // Express pitch moment in the commanded travel coordinate, just as
  // `forward` is.  For both +x and -x travel the useful windup moment has the
  // same sign in that coordinate, which lets the phase logic select the
  // physically generated tilt rather than assuming a fixed RPY sign.
  force.pitch_moment=action_c_direction_sign_x_*
      cross(lever,direction)[1];return force;
}

bool AttitudePressureController::hybridLocomotionActive() const
{
  return inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;
}

void AttitudePressureController::startHybridLocomotion(const ros::Time& now)
{
  const double relative_x=locomotionPosition()[0];
  if (inchworm_cycle_count_==1 && !action_c_preserve_adaptation_on_start_)
    {hybrid_task_start_=now.toSec();hybrid_settling_loss_=0.;
     hybrid_single_.reset();
     if(!hybrid_hold_pressure_) hybrid_grasp_pressure_.fill(base_pressure_kpa_);}
  // A new operator command starts from the nominal symmetric grasp.  An
  // automatic restart after posture recovery is different: the recovery
  // handoff has just measured a quiet, load-bearing contact equilibrium and
  // stores it in hybrid_grasp_pressure_.  Replacing that equilibrium by four
  // 30-kPa commands here recreated the roll disturbance and caused an
  // endless recovery -> restart -> recovery loop.  Preserve the measured
  // boundary for this one internal restart; the following VentFront phase
  // still commands the ordinary rear support/front release trajectory.
  if(!hybrid_retain_grasp_pressure_ &&
     !action_c_preserve_adaptation_on_start_)
    hybrid_grasp_pressure_.fill(base_pressure_kpa_);
  hybrid_single_.initialized=false;
  hybrid_hold_bottom_=false;
  hybrid_hold_pressure_=false;
  hybrid_trial_active_=true;
  hybrid_pressure_guard_.reset();hybrid_pressure_ready_gate_.reset();
  hybrid_body_rate_filter_.reset();
  hybrid_probe_budget_.reset();
  hybrid_prediction_limited_=false;
  hybrid_trial_start_=now.toSec(); hybrid_trial_x_=relative_x;
  hybrid_start_mean_valid_=hybrid_end_mean_valid_=false;hybrid_bottom_changed_=false;
  // An automatically repeated stroke starts and ends in the same
  // load-bearing 30-kPa bottom state.  The first stroke (and a restart after
  // recovery, where the bottom is empty) still measures a quiet empty
  // boundary before inflating once.
  hybrid_inflated_boundary_=hybrid_keep_bottom_between_strokes_ &&
      inchworm_cycle_count_>1 &&
      hugmy::hybrid::bottomPressureReady(bottom_pressure_kpa_,
          hybrid_single_config_.bottom_kpa,pressure_tolerance_kpa_);
  hybrid_preinflate_measured_=hybrid_inflated_boundary_;
  hybrid_terminal_settle_=false;
  hybrid_terminal_retry_=false;hybrid_identification_settle_=false;
  hybrid_regrip_thrust_zero_latched_=false;
  hybrid_regrip_complete_latched_=false;
  hybrid_regrip_terminal_rear_lift_requested_=false;
  hybrid_terminal_rear_lift_sign_.fill(0.);
  hybrid_bottom_deflated_latched_=false;
  hybrid_endpoint_captured_latched_=false;
  hybrid_ppg_wait_since_=-1.;
  hybrid_terminal_reason_.clear();
  hybrid_position_window_.reset();hybrid_position_window_.duration=.5;
  hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
  hybrid_endpoint_mocap_window_.reset();hybrid_endpoint_mocap_window_.duration=.8;
  hybrid_endpoint_pitch_window_.reset();
  hybrid_endpoint_pitch_window_.duration=
      hybrid_keep_bottom_between_strokes_ ? 1.0 : 3.0;
  hybrid_bottom_assist_.reset(0.);
  hybrid_run_start_=-1.;hybrid_ready_since_=-1.;hybrid_recovery_start_=-1.;
  hybrid_roll_rate_limit_since_=-1.;
  hybrid_last_stamp_=-1.;hybrid_speed_=0.;hybrid_phase_=0;hybrid_candidate_selected_=false;
  hybrid_result_=hugmy::hybrid::Result();
  hybrid_effort_stamp_=now.toSec();hybrid_sent_thrust_.fill(0.);
  hybrid_result_.goal=std::min(hybrid_config_.stroke,
      std::max(.0001,action_c_direction_sign_x_*(action_c_absolute_target_x_m_-relative_x)));
  hybrid_planner_.beginRun();hybrid_planner_.hold_thrust_search=false;hybrid_planner_.model.newWindow();
  action_c_recovery_gate_.reset();action_c_boundary_settle_gate_.reset();
  hybrid_ppg_settle_gate_.reset();
  action_c_roll_trend_.reset();
  for(auto& rate:hybrid_sim_rate_) rate=hugmy::hybrid::BendCoordinateRate();
  inchworm_pressure_=hybrid_grasp_pressure_;inchworm_thrust_.fill(0.);
  publishCorrectionThrust(inchworm_thrust_);
  publishPressureTarget(inchworm_pressure_,std::numeric_limits<double>::infinity());
  publishBottomDirection(-1.);
  enterState(ACTION_C_PREPARE,now);
  ROS_INFO("Hybrid C/D single pass: bottom %.1f kPa; slip once, lift once, pressure-followed pull; rear sliding release/grasp %.2f/%.2f s; target %.1f mm",hybrid_single_config_.bottom_kpa,hybrid_single_config_.rear_low_seconds,hybrid_single_config_.rear_grasp_seconds,1000.*hybrid_result_.goal);
}

void AttitudePressureController::resetRecoveryHandoffState()
{
  recovery_handoff_integral_n_ = 0.0;
  recovery_handoff_brake_error_rad_ = 0.0;
  recovery_handoff_best_error_rad_ = std::numeric_limits<double>::infinity();
  recovery_handoff_worsening_since_ = -1.0;
  recovery_handoff_thrust_worsening_since_ = -1.0;
  recovery_handoff_progress_reference_rad_ =
      std::numeric_limits<double>::infinity();
  recovery_handoff_last_progress_time_ = -1.0;
  recovery_handoff_braking_ = false;
  // A circle-model recovery may deliberately keep the selected pair flexed
  // and load bearing.  In that mode all four chambers already share support;
  // running the released-pair equalisation stage would unnecessarily lower
  // every arm to the 15-kPa floor before returning to locomotion.
  recovery_handoff_support_equalized_ =
      !recovery_selected_pair_release_;
  recovery_handoff_pair_was_wrong_ = false;
  recovery_handoff_pressure_trimmed_ = false;
  recovery_handoff_thrust_wrong_way_ = false;
  recovery_handoff_finalizing_regrasp_ = false;
  // A previous recovery may have retained an asymmetric minimum-support
  // pressure (for example [30,30,15,15] kPa).  That is a valid temporary
  // contact state, but it must not become the next locomotion baseline: a
  // front pair spanning the two pressure groups starts Motion 2 with tens of
  // degrees of bend mismatch.  Every fresh recovery therefore aims at the
  // operator/locomotion grasp floor on all arms.  The handoff controller may
  // still pause or trim this target while it is actively correcting roll,
  // but a subsequent measured-roll pass gets another opportunity to reach
  // symmetric grasp before locomotion resumes.
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      recovery_handoff_target_pressure_[arm] = std::min(
          maximum_pressure_kpa_,
          std::max(base_pressure_kpa_, grasp_pressure_[arm]));
      recovery_handoff_best_pressure_[arm] = grasp_pressure_[arm];
    }
}

void AttitudePressureController::beginHybridRegrip(const ros::Time& now,bool prediction_limited)
{
  hybrid_regrip_thrust_=inchworm_thrust_;
  // Reverse thrust is a transient pitch/traction input, not a support force.
  // Retaining it through a failed windup or normal regrip left the rear pair
  // blowing indefinitely while all four chambers were already loaded. Stop
  // it at the support-transfer boundary; front positive thrust retains its
  // pressure-proportional rundown below.
  for(size_t arm=0;arm<ARM_COUNT;++arm)
    if(hybrid_regrip_thrust_[arm]<0.) {
      hybrid_regrip_thrust_[arm]=0.;
      inchworm_thrust_[arm]=0.;
    }
  hybrid_regrip_pressure_=last_pressure_;
  hybrid_grasp_pressure_.fill(base_pressure_kpa_);
  if(hybrid_retain_grasp_pressure_) for(size_t arm=0;arm<ARM_COUNT;++arm)
    if(inchworm_front_[arm]) hybrid_grasp_pressure_[arm]=std::max(base_pressure_kpa_,last_pressure_[arm]);
  if(hybrid_retain_grasp_pressure_) {
    // Dynamic left/right corrections must not become a permanent asymmetric
    // grasp (e.g. 43/50 kPa), which tilted the stationary simulation after
    // thrust removal. Retain the larger front flexion pressure on both arms.
    const double front_grasp=std::max(hybrid_grasp_pressure_[inchworm_front_order_[0]],
                                      hybrid_grasp_pressure_[inchworm_front_order_[1]]);
    for(size_t arm=0;arm<ARM_COUNT;++arm) if(inchworm_front_[arm]) hybrid_grasp_pressure_[arm]=front_grasp;
  }
  hybrid_regrip_bottom_=hybrid_bottom_assist_.command;
  hybrid_regrip_anchor_phase_=0;
  hybrid_regrip_anchor_since_=now.toSec();
  hybrid_regrip_thrust_zero_latched_=false;
  hybrid_regrip_complete_latched_=false;
  const double remaining=action_c_direction_sign_x_*(
      action_c_absolute_target_x_m_-locomotionPosition()[0]);
  const bool rear_release_still_missing=
      hybrid_single_.rear_grasp_reference_valid &&
      hybrid_single_.rear_release_pair<
          hybrid_single_.rear_release_target-
              hybrid_single_config_.rear_release_assist_deadband;
  hybrid_regrip_terminal_rear_lift_requested_=
      hybrid_single_config_.pull_terminal_rear_lift_enabled &&
      !prediction_limited && !hybrid_terminal_settle_ && remaining>.001 &&
      rear_release_still_missing &&
      !hybrid_single_.terminal_rear_lift_complete &&
      !hybrid_single_.terminal_rear_lift_cancelled;
  hybrid_terminal_rear_lift_sign_.fill(0.);
  if(hybrid_regrip_terminal_rear_lift_requested_ &&
     !hybrid_single_.terminal_rear_lift_started) {
    hybrid_single_.terminal_rear_lift_elapsed=0.;
    hybrid_single_.terminal_rear_lift_command=0.;
    hybrid_single_.terminal_anchor_wait_elapsed=0.;
    hybrid_single_.terminal_front_anchor_stall_time=0.;
    hybrid_single_.terminal_front_anchor_best_pressure=0.;
    hybrid_single_.terminal_front_anchor_wait_started=false;
  }
  hybrid_bottom_deflated_latched_=false;
  hybrid_endpoint_captured_latched_=false;
  hybrid_ppg_wait_since_=-1.;
  hybrid_regrip_x_=hybrid_position_window_.ready(now.toSec())?
      hybrid_position_window_.mean():locomotionPosition()[0];
  hybrid_prediction_limited_=prediction_limited;
  hybrid_phase_=2;hybrid_ready_since_=-1.;
  // Motion 3 is over at this boundary.  Clear only the live golden-window
  // outputs so diagnostics do not report a powered interval while regrip is
  // already commanding zero thrust; the measured stroke summary is retained
  // for endpoint scoring and rear-pressure adaptation.
  hybrid_single_.golden_window_active=false;
  hybrid_single_.golden_desired_up=0.;
  hybrid_single_.golden_desired_forward=0.;
  hybrid_pressure_ready_gate_.reset();
  hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
  hybrid_ppg_settle_gate_.reset();
  action_c_recovery_gate_.reset();action_c_boundary_settle_gate_.reset();action_c_roll_trend_.reset();
  enterState(ACTION_C_REGRIP,now);
  ROS_INFO("Hybrid C/D regrip: prediction_limited=%d; retaining goal x %.4f and search %.2f N/rotor for automatic continuation",
      prediction_limited,action_c_absolute_target_x_m_,hybrid_planner_.thrust_search.target);
}

void AttitudePressureController::beginHybridTerminalSettle(
    const ros::Time& now,const char* reason)
{
  if(!hybrid_trial_active_) {
    stopInchwormToGrasp(now,reason);return;
  }
  hybrid_terminal_settle_=true;
  hybrid_terminal_retry_=true;
  hybrid_identification_settle_=
      std::string(reason).find("identification-only")!=std::string::npos;
  hybrid_terminal_reason_=reason;
  beginHybridRegrip(now,false);
  hybrid_regrip_bottom_=0.;
  hybrid_inflated_boundary_=false;
  // Restore grasp and begin bottom exhaust immediately. The usual phase-2
  // rundown removes thrust without an impulse; scoring waits for a quiet,
  // deflated mocap window.
  inchworm_pressure_=hybrid_grasp_pressure_;
  publishPressureTarget(inchworm_pressure_,std::numeric_limits<double>::infinity());
  publishBottomTarget(0.);
  publishCorrectionThrust(inchworm_thrust_);
  ROS_WARN("Hybrid trajectory ended; waiting for grasp and deflated endpoint before scoring: %s",
      reason);
}

void AttitudePressureController::finishHybridLocomotion(const ros::Time& now,bool completed,bool safe)
{
  if(!hybrid_trial_active_) return;
  observeHybridThrust(now,hybrid_sent_thrust_);
  const bool measured=hybrid_start_mean_valid_ && hybrid_end_mean_valid_;
  const double relative_x=locomotionPosition()[0];
  const double end_x=measured?hybrid_end_mean_x_:relative_x;
  const double raw=action_c_direction_sign_x_*(end_x-hybrid_trial_x_);
  double retained=hybrid_run_start_>=0 ? action_c_direction_sign_x_*(relative_x-hybrid_run_x_) : std::min(0.,raw);
  if(hybrid_recovery_start_>=0) {
    hybrid_result_.recovery_seconds=std::max(0.,now.toSec()-hybrid_recovery_start_);
    retained=hybrid_pre_recovery_dx_+std::min(0.,action_c_direction_sign_x_*(relative_x-hybrid_recovery_x_));
  }
  hybrid_result_.progress=measured?raw:std::min(0.,std::min(raw,retained));
  // A stable pair of comparable support boundaries is the authoritative
  // retained displacement, including recovery. Ordinary repeated strokes use
  // the inflated four-arm boundary; recovery/PPG paths may use bottom zero.
  // The powered-segment clamp remains only as a conservative fallback when a
  // comparable endpoint is unavailable.
  if(hybrid_recovery_start_>=0. && !measured)
    hybrid_result_.progress=std::min(hybrid_result_.progress,retained);
  hybrid_result_.seconds=now.toSec()-hybrid_trial_start_;
  hybrid_result_.completed=completed && measured;hybrid_result_.safe=safe;
  if(completed && safe && measured && hybrid_phase_==2 && hybrid_recovery_start_<0.) {
    const double loss=action_c_direction_sign_x_*(hybrid_regrip_x_-end_x);
    if(std::isfinite(loss) && std::abs(loss)<.05)
      hybrid_settling_loss_=std::max(0.,std::min(.03,loss));
    ROS_INFO("Hybrid C/D measured settling rollback %.2f mm; next powered endpoint allowance %.2f mm (absolute retained goal unchanged)",
        1000.*loss,1000.*hybrid_settling_loss_);
  }
  // A pending comparison can exist when resuming adaptation saved by an older
  // run; finish it once, then hand all deliberate force changes to the outer
  // measured search.  In the current policy the slip and first unloading
  // probes establish the coarse operating point.  Chaining a separate blind
  // +dF calibration here would change the front waveform at the same time as
  // a rear-support candidate and make the measured result unidentifiable.
  const double previous_pull=hybrid_single_.pull_forward;
  const int pull_calibration_result=safe && measured ?
      hybrid_single_.finishPullCalibration(hybrid_result_.progress):0;
  if(pull_calibration_result<0)
    ROS_WARN("Hybrid initial pull calibration rejected: retained %.2f mm was worse than %.2f mm; restoring horizontal pull %.2f -> %.2f N",
        1000.*hybrid_result_.progress,
        1000.*hybrid_single_.pull_calibration_baseline_progress,
        previous_pull,hybrid_single_.pull_forward);
  else if(pull_calibration_result>0)
    ROS_INFO("Hybrid initial pull calibration retained: endpoint response did not worsen; horizontal pull remains %.2f N",
        hybrid_single_.pull_forward);
  // The first identification reach has a deliberately different waveform.
  // The first subsequent measured, safe stroke is the baseline observation,
  // even when a guard shortened it: its stable retained endpoint is precisely
  // the evidence required by the bounded outer search.  Freeze the inner
  // command before recording that sample so front and rear experiments remain
  // orthogonal.
  if(safe && measured && hybrid_single_.calibrated &&
     hybrid_single_.strokes>1 && !hybrid_single_.pull_calibration_pending)
    hybrid_single_.pull_calibration_complete=true;
  // Complete the rear-support prior before recording an outer-search sample.
  // This is the second half of the one-time identification (the slip probe is
  // the first); after it, rear support is changed only by the same measured
  // candidate search as the front trajectory.
  const double previous_rear_support=hybrid_single_.rear_support_prior;
  const int rear_identification=measured?
      hybrid_single_.finishRearSupportIdentification(
          hybrid_result_.progress,safe,hybrid_single_config_):0;
  if(rear_identification!=0) {
    hybrid_single_.trajectory.samples.clear();
    hybrid_single_.trajectory.current=0;
    hybrid_single_.trajectory.baseline_revalidation_remaining=0;
    ROS_INFO("Hybrid rear-support identification: %.2f -> %.2f (%s; pressure floor %.1f kPa, peak stroke-relative roll %.1f deg, retained %.2f mm)",
        previous_rear_support,hybrid_single_.rear_support_prior,
        rear_identification>0?"more support":"more release",
        hybrid_single_.rearPressureFloor(hybrid_single_config_),
        hybrid_single_.stroke_peak_abs_gait_roll*180./M_PI,
        1000.*hybrid_result_.progress);
  }
  // The first identification reach is a different waveform and must not
  // teach the repeated-stroke model. Subsequent shortened/strengthened
  // strokes remain useful because record() retains their exact scale.
  // Missing stable endpoints are diagnostics, not positive training samples.
  // A recovery can finish before the resumed stroke ever enters a powered
  // phase.  Such a pressure-handoff-only interval is not a trajectory trial
  // and must not add a failed optimizer sample.  A guarded/interrupted stroke
  // that actually started remains useful measured evidence.
  if(hybrid_single_.lift_probe_entries>0 && hybrid_single_.strokes>1 &&
     hybrid_run_start_>=0.) {
    hybrid_single_.trajectory.enabled=hybrid_search_.enabled;
    const unsigned executed_candidate=hybrid_single_.trajectory.current;
    const auto choice=hybrid_single_.trajectory.selected();
    if(hybrid_single_.selectedRearCandidateIdentifiable()) {
      hybrid_single_.trajectory.record(hybrid_single_.context_bend,hybrid_single_.context_roll,
          hybrid_single_.context_pitch,hybrid_result_.progress,hybrid_result_.seconds,
          hybrid_result_.effort,hybrid_result_.recovery_seconds,measured,
          hybrid_result_.completed,safe,hybrid_single_.goal_scale);
      ROS_INFO("Hybrid measured trajectory optimization: candidate %u, mix offset %.2f force scale %.2f unload scale %.2f rear-support offset %.2f (prior/effective %.2f/%.2f), observations %zu; retained %.2f mm",
          executed_candidate,choice.mix_offset,choice.force_scale,
          choice.unload_scale,choice.rear_support_offset,
          hybrid_single_.rear_support_prior,
          hybrid_single_.rear_support_command,
          hybrid_single_.trajectory.samples.size(),1000.*hybrid_result_.progress);
    } else {
      // The inner posture loop replaced this rear-support request before the
      // pressure/thrust reached the hardware.  Scoring the retained motion as
      // the selected candidate would teach the optimizer the response of a
      // different input.  Return to one baseline stroke without adding a
      // fictitious observation; the candidate may be retried after the
      // contact boundary is stable.
      hybrid_single_.trajectory.current=0;
      hybrid_single_.trajectory.baseline_revalidation_remaining=1;
      ROS_WARN("Hybrid optimizer excluded unrealized rear candidate %u (offset %.2f, distinct=%d realized=%d, safety override %.2f); returning to baseline without recording this sample",
          executed_candidate,choice.rear_support_offset,
          hybrid_single_.rear_candidate_distinct,
          hybrid_single_.rear_candidate_realized,
          hybrid_single_.rear_support_safety_override);
    }
  }
  ROS_INFO("Hybrid C/D result: completed=%d safe=%d retained %.2f mm / %.2f s, effort %.3f N2s, recovery %.2f s, J %.6f, bottom %.1f kPa m_min %.2f",
      hybrid_result_.completed,safe,1000.*hybrid_result_.progress,hybrid_result_.seconds,hybrid_result_.effort,
      hybrid_result_.recovery_seconds,hybrid_result_.score(),hybrid_result_.candidate.bottom,hybrid_result_.candidate.minimum_ratio);
  ROS_INFO("Hybrid C/D evaluation: stable_support_endpoints=%d bottom_changed=%d start_mean_x=%.6f end_mean_x=%.6f bottom=%.2f kPa; %s",
      measured,hybrid_bottom_changed_,hybrid_start_mean_valid_?hybrid_trial_x_:std::numeric_limits<double>::quiet_NaN(),
      hybrid_end_mean_valid_?hybrid_end_mean_x_:std::numeric_limits<double>::quiet_NaN(),bottom_pressure_kpa_,
      measured?"comparable settled support endpoint means used":
          "no comparable settled support endpoint pair; sample excluded");
  hybrid_trial_active_=false;
}

void AttitudePressureController::updateHybridLocomotion(const ros::Time& now,double dt)
{
  using namespace hugmy::hybrid;
  if (!hybridLocomotionActive()) return;
  const double clock=now.toSec();
  const Vector3 locomotion_position=locomotionPosition();
  auto fresh=[&](const ros::Time& stamp,double limit) {
    const double age=(now-stamp).toSec();return !stamp.isZero() && age>=0. && age<=limit;
  };
  hugmy::hybrid::State s;
  const auto body_rate=hybrid_body_rate_filter_.update(
      {{body_angular_velocity_[0],body_angular_velocity_[1]}},imu_stamp_.toSec());
  s.now=clock;s.roll=wrapAngle(roll_-target_roll_);s.roll_rate=body_rate[0];
  s.pitch=pitch_;s.pitch_start=hybrid_pitch_start_;
  s.bottom=bottom_pressure_kpa_;
  // Motion 2 and Motion 3 share one short unloaded interval.  Keep the bottom
  // inflated while pressure flexion rotates the front thrust from lift into
  // pull, and while the newly placed front contact and the rear support are
  // established.  Exhausting the bottom at the Lift -> Pull boundary removed
  // support before either pair was load bearing: the powered endpoint moved
  // forward, but 8--11 mm was returned during every regrip.  Phase 2 now has
  // an explicit inflated support-transfer boundary (anchor phase 0), followed
  // by bottom exhaust and empty-bottom geometry settling (anchor phase 1/5).
  s.bottom_command=(hybrid_phase_==0 && !hybrid_preinflate_measured_) ? 0. :
      hybrid_phase_==2 ? hybrid_regrip_bottom_ :
      hybrid_single_config_.bottom_kpa;
  const bool imu_fresh=fresh(imu_stamp_,std::min(.25,imu_timeout_sec_));
  const bool arm_pressure_fresh=fresh(measured_pressure_stamp_,.25);
  const bool bottom_pressure_fresh=fresh(measured_bottom_pressure_stamp_,.25);
  // Use the configured mocap timeout.  The generic inchworm safety path does
  // the same; imposing an undocumented 0.25-s cap here aborted otherwise
  // healthy real runs on short OptiTrack gaps.
  const bool mocap_fresh=locomotionMocapFresh(now);
  s.fresh=imu_fresh && arm_pressure_fresh && bottom_pressure_fresh && mocap_fresh;
  if(grip_state_valid_) s.fresh=s.fresh && fresh(grip_state_stamp_,.25);
  for(double value:{s.roll,s.roll_rate,s.pitch,s.bottom,body_angular_velocity_[0],body_angular_velocity_[1],
      locomotion_position[0],locomotion_position[1],mocap_position_[2]})
    s.fresh=s.fresh && std::isfinite(value);
  std::array<size_t,2> arms=inchworm_front_order_;
  for(size_t arm=0;arm<ARM_COUNT;++arm) {
    s.fresh=s.fresh && fresh(neuron_stamp_[arm],std::min(.25,neuron_timeout_sec_));
  }
  const auto pressure_status=hybrid_pressure_guard_.update(measured_pressure_,clock,
      arm_pressure_fresh,
      hybrid_config_.pressure_max+hybrid_config_.pressure_measurement_margin,
      pressure_safety_limit_kpa_,pressure_safety_confirm_sec_);
  if(pressure_status==PressureGuard::Fault) {
    stopInchwormToGrasp(now,"hybrid raw pressure invalid/hard limit or sustained overpressure");return;
  }
  const bool pressure_pending=pressure_status==PressureGuard::Pending;
  PressureArray base_target,enter_tolerance;
  base_target=hybrid_grasp_pressure_;enter_tolerance.fill(pressure_tolerance_kpa_);
  const bool measuring_pressure=hybrid_phase_==2 || (hybrid_phase_==0 && !hybrid_candidate_selected_);
  if(!measuring_pressure) hybrid_pressure_ready_gate_.reset();
  const bool all_pressure=measuring_pressure && hybrid_pressure_ready_gate_.update(
      measured_pressure_,base_target,enter_tolerance,clock,fresh(measured_pressure_stamp_,.25),
      action_c_pressure_hold_tolerance_kpa_,action_c_pressure_dropout_sec_,action_c_pressure_max_deviation_kpa_);
  if(mocap_stamp_.toSec()!=hybrid_last_stamp_) {
    const double sample_dt=mocap_stamp_.toSec()-hybrid_last_stamp_;
    if(hybrid_last_stamp_>=0 && sample_dt>0 && sample_dt<=.25)
      hybrid_speed_=.7*hybrid_speed_+.3*action_c_direction_sign_x_*(locomotion_position[0]-hybrid_last_x_)/sample_dt;
    else hybrid_speed_=0.;
    hybrid_last_stamp_=mocap_stamp_.toSec();hybrid_last_x_=locomotion_position[0];
  }
  hybrid_position_window_.add(mocap_stamp_.toSec(),locomotion_position[0]);
  const bool speed_valid=hybrid_position_window_.ready(clock);
  s.speed=speed_valid?action_c_direction_sign_x_*hybrid_position_window_.speed():0.;
  const bool powered_regrip=hybrid_phase_==2 &&
      std::any_of(inchworm_thrust_.begin(),inchworm_thrust_.end(),[](double t){return std::abs(t)>.01;});
  const bool run=!hybrid_terminal_settle_ &&
      (hybrid_phase_==1 || powered_regrip);
  s.elapsed=hybrid_run_start_>=0 ? clock-hybrid_run_start_ : 0.;
  for(size_t j=0;j<2;++j) {
    const size_t arm=arms[j];
    const double rate=hugmy::relativeBendRate(neuron_gyro_[arm][1],body_angular_velocity_[0],body_angular_velocity_[1],arm_yaw_[arm]);
    // The calibrated Spinal--Neuron accelerometer attitude is the absolute
    // bend coordinate used by the real robot (straight gives local +Z ~= g).
    // Keep the relative gyro only as the bend-rate measurement/predictor.
    // Integrating that gyro across the multi-second slip/lift/catch sequence
    // accumulated the measured arm-3 bias and produced a false 30--40 deg
    // extension, even after the arm had physically re-flexed.
    const double acceleration_bend=measuredArmBendAngle(arm);
    const bool simulated=std::isfinite(grip_joint_bend_rad_[arm]) && fresh(grip_state_stamp_,.25);
    s.bend[j]=acceleration_bend;
    s.rate[j]=simulated?hybrid_sim_rate_[j].update(s.bend[j],grip_state_stamp_.toSec()):rate;
    s.pressure[j]=measured_pressure_[arm];s.command[j]=last_pressure_[arm];
    s.thrust[j]=inchworm_thrust_[arm];s.start_bend[j]=run?hybrid_start_bend_[j]:s.bend[j];
    s.floor[j]=hugmy::actionCPulseBendFloor(s.start_bend[j],action_c_min_safe_angle_rad_,
        action_c_hard_min_bend_rad_,action_c_extension_target_rad_,action_c_start_extension_margin_rad_);
    s.fresh=s.fresh && std::isfinite(rate) && std::isfinite(s.bend[j]);
  }
  Pair rear_bend{},rear_rate{};
  std::array<size_t,2> rear_arms{{0,0}};
  bool rear_kinematics_valid=true;
  size_t rear_index=0;
  for(size_t arm=0;arm<ARM_COUNT;++arm) if(!inchworm_front_[arm]) {
    if(rear_index>=rear_arms.size()) {
      rear_kinematics_valid=false;
      break;
    }
    rear_arms[rear_index]=arm;
    rear_bend[rear_index]=measuredArmBendAngle(arm);
    const double measured_rate=hugmy::relativeBendRate(
        neuron_gyro_[arm][1],body_angular_velocity_[0],
        body_angular_velocity_[1],arm_yaw_[arm]);
    const bool simulated=std::isfinite(grip_joint_bend_rad_[arm]) &&
        fresh(grip_state_stamp_,.25);
    rear_rate[rear_index]=simulated?
        hybrid_rear_sim_rate_[rear_index].update(
            rear_bend[rear_index],grip_state_stamp_.toSec()):
        measured_rate;
    rear_kinematics_valid=rear_kinematics_valid &&
        std::isfinite(rear_bend[rear_index]) &&
        std::isfinite(rear_rate[rear_index]);
    ++rear_index;
  }
  rear_kinematics_valid=rear_kinematics_valid &&
      rear_index==rear_arms.size();
  // Rear bend feedback now owns part of the actuator command, so stale or
  // invalid rear Neuron data is a safety input failure rather than an
  // optional diagnostic omission.
  s.fresh=s.fresh && rear_kinematics_valid;
  auto equilibrium=[&](size_t,double pressure,double force) {
    const auto range=bend_model_.ranges();const double coordinate=force+bend_model_thrust_offset_n_;
    if(pressure<range.first.first-5. || pressure>range.first.second ||
       coordinate<range.second.first || coordinate>range.second.second)
      return std::numeric_limits<double>::quiet_NaN();
    return bend_model_.angleRad(std::max(range.first.first,pressure),coordinate);
  };
  auto geometry=[&](size_t j,double bend,double roll_error,double pitch) {
    return hybridRotorForce(arms[j],bend,roll_error,pitch);
  };
  // A ROS timer can be dispatched twice at the same /clock stamp when
  // MuJoCo advances in larger chunks.  No physical time has elapsed, so keep
  // the preceding actuator command and skip this duplicate update.  Treating
  // dt==0 as a sensor fault stopped an otherwise healthy locomotion trial.
  // A backwards jump, a real control gap, or stale telemetry remains a
  // fail-closed condition.
  if(dt==0.) return;
  if(!s.fresh || dt<0. || dt>.1 || clock<hybrid_task_start_) {
    double oldest_neuron_age=0.;
    for(const auto& stamp:neuron_stamp_)
      oldest_neuron_age=stamp.isZero()?-1.:std::max(oldest_neuron_age,(now-stamp).toSec());
    ROS_ERROR("Hybrid input guard: imu=%d age=%.3f, arm_pressure=%d age=%.3f, "
        "bottom=%d age=%.3f, mocap=%d age=%.3f/%.3f, neuron oldest=%.3f, "
        "dt=%.3f, task_elapsed=%.2f s (anytime mode)",
        imu_fresh,(now-imu_stamp_).toSec(),arm_pressure_fresh,(now-measured_pressure_stamp_).toSec(),
        bottom_pressure_fresh,(now-measured_bottom_pressure_stamp_).toSec(),
        mocap_fresh,(now-mocap_stamp_).toSec(),inchworm_mocap_timeout_sec_,
        oldest_neuron_age,dt,clock-hybrid_task_start_);
    stopInchwormToGrasp(now,"hybrid stale/nonfinite sensors or clock discontinuity");return;
  }
  if(std::abs(s.roll)>=action_c_emergency_stop_roll_rad_) {
    stopInchwormToGrasp(now,"hybrid emergency roll reached 90-deg stop limit");return;
  }
  const bool straight_identification=hybrid_phase_==1 &&
      hybrid_single_.phase==SinglePass::Lift && !hybrid_single_.calibrated;
  const bool preparing_straight_identification=hybrid_phase_==1 &&
      hybrid_single_.phase==SinglePass::VentFront &&
      !hybrid_single_.calibrated;
  const double measured_roll_rate=body_angular_velocity_[0];
  const bool roll_rate_over_limit=
      std::abs(measured_roll_rate)>=hybrid_config_.roll_rate_limit;
  if(roll_rate_over_limit) {
    if(hybrid_roll_rate_limit_since_<0.)
      hybrid_roll_rate_limit_since_=clock;
  } else {
    // Confirmation is continuous: a single over-limit IMU sample must not be
    // remembered and combined with a later unrelated sample.
    hybrid_roll_rate_limit_since_=-1.;
  }
  const bool roll_rate_limit_confirmed=roll_rate_over_limit &&
      hybrid_roll_rate_limit_since_>=0. &&
      clock-hybrid_roll_rate_limit_since_>=hybrid_roll_rate_confirm_sec_;
  // Pull has already created the new front support when BOTH front chambers
  // carry pressure and both arms have reversed from extension into flexion.
  // At that point a hard-roll handoff to the shared posture controller is the
  // wrong topology: it releases another arm pair while the old rear anchors
  // are still exhausted.  In the high-friction MuJoCo run this happened at
  // 26--29 kPa front pressure and turned a retained +25 mm stroke into a fall.
  // Complete the four-arm support transfer first.  This does not relax the
  // hard guard before a new support exists; any missing pressure/flexion
  // evidence still takes the ordinary posture-recovery branch below.
  bool front_support_available=false;
  if(hybrid_phase_==1 &&
     (hybrid_single_.phase==SinglePass::Pull ||
      hybrid_single_.phase==SinglePass::FrontCatch) &&
     hybrid_single_.rear_release_observed) {
    bool pressure_loaded=true,flex_returned=true;
    for(size_t j=0;j<2;++j) {
      const double span=std::max(.017453,
          hybrid_single_.grasp_reference[j]-hybrid_single_.pull_start[j]);
      const double flex=clamp((s.bend[j]-hybrid_single_.pull_start[j])/span,
          0.,1.);
      pressure_loaded=pressure_loaded &&
          s.pressure[j]>=hybrid_single_config_.pull_support_transfer_pressure;
      flex_returned=flex_returned &&
          flex>=hybrid_single_config_.pull_regrip_min_flex;
    }
    front_support_available=pressure_loaded && flex_returned;
  }
  const bool guarded_support_catch=front_support_available &&
      (std::abs(s.roll)>=.75*hybrid_config_.roll_limit ||
       roll_rate_limit_confirmed);
  if(guarded_support_catch) {
    ROS_WARN("Hybrid guarded support catch: front pair is pressure-loaded and flexing at roll %.1f deg/rate %.1f deg/s; restoring rear grasp before shared posture recovery",
        s.roll*180./M_PI,measured_roll_rate*180./M_PI);
    hybrid_roll_rate_limit_since_=-1.;
    beginHybridRegrip(now,false);
    hybrid_regrip_bottom_=hybrid_single_config_.bottom_kpa;
    return;
  }
  const bool hard_roll_or_rate=std::abs(s.roll)>=hybrid_config_.roll_limit ||
      roll_rate_limit_confirmed;
  if(hard_roll_or_rate && hybrid_phase_==2) {
    // Do not open a correcting pair while rear support is still rebuilding.
    // Once all four chambers are measurably load-bearing and rotor force is
    // off, however, phase 2 is no longer an unsupported transient.  Waiting
    // for the full endpoint-settle sequence let a -20-deg residual grow past
    // -30 deg before recovery could own the actuators.
    const bool four_arm_support=std::all_of(measured_pressure_.begin(),
        measured_pressure_.end(),[&](double p) {
          return std::isfinite(p) &&
              p>=hybrid_single_config_.pull_loaded_catch_pressure;
        });
    if(four_arm_support &&
       std::abs(s.roll)>=hybrid_regrip_roll_emergency_rad_) {
      hybrid_roll_rate_limit_since_=-1.;
      requestActionCPostureRecovery(now,
          "hybrid emergency roll during four-arm support transfer");return;
    }
    ROS_WARN_THROTTLE(1.0,
        "Hybrid support transfer continuing through roll %.1f deg/rate %.1f deg/s; preserving four-arm pressure during bottom exhaust",
        s.roll*180./M_PI,measured_roll_rate*180./M_PI);
    hybrid_roll_rate_limit_since_=-1.;
  }
  if(hard_roll_or_rate && hybrid_phase_==1 &&
     (hybrid_single_.phase==SinglePass::Lift ||
      hybrid_single_.phase==SinglePass::VentFront)) {
    // Front support is absent or only partly established here. Opening a
    // second pair for posture recovery can leave no reliable anchor and was
    // the direct cause of the simulated fall after an unreleased Motion-2
    // search. Restore four-arm support first, score the retained endpoint,
    // then let the ordinary boundary logic recover roll and retry the same
    // goal. This guarded interval cannot receive completed-gait credit.
    hybrid_roll_rate_limit_since_=-1.;
    beginHybridTerminalSettle(now,
        "hybrid lift roll/rate limit; restore four-arm support before recovery");
    return;
  }
  if(hard_roll_or_rate && hybrid_phase_!=2) {
    hybrid_roll_rate_limit_since_=-1.;
    requestActionCPostureRecovery(now,"hybrid hard roll/rate limit");return;
  }
  // Use the same empty-bottom endpoint for the reverse-travel guard as for
  // retained progress.  The inflated preparation pose can be 10--20 mm
  // ahead; measuring retreat from that rocking offset stopped VentFront when
  // deflation merely returned the body to its true pre-stride position.
  const double reverse_reference_x = hybrid_start_mean_valid_
      ? hybrid_trial_x_ : hybrid_run_x_;
  if(!hybrid_terminal_settle_ && hybrid_run_start_>=0 &&
     action_c_direction_sign_x_*(locomotion_position[0]-reverse_reference_x)<=
         -action_c_abort_reverse_travel_m_) {
    beginHybridTerminalSettle(now,"hybrid reverse-travel limit; score retained endpoint and retry");return;
  }
  if(!pressure_pending) hybrid_planner_.model.observe(s,hybrid_config_,equilibrium);
  hugmy::hybrid::State safety_state=s;
  auto safety_config=hybrid_config_;
  // The nominal model ceiling is 150 deg, but a small human arm can be
  // grasped more deeply before locomotion starts (the 2026-10-26 run measured
  // 152--156 deg). Rejecting that quiet initial contact at the first active
  // tick prevented even the slip-test thrust ramp from starting. Anchor the
  // upper guard to the captured four-arm grasp instead: extension moves away
  // from this ceiling, while pressure-driven over-flexion is still stopped
  // after the configured small margin.
  safety_config.bend_max=hugmy::hybrid::contactRelativeBendMaximum(
      safety_config.bend_max,hybrid_single_.grasp_reference,
      safety_config.grasp_bend_margin);
  if(hybrid_phase_==2) {
    // Regrip has an independent emergency handoff above. Applying the
    // ordinary gait limit again in the generic planner guard would interrupt
    // the four-arm support transfer before bottom exhaust can finish.
    safety_config.roll_limit=hybrid_regrip_roll_emergency_rad_;
    // The powered Motion-2/3 excursion is over once regrip starts.  Pressure
    // now deliberately flexes the front pair back toward the cylinder, so
    // comparing its bend with the old extension start pose can only report a
    // stale gait excursion.  Retain the absolute bend, pressure, thrust,
    // pitch and emergency-roll guards, but reset this relative reference and
    // allow the identified pressure-driven return rate.
    safety_state.start_bend=s.bend;
    safety_config.bend_rate_limit=std::max(
        safety_config.bend_rate_limit,
        hybrid_single_config_.identification_return_bend_rate_limit);
  }
  const bool stopping_after_slip=hybrid_phase_==1 &&
      hybrid_single_.phase==SinglePass::StopForLift;
  const bool unpowered_preparation=hybrid_phase_==1 &&
      hybrid_single_.phase==SinglePass::VentFront &&
      std::max(s.thrust[0],s.thrust[1])<=.05;
  const bool horizontal_probe=hybrid_phase_==1 &&
      (hybrid_single_.phase==SinglePass::SlipProbe ||
       hybrid_single_.phase==SinglePass::Slide);
  if(stopping_after_slip || unpowered_preparation || horizontal_probe) {
    // StopForLift contains a fast thrust rundown.  Its changing specific
    // force can move the accelerometer angle across the ordinary 55-deg
    // pulse floor for one sample (real bag: 54.9 deg while gyro was already
    // flexing). Pressure release is likewise not a powered excursion. Keep
    // the absolute hard bend/rate/pitch/roll/freshness limits, but do not
    // classify either preparation boundary as a new powered extension.
    // SlipProbe/Slide use their own smaller per-arm extension bound relative
    // to the measured contact pose. SinglePass enforces that bound while this
    // adapter retains the absolute bend/floor, rate, roll and pitch guards;
    // do not apply the ordinary Action-C excursion a second time here.
    safety_state.start_bend=s.bend;
    if(stopping_after_slip || unpowered_preparation)
      safety_state.floor.fill(std::max(hybrid_config_.bend_min,action_c_hard_min_bend_rad_));
  }
  const bool returning_from_straight_identification=hybrid_phase_==1 &&
      hybrid_single_.identifying_straight && hybrid_single_.calibrated &&
      (hybrid_single_.phase==SinglePass::Pull ||
       hybrid_single_.phase==SinglePass::FrontCatch);
  const bool reused_absolute_motion=hybrid_phase_==1 &&
      hybrid_single_.calibrated &&
      std::isfinite(hybrid_single_.identified_bend_target[0]) &&
      std::isfinite(hybrid_single_.identified_bend_target[1]) &&
      (hybrid_single_.phase==SinglePass::VentFront ||
       hybrid_single_.phase==SinglePass::Lift ||
       hybrid_single_.phase==SinglePass::Pull ||
       hybrid_single_.phase==SinglePass::FrontCatch);
  if(straight_identification || preparing_straight_identification ||
     returning_from_straight_identification || reused_absolute_motion) {
    // The one-time identification covers the interval from the grasp bend to
    // the configured absolute unloading pose. Action C's old fixed-relative
    // excursion and pulse floor are incompatible with an absolute target when
    // the next grasp angle changes. Keep that measured envelope through the
    // pressure-driven return; bend-rate, roll, pitch, pressure and thrust
    // guards remain active.
    safety_state.floor.fill(0.);
    safety_config.bend_min=0.;
    double required_identification_travel=0.;
    for(size_t j=0;j<2;++j) {
      const double absolute_target=reused_absolute_motion?
          hybrid_single_.identified_bend_target[j]:
          hybrid_single_config_.identification_bend;
      required_identification_travel=std::max(required_identification_travel,
          std::max(0.,hybrid_single_.reference[j]-
              absolute_target)+
              hybrid_single_config_.identification_tolerance+2.*M_PI/180.);
    }
    safety_config.extension_limit=std::max(safety_config.extension_limit,
        std::max(hybrid_single_config_.identification_extension_limit,
                 required_identification_travel));
    safety_config.bend_rate_limit=std::max(safety_config.bend_rate_limit,
        straight_identification?
            hybrid_single_config_.identification_bend_rate_limit:
            hybrid_single_config_.identification_return_bend_rate_limit);
    if(returning_from_straight_identification) {
      // Motion 3 begins while the pneumatic command is reversing the fast
      // Motion-2 extension.  The arm can continue extending briefly during
      // valve/pressure delay; comparing that transient with Motion-2's grasp
      // reference aborted the golden interval before pressure could bend the
      // arm back.  The absolute bend/rate/pitch/roll limits above still own
      // safety, while the relative Motion-2 excursion is no longer relevant.
      safety_state.start_bend=s.bend;
    }
  }
  if(run && !Planner::valid(safety_state,safety_config,pressure_pending)) {
    ROS_WARN("Hybrid guard details: motion %d bend [%.1f %.1f] start [%.1f %.1f] floor [%.1f %.1f] deg, upper %.1f deg, rate [%.2f %.2f], pitch excursion %.1f deg",
        int(hybrid_single_.phase),s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
        safety_state.start_bend[0]*180./M_PI,safety_state.start_bend[1]*180./M_PI,
        safety_state.floor[0]*180./M_PI,safety_state.floor[1]*180./M_PI,
        safety_config.bend_max*180./M_PI,s.rate[0],s.rate[1],
        (s.pitch-s.pitch_start)*180./M_PI);
    beginHybridTerminalSettle(now,"hybrid measured bend/rate/tilt guard; score retained endpoint and retry");return;
  }
  // Reuse the acquired pressure latch here.  A strict +/-2-kPa comparison
  // restarted the boundary-settle clock on every short real-valve ripple,
  // even though the pressure gate correctly retained readiness through that
  // ripple.  The endpoint pitch window needs three continuous seconds, so
  // mixing the two definitions could keep PREPARE at motion 0 forever.
  // Acquisition is still strict; the latch only retains within the bounded
  // hold/grace limits, while PressureGuard independently owns hard faults.
  const double boundary_bottom_tolerance=s.bottom_command<=.01?
      action_c_bottom_deflation_tolerance_kpa_:pressure_tolerance_kpa_;
  const bool boundary=hybrid_phase_!=1 && all_pressure &&
      bottomPressureReady(s.bottom,s.bottom_command,boundary_bottom_tolerance) &&
      std::abs(s.roll_rate)<=action_c_pre_roll_rate_tolerance_rad_s_ &&
      std::all_of(inchworm_thrust_.begin(),inchworm_thrust_.end(),[](double t){return std::abs(t)<=.01;});
  action_c_roll_trend_.update(boundary,s.roll,s.roll_rate,clock);
  const bool settled=action_c_boundary_settle_gate_.update(boundary,clock,action_c_posture_settle_sec_);
  // Roll magnitude alone is not a recovery request.  A compliant contact can
  // retain a stationary offset after regrasp, and bottom inflation can move
  // that equilibrium by several degrees before the next stroke.  Treating
  // either effect as a residual error caused phase 0 <-> recovery cycling even
  // though the robot was not falling.  The ordinary boundary handoff therefore
  // requires a measured, signed outward trend.  The independent hard-angle
  // and confirmed-rate guards above remain active in every phase, so a truly
  // unsafe offset still transfers to recovery immediately.
  // A transient pulse roll is ignored until the empty-bottom, all-grasp
  // boundary is settled.  A compliant contact may then retain a stationary
  // 10--15-deg offset: repeatedly correcting that equilibrium only bends the
  // selected arms during regrasp and recreates the same offset.  The ordinary
  // threshold therefore requires a measured outward trend.  The independent
  // hard guard above still transfers immediately at roll_limit, regardless
  // of trend, so this does not relax the actual safety envelope.
  const double hybrid_residual_recovery_trigger=
      action_c_posture_recovery_trigger_rad_;
  const bool residual_requires_recovery=action_c_roll_trend_.worsening;
  const bool worsening_residual=settled && !hybrid_identification_settle_ &&
      std::abs(s.roll)>hybrid_residual_recovery_trigger &&
      residual_requires_recovery;
  // A quiet offset below the ordinary 10-deg recovery threshold is not a
  // failure by itself.  Before another unloaded Motion-2 reach, apply the
  // smaller pre-stroke budget only when that offset is measurably worsening.
  // A stable contact equilibrium is allowed to continue; the independent
  // hard-angle guard remains available if the next reach actually diverges.
  const bool pre_stroke_budget_exhausted=settled && hybrid_phase_==0 &&
      !hybrid_preinflate_measured_ &&
      action_c_roll_trend_.worsening &&
      std::abs(s.roll)>hybrid_single_config_.pre_stroke_roll_limit;
  if(action_c_recovery_gate_.update(
      worsening_residual || pre_stroke_budget_exhausted,
      clock,correction_trigger_confirm_sec_)) {
    requestActionCPostureRecovery(now,
        pre_stroke_budget_exhausted ?
            "hybrid pre-stroke roll budget exhausted at settled boundary" :
            (hybrid_phase_==2 ? "hybrid residual roll before next stroke" :
             "hybrid worsening residual roll at settled boundary"),
        pre_stroke_budget_exhausted ?
            hybrid_single_config_.pre_stroke_roll_limit :
            std::numeric_limits<double>::quiet_NaN());return;
  }
  const double stable_contact_roll_limit=hybrid_phase_==2 ?
      hybrid_regrip_roll_emergency_rad_:hybrid_config_.roll_limit;
  const bool stable_contact_offset=action_c_roll_trend_.ready &&
      !action_c_roll_trend_.worsening &&
      std::abs(s.roll)<stable_contact_roll_limit;
  const bool posture_ready=
      (std::abs(s.roll)<=hybrid_residual_recovery_trigger ||
       stable_contact_offset) &&
      bottomPressureReady(s.bottom,s.bottom_command,boundary_bottom_tolerance);
  const bool quiet=std::abs(s.roll_rate)<.1 && posture_ready;
  const bool off=std::all_of(inchworm_thrust_.begin(),inchworm_thrust_.end(),[](double t){return std::abs(t)<=.01;});
  const bool bottom_ready=bottomPressureReady(
      s.bottom,s.bottom_command,boundary_bottom_tolerance);
  // The preparation boundary still requires pressure, pose and bottom to be
  // simultaneously quiet.  At the post-motion boundary these quantities are
  // verified by independent one-way latches below: after regrip and bottom
  // exhaust have been confirmed, only the mocap position has to become stable
  // once.  Pitch noise must not restart an already completed mechanical step.
  const bool preparation_sample=hybrid_phase_==0 && all_pressure && off &&
      quiet && bottom_ready;
  const bool endpoint_sample=hybrid_phase_==2 &&
      hybrid_regrip_complete_latched_ &&
      hybrid_bottom_deflated_latched_ && off &&
      std::abs(s.roll)<hybrid_regrip_roll_emergency_rad_;
  if(preparation_sample) {
    hybrid_settled_window_.add(mocap_stamp_.toSec(),locomotion_position[0]);
    hybrid_bottom_window_.add(measured_bottom_pressure_stamp_.toSec(),s.bottom);
    hybrid_endpoint_pitch_window_.add(imu_stamp_.toSec(),s.pitch);
  } else if(endpoint_sample && !hybrid_endpoint_captured_latched_) {
    hybrid_settled_window_.add(mocap_stamp_.toSec(),locomotion_position[0]);
  } else if(!hybrid_endpoint_captured_latched_) {
    hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
    hybrid_endpoint_pitch_window_.reset();
  }
  const bool preparation_measurement_ready=preparation_sample &&
      hybrid_settled_window_.stable(clock) &&
      hybrid_bottom_window_.stable(clock,1.,1.) && hybrid_endpoint_pitch_window_.stable(clock,.004,.002);
  const bool endpoint_measurement_ready=endpoint_sample &&
      hybrid_settled_window_.stable(clock);
  SinglePassInput input;input.s=s;input.dt=dt;input.x=action_c_direction_sign_x_*hybrid_position_window_.mean();
  // Give the stride scheduler the full task remainder.  hybrid_result_.goal
  // is only the current-stroke target and used to hide a 50-mm task behind a
  // fixed 20-mm value, so the controller could not decide whether to repeat a
  // proven 10-mm stroke or strengthen a final 8-mm stroke toward 12 mm.
  const double endpoint_for_remaining=hybrid_start_mean_valid_?
      hybrid_trial_x_:hybrid_position_window_.mean();
  input.remaining=std::max(.0001,action_c_direction_sign_x_*(
      action_c_absolute_target_x_m_-endpoint_for_remaining));
  input.nominal_stride=hybrid_config_.stroke;
  input.goal_conditioning_start_ratio=
      hybrid_single_config_.goal_conditioning_start_ratio;
  input.goal_scale_min=hybrid_single_config_.goal_scale_min;
  input.goal_progress_max_scale=
      hybrid_single_config_.goal_progress_max_scale;
  input.goal_preferred_stride=hybrid_single_config_.goal_preferred_stride;
  input.height=mocap_position_[2];
  // Express useful load-relief rotation as positive for either travel
  // direction. MuJoCo publishes standard RPY, where lifting +x is negative
  // pitch; the real Spinal estimate used by the proven gait has the opposite
  // convention. Keep this source conversion explicit instead of silently
  // applying the simulation sign to hardware. The rate uses the same sign so
  // predictive windup remains consistent with the angle.
  input.pitch_rate=hybrid_directional_pitch_sign_*
      action_c_direction_sign_x_*body_rate[1];
  input.directional_pitch=hybrid_directional_pitch_sign_*
      action_c_direction_sign_x_*s.pitch;
  input.base_pressure=base_pressure_kpa_;input.speed_valid=speed_valid;
  input.rear_bend=rear_bend;input.rear_rate=rear_rate;
  input.rear_kinematics_valid=rear_kinematics_valid;
  input.rear_released=actionCRearReleased(now);
  input.rear_grasped=true;input.rear_pressure=0.;
  for(size_t arm=0;arm<ARM_COUNT;++arm) if(!inchworm_front_[arm]) {
    input.rear_grasped=input.rear_grasped &&
        measured_pressure_[arm]>=base_pressure_kpa_-3.;
    // The larger value is the rear chamber which will take longer to exhaust.
    if(!std::isfinite(measured_pressure_[arm]))
      input.rear_pressure=std::numeric_limits<double>::quiet_NaN();
    else if(std::isfinite(input.rear_pressure))
      input.rear_pressure=std::max(input.rear_pressure,measured_pressure_[arm]);
  }
  input.contact_available=grip_state_valid_ && fresh(grip_state_stamp_,.25);
  input.front_contact=grip_active_[arms[0]] && grip_active_[arms[1]];
  // `grip_active` means pressure-generated normal grip; it becomes false as
  // soon as a chamber is vented even if a high-friction tip is still rubbing
  // the cylinder. MuJoCo also reports geometric surface gap, so use a
  // two-arm clearance event as direct release evidence. Hardware has no such
  // telemetry and uses the IMU/mocap unloading path.
  constexpr double simulation_release_gap_m=.015;
  input.front_released=input.contact_available;
  for(size_t j=0;j<2;++j)
    input.front_released=input.front_released &&
        std::isfinite(grip_surface_gap_m_[arms[j]]) &&
        grip_surface_gap_m_[arms[j]]>=simulation_release_gap_m;
  SinglePassOutput command;command.bottom=hybrid_single_config_.bottom_kpa;
  if(hybrid_phase_==0) {
    inchworm_thrust_.fill(0.);inchworm_pressure_=hybrid_grasp_pressure_;
    if(preparation_measurement_ready) {
      if(!hybrid_preinflate_measured_) {
        const double empty_x=hybrid_settled_window_.mean();
        // Retained progress is measured between like support states: save the
        // quiet bottom-deflated position before inflation and do not replace
        // it with the rocked/inflated preparation pose below.
        hybrid_trial_x_=empty_x;
        hybrid_start_mean_valid_=true;
        hybrid_preinflate_measured_=true;
        hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
        hybrid_endpoint_pitch_window_.reset();
        action_c_boundary_settle_gate_.reset();
        ROS_INFO("Hybrid empty-bottom preparation measured at x %.6f; inflating bottom once to %.1f kPa before the repeated pulse/flex gait",
            empty_x,
            hybrid_single_config_.bottom_kpa);
      } else {
        const double inflated_start_x=hybrid_settled_window_.mean();
        for(size_t j=0;j<2;++j)
          hybrid_endpoint_bend_[arms[j]]=s.bend[j];
        hybrid_result_.goal=std::min(hybrid_config_.stroke,
            std::max(.0001,action_c_direction_sign_x_*(
                action_c_absolute_target_x_m_-hybrid_trial_x_)));
        hybrid_result_.candidate.bottom=hybrid_single_config_.bottom_kpa;
        hybrid_result_.candidate.minimum_ratio=1.;
        hybrid_phase_=1;hybrid_run_start_=clock;hybrid_run_x_=locomotion_position[0];
        hybrid_start_bend_=s.bend;hybrid_pitch_start_=pitch_;
        input.s.start_bend=s.bend;input.s.pitch_start=pitch_;
        hybrid_single_.begin(input);enterState(ACTION_C_HORIZONTAL_PULL,now);
        // begin() has now combined the full remaining task distance with the
        // measured retained stride. Freeze that planned distance for this
        // stroke; it must not change with bladder rocking during the motion.
        if(std::isfinite(hybrid_single_.desired_stride))
          hybrid_result_.goal=hybrid_single_.desired_stride;
        ROS_INFO("Hybrid repeated pulse/flex motion begins from inflated support x %.6f (bottom-deflated score origin %.6f); goal %.2f mm, predicted stride %.2f mm (%s), goal scale %.2f conditioned=%d; slip probes %u, lift probes %u",
            inflated_start_x,hybrid_trial_x_,
            1000.*hybrid_result_.goal,
            1000.*hybrid_single_.predicted_stride,
            hybrid_single_.stride_estimate_measured?"measured":"nominal",
            hybrid_single_.goal_scale,hybrid_single_.goal_conditioned,
            hybrid_single_.slip_entries,hybrid_single_.lift_probe_entries);
      }
    }
    if(clock-hybrid_trial_start_>hybrid_config_.prepare_timeout) {
      beginHybridTerminalSettle(now,"hybrid prepare timeout; score retained endpoint and retry");return;
    }
  } else if(hybrid_phase_==1 && !pressure_pending) {
    const auto previous_phase=hybrid_single_.phase;
    const bool previous_onset_identified=hybrid_single_.onset_identified;
    const bool previous_bend_calibrated=hybrid_single_.calibrated;
    const bool previous_load_relief=hybrid_single_.load_relief_detected;
    const bool previous_windup_capture=hybrid_single_.windup_pose_captured;
    const bool previous_reachable_peak=hybrid_single_.windup_reachable_peak;
    const bool previous_unload_event=hybrid_single_.unload_event_detected;
    const bool previous_unload_capture=hybrid_single_.unload_capture_started;
    const bool previous_rear_release=hybrid_single_.rear_release_observed;
    const bool previous_support_transfer=
        hybrid_single_.support_transfer_started;
    const std::array<unsigned,2> previous_samples{{hybrid_single_.onset[0].samples,hybrid_single_.onset[1].samples}};
    command=hybrid_single_.update(input,hybrid_config_,hybrid_single_config_,
        hybrid_planner_.model,geometry,equilibrium);
    s.bottom_command=command.bottom;
    if(previous_phase==SinglePass::VentFront && hybrid_single_.phase==SinglePass::Lift)
      hybrid_start_bend_=s.bend;
    if(command.recover_roll) {
      requestActionCPostureRecovery(now,
          hybrid_single_.phase==SinglePass::StopForLift
              ? "hybrid residual roll before front vent"
              : "hybrid predicted outward roll during lift");
      return;
    }
    if(command.calibration_regrip) {
      ROS_INFO("Hybrid first unloading event was strongly asymmetric; saved bend/thrust calibration and restoring four-arm support before the first Motion-3 pull");
      beginHybridTerminalSettle(now,
          "hybrid identification-only unload; regrip before calibrated pull");
      return;
    }
    if(command.fault) {
      ROS_WARN("Hybrid trajectory incomplete: motion %d, phase %.2f s powered %.2f s, T [%.2f %.2f], P [%.1f %.1f], reference [%.1f %.1f] bend [%.1f %.1f] deg",
          int(hybrid_single_.phase),hybrid_single_.phase_time,hybrid_single_.powered_time,
          s.thrust[0],s.thrust[1],s.pressure[0],s.pressure[1],
          hybrid_single_.reference[0]*180./M_PI,hybrid_single_.reference[1]*180./M_PI,
          s.bend[0]*180./M_PI,s.bend[1]*180./M_PI);
      beginHybridTerminalSettle(now,
          "hybrid preparation/lift/front catch timeout or unsafe trajectory");return;
    }
    // Positive Motion-3 handoff assist and the outer-search reverse assist
    // are mutually exclusive in SinglePassOutput. Keep their signs explicit
    // here so a short rear kick cannot be mistaken for reverse thrust.
    inchworm_thrust_.fill(command.rear_positive_thrust-
        command.rear_reverse_thrust);
    inchworm_pressure_.fill(command.rear_pressure);
    for(size_t j=0;j<2;++j) {
      inchworm_thrust_[arms[j]]=command.thrust[j];
      inchworm_pressure_[arms[j]]=command.pressure[j];
    }
    for(size_t j=0;j<2;++j) if(hybrid_single_.onset[j].samples!=previous_samples[j])
      ROS_INFO("Hybrid extension onset arm %zu: %.2f N at %.1f kPa, powered baseline %.1f deg (Lift reference %.1f deg); samples %u, next approach margin %.2f N",
          arms[j],hybrid_single_.onset[j].thrust,hybrid_single_.onset[j].pressure,
          hybrid_single_.powered_extension_reference[j]*180./M_PI,
          hybrid_single_.onset[j].bend*180./M_PI,
          hybrid_single_.onset[j].samples,hybrid_single_config_.onset_margin_n);
    if(!previous_onset_identified && hybrid_single_.onset_identified)
      ROS_INFO("Hybrid paired extension-onset identification complete: front arms [%zu %zu] %.2f/%.2f N; final bend trajectory calibration remains %s",
          arms[0],arms[1],hybrid_single_.onset[0].thrust,
          hybrid_single_.onset[1].thrust,
          hybrid_single_.calibrated?"complete":"pending");
    if(!previous_bend_calibrated && hybrid_single_.calibrated)
      ROS_INFO("Hybrid unloading-event calibration complete: measured event bends [%.1f %.1f] deg, required peak thrust [%.2f %.2f] N; paired onset identification %s",
          hybrid_single_.identified_bend_target[0]*180./M_PI,
          hybrid_single_.identified_bend_target[1]*180./M_PI,
          hybrid_single_.identified_extension_thrust[0],
          hybrid_single_.identified_extension_thrust[1],
          hybrid_single_.onset_identified?"complete":"pending");
    if(!previous_load_relief && hybrid_single_.load_relief_detected) {
      if(hybrid_single_config_.trajectory_golden_handoff)
        ROS_INFO("Hybrid load relief detected: immediate golden-time flexion at bend [%.1f %.1f] deg; front thrust [%.2f %.2f] N, rear pressure command %.1f kPa",
            hybrid_single_.windup_hold_bend[0]*180./M_PI,
            hybrid_single_.windup_hold_bend[1]*180./M_PI,
            hybrid_single_.load_relief_thrust[0],
            hybrid_single_.load_relief_thrust[1],
            hybrid_single_.rear_pressure_command);
      else
        ROS_INFO("Hybrid load relief detected: windup toward %.1f deg while holding bend [%.1f %.1f] deg (current/predicted pitch %.1f deg); front thrust [%.2f %.2f] N",
            hybrid_single_.windup_pitch_target*180./M_PI,
            hybrid_single_.windup_hold_bend[0]*180./M_PI,
            hybrid_single_.windup_hold_bend[1]*180./M_PI,
            hybrid_single_.windup_pitch_progress*180./M_PI,
            hybrid_single_.load_relief_thrust[0],
            hybrid_single_.load_relief_thrust[1]);
    }
    if(!previous_windup_capture && hybrid_single_.windup_pose_captured)
      ROS_INFO("Hybrid unloading pose captured: bend [%.1f %.1f] deg, pitch progress %.1f deg; pressure-driven flexion starts now",
          s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
          hybrid_single_.windup_pitch_progress*180./M_PI);
    if(!previous_reachable_peak && hybrid_single_.windup_reachable_peak)
      ROS_INFO("Hybrid windup reached contact-limited pitch peak %.1f deg below requested %.1f deg; starting pressure-driven flexion instead of timing out",
          hybrid_single_.windup_best_pitch*180./M_PI,
          hybrid_single_.windup_pitch_target*180./M_PI);
    if(!previous_unload_event && hybrid_single_.unload_event_detected)
      ROS_INFO("Hybrid measured unloading event: source=%s, pitch %.2f deg (windup target %.1f, predicted progress %.1f), height %.2f mm, front extension [%.1f %.1f] deg, bend [%.1f %.1f] deg, thrust [%.2f %.2f] N after %.2f s",
          hybrid_single_.unload_from_arm_trajectory?
              "powered relative arm travel":"body/contact cue",
          hybrid_single_.unload_pitch_delta*180./M_PI,
          hybrid_single_.windup_pitch_target*180./M_PI,
          hybrid_single_.windup_pitch_progress*180./M_PI,
          1000.*hybrid_single_.unload_height_delta,
          ((hybrid_single_.powered_extension_reference_valid[0]?
              hybrid_single_.powered_extension_reference[0]:
              hybrid_single_.reference[0])-
              hybrid_single_.unload_event_bend[0])*180./M_PI,
          ((hybrid_single_.powered_extension_reference_valid[1]?
              hybrid_single_.powered_extension_reference[1]:
              hybrid_single_.reference[1])-
              hybrid_single_.unload_event_bend[1])*180./M_PI,
          hybrid_single_.unload_event_bend[0]*180./M_PI,
          hybrid_single_.unload_event_bend[1]*180./M_PI,
          hybrid_single_.unload_event_thrust[0],
          hybrid_single_.unload_event_thrust[1],
          hybrid_single_.unload_event_elapsed);
    if(!previous_unload_capture && hybrid_single_.unload_capture_started)
      ROS_INFO("Hybrid unloading capture: paired powered extension confirmed; starting the immediate pneumatic catch with rear support retained; mismatch %.1f deg, extension rate [%.1f %.1f] deg/s",
          std::abs(((hybrid_single_.powered_extension_reference_valid[0]?
                       hybrid_single_.powered_extension_reference[0]:
                       hybrid_single_.reference[0])-s.bend[0])-
                   ((hybrid_single_.powered_extension_reference_valid[1]?
                       hybrid_single_.powered_extension_reference[1]:
                       hybrid_single_.reference[1])-s.bend[1]))*180./M_PI,
          std::max(0.,-s.rate[0])*180./M_PI,
          std::max(0.,-s.rate[1])*180./M_PI);
    if(!previous_rear_release && hybrid_single_.rear_release_observed)
      ROS_INFO("Hybrid measured rear-anchor release: pressure %.1f kPa; front support transfer may begin after the flexion/load conditions are met",
          input.rear_pressure);
    if(!previous_support_transfer && hybrid_single_.support_transfer_started)
      ROS_INFO("Hybrid rear support transfer starts after %s: front pressure [%.1f %.1f] kPa, bend [%.1f %.1f] deg, rear measured/hold %.1f/%.1f kPa, confirmation %.2f s",
          hybrid_single_.support_transfer_ready_time+1.e-9>=
              hybrid_single_config_.pull_regrip_confirm?
                  "sustained measured pressure/flexion":"roll safety override",
          s.pressure[0],s.pressure[1],
          s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
          input.rear_pressure,
          hybrid_single_config_.regrip_rear_hold_pressure,
          hybrid_single_.support_transfer_ready_time);
    if(previous_phase==SinglePass::SlipSettle &&
       hybrid_single_.phase!=SinglePass::SlipSettle)
      ROS_INFO("Hybrid slip identification: retained %.2f mm, sliding=%d unstable=%d; rear-support prior %.2f (pressure floor %.1f kPa), pull seed %.2f N",
          1000.*hybrid_single_.slip_retained_progress,
          hybrid_single_.sliding_effective,
          hybrid_single_.slip_probe_unstable,
          hybrid_single_.rear_support_prior,
          hybrid_single_.rearPressureFloor(hybrid_single_config_),
          hybrid_single_.pull_forward);
    if(previous_phase!=hybrid_single_.phase)
      ROS_INFO("Hybrid single-pass trajectory phase %d -> %d; slip probes %u lift probes %u; bend [%.1f %.1f] deg, Lift reference [%.1f %.1f] deg, current extension [%.1f %.1f] deg, extension target [%.1f %.1f] deg, bend target [%.1f %.1f] deg, straight identification %d (measured absolute target only), learned absolute bend [%.1f %.1f] deg, learned target thrust [%.2f %.2f] N, recorded extension min/per-arm %.1f/[%.1f %.1f] deg, lift %.2f N, pull %.2f N",
          int(previous_phase),int(hybrid_single_.phase),hybrid_single_.slip_entries,hybrid_single_.lift_probe_entries,
          s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
          hybrid_single_.reference[0]*180./M_PI,hybrid_single_.reference[1]*180./M_PI,
          (hybrid_single_.reference[0]-s.bend[0])*180./M_PI,
          (hybrid_single_.reference[1]-s.bend[1])*180./M_PI,
          hybrid_single_.active_extension_target[0]*180./M_PI,
          hybrid_single_.active_extension_target[1]*180./M_PI,
          hybrid_single_.active_bend_target[0]*180./M_PI,
          hybrid_single_.active_bend_target[1]*180./M_PI,
          hybrid_single_.identifying_straight,
          180./M_PI*hybrid_single_.identified_bend_target[0],
          180./M_PI*hybrid_single_.identified_bend_target[1],
          hybrid_single_.identified_extension_thrust[0],
          hybrid_single_.identified_extension_thrust[1],
          180./M_PI*hybrid_single_.observed_extension,
          180./M_PI*hybrid_single_.observed_extension_by_arm[0],
          180./M_PI*hybrid_single_.observed_extension_by_arm[1],
          hybrid_single_.lift_up,hybrid_single_.pull_forward);
    const double remaining=action_c_direction_sign_x_*(action_c_absolute_target_x_m_-hybrid_position_window_.mean());
    const double rocking=clamp(action_c_direction_sign_x_*std::max(0.,-hybrid_contact_position_[2])*
        (std::sin(s.pitch)-std::sin(hybrid_pitch_start_)),0.,.03);
    const bool horizontal=hybrid_single_.phase==SinglePass::Slide ||
        hybrid_single_.phase==SinglePass::SlipProbe;
    const bool pressure_driven_pull=
        hybrid_single_.phase==SinglePass::Pull ||
        hybrid_single_.phase==SinglePass::FrontCatch;
    // A Motion-3 pull is useful only until the task endpoint has been reached.
    // Previously this guard existed only for the sliding probe, so a short
    // lift/pull crossed a 20-mm goal and kept the rear anchor exhausted until
    // roll recovery or a fall.  Regrip at the powered endpoint; the later
    // deflated, settled measurement rejects transient rocking and
    // automatically schedules another stroke if retained progress is short.
    const bool powered_goal_reached=(horizontal || pressure_driven_pull) &&
        speed_valid &&
        remaining<=.001-std::max(rocking,hybrid_settling_loss_);
    if(command.regrip || powered_goal_reached) {
      if(powered_goal_reached)
        ROS_INFO("Hybrid powered endpoint reached during motion %d: remaining %.2f mm, settling allowance %.2f mm; restoring four-arm support",
            int(hybrid_single_.phase),1000.*remaining,
            1000.*std::max(rocking,hybrid_settling_loss_));
      if(command.contact_limited_regrip)
        ROS_INFO("Hybrid contact-limited front regrip: both front arms pressure-loaded and flexing with stable roll; retaining bounded pull until rear support is restored");
      beginHybridRegrip(now,false);
      hybrid_regrip_bottom_=hybrid_single_config_.bottom_kpa;
    }
  }
  if(hybrid_phase_==2) {
    inchworm_pressure_=hybrid_grasp_pressure_;
    // Regrip has two deliberately separate stages.  While any rotor thrust
    // remains, keep the rear pair at a compliant 10-kPa hold and slew thrust
    // to zero.  Only after all rotor commands are zero may the rear pair
    // return to the full grasp target.  This avoids recreating the old rear
    // contact while the front pair is still completing its horizontal pull.
    const bool transfer_posture_unsafe=
        std::abs(s.roll+.3*s.roll_rate)>hybrid_single_config_.soft_roll &&
        std::abs(s.roll_rate)>.02 &&
        s.roll*s.roll_rate>0.;
    if(hybrid_regrip_anchor_phase_==0 &&
       !hybrid_regrip_thrust_zero_latched_)
      for(size_t arm=0;arm<ARM_COUNT;++arm)
        inchworm_thrust_[arm]=slew(inchworm_thrust_[arm],0.,0.,2.,dt);
    // While thrust is being removed, roll feedback may preserve/add only the
    // measured restoring front side, bounded well below the gait limit.  It
    // never causes the rear pressure to jump to full grasp prematurely.
    Pair transfer_thrust{},transfer_moment{};
    for(size_t j=0;j<2;++j) {
      transfer_thrust[j]=inchworm_thrust_[arms[j]];
      transfer_moment[j]=geometry(j,s.bend[j],s.roll,s.pitch).roll_moment;
    }
    const bool transfer_roll_worsening=transfer_posture_unsafe &&
        std::abs(s.roll_rate)>.05;
    const Pair verified_transfer_moment=verifiedPairRollMoments(
        transfer_moment,hybrid_single_config_.reverse_geometry_roll_moment);
    const Pair balanced=!hybrid_regrip_thrust_zero_latched_ &&
        transfer_roll_worsening && !hybrid_terminal_settle_?
        addRestoringRollSupport(transfer_thrust,verified_transfer_moment,
            s.roll,s.roll_rate,std::min(2.,hybrid_config_.thrust_max)):
        (!hybrid_regrip_thrust_zero_latched_ && transfer_roll_worsening?
            reduceAdverseRoll(transfer_thrust,verified_transfer_moment,
                s.roll,s.roll_rate):transfer_thrust);
    for(size_t j=0;j<2;++j)
      // The transfer controller is allowed to add only the restoring side,
      // up to the independent 2-N cap above.  A zero rise rate here silently
      // disabled that path: after a guarded front catch both rotor commands
      // decayed while rear pressure was still near zero, so roll grew during
      // the very handoff intended to make recovery safe.
      inchworm_thrust_[arms[j]]=slew(transfer_thrust[j],balanced[j],
          hybrid_single_config_.identification_tracking_rise,2.,dt);
    // A normal Motion-3 catch may be recognized at a load-bearing pressure
    // below the requested 30-kPa grasp.  The real front anchor reaches its
    // full pressure here, in the outer regrip controller.  Preserve the rear
    // low-pressure state until BOTH front chambers are measured ready and the
    // old front thrust has fallen to zero, then deliver one bounded positive
    // rear lift.  Only after its falling edge do all four chambers return to
    // the full grasp target.  A completed task or a fault never requests this
    // optional progress event.
    const bool front_thrust_off=
        std::abs(inchworm_thrust_[arms[0]])<=.01 &&
        std::abs(inchworm_thrust_[arms[1]])<=.01;
    const double front_anchor_pressure=std::min(
        measured_pressure_[arms[0]],measured_pressure_[arms[1]]);
    double rear_anchor_pressure=0.;
    for(size_t arm=0;arm<ARM_COUNT;++arm)
      if(!inchworm_front_[arm])
        rear_anchor_pressure=std::max(
            rear_anchor_pressure,measured_pressure_[arm]);
    const bool terminal_posture_safe=
        std::abs(s.roll+.2*s.roll_rate)<
            hybrid_single_config_.catch_roll_angle &&
        std::abs(s.roll_rate)<=
            2.*hybrid_single_config_.catch_roll_rate;
    const bool terminal_front_pressure_ready=
        front_anchor_pressure>=base_pressure_kpa_-
            hybrid_single_config_.pull_terminal_front_pressure_tolerance;
    const bool terminal_rear_pressure_ready=
        rear_anchor_pressure<=hybrid_single_config_.rear_motion_pressure_cap+
            hybrid_single_config_.pull_terminal_rear_pressure_tolerance;
    const bool terminal_front_anchor_ready=front_thrust_off &&
        terminal_front_pressure_ready && terminal_rear_pressure_ready;
    bool terminal_lift_geometry_ready=true;
    for(size_t arm=0;arm<ARM_COUNT;++arm)
      if(!inchworm_front_[arm]) {
        const auto force=hybridRotorForce(arm,measuredArmBendAngle(arm),
            s.roll,s.pitch);
        terminal_lift_geometry_ready=terminal_lift_geometry_ready &&
            std::isfinite(force.up) && std::abs(force.up)>=.20;
      }
    bool terminal_sequence_active=
        hybrid_regrip_terminal_rear_lift_requested_ &&
        !hybrid_single_.terminal_rear_lift_complete;
    if(terminal_sequence_active && front_thrust_off &&
       !hybrid_single_.terminal_rear_lift_started) {
      hybrid_single_.terminal_anchor_wait_elapsed+=dt;
      if(!terminal_posture_safe) {
        hybrid_single_.terminal_rear_lift_cancelled=true;
      } else if(terminal_front_anchor_ready &&
                terminal_lift_geometry_ready) {
        hybrid_single_.terminal_rear_lift_started=true;
        hybrid_single_.terminal_rear_lift_elapsed=0.;
        hybrid_single_.terminal_rear_lift_command=0.;
        for(size_t arm=0;arm<ARM_COUNT;++arm)
          if(!inchworm_front_[arm]) {
            const auto force=hybridRotorForce(arm,
                measuredArmBendAngle(arm),s.roll,s.pitch);
            hybrid_terminal_rear_lift_sign_[arm]=
                force.up>=0. ? 1. : -1.;
          }
      } else if(terminal_front_anchor_ready &&
                !terminal_lift_geometry_ready) {
        hybrid_single_.terminal_rear_lift_cancelled=true;
      } else if(hybrid_single_.terminal_anchor_wait_elapsed>=
          hybrid_single_config_.pull_terminal_anchor_wait_timeout) {
        hybrid_single_.terminal_rear_lift_cancelled=true;
      } else if(terminal_front_pressure_ready) {
        // The front anchor is ready. Keep exhausting the rear pair until both
        // rear chambers are actually compliant; a front-pressure stall must
        // not cancel this deliberate rear-pressure wait.
        hybrid_single_.terminal_front_anchor_stall_time=0.;
      } else if(!hybrid_single_.terminal_front_anchor_wait_started) {
        hybrid_single_.terminal_front_anchor_wait_started=true;
        hybrid_single_.terminal_front_anchor_best_pressure=
            front_anchor_pressure;
        hybrid_single_.terminal_front_anchor_stall_time=0.;
      } else if(front_anchor_pressure>=
          hybrid_single_.terminal_front_anchor_best_pressure+
          hybrid_single_config_.pull_terminal_front_pressure_progress_kpa) {
        hybrid_single_.terminal_front_anchor_best_pressure=
            front_anchor_pressure;
        hybrid_single_.terminal_front_anchor_stall_time=0.;
      } else {
        hybrid_single_.terminal_front_anchor_stall_time+=dt;
        if(hybrid_single_.terminal_front_anchor_stall_time>=
           hybrid_single_config_.pull_terminal_front_pressure_stall_seconds)
          hybrid_single_.terminal_rear_lift_cancelled=true;
      }
    }
    if(terminal_sequence_active &&
       hybrid_single_.terminal_rear_lift_started &&
       (!terminal_posture_safe ||
        front_anchor_pressure<base_pressure_kpa_-
            hybrid_single_config_.pull_terminal_front_pressure_tolerance ||
        rear_anchor_pressure>hybrid_single_config_.rear_motion_pressure_cap+
            hybrid_single_config_.pull_terminal_rear_pressure_tolerance))
      hybrid_single_.terminal_rear_lift_cancelled=true;
    const bool terminal_drive=terminal_sequence_active &&
        hybrid_single_.terminal_rear_lift_started &&
        !hybrid_single_.terminal_rear_lift_cancelled &&
        hybrid_single_.terminal_rear_lift_elapsed<
            hybrid_single_config_.pull_terminal_rear_lift_duration;
    hybrid_single_.terminal_rear_lift_command=slew(
        hybrid_single_.terminal_rear_lift_command,
        terminal_drive?
            hybrid_single_config_.pull_terminal_rear_lift_thrust:0.,
        hybrid_single_config_.pull_terminal_rear_lift_rise,
        hybrid_single_config_.pull_terminal_rear_lift_fall,dt);
    if(terminal_drive &&
       hybrid_single_.terminal_rear_lift_command>.05)
      hybrid_single_.terminal_rear_lift_elapsed=std::min(
          hybrid_single_config_.pull_terminal_rear_lift_duration,
          hybrid_single_.terminal_rear_lift_elapsed+dt);
    if(terminal_sequence_active &&
       (hybrid_single_.terminal_rear_lift_started ||
        hybrid_single_.terminal_rear_lift_cancelled) &&
       !terminal_drive &&
       hybrid_single_.terminal_rear_lift_command<=.05) {
      hybrid_single_.terminal_rear_lift_command=0.;
      hybrid_single_.terminal_rear_lift_complete=true;
      hybrid_regrip_terminal_rear_lift_requested_=false;
      terminal_sequence_active=false;
      // The event command can cross the 0.05-N completion threshold one
      // control tick before the cached rear commands reach the stricter
      // 0.01-N regrip threshold.  Leaving that sub-PWM command cached keeps
      // `thrust_off` false forever, so the rear chambers remain at the
      // compliant 10-kPa hold until the global regrip timeout.  The front
      // pair was already verified off before this event could start and the
      // rear falling edge has completed, therefore zero all four cached
      // commands and latch the support-transfer boundary explicitly.
      inchworm_thrust_.fill(0.);
      hybrid_regrip_thrust_zero_latched_=true;
    }
    if(terminal_sequence_active) {
      for(size_t arm=0;arm<ARM_COUNT;++arm)
        if(!inchworm_front_[arm]) {
          inchworm_pressure_[arm]=hybrid_single_config_.rear_low_kpa;
          // Positive/negative motor direction is not synonymous with upward
          // force once a compliant arm has flexed beyond 90 degrees. Freeze
          // the sign selected from the measured rotor direction at event
          // entry, while retaining one common magnitude for zero roll bias.
          inchworm_thrust_[arm]=hybrid_terminal_rear_lift_sign_[arm]*
              hybrid_single_.terminal_rear_lift_command;
        }
    }
    // Once zero thrust has been observed, keep it latched at zero while the
    // rear pair fills from the compliant hold to full grasp.  A previous
    // full-grip roll trim could reappear during this fill and reopen the rear
    // chambers, trapping an otherwise successful stroke in regrip.
    if((hybrid_regrip_anchor_phase_>=1 ||
        hybrid_regrip_thrust_zero_latched_) &&
       !terminal_sequence_active) {
      for(size_t arm=0;arm<ARM_COUNT;++arm)
        inchworm_thrust_[arm]=slew(inchworm_thrust_[arm],0.,0.,3.,dt);
    }
    const bool thrust_off=std::all_of(inchworm_thrust_.begin(),
        inchworm_thrust_.end(),[](double value){return std::abs(value)<=.01;});
    if(thrust_off && !terminal_sequence_active)
      hybrid_regrip_thrust_zero_latched_=true;
    if(!hybrid_regrip_thrust_zero_latched_) {
      const double rear_hold=terminal_sequence_active?
          hybrid_single_config_.rear_low_kpa:clamp(
              hybrid_single_config_.regrip_rear_hold_pressure,
              hybrid_single_config_.rear_low_kpa,base_pressure_kpa_);
      for(size_t arm=0;arm<ARM_COUNT;++arm)
        if(!inchworm_front_[arm]) inchworm_pressure_[arm]=rear_hold;
    }
    // Regrip, bottom exhaust, endpoint capture and optional PPG dwell are
    // deliberately separate latches.  The former implementation required
    // pressure, bend, bottom, pitch and mocap stability to remain true in the
    // same long window.  One benign sensor fluctuation therefore restarted
    // the whole sequence even though all four arms were already supporting
    // the body.
    const bool regrip_roll_safe=
        std::abs(s.roll)<hybrid_regrip_roll_emergency_rad_;
    const bool mechanical_regrip_ready=all_pressure && thrust_off &&
        regrip_roll_safe;
    if(!hybrid_regrip_complete_latched_) {
      if(!mechanical_regrip_ready) {
        hybrid_regrip_anchor_since_=clock;
      } else if(clock-hybrid_regrip_anchor_since_>=.30) {
        hybrid_regrip_complete_latched_=true;
        hybrid_regrip_anchor_phase_=1;
        hybrid_regrip_anchor_since_=clock;
        hybrid_settled_window_.reset();
        hybrid_bottom_window_.reset();
        hybrid_endpoint_pitch_window_.reset();
        ROS_INFO("Hybrid regrip complete: all arm pressures ready, rotor thrust zero and roll safe for 0.30 s; beginning bottom exhaust");
      }
    }

    if(hybrid_regrip_complete_latched_ &&
       !hybrid_bottom_deflated_latched_) {
      // Preserve the verified four-arm grasp while removing the bottom
      // support.  Actual pressure, rather than command age or pitch, owns the
      // completion latch.
      hybrid_regrip_bottom_=std::max(0.,hybrid_regrip_bottom_-
          hybrid_regrip_bottom_ramp_kpa_s_*dt);
      if(bottomPressureReady(s.bottom,0.,
          action_c_bottom_deflation_tolerance_kpa_)) {
        hybrid_bottom_deflated_latched_=true;
        hybrid_regrip_anchor_phase_=5;
        hybrid_settled_window_.reset();
        ROS_INFO("Hybrid bottom deflation complete: measured %.1f kPa <= %.1f kPa; acquiring one stable mocap endpoint",
            s.bottom,action_c_bottom_deflation_tolerance_kpa_);
      }
    }

    // Once regrip has been latched, ignore small pressure ripple.  Clear the
    // sequence only for a real loss of support, excessive roll, or renewed
    // rotor actuation.  These are also the only events that restart an
    // optional PPG wait after endpoint capture.
    bool arm_support_lost=false;
    for(size_t arm=0;arm<ARM_COUNT;++arm) {
      const double support_floor=std::max(0.,hybrid_grasp_pressure_[arm]-
          action_c_pressure_max_deviation_kpa_);
      arm_support_lost=arm_support_lost ||
          !std::isfinite(measured_pressure_[arm]) ||
          measured_pressure_[arm]<support_floor;
    }
    const bool thrust_reappeared=std::any_of(inchworm_thrust_.begin(),
        inchworm_thrust_.end(),[](double value){return std::abs(value)>.05;});
    const bool true_endpoint_safety_loss=hybrid_regrip_complete_latched_ &&
        (arm_support_lost ||
         std::abs(s.roll)>=hybrid_regrip_roll_emergency_rad_ ||
         thrust_reappeared);
    if(true_endpoint_safety_loss) {
      ROS_WARN("Hybrid endpoint safety state lost; reacquiring regrip (pressure_lost=%d roll=%.1f deg thrust_reappeared=%d)",
          arm_support_lost,s.roll*180./M_PI,thrust_reappeared);
      hybrid_regrip_complete_latched_=false;
      hybrid_regrip_thrust_zero_latched_=false;
      hybrid_bottom_deflated_latched_=false;
      hybrid_endpoint_captured_latched_=false;
      hybrid_end_mean_valid_=false;
      hybrid_ppg_wait_since_=-1.;
      hybrid_regrip_anchor_phase_=0;
      hybrid_regrip_anchor_since_=clock;
      hybrid_settled_window_.reset();
    }

    if(!true_endpoint_safety_loss &&
       !hybrid_endpoint_captured_latched_ &&
       endpoint_measurement_ready) {
      hybrid_end_mean_x_=hybrid_settled_window_.mean();
      hybrid_end_mean_valid_=true;
      hybrid_endpoint_captured_latched_=true;
      hybrid_ppg_wait_since_=clock;
      ROS_INFO("Hybrid endpoint fixed at x %.6f after independent regrip and bottom-deflation latches",
          hybrid_end_mean_x_);
    }

    // Do not relinquish control on the first quiet sample after a terminal
    // stroke.  Four-arm regrasp can unload the bottom cleanly and then drift
    // several degrees over the following seconds.  Keeping phase 2 alive at
    // the already-safe zero-thrust/zero-bottom boundary allows the boundary
    // trend detector above to request posture recovery if (and only if) roll
    // keeps worsening.  Intermediate strokes retain the fast PPG-only wait.
    const double remaining_at_endpoint=hybrid_end_mean_valid_ ?
        action_c_direction_sign_x_*(
            action_c_absolute_target_x_m_-hybrid_end_mean_x_) :
        std::numeric_limits<double>::infinity();
    const bool target_reached_at_endpoint=
        hybrid_endpoint_captured_latched_ && remaining_at_endpoint<=.001;
    const double endpoint_hold_sec=target_reached_at_endpoint ?
        std::max(ppg_settle_sec_,hybrid_terminal_posture_monitor_sec_) :
        ppg_settle_sec_;
    const bool ppg_settle_ready=hybrid_endpoint_captured_latched_ &&
        hybrid_ppg_wait_since_>=0. &&
        clock-hybrid_ppg_wait_since_>=endpoint_hold_sec;
    if(hybrid_endpoint_captured_latched_ && !ppg_settle_ready)
      ROS_INFO_THROTTLE(1.0,
          "Hybrid endpoint hold: safe outputs for %.1f/%.1f s (terminal posture monitor=%d, PPG dwell %.1f s)",
          clock-hybrid_ppg_wait_since_,endpoint_hold_sec,
          target_reached_at_endpoint,ppg_settle_sec_);
    // The first quiet endpoint can still creep backwards while the terminal
    // posture monitor owns the robot.  Re-measure the retained position at
    // the end of that monitor instead of declaring success from the stale
    // pre-hold sample.  If more than the 1-mm goal tolerance was lost, the
    // ordinary continuation below schedules another measured stroke.
    const bool terminal_position_ready=!target_reached_at_endpoint ||
        hybrid_position_window_.stable(clock);
    if(ppg_settle_ready && target_reached_at_endpoint &&
       !terminal_position_ready)
      ROS_INFO_THROTTLE(1.0,
          "Hybrid terminal position is still settling; retaining safe zero-output ownership before final goal validation");
    if(ppg_settle_ready && target_reached_at_endpoint &&
       terminal_position_ready) {
      const double monitored_endpoint=hybrid_position_window_.mean();
      if(std::isfinite(monitored_endpoint)) {
        const double previous_endpoint=hybrid_end_mean_x_;
        hybrid_end_mean_x_=monitored_endpoint;
        hybrid_end_mean_valid_=true;
        const double monitored_remaining=action_c_direction_sign_x_*(
            action_c_absolute_target_x_m_-hybrid_end_mean_x_);
        if(monitored_remaining>.001)
          ROS_WARN("Hybrid terminal hold retained %.2f mm less progress than its first endpoint; %.2f mm remains, so locomotion will continue",
              1000.*action_c_direction_sign_x_*(
                  previous_endpoint-hybrid_end_mean_x_),
              1000.*monitored_remaining);
      }
    }
    if(ppg_settle_ready && terminal_position_ready) {
      const bool terminal=hybrid_terminal_settle_;
      const bool retry=hybrid_terminal_retry_;
      const std::string terminal_reason=hybrid_terminal_reason_;
      finishHybridLocomotion(now,!terminal,true);
      if(terminal) {
        hybrid_terminal_settle_=false;
        hybrid_terminal_retry_=false;
        hybrid_identification_settle_=false;
        if(!retry) {
          stopInchwormToGrasp(now,terminal_reason.c_str());return;
        }
        const double remaining=action_c_direction_sign_x_*(
            action_c_absolute_target_x_m_-hybrid_end_mean_x_);
        if(remaining<=.001) {
          hybrid_hold_pressure_=hybrid_retain_grasp_pressure_;
          stopInchwormToGrasp(now,"hybrid target reached during guarded trial");return;
        }
        ROS_WARN("Hybrid guarded trial retained %.2f mm; automatically retrying toward the same goal instead of waiting for a new command: %s",
            1000.*hybrid_result_.progress,terminal_reason.c_str());
        startInchwormCycle(now);return;
      }
      const double remaining=action_c_direction_sign_x_*(action_c_absolute_target_x_m_-hybrid_end_mean_x_);
      if(remaining<=.001) {
        hybrid_hold_pressure_=hybrid_retain_grasp_pressure_;
        stopInchwormToGrasp(now,"hybrid target reached");return;
      }
      ROS_INFO("Hybrid single-pass reuse (no new identification): remaining %.2f mm; slip probes %u lift probes %u",
          1000.*remaining,hybrid_single_.slip_entries,hybrid_single_.lift_probe_entries);
      startInchwormCycle(now);return;
    }
    // The timeout protects only the powered-to-pneumatic support transfer.
    // Once mechanical regrip is latched, a slow bottom exhaust, mocap capture
    // or fixed PPG interval must not turn a safe trial into GRASP.
    if(!hybrid_regrip_complete_latched_ &&
       (now-state_start_).toSec()>hybrid_config_.regrip_timeout) {
      stopInchwormToGrasp(now,"hybrid regrip timeout");return;
    }
  }
  publishPressureTarget(inchworm_pressure_,std::numeric_limits<double>::infinity());
  publishBottomTarget(s.bottom_command);
  publishCorrectionThrust(inchworm_thrust_);
  // Versioned diagnostics: v3 replaces the obsolete contact-limited
  // calibration flag with the two measured target-reach thrusts.
  std_msgs::Float32MultiArray debug;
  double rear_command=0.;for(size_t arm=0;arm<ARM_COUNT;++arm)
    if(!inchworm_front_[arm]) rear_command+=.5*inchworm_pressure_[arm];
  double fx=0.,up=0.;for(size_t j=0;j<2;++j) {
    const auto f=geometry(j,s.bend[j],s.roll,s.pitch);
    fx+=std::max(0.,f.forward)*inchworm_thrust_[arms[j]];
    up+=std::max(0.,f.up)*inchworm_thrust_[arms[j]];
  }
  const auto trajectory_choice=hybrid_single_.trajectory.selected();
  debug.data={3.f,float(hybrid_phase_),float(hybrid_single_.phase),float(hybrid_single_.slip_entries),float(hybrid_single_.lift_probe_entries),
      float(hybrid_single_.ramp),float(fx),float(up),float((fx+up)>1e-6?fx/(fx+up):1.),
      float(rear_command),float(hybrid_single_config_.bottom_kpa),float(s.speed),
      float(s.bend[0]),float(s.bend[1]),float(s.rate[0]),float(s.rate[1]),
      float(inchworm_thrust_[arms[0]]),float(inchworm_thrust_[arms[1]]),
      float(inchworm_pressure_[arms[0]]),float(inchworm_pressure_[arms[1]]),
      float(command.angle_target[0]),float(command.angle_target[1]),float(hybrid_trial_x_),
      float(hybrid_start_mean_valid_),float(pressure_pending),float(all_pressure),
      float(hybrid_single_.onset[0].samples),float(hybrid_single_.onset[1].samples),
      float(hybrid_single_.onset[0].thrust),float(hybrid_single_.onset[1].thrust),
      float(hybrid_single_.onset[0].pressure),float(hybrid_single_.onset[1].pressure),
      float(hybrid_single_.phase==SinglePass::Lift && hybrid_single_.fast_approach[0]),
      float(hybrid_single_.phase==SinglePass::Lift && hybrid_single_.fast_approach[1]),
      float(hybrid_single_.calibrated),float(hybrid_single_.stroke_lifted),
      float(hybrid_single_.learned_lift_thrust),float(hybrid_single_.learned_ratio),
      float(hybrid_single_.trajectory.current),float(hybrid_single_.planned_mix),
      float(hybrid_single_.pressure_lead_seconds),float(hybrid_single_.powered_time),
      float(hybrid_single_.identifying_straight),
      float(hybrid_single_.active_bend_target[0]),float(hybrid_single_.active_bend_target[1]),
      float(hybrid_single_.active_extension_target[0]),float(hybrid_single_.active_extension_target[1]),
      float(input.rear_pressure),float(hybrid_single_.predicted_lift_completion_seconds),
      float(hybrid_single_.rear_vent_lead_seconds),
      float(hybrid_single_.front_target_latched),
      float(hybrid_single_.slope_overlap_active),
      float(hybrid_single_.lift_prefill_command[0]),
      float(hybrid_single_.lift_prefill_command[1]),
      float(input.directional_pitch),
      float(hybrid_single_.overlap_extension_latched),
      float(hybrid_single_.lift_target_reached[0]),
      float(hybrid_single_.lift_target_reached[1]),
      float(hybrid_single_.lift_angle_pressure_command[0]),
      float(hybrid_single_.lift_angle_pressure_command[1]),
      float(hybrid_result_.goal),
      float(hybrid_single_.predicted_stride),
      float(hybrid_single_.goal_scale),
      float(hybrid_single_.goal_extension_scale),
      float(hybrid_single_.goal_pull_envelope),
      float(hybrid_single_.goal_conditioned),
      float(hybrid_single_.stride_estimate_measured),
      float(hybrid_single_.identified_extension_thrust[0]),
      float(hybrid_single_.identified_extension_thrust[1]),
      float(hybrid_single_.observed_extension_by_arm[0]),
      float(hybrid_single_.observed_extension_by_arm[1]),
      float(hybrid_single_.identified_bend_target[0]),
      float(hybrid_single_.identified_bend_target[1]),
      float(hybrid_single_.onset_identified),
      float(hybrid_single_.onset_approach_ready[0]),
      float(hybrid_single_.onset_approach_ready[1]),
      float(hybrid_single_.roll_risk_time),
      float(hybrid_roll_rate_limit_since_>=0. ?
          clock-hybrid_roll_rate_limit_since_:0.),
      float(hybrid_single_.unload_event_detected),
      float(hybrid_single_.unload_evidence_time),
      float(hybrid_single_.unload_pitch_delta),
      float(hybrid_single_.unload_height_delta),
      float(hybrid_single_.predicted_unload_bend[0]),
      float(hybrid_single_.predicted_unload_bend[1]),
      float(hybrid_single_.unload_event_elapsed),
      float(hybrid_single_.unload_capture_started),
      float(hybrid_single_.unload_alignment_time),
      float(std::abs((hybrid_single_.reference[0]-s.bend[0])-
                     (hybrid_single_.reference[1]-s.bend[1]))),
      float(std::max(std::max(0.,-s.rate[0]),
                     std::max(0.,-s.rate[1]))),
      // Keep the complete outer-search command in the bag.  Candidate index
      // alone is not enough to reproduce or compare a run after the bounded
      // candidate table evolves.
      float(trajectory_choice.mix_offset),
      float(trajectory_choice.force_scale),
      float(trajectory_choice.unload_scale),
      float(trajectory_choice.rear_support_offset),
      float(command.rear_reverse_thrust),
      // Unified Motion-2/3 trajectory diagnostics (appended to preserve every
      // existing v3 field index used by bag-analysis scripts).
      float(hybrid_single_.trajectory_phase),
      float(hybrid_single_.trajectory_clock_scale),
      float(hybrid_single_.trajectory_angle_error[0]),
      float(hybrid_single_.trajectory_angle_error[1]),
      float(hybrid_single_.trajectory_rate_error[0]),
      float(hybrid_single_.trajectory_rate_error[1]),
      float(hybrid_single_.extension_thrust_trim[0]),
      float(hybrid_single_.extension_thrust_trim[1]),
      float(hybrid_single_.flex_pressure_trim[0]),
      float(hybrid_single_.flex_pressure_trim[1]),
      // Gyro/acceleration-aided trajectory prediction diagnostics.
      float(hybrid_single_.trajectory_rate_filtered[0]),
      float(hybrid_single_.trajectory_rate_filtered[1]),
      float(hybrid_single_.trajectory_acceleration[0]),
      float(hybrid_single_.trajectory_acceleration[1]),
      float(hybrid_single_.trajectory_predicted_bend[0]),
      float(hybrid_single_.trajectory_predicted_bend[1]),
      float(hybrid_single_.phase==SinglePass::Lift?
          hybrid_single_.extension_trajectory.acceleration[0]:
          hybrid_single_.flex_trajectory.acceleration[0]),
      float(hybrid_single_.phase==SinglePass::Lift?
          hybrid_single_.extension_trajectory.acceleration[1]:
          hybrid_single_.flex_trajectory.acceleration[1]),
      float(hybrid_single_.phase==SinglePass::Lift?
          hybrid_single_.extension_trajectory.observed_phase:
          hybrid_single_.flex_trajectory.observed_phase),
      float(hybrid_single_.phase==SinglePass::Lift?
          hybrid_single_.extension_trajectory.prediction_lag:
          hybrid_single_.flex_trajectory.prediction_lag),
      // Deep-windup diagnostics are appended so all established bag indices
      // remain unchanged.
      float(hybrid_single_.load_relief_detected),
      float(hybrid_single_.windup_pitch_target),
      float(hybrid_single_.windup_pitch_progress),
      float(hybrid_single_.windup_evidence_time),
      // Continuous rear-support transfer diagnostics.
      float(hybrid_single_.lift_rear_release_progress),
      float(hybrid_single_.rear_release_progress),
      float(hybrid_single_.rear_pressure_command),
      // Golden-time force path and contact-dependent rear support.
      float(hybrid_single_.golden_flex_progress),
      float(hybrid_single_.golden_desired_up),
      float(hybrid_single_.golden_desired_forward),
      float(hybrid_single_.rearPressureFloor(hybrid_single_config_)),
      float(hybrid_single_.stroke_peak_abs_gait_roll),
      float(hybrid_single_.stroke_min_rear_pressure),
      float(hybrid_single_.golden_window_active),
      // Motion-3 rear positive handoff pulse (appended; old field indices are
      // intentionally unchanged for existing rosbag analysis scripts).
      float(command.rear_positive_thrust),
      // Event-synchronized rear-positive diagnostics. The elapsed value is
      // delivered-force time, not wall time since the first request.
      float(hybrid_single_.rear_positive_front_envelope),
      float(hybrid_single_.rear_positive_pressure_envelope),
      float(hybrid_single_.rear_positive_elapsed),
      float(hybrid_single_.rear_positive_started),
      // Paired load-relief diagnostics (v3 indices 133--136).  They make it
      // possible to distinguish a missing arm event from a pair-mismatch
      // rejection in the next hardware bag without changing older indices.
      float(hybrid_single_.unload_extension_memory[0]),
      float(hybrid_single_.unload_extension_memory[1]),
      float(hybrid_single_.unload_pair_mismatch_measured),
      float(hybrid_single_config_.unload_pair_mismatch),
      // Unified rear-support diagnostics. These are appended so older bag
      // parsers keep all established indices.
      float(hybrid_single_.rear_support_prior),
      float(hybrid_single_.rear_support_nominal),
      float(hybrid_single_.rear_support_command),
      float(hybrid_single_.rear_support_safety_override),
      float(hybrid_single_.rear_support_slip_identified),
      float(hybrid_single_.rear_support_identified),
      // Early event and optimizer-identifiability diagnostics.  A rear
      // candidate is learnable only when the selected pressure/thrust reached
      // the plant without the inner roll loop replacing it.
      float(hybrid_single_.load_relief_candidate),
      float(hybrid_single_.rear_candidate_requested),
      float(hybrid_single_.rear_candidate_distinct),
      float(hybrid_single_.rear_candidate_realized),
      float(hybrid_single_.rear_candidate_realization_time),
      // Measured rear-angle feedback (append-only for bag compatibility).
      float(input.rear_bend[0]),float(input.rear_bend[1]),
      float(input.rear_rate[0]),float(input.rear_rate[1]),
      float(hybrid_single_.rear_grasp_reference[0]),
      float(hybrid_single_.rear_grasp_reference[1]),
      float(hybrid_single_.rear_release_by_arm[0]),
      float(hybrid_single_.rear_release_by_arm[1]),
      float(hybrid_single_.rear_release_pair),
      float(hybrid_single_.rear_release_target),
      float(hybrid_single_.rear_release_pair_rate),
      float(hybrid_single_.rear_release_mismatch),
      float(hybrid_single_.rear_release_balance_envelope),
      float(hybrid_single_.rear_angle_pressure_trim),
      float(hybrid_single_.rear_angle_thrust_trim),
      float(hybrid_single_.rear_angle_feedback_active),
      // Full-pressure front-anchor / terminal rear-lift diagnostics.
      float(hybrid_single_.terminal_rear_lift_started),
      float(hybrid_single_.terminal_rear_lift_cancelled),
      float(hybrid_single_.terminal_rear_lift_complete),
      float(hybrid_single_.terminal_rear_lift_elapsed),
      float(hybrid_single_.terminal_rear_lift_command),
      // Rear-release event diagnostics (append-only): a positive assist is
      // admitted only after this measured deficit timer reaches its limit.
      float(hybrid_single_.rear_release_deficit_time),
      float(hybrid_single_.rear_release_angle_achieved)};
  hybrid_diagnostic_pub_.publish(debug);
  ROS_INFO_THROTTLE(.5,
      "Hybrid golden-time: active=%d u=%.2f, desired Fup/Fx %.2f/%.2f N, rear +/reverse %.2f/%.2f N env front/pressure %.2f/%.2f delivered %.2f s, terminal rear lift start/cancel/done=%d/%d/%d %.2f N %.2f s, support prior/nominal/effective %.2f/%.2f/%.2f (safety %.2f), rear floor/cmd/measured %.1f/%.1f/%.1f kPa, rear release [%.1f %.1f]/%.1f deg achieved=%d deficit %.2f s rate %.1f deg/s mismatch %.1f deg trim P/T %+.1f/%+.2f, peak stroke roll %.1f deg",
      hybrid_single_.golden_window_active,
      hybrid_single_.golden_flex_progress,
      hybrid_single_.golden_desired_up,
      hybrid_single_.golden_desired_forward,
      command.rear_positive_thrust,command.rear_reverse_thrust,
      hybrid_single_.rear_positive_front_envelope,
      hybrid_single_.rear_positive_pressure_envelope,
      hybrid_single_.rear_positive_elapsed,
      hybrid_single_.terminal_rear_lift_started,
      hybrid_single_.terminal_rear_lift_cancelled,
      hybrid_single_.terminal_rear_lift_complete,
      hybrid_single_.terminal_rear_lift_command,
      hybrid_single_.terminal_rear_lift_elapsed,
      hybrid_single_.rear_support_prior,
      hybrid_single_.rear_support_nominal,
      hybrid_single_.rear_support_command,
      hybrid_single_.rear_support_safety_override,
      hybrid_single_.rearPressureFloor(hybrid_single_config_),
      hybrid_single_.rear_pressure_command,input.rear_pressure,
      hybrid_single_.rear_release_by_arm[0]*180./M_PI,
      hybrid_single_.rear_release_by_arm[1]*180./M_PI,
      hybrid_single_.rear_release_target*180./M_PI,
      hybrid_single_.rear_release_angle_achieved,
      hybrid_single_.rear_release_deficit_time,
      hybrid_single_.rear_release_pair_rate*180./M_PI,
      hybrid_single_.rear_release_mismatch*180./M_PI,
      hybrid_single_.rear_angle_pressure_trim,
      hybrid_single_.rear_angle_thrust_trim,
      hybrid_single_.stroke_peak_abs_gait_roll*180./M_PI);
  ROS_INFO_THROTTLE(.5,"Hybrid single-pass: phase=%d motion=%d T [%.2f %.2f] P [%.1f %.1f] rear cmd/measured %.1f/%.1f bottom %.1f, bend [%.1f %.1f] deg ref [%.1f %.1f] deg dExt [%.1f %.1f]/[%.1f %.1f] deg safetyBend [%.1f %.1f] deg unload=%d evidence=%.2f s bodyMemory=%.2f s dPitch=%.1f deg dZ=%.1f mm predictBend=[%.1f %.1f] deg identify=%d armLimit=[%d %d] prefill [%.1f %.1f] angleHoldP [%.1f %.1f] kPa dirPitch=%.1f deg rate [%.1f %.1f] deg/s, rear vent prediction %.2f/%.2f s, Fx %.2f Fup %.2f N; slip/lift probes %u/%u",
      hybrid_phase_,int(hybrid_single_.phase),inchworm_thrust_[arms[0]],inchworm_thrust_[arms[1]],
      inchworm_pressure_[arms[0]],inchworm_pressure_[arms[1]],rear_command,input.rear_pressure,
      hybrid_single_config_.bottom_kpa,
      s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
      hybrid_single_.reference[0]*180./M_PI,hybrid_single_.reference[1]*180./M_PI,
      (hybrid_single_.reference[0]-s.bend[0])*180./M_PI,
      (hybrid_single_.reference[1]-s.bend[1])*180./M_PI,
      hybrid_single_.active_extension_target[0]*180./M_PI,
      hybrid_single_.active_extension_target[1]*180./M_PI,
      hybrid_single_.active_bend_target[0]*180./M_PI,
      hybrid_single_.active_bend_target[1]*180./M_PI,
      hybrid_single_.unload_event_detected,
      hybrid_single_.unload_evidence_time,
      hybrid_single_.unload_body_memory_time,
      hybrid_single_.unload_pitch_delta*180./M_PI,
      hybrid_single_.unload_height_delta*1000.,
      hybrid_single_.predicted_unload_bend[0]*180./M_PI,
      hybrid_single_.predicted_unload_bend[1]*180./M_PI,
      hybrid_single_.identifying_straight,
      hybrid_single_.lift_target_reached[0],
      hybrid_single_.lift_target_reached[1],
      hybrid_single_.lift_prefill_command[0],
      hybrid_single_.lift_prefill_command[1],
      hybrid_single_.lift_angle_pressure_command[0],
      hybrid_single_.lift_angle_pressure_command[1],
      input.directional_pitch*180./M_PI,
      s.rate[0]*180./M_PI,s.rate[1]*180./M_PI,
      hybrid_single_.predicted_lift_completion_seconds,
      hybrid_single_.rear_vent_lead_seconds,
      fx,up,hybrid_single_.slip_entries,hybrid_single_.lift_probe_entries);
  if(hybrid_phase_==1 &&
     (hybrid_single_.phase==hugmy::hybrid::SinglePass::Lift ||
      hybrid_single_.phase==hugmy::hybrid::SinglePass::Pull))
    ROS_INFO_THROTTLE(.5,
        "Hybrid bend trajectory: motion=%d path=%.2f observed=%.2f clock=%.2f target [%.1f %.1f] measured/predicted [%.1f %.1f]/[%.1f %.1f] deg, rate [%.1f %.1f] deg/s acceleration [%.1f %.1f] deg/s2, angle error [%.1f %.1f] deg, rate error [%.1f %.1f] deg/s, learned trim T [%.2f %.2f] N P [%.1f %.1f] kPa",
        int(hybrid_single_.phase),hybrid_single_.trajectory_phase,
        hybrid_single_.phase==hugmy::hybrid::SinglePass::Lift?
            hybrid_single_.extension_trajectory.observed_phase:
            hybrid_single_.flex_trajectory.observed_phase,
        hybrid_single_.trajectory_clock_scale,
        command.angle_target[0]*180./M_PI,command.angle_target[1]*180./M_PI,
        s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
        hybrid_single_.trajectory_predicted_bend[0]*180./M_PI,
        hybrid_single_.trajectory_predicted_bend[1]*180./M_PI,
        hybrid_single_.trajectory_rate_filtered[0]*180./M_PI,
        hybrid_single_.trajectory_rate_filtered[1]*180./M_PI,
        hybrid_single_.trajectory_acceleration[0]*180./M_PI,
        hybrid_single_.trajectory_acceleration[1]*180./M_PI,
        hybrid_single_.trajectory_angle_error[0]*180./M_PI,
        hybrid_single_.trajectory_angle_error[1]*180./M_PI,
        hybrid_single_.trajectory_rate_error[0]*180./M_PI,
        hybrid_single_.trajectory_rate_error[1]*180./M_PI,
        hybrid_single_.extension_thrust_trim[0],
        hybrid_single_.extension_thrust_trim[1],
        hybrid_single_.flex_pressure_trim[0],
        hybrid_single_.flex_pressure_trim[1]);
  if(hybrid_single_.phase==hugmy::hybrid::SinglePass::Lift &&
     hybrid_single_.identifying_straight)
    ROS_INFO_THROTTLE(.5,
        "Hybrid initial extension-rate FB: filtered [%.1f %.1f] deg/s, target %.1f +/- %.1f deg/s, thrust slew [%.2f %.2f] N/s",
        hybrid_single_.identification_extension_rate_filtered[0]*180./M_PI,
        hybrid_single_.identification_extension_rate_filtered[1]*180./M_PI,
        hybrid_single_config_.extension_rate*180./M_PI,
        hybrid_single_config_.identification_rate_deadband*180./M_PI,
        hybrid_single_.identification_rate_command_slope[0],
        hybrid_single_.identification_rate_command_slope[1]);
  if(hybrid_single_.phase==hugmy::hybrid::SinglePass::SlipProbe)
    ROS_INFO_THROTTLE(.5,
        "Hybrid slip common ramp: scale %.2f pressure_wait=%d extension_brake=%d error [%.1f %.1f] kPa, extension remaining [%.1f %.1f] deg",
        hybrid_single_.slip_ramp_scale,
        hybrid_single_.slip_waiting_for_pressure,
        hybrid_single_.slip_extension_braking,
        hybrid_single_.slip_pressure_error[0],
        hybrid_single_.slip_pressure_error[1],
        hybrid_single_.slip_extension_remaining[0]*180./M_PI,
        hybrid_single_.slip_extension_remaining[1]*180./M_PI);
}
