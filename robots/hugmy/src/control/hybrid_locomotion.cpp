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
  force.pitch_moment=cross(lever,direction)[1];return force;
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
  hybrid_preinflate_measured_=false;hybrid_terminal_settle_=false;
  hybrid_terminal_retry_=false;
  hybrid_terminal_reason_.clear();
  hybrid_position_window_.reset();hybrid_position_window_.duration=.5;
  hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
  hybrid_endpoint_mocap_window_.reset();hybrid_endpoint_mocap_window_.duration=.8;
  hybrid_endpoint_pitch_window_.reset();hybrid_endpoint_pitch_window_.duration=3.;
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
  action_c_roll_trend_.reset();
  for(auto& rate:hybrid_sim_rate_) rate=hugmy::hybrid::BendCoordinateRate();
  inchworm_pressure_=hybrid_grasp_pressure_;inchworm_thrust_.fill(0.);
  publishCorrectionThrust(inchworm_thrust_);
  publishPressureTarget(inchworm_pressure_,std::numeric_limits<double>::infinity());
  publishBottomDirection(-1.);
  enterState(ACTION_C_PREPARE,now);
  ROS_INFO("Hybrid C/D single pass: bottom %.1f kPa; slip once, lift once, pressure-followed pull; rear sliding release/grasp %.2f/%.2f s; target %.1f mm",hybrid_single_config_.bottom_kpa,hybrid_single_config_.rear_low_seconds,hybrid_single_config_.rear_grasp_seconds,1000.*hybrid_result_.goal);
}

void AttitudePressureController::beginHybridRegrip(const ros::Time& now,bool prediction_limited)
{
  hybrid_regrip_thrust_=inchworm_thrust_;
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
  hybrid_regrip_x_=hybrid_position_window_.ready(now.toSec())?
      hybrid_position_window_.mean():locomotionPosition()[0];
  hybrid_prediction_limited_=prediction_limited;
  hybrid_phase_=2;hybrid_ready_since_=-1.;
  hybrid_pressure_ready_gate_.reset();
  hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
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
  hybrid_terminal_reason_=reason;
  beginHybridRegrip(now,false);
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
  // A stable pre-inflation/post-deflation pair is the authoritative retained
  // displacement, including recovery. The powered-segment clamp remains only
  // as a conservative fallback when a comparable endpoint is unavailable.
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
  // If the complete lift/flex/regrip stroke was safe but did not retain useful
  // forward travel, strengthen only the next pressure-driven pull.  Do not
  // relearn the lift threshold or increase its extension thrust: the real bag
  // showed that lift already moved the body backward while Pull recovered
  // most of that distance.  Stable endpoint measurement makes this an inner
  // response adaptation, independent of the optional outer trajectory search.
  const double previous_pull=hybrid_single_.pull_forward;
  if(completed && safe && measured && !hybrid_single_.goal_conditioned &&
     hybrid_single_.adaptPullAfterMeasuredStroke(
         hybrid_result_.progress,hybrid_single_config_))
    ROS_INFO("Hybrid low-progress pull adaptation: retained %.2f mm < %.2f mm; horizontal pull %.2f -> %.2f N (lift force unchanged)",
        1000.*hybrid_result_.progress,
        1000.*hybrid_single_config_.pull_progress_threshold,
        previous_pull,hybrid_single_.pull_forward);
  // The first identification reach is a different waveform and must not
  // teach the repeated-stroke model. Subsequent shortened/strengthened
  // strokes remain useful because record() retains their exact scale.
  // Missing stable endpoints are diagnostics, not positive training samples.
  if(hybrid_single_.calibrated && hybrid_single_.strokes>1) {
    hybrid_single_.trajectory.enabled=hybrid_search_.enabled;
    hybrid_single_.trajectory.record(hybrid_single_.context_bend,hybrid_single_.context_roll,
        hybrid_single_.context_pitch,hybrid_result_.progress,hybrid_result_.seconds,
        hybrid_result_.effort,hybrid_result_.recovery_seconds,measured,
        hybrid_result_.completed,safe,hybrid_single_.goal_scale);
    const auto choice=hybrid_single_.trajectory.selected();
    ROS_INFO("Hybrid measured trajectory optimization: candidate %u, mix offset %.2f force scale %.2f, observations %zu; retained %.2f mm",
        hybrid_single_.trajectory.current,choice.mix_offset,choice.force_scale,
        hybrid_single_.trajectory.samples.size(),1000.*hybrid_result_.progress);
  }
  ROS_INFO("Hybrid C/D result: completed=%d safe=%d retained %.2f mm / %.2f s, effort %.3f N2s, recovery %.2f s, J %.6f, bottom %.1f kPa m_min %.2f",
      hybrid_result_.completed,safe,1000.*hybrid_result_.progress,hybrid_result_.seconds,hybrid_result_.effort,
      hybrid_result_.recovery_seconds,hybrid_result_.score(),hybrid_result_.candidate.bottom,hybrid_result_.candidate.minimum_ratio);
  ROS_INFO("Hybrid C/D evaluation: stable_deflated_endpoints=%d bottom_changed=%d start_mean_x=%.6f end_mean_x=%.6f bottom=%.2f kPa; %s",
      measured,hybrid_bottom_changed_,hybrid_start_mean_valid_?hybrid_trial_x_:std::numeric_limits<double>::quiet_NaN(),
      hybrid_end_mean_valid_?hybrid_end_mean_x_:std::numeric_limits<double>::quiet_NaN(),bottom_pressure_kpa_,
      measured?"pre-inflation/post-deflation endpoint means used":
          "no stable deflated endpoint pair; sample excluded");
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
  // First record the grasp with an empty bottom, then inflate for motion.
  // Regrip/terminal evaluation always deflates again before scoring.
  s.bottom_command=((hybrid_phase_==0 && !hybrid_preinflate_measured_) ||
      hybrid_phase_==2)?0.:hybrid_single_config_.bottom_kpa;
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
  if(!s.fresh || dt<=0 || dt>.1 || clock<hybrid_task_start_) {
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
  if(std::abs(s.roll)>=hybrid_config_.roll_limit ||
      roll_rate_limit_confirmed) {
    hybrid_roll_rate_limit_since_=-1.;
    requestActionCPostureRecovery(now,"hybrid hard roll/rate limit");return;
  }
  if(!hybrid_terminal_settle_ && hybrid_run_start_>=0 &&
     action_c_direction_sign_x_*(locomotion_position[0]-hybrid_run_x_)<=-action_c_abort_reverse_travel_m_) {
    beginHybridTerminalSettle(now,"hybrid reverse-travel limit; score retained endpoint and retry");return;
  }
  if(!pressure_pending) hybrid_planner_.model.observe(s,hybrid_config_,equilibrium);
  hugmy::hybrid::State safety_state=s;
  auto safety_config=hybrid_config_;
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
  }
  if(run && !Planner::valid(safety_state,safety_config,pressure_pending)) {
    ROS_WARN("Hybrid guard details: motion %d bend [%.1f %.1f] start [%.1f %.1f] floor [%.1f %.1f] deg, rate [%.2f %.2f], pitch excursion %.1f deg",
        int(hybrid_single_.phase),s.bend[0]*180./M_PI,s.bend[1]*180./M_PI,
        safety_state.start_bend[0]*180./M_PI,safety_state.start_bend[1]*180./M_PI,
        safety_state.floor[0]*180./M_PI,safety_state.floor[1]*180./M_PI,
        s.rate[0],s.rate[1],(s.pitch-s.pitch_start)*180./M_PI);
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
  if(action_c_recovery_gate_.update(settled && std::abs(s.roll)>action_c_posture_recovery_trigger_rad_ &&
      action_c_roll_trend_.worsening,clock,correction_trigger_confirm_sec_)) {
    requestActionCPostureRecovery(now,"hybrid worsening residual roll at settled boundary");return;
  }
  const bool posture_ready=std::abs(s.roll)<=action_c_posture_recovery_trigger_rad_ ||
      (settled && action_c_roll_trend_.ready && !action_c_roll_trend_.worsening);
  const bool quiet=std::abs(s.roll_rate)<.1 && posture_ready;
  const bool off=std::all_of(inchworm_thrust_.begin(),inchworm_thrust_.end(),[](double t){return std::abs(t)<=.01;});
  const bool bottom_ready=bottomPressureReady(
      s.bottom,s.bottom_command,boundary_bottom_tolerance);
  // Compare like with like: both displacement endpoints use an empty bottom.
  // The inflated preparation boundary is still settled before motion, but is
  // deliberately not used as the displacement origin.
  if(hybrid_phase_!=1 && all_pressure && off && quiet && bottom_ready) {
    hybrid_settled_window_.add(mocap_stamp_.toSec(),locomotion_position[0]);
    hybrid_bottom_window_.add(measured_bottom_pressure_stamp_.toSec(),s.bottom);
    hybrid_endpoint_pitch_window_.add(imu_stamp_.toSec(),s.pitch);
  } else {hybrid_settled_window_.reset();hybrid_bottom_window_.reset();hybrid_endpoint_pitch_window_.reset();}
  const bool measurement_ready=hybrid_settled_window_.stable(clock) &&
      hybrid_bottom_window_.stable(clock,1.,1.) && hybrid_endpoint_pitch_window_.stable(clock,.004,.002);
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
  input.height=mocap_position_[2];input.pitch_rate=body_rate[1];
  input.directional_pitch=action_c_direction_sign_x_*s.pitch;
  input.base_pressure=base_pressure_kpa_;input.speed_valid=speed_valid;
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
  SinglePassOutput command;command.bottom=hybrid_single_config_.bottom_kpa;
  if(hybrid_phase_==0) {
    inchworm_thrust_.fill(0.);inchworm_pressure_=hybrid_grasp_pressure_;
    if(measurement_ready) {
      if(!hybrid_preinflate_measured_) {
        hybrid_trial_x_=hybrid_settled_window_.mean();
        hybrid_result_.goal=std::min(hybrid_config_.stroke,
            std::max(.0001,action_c_direction_sign_x_*(
                action_c_absolute_target_x_m_-hybrid_trial_x_)));
        hybrid_start_mean_valid_=true;hybrid_preinflate_measured_=true;
        hybrid_settled_window_.reset();hybrid_bottom_window_.reset();
        hybrid_endpoint_pitch_window_.reset();
        action_c_boundary_settle_gate_.reset();
        ROS_INFO("Hybrid pre-inflation endpoint measured at x %.6f; retained goal %.2f mm; inflating bottom to %.1f kPa",
            hybrid_trial_x_,1000.*hybrid_result_.goal,
            hybrid_single_config_.bottom_kpa);
      } else {
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
        ROS_INFO("Hybrid single-pass motion begins: pre-inflation x %.6f, inflated x %.6f; goal %.2f mm, predicted stride %.2f mm (%s), goal scale %.2f conditioned=%d; slip probes %u, lift probes %u",
            hybrid_trial_x_,hybrid_settled_window_.mean(),
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
    const std::array<unsigned,2> previous_samples{{hybrid_single_.onset[0].samples,hybrid_single_.onset[1].samples}};
    command=hybrid_single_.update(input,hybrid_config_,hybrid_single_config_,hybrid_planner_.model,geometry);
    if(previous_phase==SinglePass::VentFront && hybrid_single_.phase==SinglePass::Lift)
      hybrid_start_bend_=s.bend;
    if(command.recover_roll) {
      requestActionCPostureRecovery(now,
          hybrid_single_.phase==SinglePass::StopForLift
              ? "hybrid residual roll before front vent"
              : "hybrid predicted outward roll during lift");
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
    inchworm_thrust_.fill(0.);inchworm_pressure_.fill(command.rear_pressure);
    for(size_t j=0;j<2;++j) {inchworm_thrust_[arms[j]]=command.thrust[j];inchworm_pressure_[arms[j]]=command.pressure[j];}
    for(size_t j=0;j<2;++j) if(hybrid_single_.onset[j].samples!=previous_samples[j])
      ROS_INFO("Hybrid extension onset arm %zu: %.2f N at %.1f kPa, reference %.1f deg; samples %u, next approach margin %.2f N",
          arms[j],hybrid_single_.onset[j].thrust,hybrid_single_.onset[j].pressure,
          hybrid_single_.onset[j].bend*180./M_PI,hybrid_single_.onset[j].samples,hybrid_single_config_.onset_margin_n);
    if(!previous_onset_identified && hybrid_single_.onset_identified)
      ROS_INFO("Hybrid paired extension-onset identification complete: front arms [%zu %zu] %.2f/%.2f N; final bend trajectory calibration remains %s",
          arms[0],arms[1],hybrid_single_.onset[0].thrust,
          hybrid_single_.onset[1].thrust,
          hybrid_single_.calibrated?"complete":"pending");
    if(!previous_bend_calibrated && hybrid_single_.calibrated)
      ROS_INFO("Hybrid final bend trajectory calibration complete: measured absolute targets [%.1f %.1f] deg, required peak thrust [%.2f %.2f] N; paired onset identification %s",
          hybrid_single_.identified_bend_target[0]*180./M_PI,
          hybrid_single_.identified_bend_target[1]*180./M_PI,
          hybrid_single_.identified_extension_thrust[0],
          hybrid_single_.identified_extension_thrust[1],
          hybrid_single_.onset_identified?"complete":"pending");
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
    const bool horizontal=hybrid_single_.phase==SinglePass::Slide || hybrid_single_.phase==SinglePass::SlipProbe;
    if(command.regrip || (horizontal && speed_valid && remaining<=.001-std::max(rocking,hybrid_settling_loss_))) {
      if(command.contact_limited_regrip)
        ROS_INFO("Hybrid contact-limited front regrip: high pressure, safe stalled bends and stable roll confirmed; residual thrust removed");
      beginHybridRegrip(now,false);
      hybrid_regrip_bottom_=hybrid_single_config_.bottom_kpa;
    }
  }
  if(hybrid_phase_==2) {
    inchworm_pressure_=hybrid_grasp_pressure_;
    double rear_pressure=base_pressure_kpa_;
    for(size_t arm=0;arm<ARM_COUNT;++arm) if(!inchworm_front_[arm]) rear_pressure=std::min(rear_pressure,measured_pressure_[arm]);
    const double transfer=clamp((rear_pressure-std::max(0.,base_pressure_kpa_-6.))/4.,0.,1.);
    for(size_t arm=0;arm<ARM_COUNT;++arm) {
      inchworm_thrust_[arm]=slew(inchworm_thrust_[arm],
          std::min(inchworm_thrust_[arm],hybrid_regrip_thrust_[arm]*(1.-transfer)),0.,2.,dt);
    }
    // The pressure-driven transfer can last longer than the pull itself.
    // Keep reduction-only roll feedback during this handover; otherwise the
    // last (possibly unbalanced) pull command is held until rear pressure rises.
    // Never restart thrust or raise either rotor in a terminal regrasp.
    Pair transfer_thrust{},transfer_moment{};
    for(size_t j=0;j<2;++j) {
      transfer_thrust[j]=inchworm_thrust_[arms[j]];
      transfer_moment[j]=geometry(j,s.bend[j],s.roll,s.pitch).roll_moment;
    }
    const bool transfer_roll_worsening=std::abs(s.roll+.3*s.roll_rate)>
        hybrid_single_config_.soft_roll && std::abs(s.roll_rate)>.05 &&
        s.roll*s.roll_rate>0.;
    const Pair balanced=transfer_roll_worsening?
        reduceAdverseRoll(transfer_thrust,
            verifiedPairRollMoments(transfer_moment),s.roll,s.roll_rate):
        transfer_thrust;
    for(size_t j=0;j<2;++j)
      inchworm_thrust_[arms[j]]=slew(transfer_thrust[j],balanced[j],0.,2.,dt);
    if(measurement_ready) {
      hybrid_end_mean_x_=hybrid_settled_window_.mean();hybrid_end_mean_valid_=true;
      const bool terminal=hybrid_terminal_settle_;
      const bool retry=hybrid_terminal_retry_;
      const std::string terminal_reason=hybrid_terminal_reason_;
      finishHybridLocomotion(now,!terminal,true);
      if(terminal) {
        hybrid_terminal_settle_=false;
        hybrid_terminal_retry_=false;
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
    if((now-state_start_).toSec()>hybrid_config_.regrip_timeout) {
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
          clock-hybrid_roll_rate_limit_since_:0.)};
  hybrid_diagnostic_pub_.publish(debug);
  ROS_INFO_THROTTLE(.5,"Hybrid single-pass: phase=%d motion=%d T [%.2f %.2f] P [%.1f %.1f] rear cmd/measured %.1f/%.1f bottom %.1f, bend [%.1f %.1f] deg ref [%.1f %.1f] deg dExt [%.1f %.1f]/[%.1f %.1f] deg bendTarget [%.1f %.1f] deg identifyStraight=%d targetLatch=%d armLatch=[%d %d] slopeOverlap=%d prefill [%.1f %.1f] angleHoldP [%.1f %.1f] kPa dirPitch=%.1f deg rate [%.1f %.1f] deg/s, rear vent prediction %.2f/%.2f s, Fx %.2f Fup %.2f N; slip/lift probes %u/%u",
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
      hybrid_single_.identifying_straight,
      hybrid_single_.front_target_latched,
      hybrid_single_.lift_target_reached[0],
      hybrid_single_.lift_target_reached[1],
      hybrid_single_.slope_overlap_active,
      hybrid_single_.lift_prefill_command[0],
      hybrid_single_.lift_prefill_command[1],
      hybrid_single_.lift_angle_pressure_command[0],
      hybrid_single_.lift_angle_pressure_command[1],
      input.directional_pitch*180./M_PI,
      s.rate[0]*180./M_PI,s.rate[1]*180./M_PI,
      hybrid_single_.predicted_lift_completion_seconds,
      hybrid_single_.rear_vent_lead_seconds,
      fx,up,hybrid_single_.slip_entries,hybrid_single_.lift_probe_entries);
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
