#pragma once
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <hugmy/control/hybrid_trajectory_adaptation.h>

namespace hugmy { namespace hybrid {
// Sim bend is a sum of rotor-link hinge angles, not a single local gyro axis.
// Differentiate that SAME coordinate; never mix its angle with opposite gyro.
struct BendCoordinateRate {
  double stamp=-1.,bend=0.,rate=0.;
  double update(double angle,double time) {
    if(!std::isfinite(angle) || !std::isfinite(time)) {stamp=-1.;rate=0.;return rate;}
    if(time==stamp) return rate;
    const double dt=time-stamp;
    if(stamp<0. || dt<=0. || dt>.25) rate=0.;
    else {const double alpha=dt/(.06+dt);rate+=alpha*((angle-bend)/dt-rate);}
    stamp=time;bend=angle;return rate;
  }
};
// Use the signed moment arm, not lateral force as a proxy for roll moment.
// The latter changes sign depending on whether support is above/below rotor.
inline Pair reduceAdverseRoll(Pair thrust,const Pair& moment,double roll,double rate) {
  const double target=clamp(-.15*roll-.04*rate,-.025,.025);
  double excess=thrust[0]*moment[0]+thrust[1]*moment[1]-target;
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.005 && excess*moment[j]>0.) {
    const double reduction=std::min(thrust[j],excess/moment[j]);
    thrust[j]-=reduction;excess-=reduction*moment[j];
  }
  return thrust;
}
// During free extension, reducing the adverse rotor also removes the very
// force needed to extend that arm.  walk_sim_3 consequently learned arm 3's
// reduced output as its next search ceiling and could never reproduce the
// two-arm Action-C extension.  Preserve the requested extension force first:
// add the roll-helpful rotor up to the physical per-rotor limit, and only use
// the reduction-only allocator for any correction that cannot be supplied by
// that headroom.  Final slew, bend/rate and total-up-force guards still apply.
inline Pair supportExtensionAndRoll(Pair thrust,const Pair& moment,
    double roll,double rate,double limit) {
  const double target=clamp(-.15*roll-.04*rate,-.025,.025);
  double missing=target-(thrust[0]*moment[0]+thrust[1]*moment[1]);
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.005 && missing*moment[j]>0.) {
    const double addition=std::min(std::max(0.,limit-thrust[j]),missing/moment[j]);
    thrust[j]+=addition;missing-=addition*moment[j];
  }
  return reduceAdverseRoll(thrust,moment,roll,rate);
}
// Pair index 0 is body +Y (arms 0/1), and pair index 1 is body -Y
// (arms 3/2). The hardware-verified correction is therefore positive body
// roll -> pair 1 and negative body roll -> pair 0. Do not inherit the sign
// from the estimated contact lever: around a 104-deg bend that estimate
// changes sign and selected the opposite rotor in the real bags.
inline Pair verifiedPairRollMoments(const Pair& estimated) {
  return {{std::max(.005,std::abs(estimated[0])),
          -std::max(.005,std::abs(estimated[1]))}};
}
// Free extension owns both rotor demands. Roll feedback may add restoring
// authority when roll is actually growing, but it must never remove the
// lagging arm's synchronization demand. If addition is insufficient, the
// independent hard-roll handoff owns safety recovery.
inline Pair addRestoringRollSupport(Pair thrust,const Pair& moment,
    double roll,double rate,double limit) {
  const double target=clamp(-.15*roll-.04*rate,-.025,.025);
  double missing=target-(thrust[0]*moment[0]+thrust[1]*moment[1]);
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 && missing*moment[j]>0.) {
    const double addition=std::min(std::max(0.,limit-thrust[j]),missing/moment[j]);
    thrust[j]+=addition;missing-=addition*moment[j];
  }
  return thrust;
}
// One identification pass per operator task. Subsequent strokes reuse the
// measured envelope; they are locomotion, not repeated calibration trials.
struct SinglePassConfig {
  double slip_seconds=20., lift_seconds=20., pull_seconds=4., front_wait_seconds=8.;
  // Sliding needs both net travel and velocity evidence.  The leaky time
  // confirmation accepts intermittent stick-slip motion, while the separate
  // displacement condition prevents stationary mocap noise from passing.
  double slip_confirm_seconds=.3, slip_min_progress=.002, slip_min_speed=.0015;
  // Motion observed while bottom/rear pressure is merely settling is not a
  // rotor-driven sliding result.  Start measuring at actual rear release and
  // require a small, deliberate pair thrust before accepting the probe.
  double slip_min_thrust=.75;
  // A single angular-rate sample may be an impact or IMU spike.  Confirm the
  // soft roll brake briefly; absolute roll and the outer hard guard remain
  // independent safety limits.
  double soft_roll_confirm=.10;
  double pull_min_seconds=.5, front_catch_tolerance=.06981317;
  // Stable stroke-to-stroke adaptation strengthens only the horizontal pull.
  // Lift/extension force remains the measured contact-specific value.
  double pull_progress_threshold=.002, pull_adaptation_step=.5;
  double pull_adaptation_max=6., slip_seed_pull_max=3.;
  // Goal conditioning initially uses the configured nominal stride and is
  // replaced by a comparable retained measurement when available. Keep the
  // breakaway force intact, shorten the motion-2 reach, then taper the
  // motion-3 impulse while pressure completes the regrasp.
  double goal_scale_min=.25, goal_conditioning_start_ratio=.55;
  // A known baseline stroke is repeated while the goal is far away.  Only
  // when the whole remaining distance fits in one bounded stronger stroke do
  // we scale up to finish it, e.g. 12 mm remaining after an 8-mm baseline.
  double goal_progress_max_scale=1.5;
  // Preferred repeatable stride used to tile long-distance goals. A larger
  // measured stable stride is never reduced merely to match this value.
  double goal_preferred_stride=.010;
  double goal_extension_min_scale=.55;
  double goal_pull_taper_fraction=.15;
  // A real front contact need not reproduce the pre-lift free-space angle.
  // In FrontCatch, high pressure plus a safe, stationary bend is accepted as
  // contact-limited regrasp after a bounded confirmation interval.
  double catch_stall_confirm=.5, catch_stall_rate=.06;
  double catch_safe_margin=.0174533, catch_pressure_tolerance=3.;
  double catch_roll_rate=.08, catch_roll_prediction_tolerance=.01;
  double vent_seconds=30., vent_tolerance=5., tilt_target=.05236;
  double rear_low_seconds=2.0, rear_grasp_seconds=1.0;
  double rear_low_kpa=0., rear_vent_lead_margin=.10;
  // Predictive exhaust is still started before motion 3, but only when BOTH
  // front arms are in the last two degrees. Five degrees allowed one transient
  // arm latch to remove the rear anchor while its partner was still short.
  double rear_vent_start_margin=.03490659;
  // Rear pressure continues exhausting in Pull.  It is not allowed to keep
  // the already-extended front pair in the nonlinear Lift controller.
  double bottom_kpa=30., extension_rad=.13962634;
  // Zero is the mechanical straight coordinate. Forty-five degrees is the
  // absolute unloading target: it provides more reach than 60 deg without
  // continuing all the way to the unstable former 20-deg target.
  // The first motion-2 test is complete
  // only after BOTH measured rotor-side bend angles reach this target.  A
  // stalled arm is evidence that more thrust is needed, never an alternative
  // target: otherwise the test cannot identify the force needed to extend.
  double identification_bend=.78539816, identification_tolerance=.03490659;
  double identification_confirm=.06;
  // Independent hard excursion guard. The normal completion condition is the
  // absolute bend above; this limit must not silently replace 45 deg with a
  // start-relative target during an ordinary real-arm grasp.
  double identification_extension_limit=1.3962634;
  double identification_sync_deadband=.0523599, identification_sync_gain=10.;
  double identification_sync_assist_max=1.;
  double identification_tracking_start=.13962634;
  double identification_angle_kp=4., identification_rate_kd=.8;
  double identification_tracking_rise=4.;
  // The rotor-side IMU can produce a one-sample bend-rate spike during
  // breakaway. Above 45 deg, the first identification reach filters that rate
  // and closes a loop around extension_rate: thrust rises progressively more
  // slowly as the target rate is approached, holds inside a deadband, and is
  // reduced gently as soon as the filtered speed exceeds the band. The larger
  // fall limit is enabled only after continuously fast extension is confirmed.
  // Below 45 deg the rate feedback is disabled; measured angle alone owns the
  // final approach to the configured target. Reused reaches do not use this loop.
  double identification_rate_feedback_min_bend=.78539816;
  double identification_rate_filter_tau=.25;
  double identification_rate_confirm=.15;
  double identification_rate_deadband=.03490659;
  // Multiplying extension-rate error [rad/s] by this gain produces the
  // permitted thrust decrease rate [N/s]. Increase remains bounded by
  // lift_rise, so this cannot turn the first test into a thrust step.
  double identification_rate_control_gain=1.5;
  double identification_rate_control_fall=3.;
  double identification_bend_rate_limit=6.;
  double identification_return_bend_rate_limit=8.;
  double identification_return_pressure_slew=30.;
  double horizontal_bend=1.570796, pressure_gain=20.;
  double lift_pressure_gain=20.;
  double lift_rise=.7, reused_lift_rise=8.;
  // On an adverse longitudinal slope, do not wait for an almost-straight arm
  // before filling the front chambers.  Predict a small pressure lead, latch
  // a substantial forward reach, and let pressure flexion rotate the same
  // thrust continuously from lift toward pull.
  double lift_overlap_pitch_start=.08726646;
  double lift_overlap_pressure_start_ratio=.55;
  double lift_overlap_handoff_ratio=.60;
  double lift_overlap_min_reach_ratio=.50;
  double lift_overlap_prefill_kpa=3.;
  double lift_overlap_min_measured_kpa=2.;
  double lift_overlap_max_prediction_sec=.60;
  double lift_overlap_max_prediction_lead_ratio=.12;
  double lift_overlap_min_up_scale=.65;
  // A reused stroke rises quickly to the measured breakaway force, but the
  // zero-pressure arm has a sharp delayed extension.  Begin reducing thrust
  // before the measured target so pneumatic rear release cannot turn the
  // remaining actuator/mass momentum into a floor violation.
  double reused_lift_brake_margin=.06981317;
  // Reused motion 2 captures each arm separately.  Once measured extension
  // begins, a delay-compensated pressure ramp brakes the terminal reach.  A
  // reached arm remains pressure-held while its rotor thrust is reduced and
  // the other arm catches up. Full flexion pressure still belongs to Pull.
  double reused_lift_hold_pressure_max=10.;
  // Do not let a small early holding pressure cancel the reused free
  // extension. Begin pressure braking only after 70% measured/predicted reach.
  double reused_lift_pressure_start_ratio=.90;
  // The diagnostic sliding sweep is deliberately much slower than motion 2.
  // Rotor thrust must not outrun the pneumatic angle loop: pause the common
  // ramp while either measured chamber pressure is still following its last
  // command, and smoothly stop increasing near the requested extension.
  double probe_rise=.25, slip_thrust_max=5., slip_pressure_max=40.;
  double slip_pressure_tracking_tolerance=2.;
  double slip_extension_hold_margin=.03490659; // last 2 deg
  // Normal angle regulation freezes the common thrust instead of chopping one
  // rotor.  Reduction remains available only beyond this independent hard
  // overrun (or the existing absolute bend/attitude safety guards).
  double slip_hard_extension_margin=.05235988; // 3 deg past target
  // A hard slip-probe extension overrun is still a controlled diagnostic,
  // not an Action-C free-extension emergency. Reduce the COMMON command
  // slowly while pressure bends both arms back into the corridor. The old
  // generic 12-N/s per-arm cut produced 2--3 N sawteeth on the real arm even
  // though body roll remained near 2.5 deg.
  double slip_extension_brake_fall=.5;
  // The slip probe measures the present grasp; it must not become a second
  // free-extension test. A human-arm grasp can begin near [95, 125] deg. A
  // blind common 90-deg target then exhausts only the 125-deg arm while both
  // rotors rise toward 5 N, directly exciting roll. Permit only a small
  // per-arm extension from its measured start and retain most grasp pressure.
  // The dedicated Lift phase owns larger extension when sliding is ineffective.
  double slip_max_extension=.17453293; // 10 deg from each initial bend
  double slip_pressure_drop_max=10.;   // kPa below base grasp pressure
  double approach_rise=2., onset_margin_n=.6, onset_angle=.0174533;
  double onset_rate=.02, onset_confirm=.10, extension_rate=.20;
  double soft_roll=.139626, soft_pitch=.104720, lift_height=.003;
  bool valid() const {
    for(double x:{slip_seconds,lift_seconds,pull_seconds,front_wait_seconds,
        slip_confirm_seconds,slip_min_progress,slip_min_speed,slip_min_thrust,
        soft_roll_confirm,
        pull_min_seconds,
        front_catch_tolerance,pull_progress_threshold,pull_adaptation_step,
        pull_adaptation_max,slip_seed_pull_max,goal_scale_min,
        goal_conditioning_start_ratio,goal_progress_max_scale,
        goal_preferred_stride,
        goal_extension_min_scale,goal_pull_taper_fraction,
        catch_stall_confirm,catch_stall_rate,catch_safe_margin,
        catch_pressure_tolerance,catch_roll_rate,catch_roll_prediction_tolerance,
        vent_seconds,vent_tolerance,tilt_target,
        rear_low_seconds,rear_grasp_seconds,rear_low_kpa,rear_vent_lead_margin,
        rear_vent_start_margin,
        bottom_kpa,extension_rad,
        identification_bend,identification_tolerance,identification_confirm,
        identification_extension_limit,
        identification_sync_deadband,identification_sync_gain,
        identification_sync_assist_max,
        identification_tracking_start,identification_angle_kp,
        identification_rate_kd,identification_tracking_rise,
        identification_rate_feedback_min_bend,
        identification_rate_filter_tau,identification_rate_confirm,
        identification_rate_deadband,identification_rate_control_gain,
        identification_rate_control_fall,
        identification_bend_rate_limit,identification_return_bend_rate_limit,
        identification_return_pressure_slew,
        horizontal_bend,pressure_gain,lift_pressure_gain,lift_rise,reused_lift_rise,
        lift_overlap_pitch_start,lift_overlap_pressure_start_ratio,
        lift_overlap_handoff_ratio,lift_overlap_min_reach_ratio,
        lift_overlap_prefill_kpa,
        lift_overlap_min_measured_kpa,lift_overlap_max_prediction_sec,
        lift_overlap_max_prediction_lead_ratio,
        lift_overlap_min_up_scale,
        reused_lift_brake_margin,reused_lift_hold_pressure_max,
        reused_lift_pressure_start_ratio,
        probe_rise,slip_thrust_max,slip_pressure_max,
        slip_pressure_tracking_tolerance,slip_extension_hold_margin,
        slip_hard_extension_margin,slip_extension_brake_fall,
        slip_max_extension,slip_pressure_drop_max,approach_rise,
        onset_margin_n,onset_angle,onset_rate,onset_confirm,extension_rate,
        soft_roll,soft_pitch,lift_height}) if(!std::isfinite(x) || x<0.) return false;
    return slip_seconds>=1 && slip_seconds<=20 && lift_seconds>=1 && lift_seconds<=20 &&
      pull_seconds>=.5 && pull_seconds<=10 && front_wait_seconds>=1 && front_wait_seconds<=30 &&
      slip_confirm_seconds>=.2 && slip_confirm_seconds<=2. &&
      slip_min_progress>=.001 && slip_min_progress<=.02 &&
      slip_min_speed>=.0005 && slip_min_speed<=.02 &&
      slip_min_thrust>=.3 && slip_min_thrust<=2. &&
      soft_roll_confirm>=.04 && soft_roll_confirm<=.5 &&
      pull_min_seconds>=.5 && pull_min_seconds<=pull_seconds &&
      front_catch_tolerance>=.017453 && front_catch_tolerance<=.174533 &&
      pull_progress_threshold>=.001 && pull_progress_threshold<=.01 &&
      pull_adaptation_step>0. && pull_adaptation_step<=1. &&
      pull_adaptation_max>=1. && pull_adaptation_max<=12. &&
      slip_seed_pull_max>=1. && slip_seed_pull_max<=pull_adaptation_max &&
      goal_scale_min>=.1 && goal_scale_min<=.75 &&
      goal_conditioning_start_ratio>=.5 && goal_conditioning_start_ratio<=.98 &&
      goal_progress_max_scale>=1. && goal_progress_max_scale<=1.5 &&
      goal_preferred_stride>=.005 && goal_preferred_stride<=.03 &&
      goal_extension_min_scale>=.4 && goal_extension_min_scale<=1. &&
      goal_pull_taper_fraction>=.05 && goal_pull_taper_fraction<=.4 &&
      catch_stall_confirm>=.2 && catch_stall_confirm<=2. &&
      catch_stall_rate>=.01 && catch_stall_rate<=.2 &&
      catch_safe_margin>=0. && catch_safe_margin<=.087267 &&
      catch_pressure_tolerance>=1. && catch_pressure_tolerance<=8. &&
      catch_roll_rate>=.02 && catch_roll_rate<=.2 &&
      catch_roll_prediction_tolerance>=0. && catch_roll_prediction_tolerance<=.034907 &&
      vent_seconds>=2. && vent_seconds<=60. && vent_tolerance>0. && vent_tolerance<=5. &&
      tilt_target>=.017453 && tilt_target<=.10472 &&
      rear_low_seconds>=.2 && rear_low_seconds<=10 && rear_grasp_seconds>=.2 && rear_grasp_seconds<=10 &&
      rear_low_kpa<=6 && rear_vent_lead_margin<=1. &&
      rear_vent_start_margin>=.017453 && rear_vent_start_margin<=.2618 &&
      bottom_kpa<=30 && extension_rad>=.035 && extension_rad<=.262 &&
      identification_bend>=0. && identification_bend<=1.047198 &&
      identification_tolerance>0. && identification_tolerance<=.087267 &&
      identification_confirm>=.04 && identification_confirm<=.25 &&
      identification_extension_limit>=.349066 && identification_extension_limit<=1.745329 &&
      identification_sync_deadband>=.017453 && identification_sync_deadband<=.174533 &&
      identification_sync_gain>0. && identification_sync_gain<=30. &&
      identification_sync_assist_max>=.25 && identification_sync_assist_max<=2. &&
      identification_tracking_start>=.087266 && identification_tracking_start<=.785399 &&
      identification_angle_kp>0. && identification_angle_kp<=12. &&
      identification_rate_kd>=0. && identification_rate_kd<=1. &&
      identification_tracking_rise>=lift_rise && identification_tracking_rise<=6. &&
      identification_rate_feedback_min_bend>=.523599 &&
      identification_rate_feedback_min_bend<=1.047198 &&
      identification_rate_filter_tau>=.05 && identification_rate_filter_tau<=.5 &&
      identification_rate_confirm>=.05 && identification_rate_confirm<=.5 &&
      identification_rate_deadband>=.0087266 &&
      identification_rate_deadband<=.174533 &&
      identification_rate_deadband<extension_rate &&
      identification_rate_control_gain>=.1 &&
      identification_rate_control_gain<=10. &&
      identification_rate_control_fall>=.5 &&
      identification_rate_control_fall<=12. &&
      identification_bend_rate_limit>=3.14 && identification_bend_rate_limit<=6. &&
      identification_return_bend_rate_limit>=identification_bend_rate_limit &&
      identification_return_bend_rate_limit<=10. &&
      identification_return_pressure_slew>=10. && identification_return_pressure_slew<=80. &&
      horizontal_bend>=1.2 && horizontal_bend<=1.9 && pressure_gain>0 &&
      lift_pressure_gain>0 && lift_pressure_gain<=150. &&
      lift_rise>0 && lift_rise<=1 &&
      reused_lift_rise>=lift_rise && reused_lift_rise<=8. &&
      lift_overlap_pitch_start>=.0349066 && lift_overlap_pitch_start<=.2618 &&
      lift_overlap_pressure_start_ratio>=.2 && lift_overlap_pressure_start_ratio<.9 &&
      lift_overlap_handoff_ratio>lift_overlap_pressure_start_ratio &&
      lift_overlap_handoff_ratio<=1. && lift_overlap_min_reach_ratio>=.3 &&
      lift_overlap_min_reach_ratio<=lift_overlap_handoff_ratio &&
      lift_overlap_prefill_kpa>=2. &&
      lift_overlap_prefill_kpa<=20. && lift_overlap_min_measured_kpa>=.5 &&
      lift_overlap_min_measured_kpa<=lift_overlap_prefill_kpa &&
      lift_overlap_max_prediction_sec>=.1 && lift_overlap_max_prediction_sec<=1.2 &&
      lift_overlap_max_prediction_lead_ratio>=.05 &&
      lift_overlap_max_prediction_lead_ratio<=.3 &&
      lift_overlap_min_up_scale>=.4 && lift_overlap_min_up_scale<=1. &&
      reused_lift_brake_margin>=.017453 && reused_lift_brake_margin<=.10472 &&
      reused_lift_hold_pressure_max>=3. && reused_lift_hold_pressure_max<=20. &&
      reused_lift_pressure_start_ratio>=.4 &&
      reused_lift_pressure_start_ratio<=.9 &&
      approach_rise>=lift_rise && approach_rise<=3. && onset_margin_n>=.2 && onset_margin_n<=2. &&
      onset_angle>=.017453 && onset_angle<=.087267 && onset_rate>=.01 && onset_rate<=.2 &&
      onset_confirm>=.06 && onset_confirm<=.5 && extension_rate>=.05 && extension_rate<=.5 &&
      probe_rise>0 && probe_rise<=1 && slip_thrust_max>0 && slip_thrust_max<=6. &&
      slip_pressure_max>=30. && slip_pressure_max<=50. &&
      slip_pressure_tracking_tolerance>=.5 &&
      slip_pressure_tracking_tolerance<=8. &&
      slip_extension_hold_margin>=.017453 &&
      slip_extension_hold_margin<=.087267 &&
      slip_hard_extension_margin>=.017453 &&
      slip_hard_extension_margin<=.174533 &&
      slip_extension_brake_fall>=probe_rise &&
      slip_extension_brake_fall<=3. &&
      slip_max_extension>=.087266 && slip_max_extension<=.349066 &&
      slip_pressure_drop_max>=5. && slip_pressure_drop_max<=20. &&
      soft_roll>0 && soft_pitch>0 && lift_height>0;
  }
};
// A local breakaway observation, not a globally valid inverse dynamics model.
// One accepted sample per arm per stroke; no fixed round count. Context checks
// below prevent reusing a high-force threshold at a softer pressure/geometry.
struct ExtensionOnset {
  unsigned samples=0;
  double thrust=0.,pressure=0.,bend=0.;
  void observe(double t,double p,double a) {
    if(!std::isfinite(t)||!std::isfinite(p)||!std::isfinite(a)) return;
    const double alpha=samples && std::abs(p-pressure)<=8.?.25:1.;
    thrust+=alpha*(t-thrust);pressure+=alpha*(p-pressure);bend+=alpha*(a-bend);
    ++samples;
  }
  bool compatible(double p,double a) const {
    return samples && p>=pressure-3. && p<=pressure+8. && std::abs(a-bend)<.174533;
  }
};
struct SinglePassInput {
  State s;
  double dt=.02, x=0., height=0., pitch_rate=0., base_pressure=30.;
  // Retained distance still required from the pre-inflation endpoint.  It is
  // constant during one stroke so bladder rocking cannot shrink the goal.
  double remaining=std::numeric_limits<double>::infinity();
  double nominal_stride=.02;
  double goal_conditioning_start_ratio=.55;
  double goal_scale_min=.25, goal_progress_max_scale=1.5;
  double goal_preferred_stride=.010;
  // Positive is pitch in the requested travel direction.  A negative value
  // means the front direction is downhill in the body convention observed on
  // the real fixture and makes pure lift produce reverse travel.
  double directional_pitch=0.;
  // Maximum of the two measured rear pressures.  Using the slower side makes
  // the Lift -> Pull handoff wait until both rear supports are released.
  double rear_pressure=0.;
  bool speed_valid=false, front_contact=true, contact_available=false;
  bool rear_released=false, rear_grasped=false;
};
struct SinglePassOutput {
  Pair thrust{}, pressure{}, angle_target{};
  double rear_pressure=0., bottom=30., desired_up=0., desired_forward=0., ratio=1.;
  bool regrip=false, contact_limited_regrip=false;
  bool fault=false, recover_roll=false, rear_low=true;
};
class SinglePass {
 public:
  enum Phase { SlipProbe=0, Slide=1, Lift=2, Pull=3, FrontCatch=4, StopForLift=5, VentFront=6 };
  Phase phase=SlipProbe;
  bool slip_tested=false,lift_tested=false,sliding_effective=false,initialized=false;
  unsigned slip_entries=0,lift_probe_entries=0,strokes=0;
  double phase_time=0., rear_time=0., motion_time=0., stall_time=0., ready_time=0., probe_time=0.;
  double roll_risk_time=0.;
  Pair catch_stall_time{};
  double identification_ready_time=0.;
  double ramp=0., slip_thrust=0., pull_forward=0., lift_up=0., lift_force=0.;
  // Slip-probe diagnostics. A scale of zero means that the common thrust is
  // held, not reduced. Pressure error is measured command minus sensor value.
  double slip_ramp_scale=1.;
  Pair slip_pressure_error{},slip_extension_remaining{};
  bool slip_waiting_for_pressure=false,slip_extension_braking=false;
  double start_height=0., start_pitch=0., observed_extension=0., initial_x=0.;
  double slide_best_x=std::numeric_limits<double>::quiet_NaN();
  Pair observed_extension_by_arm{};
  // Per-arm absolute rotor-side bend reached during the first useful lift.
  // This, rather than a fixed excursion from the next starting pose, is the
  // nominal target reused by subsequent motion-2 strokes.
  Pair identified_bend_target{{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  // Diagnostic copy of the target actually used by the current Lift stroke.
  // This may differ from extension_rad after the first identified stroke, so
  // logging only the configured value hides an early Lift -> Pull handoff.
  Pair active_extension_target{};
  Pair active_bend_target{};
  bool identifying_straight=false;
  bool front_target_latched=false;
  std::array<bool,2> lift_target_reached{{false,false}};
  bool slope_overlap_active=false, overlap_extension_latched=false;
  Pair reference{}, minimum{}, pull_start{}, pressure_target{};
  Pair lift_prefill_command{};
  Pair lift_angle_pressure_command{};
  Pair lift_target_hold_thrust{};
  Pair overlap_peak_extension{};
  std::array<ExtensionOnset,2> onset;
  Pair lift_ramp{},onset_time{},onset_candidate_thrust{},onset_candidate_pressure{},onset_candidate_bend{};
  std::array<bool,2> onset_this_stroke{{false,false}},fast_approach{{false,false}};
  // Breakaway identification and final bend-trajectory calibration are
  // deliberately separate.  Both arms must be observed in a zero-pressure
  // Lift before either is allowed to use the next-stroke shortcut.
  bool onset_identified=false;
  std::array<bool,2> lift_onset_seen{{false,false}};
  std::array<bool,2> onset_approach_ready{{false,false}};
  std::array<bool,2> identification_tracking{{false,false}};
  Pair identification_hold_thrust{};
  Pair identification_extension_rate_filtered{};
  Pair identification_fast_rate_time{};
  // Diagnostic value: signed thrust slew selected by the first-pass angular
  // rate loop. Positive means continued search, negative means overspeed
  // braking. It remains zero for reused motion-2 strokes.
  Pair identification_rate_command_slope{};
  // The first test records two distinct quantities per arm: breakaway force
  // in onset[] and the largest force required before the absolute bend target
  // was measured. Later strokes may approach the latter quickly but still
  // retain measured-angle feedback and all safety guards.
  Pair identification_peak_thrust{};
  Pair identified_extension_thrust{{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  bool rear_low=true, release_seen=false,slip_measurement_started=false;
  bool calibrated=false, stroke_lifted=false;
  double powered_time=0., learned_lift_thrust=0., learned_ratio=.5;
  Pair grasp_reference{};
  Pair passive_extension_rate{};
  Pair context_bend{};
  double context_roll=0.,context_pitch=0.,planned_mix=1.,pressure_lead_seconds=0.;
  double predicted_stride=std::numeric_limits<double>::quiet_NaN();
  double desired_stride=std::numeric_limits<double>::quiet_NaN();
  double goal_scale=1.,goal_extension_scale=1.,goal_pull_envelope=1.;
  bool goal_conditioned=false,stride_estimate_measured=false;
  double predicted_lift_completion_seconds=std::numeric_limits<double>::infinity();
  double rear_vent_lead_seconds=0.;
  TrajectoryAdaptation trajectory;
  bool adaptPullAfterMeasuredStroke(double progress,const SinglePassConfig& k) {
    if(!stroke_lifted || !std::isfinite(progress) ||
       progress>=k.pull_progress_threshold ||
       pull_forward>=k.pull_adaptation_max) return false;
    const double previous=pull_forward;
    pull_forward=std::min(k.pull_adaptation_max,
        pull_forward+k.pull_adaptation_step);
    return pull_forward>previous+1e-9;
  }
  void reset() {*this=SinglePass();}
  void enter(Phase p,const SinglePassInput& in) {
    phase=p;phase_time=0.;ready_time=0.;motion_time=0.;stall_time=0.;roll_risk_time=0.;
    if(p==Pull || p==FrontCatch) catch_stall_time.fill(0.);
    if(p!=Lift) fast_approach.fill(false);
    if(p==StopForLift) {grasp_reference=in.s.bend;rear_low=false;stroke_lifted=false;}
    if(p==VentFront) {rear_low=false;grasp_reference=in.s.bend;}
    if(p==Slide) {slide_best_x=in.x;stall_time=0.;}
    if(p==Lift) {reference=in.s.bend;minimum=in.s.bend;start_height=in.height;start_pitch=in.s.pitch;
      active_extension_target.fill(0.);active_bend_target=in.s.bend;
      identifying_straight=!calibrated;
      front_target_latched=false;
      lift_target_reached.fill(false);
      slope_overlap_active=false;overlap_extension_latched=false;
      lift_prefill_command.fill(0.);
      lift_angle_pressure_command.fill(0.);
      lift_target_hold_thrust.fill(0.);
      overlap_peak_extension.fill(0.);
      pressure_lead_seconds=0.;
      identification_ready_time=0.;
      ramp=std::max(in.s.thrust[0],in.s.thrust[1]);pressure_target=in.s.command;
      lift_ramp=in.s.thrust;onset_time.fill(0.);onset_this_stroke.fill(false);
      fast_approach.fill(false);onset_approach_ready.fill(false);
      identification_tracking.fill(false);identification_hold_thrust.fill(0.);
      identification_extension_rate_filtered.fill(0.);
      identification_fast_rate_time.fill(0.);
      identification_rate_command_slope.fill(0.);
      identification_peak_thrust.fill(0.);
      for(size_t j=0;j<2;++j) passive_extension_rate[j]=std::max(0.,-in.s.rate[j]);
      for(size_t j=0;j<2;++j) if(!onset[j].compatible(in.s.pressure[j],reference[j])) onset_this_stroke[j]=false;
      powered_time=0.;
      predicted_lift_completion_seconds=std::numeric_limits<double>::infinity();
      rear_vent_lead_seconds=0.;
      if(!lift_tested) {++lift_probe_entries;lift_tested=true;}
      // A reused Action-C stroke rises much faster than the first sweep, but
      // motion 2 still needs the rear pair as its anchor.  Do not release the
      // rear merely because a calibration exists: that removed support at
      // exactly the fast-ramp edge and converted unequal front breakaway into
      // body roll.  The measured completion-time predictor below starts rear
      // exhaust only after front extension is visible and close enough to the
      // target for pressure to be low when motion 3 begins.
      rear_low=false;rear_time=0.;}
    if(p==Pull) {pull_start=in.s.bend;minimum=in.s.bend;rear_low=true;}
  }
  void begin(const SinglePassInput& in) {
    // A recovery/short goal must not silently restart an already attempted
    // identification pass. Keep its bounded observations for the next stroke.
    if(slip_entries>0) slip_tested=true;
    if(lift_probe_entries>0) lift_tested=true;
    initialized=true;++strokes;reference=in.s.bend;grasp_reference=reference;minimum=reference;start_pitch=in.s.pitch;
    stroke_lifted=false;
    context_bend=in.s.bend;context_roll=in.s.roll;context_pitch=in.s.pitch;
    goal_conditioned=false;goal_scale=goal_extension_scale=goal_pull_envelope=1.;
    predicted_stride=std::numeric_limits<double>::quiet_NaN();
    desired_stride=std::numeric_limits<double>::quiet_NaN();
    stride_estimate_measured=false;
    if(calibrated) {
      predicted_stride=trajectory.estimatedProgress(
          context_bend,context_roll,context_pitch);
      stride_estimate_measured=std::isfinite(predicted_stride);
    }
    // Before the first retained sample, use the configured nominal stroke as
    // a conservative scale reference. A later contact-matched measurement
    // replaces it. Identification reach stays full; only its pull is bounded.
    if(!stride_estimate_measured && std::isfinite(in.nominal_stride) &&
       in.nominal_stride>=.001) predicted_stride=in.nominal_stride;
    if(std::isfinite(predicted_stride) && predicted_stride>=.001 &&
       std::isfinite(in.remaining) && in.remaining>0.) {
      const double lower_deadband=clamp(
          in.goal_conditioning_start_ratio,.5,.98)*predicted_stride;
      desired_stride=predicted_stride;
      if(stride_estimate_measured) {
        desired_stride=TrajectoryAdaptation::planStride(in.remaining,
            predicted_stride,in.goal_preferred_stride,in.goal_scale_min,
            in.goal_progress_max_scale);
      } else if(in.remaining<lower_deadband) {
        // Final short stroke: match the remaining distance instead of taking
        // another complete baseline step.
        desired_stride=in.remaining;
      }
      goal_scale=clamp(desired_stride/predicted_stride,
          in.goal_scale_min,in.goal_progress_max_scale);
      goal_conditioned=std::abs(goal_scale-1.)>.02;
    }
    // Do not change the outer mix/force candidate in the same stroke that
    // distance scheduling changes extension/pull amplitude. This keeps the
    // two effects identifiable. Exact-repeat strokes remain available for
    // the local mix-first, force-second optimization.
    if(calibrated && !goal_conditioned)
      trajectory.select(context_bend,context_roll,context_pitch);
    pressure_target=in.s.command;initial_x=in.x;rear_time=0.;rear_low=true;
    release_seen=false;slip_measurement_started=false;
    onset_time.fill(0.);onset_this_stroke.fill(false);fast_approach.fill(false);
    ramp=std::max(in.s.thrust[0],in.s.thrust[1]);
    if(!slip_tested) {++slip_entries;enter(SlipProbe,in);}
    else if(sliding_effective) enter(Slide,in);
    else enter(StopForLift,in);
  }
  template<class Geometry>
  SinglePassOutput update(const SinglePassInput& in,const Config& c,
      const SinglePassConfig& k,const Model& model,Geometry geometry) {
    SinglePassOutput out;out.bottom=k.bottom_kpa;
    if(!initialized) begin(in);
    const auto& s=in.s;const double dt=in.dt;
    if(!k.valid() || !s.fresh || !std::isfinite(dt) || dt<=0 || dt>.1 ||
       !std::isfinite(in.rear_pressure) || !std::isfinite(in.directional_pitch))
      {out.fault=true;return out;}
    phase_time+=dt;
    Pair ux{},uz{},moment{},raw{},rise_limit{{c.thrust_rise,c.thrust_rise}};
    std::array<bool,2> lift_terminal_braking{{false,false}};
    double fx=0.,up=0.,extension=0.;
    for(size_t j=0;j<2;++j) {
      const auto f=geometry(j,s.bend[j],s.roll,s.pitch);
      ux[j]=f.forward;uz[j]=std::max(0.,f.up);
      moment[j]=f.roll_moment;
      if(!std::isfinite(f.forward) || !std::isfinite(f.up) || !std::isfinite(f.roll_moment) ||
         !std::isfinite(f.pitch_moment)) {out.fault=true;return out;}
      fx+=s.thrust[j]*std::max(0.,ux[j]);up+=s.thrust[j]*uz[j];
      minimum[j]=std::min(minimum[j],s.bend[j]);
      extension=std::max(extension,reference[j]-s.bend[j]);
    }
    const double predicted_roll=s.roll+.3*s.roll_rate;
    const bool roll_risk_sample=std::abs(predicted_roll)>k.soft_roll &&
      std::abs(s.roll_rate)>.05 && s.roll*s.roll_rate>0.;
    roll_risk_time=roll_risk_sample?
        std::min(k.soft_roll_confirm,roll_risk_time+dt):
        std::max(0.,roll_risk_time-2.*dt);
    const bool roll_brake=roll_risk_time+1e-9>=k.soft_roll_confirm;
    // Pitch is an intended gait coordinate: bottom inflation, free extension
    // and pressure-driven flexion all rotate the body.  Treating that motion
    // as a soft posture error cut Pull thrust from about 2 N to zero exactly
    // during the short unloaded interval.  Roll still receives this fast
    // brake, while the adapter's independent absolute pitch/rate guard keeps
    // ownership of genuinely unsafe pitch excursions.
    const bool posture_brake=roll_brake;
    // Sliding pressure and its absolute 90-deg target are a different load
    // case.  Learn the reusable free-extension threshold only in Lift, after
    // both front chambers have been exhausted.  This prevents one arm's
    // pressure-assisted slip onset from enabling a one-sided shortcut.
    if(phase==Lift) for(size_t j=0;j<2;++j) {
      const bool moving=reference[j]-s.bend[j]>=k.onset_angle && -s.rate[j]>=k.onset_rate;
      const bool learn=moving && s.thrust[j]>.3 && !posture_brake &&
          std::abs(s.roll_rate)<.1 && std::abs(in.pitch_rate)<.1;
      if(!onset_this_stroke[j]) {
        if(learn) {
          if(onset_time[j]==0.) {
            onset_candidate_thrust[j]=s.thrust[j];onset_candidate_pressure[j]=s.pressure[j];
            onset_candidate_bend[j]=s.bend[j];
          }
          onset_time[j]+=dt;
          if(onset_time[j]+1e-9>=k.onset_confirm &&
              onset_candidate_bend[j]-s.bend[j]>=.5*k.onset_rate*k.onset_confirm) {
            onset[j].observe(onset_candidate_thrust[j],onset_candidate_pressure[j],reference[j]);
            onset_this_stroke[j]=true;
            lift_onset_seen[j]=true;
          }
        } else onset_time[j]=0.;
      }
    }
    onset_identified=onset_identified ||
        (lift_onset_seen[0] && lift_onset_seen[1]);
    if(phase==SlipProbe || phase==Slide) {
      // Only horizontal motion cycles the rear pair. No pressure-ready gate
      // incorrectly interprets the intentional regrasp as a release fault.
      rear_time+=dt;
      if(rear_time>=(rear_low?k.rear_low_seconds:k.rear_grasp_seconds)) {
        rear_low=!rear_low;rear_time=0.;
      }
      // Keep two independent pieces of evidence.  A velocity trace alone can
      // be mocap noise, while net displacement alone can be fixture/hand
      // motion.  Requiring both accepts intermittent stick-slip travel but
      // never classifies a stationary robot as sliding.  The leaky timer also
      // avoids throwing away a real response because of one slow sample.
      // The initial reference includes bottom inflation and rear-valve
      // settling.  Reset it exactly once when the rear pair is measurably
      // released, so those passive motions cannot produce a low-thrust
      // "successful" slip such as the 0.26-N event in the real-arm bag.
      if(!release_seen && in.rear_released) {
        initial_x=in.x;motion_time=0.;stall_time=0.;
      }
      release_seen=release_seen || in.rear_released;
      const bool deliberate_thrust=
          std::max(s.thrust[0],s.thrust[1])>=k.slip_min_thrust;
      if(release_seen && deliberate_thrust && !slip_measurement_started) {
        // Discard any motion accumulated while the rotor pair was below the
        // deliberate test force.  This is the second half of excluding the
        // low-thrust pressure-settling transient from the slip result.
        initial_x=in.x;motion_time=0.;stall_time=0.;
        slip_measurement_started=true;
      }
      const double forward_progress=in.x-initial_x;
      const bool velocity_evidence=slip_measurement_started &&
          in.speed_valid &&
          s.speed>=k.slip_min_speed;
      motion_time=velocity_evidence?
          std::min(k.slip_confirm_seconds,motion_time+dt):
          std::max(0.,motion_time-dt);
      const bool moving=forward_progress>=k.slip_min_progress &&
          motion_time+1e-9>=k.slip_confirm_seconds;
      if(phase==SlipProbe && velocity_evidence)
        slip_thrust=std::max(s.thrust[0],s.thrust[1]);
      if(phase==SlipProbe && moving) {
        sliding_effective=true;slip_tested=true;enter(Slide,in);
      }
      if(phase==Slide) {
        // Once sliding has been detected, continue it only while it produces
        // NEW retained progress. MuJoCo/contact oscillation can alternate the
        // instantaneous speed sign forever around one position; the old
        // velocity-only timer interpreted each positive half-cycle as useful
        // locomotion and never handed control to motion 2.
        const double progress_epsilon=std::max(.001,.5*k.slip_min_progress);
        if(!std::isfinite(slide_best_x) ||
           in.x>=slide_best_x+progress_epsilon) {
          slide_best_x=in.x;
          stall_time=0.;
        } else {
          stall_time+=dt;
        }
      } else {
        stall_time=0.;
      }
      slip_ramp_scale=1.;
      slip_waiting_for_pressure=false;
      slip_extension_braking=false;
      for(size_t j=0;j<2;++j) {
        slip_pressure_error[j]=s.command[j]-s.pressure[j];
        const double measured_extension=std::max(0.,reference[j]-s.bend[j]);
        slip_extension_remaining[j]=k.slip_max_extension-measured_extension;
        if(std::abs(slip_pressure_error[j])>
            k.slip_pressure_tracking_tolerance)
          slip_waiting_for_pressure=true;
        // Slow both rotors together during the last part of the permitted
        // reach. At the target, hold the current thrust until pressure bends
        // the arm back into the corridor; never alternate one-arm cuts.
        slip_ramp_scale=std::min(slip_ramp_scale,clamp(
            slip_extension_remaining[j]/k.slip_extension_hold_margin,0.,1.));
        slip_extension_braking=slip_extension_braking ||
            slip_extension_remaining[j]<-k.slip_hard_extension_margin;
      }
      if(slip_waiting_for_pressure) slip_ramp_scale=0.;
      if(slip_extension_braking) {
        // One noisy/fast arm must not be chopped independently and then race
        // back toward the unchanged common ramp. Keep one common state and
        // let the already-commanded pressure do the bending.
        slip_ramp_scale=0.;
        ramp=std::max(0.,ramp-k.slip_extension_brake_fall*dt);
      }
      if(phase==SlipProbe && !posture_brake &&
         !slip_extension_braking && release_seen) {
        // The one-off horizontal sliding test has its own force ceiling.
        // Lift/pull may still use the independent physical maximum in Config.
        ramp=std::min(std::min(c.thrust_max,k.slip_thrust_max),
            ramp+k.probe_rise*slip_ramp_scale*dt);
        // Pneumatic settling is not search time. This preserves the intended
        // sweep up to 5 N without raising the force while pressure lags. The
        // independent 3x wall limit below still bounds a leaking chamber.
        if(!slip_waiting_for_pressure) probe_time+=dt;
      }
      if((phase==SlipProbe && (probe_time>=k.slip_seconds || phase_time>=3.*k.slip_seconds)) ||
          (phase==Slide && stall_time>=3.)) {
        slip_tested=true;sliding_effective=false;
        // No sustained slide was identified.  The last instantaneous force
        // is a lower bound on static friction, not a safe pressure-assisted
        // pull command.  Seed a moderate pull and let completed, retained
        // strokes adapt it; do not inherit the 5-N probe endpoint directly.
        pull_forward=std::max(1.,std::min(k.slip_seed_pull_max,fx));
        enter(StopForLift,in);
      }
      for(size_t j=0;j<2;++j) {
        // Approach horizontal only within a bounded excursion from THIS
        // contact pose.  In walk_arm_pink_7 the two starts were 95/127 deg;
        // demanding 90/90 exhausted only arm 3 and produced roll before the
        // actual Lift test. Here their safe targets become 90/117 deg.
        const double slip_angle_target=std::max(
            k.horizontal_bend,reference[j]-k.slip_max_extension);
        out.angle_target[j]=slip_angle_target;
        const double error=slip_angle_target-s.bend[j];
        const double correction=std::abs(error)>.0174533?
            error-std::copysign(.0174533,error):0.;
        // Sliding keeps the arm near the horizontal-force pose.  It may need
        // more than the 30-kPa grasp pressure, but owns a separate 40-kPa cap
        // and can never exceed the controller-wide arm-pressure limit.  Do
        // not fully release one arm during this diagnostic: larger extension
        // belongs to motion 2. If roll is already worsening, restore both
        // fronts toward grasp instead of continuing the angle transient.
        const double slip_pressure_limit=std::min(c.pressure_max,k.slip_pressure_max);
        const double slip_pressure_min=std::max(0.,
            in.base_pressure-k.slip_pressure_drop_max);
        pressure_target[j]=(posture_brake || slip_extension_braking)?
            std::max(pressure_target[j],in.base_pressure):
            clamp(pressure_target[j]+k.pressure_gain*correction*dt,
                slip_pressure_min,slip_pressure_limit);
        raw[j]=phase==Slide?slip_thrust:ramp;
      }
    }
    if(phase==StopForLift || phase==VentFront) {
      rear_low=false;
      raw.fill(0.);out.angle_target=grasp_reference;
      if(phase==StopForLift) {
        // Regain rear support before venting the front. Never dump pressure
        // under the high thrust inherited from the unsuccessful slip test.
        pressure_target.fill(std::min(in.base_pressure,c.pressure_max));
        const bool stopped_and_supported=
            std::max(s.thrust[0],s.thrust[1])<=.05 && in.rear_grasped;
        if(stopped_and_supported && std::abs(s.roll)>k.soft_roll) {
          // The slip probe can leave a residual roll although it did not
          // produce useful forward motion. Venting both front arms from that
          // biased state removes support first and makes Lift hand over to
          // recovery before thrust can extend either arm. Recover with all
          // arms grasping, then resume here without repeating the slip probe.
          out.recover_roll=true;
        } else if(stopped_and_supported) {
          enter(VentFront,in);
        }
      }
      if(phase==VentFront) {
        // Motion 2 starts with the rear pair still gripping at base pressure.
        // Only the front pair is exhausted in this preparation phase.
        rear_low=false;
        pressure_target.fill(0.);
        const bool empty=std::max(s.pressure[0],s.pressure[1])<=
                k.vent_tolerance &&
            std::max(s.thrust[0],s.thrust[1])<=.05;
        ready_time=empty?ready_time+dt:0.;
        if(ready_time>=.2) enter(Lift,in);
      }
      if(phase_time>=k.vent_seconds) out.fault=true;
      // A phase entry cannot also consume stale extension from the old
      // reference in this tick. Start the powered search on the next tick.
    } else if(phase==Lift) {
      // Keep the rear pair at grasp pressure through most of motion 2.  A
      // measured-time prediction below starts its exhaust just early enough
      // for the pair to be at low pressure when motion 3 begins.
      // A timeout/brake with negligible extension is not an identified lift.
      // Otherwise it teaches a 2-deg target which rear release alone can meet,
      // producing repeated unpowered "completed" strokes on a sticky fixture.
      const auto selected=trajectory.selected();
      identifying_straight=!calibrated;
      // The adverse-slope overlap is a locomotion trajectory variation, not
      // a calibration shortcut. It is available only after a complete
      // zero-pressure absolute-bend test has identified both arms.
      slope_overlap_active=slope_overlap_active ||
          (calibrated && -in.directional_pitch>=k.lift_overlap_pitch_start);
      const double minimum_mix=clamp(learned_ratio+selected.mix_offset,.1,.95);
      const double mix_scale=clamp((1.-minimum_mix)/std::max(.1,1.-learned_ratio),.6,1.2);
      extension=0.;
      identification_rate_command_slope.fill(0.);
      double current_minimum_extension=std::numeric_limits<double>::infinity();
      powered_time+=dt;
      // Update the passive reference first for BOTH arms.  Identification
      // tracking is pair-gated below; deciding it inside the per-arm loop let
      // the first arm enter absolute-angle feedback one control sample before
      // the second arm was even evaluated.
      for(size_t j=0;j<2;++j) {
        if(!onset_this_stroke[j] && s.bend[j]>reference[j]) {
          reference[j]=s.bend[j];
          minimum[j]=reference[j];
        }
      }
      if(identifying_straight) {
        for(size_t j=0;j<2;++j) {
          const double ext=reference[j]-s.bend[j];
          if(!identification_tracking[j] &&
             ext>=k.identification_tracking_start) {
            // Latch the measured breakaway force, but do not yet drive this
            // arm harder toward the absolute target.  It must hold support
            // while the lagging arm reaches the same regime.
            identification_tracking[j]=true;
            identification_hold_thrust[j]=std::max(s.thrust[j],lift_ramp[j]);
          }
        }
      }
      const bool identification_pair_tracking=identifying_straight &&
          identification_tracking[0] && identification_tracking[1];
      const double lift_search_rise=calibrated?k.reused_lift_rise:k.lift_rise;
      // Reuse the two measured breakaway forces as a PAIR.  A partial
      // one-arm observation is deliberately insufficient: accelerating only
      // that arm on the following stroke creates roll before the other arm
      // starts to extend.  Both arms first approach their own sub-onset force,
      // the early arm waits there, and only then do both resume the measured
      // extension search together.  This state is independent of the final
      // absolute-bend trajectory calibration below.
      Pair onset_approach_target{};
      bool onset_pair_context=onset_identified;
      for(size_t j=0;j<2;++j) {
        const double learned_reach=calibrated &&
            std::isfinite(identified_extension_thrust[j])?
            identified_extension_thrust[j]:onset[j].thrust;
        onset_approach_target[j]=std::max(0.,
            learned_reach-k.onset_margin_n);
        onset_pair_context=onset_pair_context &&
            onset[j].compatible(s.pressure[j],reference[j]);
        if(onset_pair_context &&
           s.thrust[j]+.02>=onset_approach_target[j])
          onset_approach_ready[j]=true;
      }
      if(!onset_pair_context) onset_approach_ready.fill(false);
      const bool onset_pair_approach_pending=onset_pair_context &&
          !(onset_approach_ready[0] && onset_approach_ready[1]) &&
          !posture_brake;
      for(size_t j=0;j<2;++j) {
        // Vent completion is a pressure condition, not proof that the arm has
        // reached its passive mechanical equilibrium.  In MuJoCo the arm can
        // continue flexing for several degrees after Lift begins.  Freezing
        // the reference at that instant made the later, genuine extension
        // appear smaller (or negative) and timed out the third stroke.  Until
        // a powered extension onset is confirmed, retain the most-flexed bend
        // as the physical start of this stroke.  This only moves the reference
        // in the flexion direction; it cannot manufacture extension progress.
        const double ext=reference[j]-s.bend[j];
        overlap_peak_extension[j]=std::max(overlap_peak_extension[j],ext);
        // The first sweep aims at the configured absolute unloading bend.
        // Later sweeps reuse the independently measured absolute bend of this
        // arm. A different grasp normally changes the necessary excursion;
        // the independent start-relative cap may choose a more-flexed final
        // pose when reaching the absolute target would require excessive travel.
        double nominal_bend_target=identifying_straight?k.identification_bend:
            identified_bend_target[j];
        if(!std::isfinite(nominal_bend_target) || nominal_bend_target<0. ||
           nominal_bend_target>=reference[j]-k.onset_angle) {
          // Compatibility fallback for an in-memory calibration made before
          // absolute targets were introduced. New calibrations never use it.
          const double learned=observed_extension_by_arm[j]>=k.onset_angle?
              observed_extension_by_arm[j]:std::max(k.extension_rad,observed_extension);
          nominal_bend_target=reference[j]-learned;
        }
        const double identified_extension=clamp(
            reference[j]-nominal_bend_target,k.onset_angle,
            k.identification_extension_limit);
        double nominal_extension_target=clamp(
            identified_extension*mix_scale,k.onset_angle,identified_extension);
        // Do not weaken the breakaway thrust: reduce only the requested reach
        // once a nominal retained stride is known. The first identification
        // remains full length, and a bounded minimum preserves unloading.
        if(!identifying_straight && goal_conditioned) {
          // Apply half of the progress ratio to reach and half to pull force.
          // Their product is approximately the requested stride ratio while
          // neither actuator receives the full change alone.
          // A short final stroke may stop at a more flexed absolute bend. A
          // larger requested stride never extends beyond the identified safe
          // absolute target; its extra demand is assigned to horizontal pull.
          goal_extension_scale=clamp(std::sqrt(goal_scale),
              k.goal_extension_min_scale,1.);
          nominal_extension_target=clamp(
              nominal_extension_target*goal_extension_scale,
              k.onset_angle,identified_extension);
        }
        // Only the slope-aware first reach hands over before the nominal
        // unloading calibration pose. Reused strokes retain their learned
        // absolute target while still receiving predictive prefill.
        const double handoff_ratio=slope_overlap_active && identifying_straight?
            k.lift_overlap_handoff_ratio:1.;
        const double arm_extension_target=nominal_extension_target*handoff_ratio;
        active_extension_target[j]=arm_extension_target;
        active_bend_target[j]=std::max(.035,
            reference[j]-arm_extension_target);
        const auto& reach_pneumatic=model.pneumatic[j][Model::load(s)];
        // Predict against the pressure which can actually stop a normal
        // reused reach.  The old 3-kPa pilot estimate looked only a fraction
        // of the real valve/fill delay ahead and reached 0.2--0.3 kPa when a
        // sharp MuJoCo breakaway was already four degrees from the target.
        const double reach_pressure=std::min(
            k.reused_lift_hold_pressure_max,c.pressure_max);
        const double reach_pressure_lead=clamp(reach_pneumatic.delay+
            (std::max(0.,reach_pressure-s.pressure[j])+reach_pneumatic.error)/
                std::max(1.,reach_pneumatic.fill),
            0.,k.lift_overlap_max_prediction_sec);
        // Use the measured pneumatic delay to start braking before the angle
        // crosses the target.  Bound the extrapolation to the same four-degree
        // terminal region so a breakaway-rate spike cannot assert completion.
        const double predicted_extension=ext+std::min(
            std::max(0.,-s.rate[j])*reach_pressure_lead,
            k.reused_lift_brake_margin);
        const double predicted_remaining=arm_extension_target-predicted_extension;
        if(identifying_straight && !lift_target_reached[j])
          // The first extension test is deliberately a zero-pressure test.
          // Prediction may reduce rotor thrust, but pressure is not allowed
          // to reverse the arm before its measured bend reaches the target.
          lift_angle_pressure_command[j]=0.;
        // The first unloading reach is a zero-pressure identification.
        // Do not extrapolate a one-sample IMU rate spike into the terminal
        // region. Only the measured angle starts the final thrust rundown.
        lift_terminal_braking[j]=identifying_straight && !slope_overlap_active &&
            s.bend[j]<=active_bend_target[j]+
                k.reused_lift_brake_margin;
        const bool measured_target_reached=
            s.bend[j]<=active_bend_target[j]+k.identification_tolerance;
        // Prediction is allowed to brake thrust before the target, but only
        // the measured absolute bend may complete an arm. A zero angular rate
        // far from the target is a stall and keeps the bounded rate controller
        // searching; it must never become a learned successful trajectory.
        if(!lift_target_reached[j] && measured_target_reached) {
          lift_target_reached[j]=true;
          identification_peak_thrust[j]=std::max(
              identification_peak_thrust[j],std::max(s.thrust[j],lift_ramp[j]));
          // Reduce gently from the force which actually reached this target.
          // Falling all the way back to onset-minus-margin made the first arm
          // lose support while its partner was still extending, converting
          // the remaining partner thrust directly into roll.  The target
          // approach below has already bounded the bend rate, so retaining
          // the current force minus half a margin is the safer pair handoff.
          const double measured_hold=std::max(
              onset[j].samples?onset[j].thrust-k.onset_margin_n:0.,
              s.thrust[j]-.5*k.onset_margin_n);
          lift_target_hold_thrust[j]=clamp(measured_hold,0.,c.thrust_max);
        }
        extension=std::max(extension,ext);
        if(identifying_straight && !lift_target_reached[j])
          identification_peak_thrust[j]=std::max(
              identification_peak_thrust[j],std::max(s.thrust[j],lift_ramp[j]));
        // Initial identification retains a bounded per-arm reach event. The
        // pair handoff below requires both latches, rather than an unrelated
        // body-height/pitch shortcut or an unregulated historical minimum.
        current_minimum_extension=std::min(current_minimum_extension,ext);
        // Do not hold thrust at zero because venting itself already moves the
        // arm. Rate limiting applies to the extra powered response; the raw
        // hard rate and absolute bend guards remain active in the adapter.
        const double extending=std::max(0.,-s.rate[j]-passive_extension_rate[j]);
        if(identifying_straight) {
          if(s.bend[j]>=k.identification_rate_feedback_min_bend) {
            const double alpha=dt/(k.identification_rate_filter_tau+dt);
            identification_extension_rate_filtered[j]+=alpha*
                (extending-identification_extension_rate_filtered[j]);
            identification_fast_rate_time[j]=
                identification_extension_rate_filtered[j]>
                    k.extension_rate+k.identification_rate_deadband?
                identification_fast_rate_time[j]+dt:0.;
          } else {
            // Once inside 45 deg, angular-rate spikes are not useful for
            // shaping the first reach. Clear their memory so they cannot
            // suppress the remaining thrust search.
            identification_extension_rate_filtered[j]=0.;
            identification_fast_rate_time[j]=0.;
          }
        }
        const bool sustained_fast_extension=identifying_straight &&
            s.bend[j]>=k.identification_rate_feedback_min_bend &&
            identification_fast_rate_time[j]+1e-9>=
                k.identification_rate_confirm;
        const bool instantaneous_fast_extension=identifying_straight &&
            s.bend[j]>=k.identification_rate_feedback_min_bend &&
            extending>k.extension_rate+k.identification_rate_deadband;
        const bool moving=ext>=k.onset_angle && extending>=k.onset_rate;
        const double approach_target=onset_approach_target[j];
        fast_approach[j]=onset_pair_approach_pending &&
            !onset_approach_ready[j] && !moving &&
            !onset_this_stroke[j] && ext<k.onset_angle &&
            extending<k.onset_rate;
        // Keep the common extension search independent of differential roll
        // allocation.  Anchoring this value to the reduced actuator output
        // permanently lowered only the lagging arm.  The final actuator slew
        // below already prevents a hidden demand from becoming a thrust step.
        if(lift_target_reached[j]) {
          if(identifying_straight && !slope_overlap_active) {
            // The first arm to reach the absolute target must not chase it a
            // second time while its partner is still extending.  In MuJoCo,
            // immediately applying 30 kPa at the first latch rebounded both
            // arms from about 20 to 40--70 deg and the angle controller then
            // re-accelerated only the first arm, exciting hard roll.  Retain
            // the measured latch, remove extension thrust toward the local
            // breakaway hold, and keep pressure at zero.  Once both latches
            // exist, Pull owns the single pressure-driven flexion event.
            lift_ramp[j]=std::min(lift_ramp[j],lift_target_hold_thrust[j]);
            lift_angle_pressure_command[j]=0.;
          } else {
            // Reused short strokes and slope-overlap reaches keep active
            // angle holding because their travel is small and pressure is
            // already part of the intended continuous lift-to-pull motion.
            const double bend_error=s.bend[j]-active_bend_target[j];
            const double hold=lift_target_hold_thrust[j]+
                k.identification_angle_kp*bend_error+
                k.identification_rate_kd*s.rate[j];
            lift_ramp[j]=clamp(hold,0.,c.thrust_max);
            rise_limit[j]=std::max(rise_limit[j],k.approach_rise);
            const double predicted_bend=s.bend[j]+
                std::min(0.,s.rate[j])*reach_pressure_lead;
            // Pressure acts only toward flexion. Use it strongly at and beyond
            // the target, then release it continuously if the arm rebounds by
            // the four-degree holding corridor. Rotor thrust supplies the
            // opposite (extension) correction.
            const double rebound=std::max(0.,
                predicted_bend-active_bend_target[j]);
            const double hold_fraction=clamp(
                1.-rebound/k.reused_lift_brake_margin,0.,1.);
            const double hold_pressure=std::min(
                k.reused_lift_hold_pressure_max,c.pressure_max);
            lift_angle_pressure_command[j]=hold_pressure*hold_fraction;
          }
        } else if(identification_pair_tracking) {
          // BOTH breakaway forces are known, but they are not completion
          // forces. During this first test only, regulate the filtered
          // extension speed instead of blindly continuing the 0.7-N/s ramp.
          // The rise rate tends continuously to zero at the requested speed.
          // An overspeed sample pauses the search instead of lowering thrust:
          // cutting an already-established force made the arm rebound and
          // repeatedly restarted the initial reach.  The absolute bend guard
          // still owns terminal braking, and the independent hard rate/roll
          // guards remain available for a genuinely unsafe response. Below
          // 45 deg, rate feedback is deliberately ignored and measured angle
          // owns terminal braking.
          double rate_slope=k.lift_rise;
          if(roll_brake) {
            lift_ramp[j]=std::min(lift_ramp[j],
                std::max(0.,s.thrust[j]-c.thrust_fall*dt));
            rate_slope=-c.thrust_fall;
          } else if(s.bend[j]>=k.identification_rate_feedback_min_bend) {
            const double filtered=identification_extension_rate_filtered[j];
            if(filtered<k.extension_rate-k.identification_rate_deadband &&
               !instantaneous_fast_extension) {
              // Full search when stalled, progressively slower as the
              // measured rate approaches the lower edge of the deadband.
              rate_slope=k.lift_rise*clamp(
                  (k.extension_rate-k.identification_rate_deadband-filtered)/
                  std::max(.01,k.extension_rate-k.identification_rate_deadband),
                  0.,1.);
            } else if(filtered>
                       k.extension_rate+k.identification_rate_deadband) {
              rate_slope=0.;
            } else {
              // Hold through the target band and during the confirmation
              // interval of a possible overspeed event.
              rate_slope=0.;
            }
            lift_ramp[j]=clamp(s.thrust[j]+rate_slope*dt,0.,c.thrust_max);
          } else {
            lift_ramp[j]=std::min(c.thrust_max,
                std::max(lift_ramp[j],s.thrust[j])+k.lift_rise*dt);
          }
          identification_rate_command_slope[j]=rate_slope;
          rise_limit[j]=k.lift_rise;
        } else if(identifying_straight && identification_tracking[j]) {
          // The leading arm holds its locally measured breakaway force.  Do
          // not cut it (which makes it rebound), and do not keep increasing
          // it (which converts the mismatch directly into body roll). If it
          // is already extending too quickly while waiting for the second
          // onset, however, pause the leader's increase in the same way as the
          // paired controller. This closes the dangerous interval between the
          // first and second breakaway without accelerating the leader or
          // unloading it into a rebound.
          if(identification_extension_rate_filtered[j]>
             k.extension_rate+k.identification_rate_deadband) {
            lift_ramp[j]=s.thrust[j];
            identification_rate_command_slope[j]=0.;
          } else {
            lift_ramp[j]=identification_hold_thrust[j];
          }
        } else if(onset_pair_approach_pending) {
          // Hold an early arm below its measured breakaway while its partner
          // catches up.  Only the active arm gets the fast feed-forward ramp;
          // neither proceeds into the slow onset search until both are ready.
          if(fast_approach[j]) {
            const double approach_rise=calibrated?
                std::max(k.approach_rise,k.reused_lift_rise):k.approach_rise;
            lift_ramp[j]=std::min(approach_target,lift_ramp[j]+approach_rise*dt);
            rise_limit[j]=approach_rise;
          } else {
            lift_ramp[j]=std::min(lift_ramp[j],approach_target);
          }
        } else if(identifying_straight && !posture_brake) {
          // The same spike rejection applies before both arms have crossed the
          // 8-deg tracking boundary. A single rate sample may end the learned
          // fast approach, but it must not reverse the gentle identification
          // search. The leading-arm hold branch above still owns pair sync.
          if(instantaneous_fast_extension || sustained_fast_extension)
            lift_ramp[j]=s.thrust[j];
          else
            lift_ramp[j]=std::min(c.thrust_max,
                std::max(lift_ramp[j],s.thrust[j])+k.lift_rise*dt);
          rise_limit[j]=k.lift_rise;
        } else if(!posture_brake) {
          if(ext>=arm_extension_target) {
            // Action C waits for BOTH measured front-arm bends.  Hold a
            // leading arm instead of continuing to increase it while the
            // other arm reaches its independently identified absolute target.
            lift_ramp[j]=std::min(lift_ramp[j],s.thrust[j]);
          } else if(extending>k.extension_rate) {
            lift_ramp[j]=std::max(0.,s.thrust[j]-3.*(extending-k.extension_rate)*dt);
          } else {
            lift_ramp[j]+=lift_search_rise*
                clamp(1.-extending/k.extension_rate,0.,1.)*dt;
            rise_limit[j]=std::max(rise_limit[j],lift_search_rise);
          }
        }
        if(lift_terminal_braking[j] && !lift_target_reached[j]) {
          // Taper only from the measured final-angle corridor. Use the same
          // gentle rate as the identification rise: the old 12-N/s rundown
          // could stop a slowly moving real arm before it reached its target.
          // Keep the released chamber at zero until measured completion.
          lift_ramp[j]=std::min(lift_ramp[j],
              std::max(0.,s.thrust[j]-k.lift_rise*dt));
          lift_angle_pressure_command[j]=0.;
        }
        // An earlier breakaway is a feed-forward hint, NOT a new safety cap.
        // Contact changes after each stroke: keep the slow measured search
        // available up to the configured physical limit until extension is
        // observed. Otherwise one easy first lift permanently starves later
        // strokes. Rate/roll/floor/up-force guards below still take priority.
        if(!identifying_straight && !lift_target_reached[j]) {
          // The reused Action-C stroke rises faster than the first
          // identification pass. With an 8 N/s rise, waiting until the
          // target sample to brake allowed a further 4--5 deg of extension
          // while rear pressure was exhausting.  Shape only the final part
          // of motion 2: retain the fast approach, then continuously reduce
          // the demand in proportion to the remaining measured angle.  If
          // the arm stops early the ordinary bounded search resumes, so this
          // is a predictive brake rather than a new completion condition.
          const double remaining=predicted_remaining;
          if(remaining<k.reused_lift_brake_margin) {
            const double scale=clamp(remaining/k.reused_lift_brake_margin,0.,1.);
            // Do not brake all the way to zero.  The released arm then
            // rebounds before its partner reaches the simultaneous target,
            // creating the very roll drift this shaping is meant to avoid.
            // The locally observed breakaway minus its approach margin is the
            // smallest measured hold force; it is contact-specific and stays
            // below the force which initiated extension.
            const double hold=onset[j].samples?
                std::max(0.,onset[j].thrust-k.onset_margin_n):0.;
            const double shaped=hold+std::max(0.,s.thrust[j]-hold)*scale;
            lift_ramp[j]=std::min(lift_ramp[j],shaped);
          }
        }
        lift_ramp[j]=clamp(lift_ramp[j],0.,c.thrust_max);
        out.angle_target[j]=active_bend_target[j];
        pressure_target[j]=0.;
        if(!identifying_straight && !lift_target_reached[j] &&
           predicted_extension>=k.onset_angle) {
          // After measured breakaway, use the learned pneumatic delay to
          // ramp toward the per-arm holding pressure over the remaining
          // extension coordinate.  Pressure is still zero before genuine
          // motion, so this cannot suppress the motion-2 extension test.
          // Reaching the target itself is latched only from measured bend.
          // The second MuJoCo stroke stalled at 40 deg with 4 N because the
          // old ramp started at the 8-deg onset and had already built 2.5 kPa
          // while the target was still 17 deg. A later 70% start still made a
          // 1.8-kPa equilibrium five to six degrees before the learned target
          // on the next high-friction stroke. Keep the reused reach truly free
          // through its first 90%; pressure then provides terminal braking
          // rather than an opposing equilibrium far from the target.
          const double pressure_start=std::max(k.onset_angle,
              k.reused_lift_pressure_start_ratio*arm_extension_target);
          const double controlled_span=std::max(k.onset_angle,
              arm_extension_target-pressure_start);
          const double fraction=clamp((predicted_extension-pressure_start)/
              controlled_span,0.,1.);
          lift_angle_pressure_command[j]=
              std::min(k.reused_lift_hold_pressure_max,c.pressure_max)*fraction;
        } else if(!identifying_straight && !lift_target_reached[j]) {
          lift_angle_pressure_command[j]=0.;
        }
        if(slope_overlap_active) {
          const auto& pneumatic=reach_pneumatic;
          // Predict when a modest prefill, not the full 30-kPa grasp, begins
          // to affect the chamber.  The full-fill estimate in walk_c_new_3 was
          // 1.18 s and arrived far too late after the 12/8-deg overshoot.
          const double lead=clamp(pneumatic.delay+
              (std::max(0.,std::min(k.lift_overlap_prefill_kpa,in.base_pressure)-
                   s.pressure[j])+pneumatic.error)/std::max(1.,pneumatic.fill),
              0.,k.lift_overlap_max_prediction_sec);
          pressure_lead_seconds=std::max(pressure_lead_seconds,lead);
          const double nominal_target=std::max(k.onset_angle,
              nominal_extension_target);
          const double measured_ratio=clamp(ext/nominal_target,0.,1.);
          // A breakaway-rate spike is not a trustworthy constant-velocity
          // prediction over the whole pneumatic delay.  walk_sim at -15 deg
          // predicted almost the complete reach from a 29-deg/s sample,
          // filled to 6 kPa at 35% measured progress and stalled the arm.
          // Keep using the learned delay/rate, but bound its look-ahead in
          // trajectory coordinates so pressure cannot cancel the extension
          // which is meant to create the forward contact point.
          const double velocity_prediction=clamp(
              (ext+std::max(0.,-s.rate[j])*lead)/nominal_target,0.,1.);
          const double predicted_ratio=std::min(velocity_prediction,
              measured_ratio+k.lift_overlap_max_prediction_lead_ratio);
          const double fill_fraction=clamp((predicted_ratio-
              k.lift_overlap_pressure_start_ratio)/
              (k.lift_overlap_handoff_ratio-
               k.lift_overlap_pressure_start_ratio),0.,1.);
          const double request=std::min(in.base_pressure,
              k.lift_overlap_prefill_kpa)*fill_fraction;
          // Monotonic within one reach: gyro noise must not chatter the valve.
          lift_prefill_command[j]=std::max(lift_prefill_command[j],request);
        }
        pressure_target[j]=std::max(lift_prefill_command[j],
            lift_angle_pressure_command[j]);
        raw[j]=lift_ramp[j];
      }
      ramp=std::max(lift_ramp[0],lift_ramp[1]);
      const bool powered=std::max(s.thrust[0],s.thrust[1])>.3;
      // Predict when both front bends will reach the current motion-2 target.
      // Rear exhaust time is estimated conservatively from the larger measured
      // rear pressure, the configured pneumatic delay and exhaust rate.  Do not
      // pre-vent before actual extension motion is visible: the rear pair must
      // remain the anchor during the force search.
      predicted_lift_completion_seconds=0.;
      double maximum_remaining_extension=0.;
      bool completion_prediction_valid=powered;
      bool extension_motion_seen=false;
      for(size_t j=0;j<2;++j) {
        const double remaining=std::max(0.,s.bend[j]-active_bend_target[j]);
        const double extending=std::max(0.,-s.rate[j]);
        maximum_remaining_extension=std::max(maximum_remaining_extension,remaining);
        extension_motion_seen=extension_motion_seen ||
            (extending>=k.onset_rate && reference[j]-s.bend[j]>=k.onset_angle);
        if(remaining>k.onset_angle) {
          if(extending<k.onset_rate) completion_prediction_valid=false;
          else predicted_lift_completion_seconds=std::max(
              predicted_lift_completion_seconds,remaining/extending);
        }
      }
      rear_vent_lead_seconds=c.pressure_delay+
          std::max(0.,in.rear_pressure-k.rear_low_kpa)/
              std::max(1.,c.exhaust_rate)+k.rear_vent_lead_margin;
      if(!completion_prediction_valid)
        predicted_lift_completion_seconds=std::numeric_limits<double>::infinity();
      if(!rear_low && extension_motion_seen && completion_prediction_valid &&
         maximum_remaining_extension<=k.rear_vent_start_margin &&
         predicted_lift_completion_seconds<=rear_vent_lead_seconds)
        rear_low=true;
      // Body height/pitch alone is not reach completion. Each arm must acquire
      // its bounded measured/predicted target latch. The reached arm waits at
      // low thrust and zero pressure while its partner catches up; requiring
      // both CURRENT angles to occupy a narrow corridor simultaneously made
      // the controller chase rebound instead of handing the stroke to Pull.
      bool pair_reached=std::isfinite(current_minimum_extension);
      for(size_t j=0;j<2;++j) {
        if(identifying_straight) {
          pair_reached=pair_reached && lift_target_reached[j];
        } else {
          // Reused strokes retain their identified absolute bend corridor: a
          // large rebound means the reached-arm FB has not caught it yet.
          pair_reached=pair_reached && lift_target_reached[j] &&
              s.bend[j]<=active_bend_target[j]+k.reused_lift_brake_margin;
        }
      }
      // The breakaway in the zero-pressure model is sharp.  A single sample
      // below the threshold can be a dynamic overshoot. Require both measured
      // per-arm reach latches to persist briefly. The first arm waits at low
      // thrust and zero pressure; Pull applies pressure only after both arrive.
      if(identifying_straight) {
        identification_ready_time=pair_reached?
            identification_ready_time+dt:0.;
        pair_reached=identification_ready_time+1e-9>=k.identification_confirm;
      }
      if(slope_overlap_active && pair_reached)
        overlap_extension_latched=true;
      bool overlap_pressure_arrived=slope_overlap_active;
      bool overlap_minimum_reach=slope_overlap_active;
      bool pressure_driven_flexion=false;
      for(size_t j=0;j<2;++j)
      {
        overlap_pressure_arrived=overlap_pressure_arrived &&
            s.pressure[j]>=k.lift_overlap_min_measured_kpa;
        const double minimum_reach=active_extension_target[j]*
            k.lift_overlap_min_reach_ratio/k.lift_overlap_handoff_ratio;
        overlap_minimum_reach=overlap_minimum_reach &&
            overlap_peak_extension[j]+k.onset_angle>=minimum_reach;
        pressure_driven_flexion=pressure_driven_flexion ||
            s.rate[j]>=k.onset_rate;
      }
      // With pilot pressure already present, the physically attainable end
      // of reach is the measured reversal from extension to flexion.  The two
      // arms need not cross the same angle in the same sample: retain each
      // measured peak, require a substantial reach on both, and hand off when
      // the pressure has demonstrably begun the intended flexion.  This is
      // used only on the adverse-slope overlap path; ordinary identification
      // keeps the two bounded per-arm unloading-pose reach latches.
      if(slope_overlap_active && overlap_pressure_arrived &&
         overlap_minimum_reach && pressure_driven_flexion)
        overlap_extension_latched=true;
      const bool achieved=slope_overlap_active?
          overlap_extension_latched && overlap_pressure_arrived:pair_reached;
      // The front geometry alone owns the motion-2 -> motion-3 transition.
      // Waiting for a measured rear-pressure threshold kept the zero-pressure
      // front pair under unloading-angle feedback after it had already
      // reached the target.  Around the contact breakaway this has no stable
      // equilibrium and produced the observed 0--30-deg chatter.  Keep the
      // rear exhaust command active through Pull, but never wait for its
      // pneumatic settling before beginning pressure-driven front flexion.
      if(achieved) front_target_latched=true;
      if(front_target_latched) rear_low=true;
      const bool handoff_due=front_target_latched;
      if(handoff_due) {
        stroke_lifted=true;
        if(!calibrated) {
          for(size_t j=0;j<2;++j) {
            // Dynamic travel continues for a few samples after the target is
            // crossed. Reusing minimum[] directly therefore made every next
            // stroke more aggressive than the safe target which completed
            // calibration. Preserve per-arm absolute targets, but never learn
            // an overshoot closer to straight than the configured unloading
            // bend. If an arm starts already beyond that pose, retain only a
            // small genuine extension from its measured start.
            const double safe_bend_target=active_bend_target[j];
            identified_bend_target[j]=clamp(
                std::max(minimum[j],safe_bend_target),.035,
                reference[j]-k.onset_angle);
            identified_extension_thrust[j]=clamp(
                identification_peak_thrust[j],0.,c.thrust_max);
            observed_extension_by_arm[j]=clamp(overlap_peak_extension[j],
                k.onset_angle,k.identification_extension_limit);
          }
          observed_extension=std::min(observed_extension_by_arm[0],
              observed_extension_by_arm[1]);
          learned_lift_thrust=std::max(identified_extension_thrust[0],
              identified_extension_thrust[1]);
          learned_ratio=clamp(fx/std::max(.01,fx+up),0.,1.);
          calibrated=true;
        }
        lift_up=std::min(c.up_force_max*.7,std::max(.2,up));
        lift_force=std::max(s.thrust[0],s.thrust[1]);
        pull_forward=std::max(pull_forward,std::max(1.,fx));
        enter(Pull,in);
      } else if(roll_brake && !identifying_straight) {
        // Preserve the goal/identified envelope, but give exclusive actuator
        // ownership to the existing shared roll recovery. This is NOT a
        // completed lift or a positive optimizer sample.
        out.recover_roll=true;
      } else if(powered_time>=k.lift_seconds || phase_time>=k.lift_seconds+k.vent_seconds) {
        // A transient roll brake only decreases thrust and can resume if it
        // settles before the deadline. Intended gait pitch is not a soft
        // failure here; the adapter retains the independent hard pitch guard.
        // An unachieved lift at the deadline is not a normal pull/sample.
        out.fault=true;
      }
    }
    if(phase==Pull || phase==FrontCatch) {
      rear_low=true;
      bool angle_caught=true, stalled_caught=phase==FrontCatch;
      planned_mix=0.;pressure_lead_seconds=0.;
      Pair measured_flex{};
      for(size_t j=0;j<2;++j) {
        const double span=std::max(.017453,grasp_reference[j]-pull_start[j]);
        measured_flex[j]=clamp((s.bend[j]-pull_start[j])/span,0.,1.);
      }
      const double common_flex=.5*(measured_flex[0]+measured_flex[1]);
      goal_pull_envelope=1.;
      if(goal_conditioned && goal_scale<1.) {
        const double cutoff=clamp(goal_scale,k.goal_scale_min,1.);
        goal_pull_envelope=clamp((cutoff-common_flex)/
            k.goal_pull_taper_fraction,0.,1.);
      }
      for(size_t j=0;j<2;++j) {
        // Return to the measured feasible grasp, not an unreachable exact
        // 90 degrees imposed through the cylinder's contact constraints.
        const double target=grasp_reference[j];
        out.angle_target[j]=target;
        const double span=std::max(.017453,target-pull_start[j]);
        const double actual=measured_flex[j];
        // Prediction helps timing but cannot assert unobserved flexion.
        const auto& pneumatic=model.pneumatic[j][Model::load(s)];
        const double fill_target=std::min(c.pressure_max,in.base_pressure+10.);
        const double fill_time=pneumatic.delay+
            (std::max(0.,fill_target-s.pressure[j])+pneumatic.error)/std::max(1.,pneumatic.fill);
        pressure_lead_seconds=std::max(pressure_lead_seconds,fill_time);
        const double horizon=clamp(fill_time,.05,.5);
        const double progress=clamp(actual+clamp(s.rate[j]*horizon/span,0.,.1),0.,1.);
        pressure_target[j]=std::min(c.pressure_max,in.base_pressure+10.);
        const auto selected=trajectory.selected();
        const double minimum_mix=calibrated?clamp(learned_ratio+selected.mix_offset,.1,.95):.5;
        const double mix=minimum_mix+(1.-minimum_mix)*progress;
        planned_mix+=.5*mix;
        const double distance_force_scale=goal_conditioned?
            std::sqrt(goal_scale):1.;
        const double magnitude=(lift_up+pull_forward)*selected.force_scale*
            distance_force_scale;
        // Both front rotors share one envelope to avoid introducing roll.
        // Pressure continues to the full grasp even after the final-stroke
        // thrust impulse has ended.
        const double desired_up=.5*magnitude*(1.-mix)*goal_pull_envelope;
        const double desired_x=.5*magnitude*mix*goal_pull_envelope;
        // Project the desired force onto the measured rotor direction. Do
        // not divide by cos(bend) near horizontal and request infinite lift.
        const double norm=ux[j]*ux[j]+uz[j]*uz[j];
        const double projected=norm>.05?(desired_x*ux[j]+desired_up*uz[j])/norm:0.;
        const double allowed=lift_force+2.*c.thrust_max*actual;
        raw[j]=std::min(projected,allowed);
        out.desired_up+=desired_up;out.desired_forward+=desired_x;
        const bool this_angle_caught=s.bend[j]>=target-k.front_catch_tolerance &&
            s.pressure[j]>=in.base_pressure-3. && std::abs(s.rate[j])<.2;
        angle_caught=angle_caught && this_angle_caught;
        const bool pressure_loaded=s.pressure[j]>=fill_target-k.catch_pressure_tolerance;
        const bool safely_stalled=s.bend[j]>=s.floor[j]+k.catch_safe_margin &&
            std::abs(s.rate[j])<=k.catch_stall_rate;
        catch_stall_time[j]=(phase==FrontCatch && pressure_loaded && safely_stalled)?
            catch_stall_time[j]+dt:0.;
        stalled_caught=stalled_caught &&
            catch_stall_time[j]+1e-9>=k.catch_stall_confirm;
      }
      const double predicted_roll=s.roll+.3*s.roll_rate;
      const bool roll_stable=std::abs(s.roll_rate)<=k.catch_roll_rate &&
          std::abs(predicted_roll)<=std::abs(s.roll)+k.catch_roll_prediction_tolerance;
      stalled_caught=stalled_caught && roll_stable;
      const bool contact_ok=!in.contact_available || in.front_contact;
      const bool caught=(angle_caught || stalled_caught) && contact_ok;
      ready_time=caught?ready_time+dt:0.;
      if(ready_time>=.2 && phase_time>=k.pull_min_seconds) {
        out.regrip=true;
        out.contact_limited_regrip=stalled_caught && !angle_caught;
      }
      if(phase==Pull && phase_time>=k.pull_seconds) enter(FrontCatch,in);
      if(phase==FrontCatch) {
        // Stop spending thrust on an unproductive lift; leave the rear LOW
        // until front catch, or hand over to explicit fault regrasp at timeout.
        for(double& t:raw) t=std::min(t,.5);
        if(phase_time>=k.front_wait_seconds) out.fault=true;
      }
    }
    // Keep the first unloading identification envelope through its whole
    // pressure-driven Pull/FrontCatch.  Re-enabling the ordinary pulse floor
    // as soon as the bend crossed it once made a later 1--2 degree settling
    // drift abort an otherwise successful regrip.  The next Lift sets this
    // flag from !calibrated, so ordinary strokes still regain the normal
    // Action-C floor.  Absolute bend/rate and body roll guards remain active.
    out.rear_low=rear_low;out.rear_pressure=rear_low?k.rear_low_kpa:in.base_pressure;
    const bool has_identified_absolute_target=calibrated &&
        std::isfinite(identified_bend_target[0]) &&
        std::isfinite(identified_bend_target[1]);
    const bool identified_absolute_envelope=
        (identifying_straight || has_identified_absolute_target) &&
        (phase==Lift || phase==Pull || phase==FrontCatch);
    double proposed_up=0.;
    for(size_t j=0;j<2;++j) {
      raw[j]=clamp(raw[j],0.,c.thrust_max);
      if(posture_brake && !(phase==Lift && identifying_straight)) {
        // During pressure-driven flexion the front chambers are already
        // rebuilding support. Remove residual pull promptly when roll keeps
        // growing; intended pitch is handled by the independent hard guard.
        // Lift remains deliberately gentle because its front pressure is
        // still zero or only beginning the bounded slope-aware prefill.
        const double posture_fall=phase==Lift?2.:c.thrust_fall;
        raw[j]=std::min(raw[j],std::max(0.,s.thrust[j]-posture_fall*dt));
      }
      const double ext=reference[j]-s.bend[j];
      const double anticipated=s.bend[j]+std::min(0.,s.rate[j])*.15;
      // A fixed relative travel cap cannot coexist with a reused absolute bend
      // target when the grasp angle changes (the real fixture starts around
      // 106--134 deg). Extend the hybrid envelope just far enough to reach the
      // active absolute target. Absolute bend, bend-rate, roll and thrust
      // guards remain active.
      const double absolute_target_travel=std::max(0.,
          reference[j]-active_bend_target[j])+k.identification_tolerance+.035;
      const double extension_limit=identified_absolute_envelope?
          std::max(k.identification_extension_limit,
              absolute_target_travel):c.extension_limit;
      const double bend_floor=identified_absolute_envelope?0.:s.floor[j];
      // SlipProbe/Slide have their own smaller per-arm excursion.  The slip
      // test must not perform the large release reserved for Lift.
      const bool relative_extension_exceeded=
          phase!=SlipProbe && phase!=Slide && ext>extension_limit-.035;
      if((phase==Lift || phase==Pull || phase==SlipProbe || phase==Slide || phase==FrontCatch) &&
          (relative_extension_exceeded || anticipated<=bend_floor+.035)) {
        raw[j]=std::min(raw[j],std::max(0.,s.thrust[j]-c.thrust_fall*dt));
        // Lift deliberately reproduces Action C's free-extension interval:
        // front pressure remains zero while thrust finds the extension onset.
        // On an adverse slope, only the late part of that interval receives
        // the bounded predictive prefill computed above.
        // Repressurising here used to drive the command back toward 30 kPa at
        // roughly 2 N, exactly cancelling the requested extension.  During
        // Lift the angle/floor guard owns thrust only; pneumatic regrasp starts
        // after the measured extension causes the transition to Pull.
        if(phase!=Lift)
          pressure_target[j]=std::max(pressure_target[j],in.base_pressure);
      }
      // Keep one unambiguous owner for Lift pressure.  It is zero in free
      // extension, then becomes the larger of trajectory prefill and the
      // individual reached-arm angle brake.
      if(phase==Lift) pressure_target[j]=std::max(
          lift_prefill_command[j],lift_angle_pressure_command[j]);
      // Unequal arm angles are allowed. Individual extension/floor guards
      // remain; body roll feedback, not angle matching, owns differential thrust.
    }
    bool identification_sync_active=false;
    size_t identification_leading=0;
    if(phase==Lift && identifying_straight &&
       !(lift_target_reached[0] && lift_target_reached[1])) {
      // The zero-pressure model has a sharp breakaway: one arm can move tens
      // of degrees before the other starts.  Cutting the leading rotor made it
      // rebound before the lagging arm could catch up, so the pair could never
      // acquire both absolute-angle reach events. Keep the leading
      // arm at its bounded hold demand and temporarily accelerate only the
      // lagging arm's search.  The common up-force cap and hard roll/rate
      // limits still apply after this correction.
      // Continue the synchronization after both onsets: walk_arm_pink_7 had
      // measured both breakaways, then arm 3 travelled 30--70 deg farther
      // because synchronization was disabled at that exact boundary.  Bound
      // the addition to 1 N so this cannot recreate the old 4--5 N runaway.
      // Ordinary Action-C strokes remain unchanged.
      // Use a short filtered angle-rate prediction only above 45 deg. At the sharp breakaway the
      // current angles can still look equal while one arm is already moving
      // 10--50 deg/s faster; waiting for the measured mismatch reacts one
      // sample too late and turns that motion directly into body roll. Below
      // 45 deg, use measured angles so an IMU spike cannot hold one rotor.
      const Pair predicted_bend{{
          s.bend[0]+(s.bend[0]>=k.identification_rate_feedback_min_bend?
              -identification_extension_rate_filtered[0]*.15:0.),
          s.bend[1]+(s.bend[1]>=k.identification_rate_feedback_min_bend?
              -identification_extension_rate_filtered[1]*.15:0.)}};
      // Initial bends depend on where each arm contacts the cylinder and can
      // legitimately differ by tens of degrees. Synchronize extension made
      // during THIS reach, never the two absolute bend readings. In the real
      // bag [113.6,131.9] deg represented [0,0] deg extension; the old test
      // froze arm 0 at zero thrust solely because of that initial offset.
      const Pair predicted_extension{{
          reference[0]-predicted_bend[0],
          reference[1]-predicted_bend[1]}};
      const double mismatch=std::abs(
          predicted_extension[0]-predicted_extension[1]);
      if(mismatch>k.identification_sync_deadband) {
        identification_sync_active=true;
        identification_leading=
            predicted_extension[0]>predicted_extension[1]?0:1;
        const size_t lagging=1-identification_leading;
        const double assist=std::min(k.identification_sync_assist_max,
            k.identification_sync_gain*
                (mismatch-k.identification_sync_deadband));
        // Freeze (rather than cut) the arm that has begun extending first.
        // The lagging arm receives the extra search demand below.
        raw[identification_leading]=std::min(raw[identification_leading],
            s.thrust[identification_leading]);
        // Once both arms are in first-pass rate control, synchronization may
        // accelerate only an actually slow lagging arm. Otherwise it would
        // overwrite the speed controller and recreate the overspeed which the
        // controller just removed. Before pair tracking, retain the original
        // catch-up assist so the second breakaway can still be acquired.
        const bool lagging_rate_allows_assist=
            !(identification_tracking[0] && identification_tracking[1]) ||
            (identification_extension_rate_filtered[lagging]<
                 k.extension_rate-k.identification_rate_deadband &&
             std::max(0.,-s.rate[lagging]-passive_extension_rate[lagging])<=
                 k.extension_rate+k.identification_rate_deadband);
        if(lagging_rate_allows_assist) {
          raw[lagging]=std::min(c.thrust_max,raw[lagging]+assist);
          rise_limit[lagging]=std::max(rise_limit[lagging],
              k.identification_tracking_rise);
        }
      }
    }
    // A small, stationary roll bias is part of the contact condition, not a
    // reason to fight the extension search. Only a measured outward trend
    // activates the inner roll allocator. In Lift it is addition-only, so
    // synchronization remains the primary owner and cannot deadlock with a
    // later reduction. Use the hardware-verified pair sign rather than the
    // contact-lever sign, which can reverse near a 104-deg bend.
    const Pair verified_roll_moment=verifiedPairRollMoments(moment);
    if(roll_brake)
      raw=phase==Lift?
          addRestoringRollSupport(raw,verified_roll_moment,
              s.roll,s.roll_rate,c.thrust_max):
          reduceAdverseRoll(raw,verified_roll_moment,s.roll,s.roll_rate);
    if(identification_sync_active)
      raw[identification_leading]=std::min(raw[identification_leading],
          s.thrust[identification_leading]);
    if(phase==Lift)
      for(size_t j=0;j<2;++j)
        if(lift_target_reached[j] || lift_terminal_braking[j])
        // Roll allocation may help the lagging arm, but may not re-accelerate
        // an arm whose individual reach controller is already braking. This
        // includes predictive braking during first identification.
          raw[j]=std::min(raw[j],lift_ramp[j]);
    for(size_t j=0;j<2;++j) {
      out.thrust[j]=slew(s.thrust[j],raw[j],rise_limit[j],c.thrust_fall,dt);
      const double pressure_rise=(identifying_straight &&
          (phase==Pull || phase==FrontCatch))?
          k.identification_return_pressure_slew:c.pressure_slew;
      out.pressure[j]=slew(s.command[j],pressure_target[j],pressure_rise,c.pressure_slew,dt);
      proposed_up+=out.thrust[j]*uz[j];
    }
    double allowed_up=c.up_force_max;
    const bool overlap_prefilling=phase==Lift && slope_overlap_active &&
        std::max(lift_prefill_command[0],lift_prefill_command[1])>.1;
    if(overlap_prefilling) {
      const double adverse=std::max(0.,-in.directional_pitch);
      const double severity=clamp((adverse-k.lift_overlap_pitch_start)/
          std::max(.0174533,k.soft_pitch),0.,1.);
      allowed_up*=1.-(1.-k.lift_overlap_min_up_scale)*severity;
    }
    if(proposed_up>allowed_up)
      for(double& t:out.thrust) t*=allowed_up/proposed_up;
    // Regrasp has been confirmed.  Residual pull is useful while the chamber
    // is flexing, but must not remain after contact; hand the following rear
    // pressure transfer an explicit zero-thrust state.
    if(out.regrip) out.thrust.fill(0.);
    double outx=0.,outz=0.;for(size_t j=0;j<2;++j) {outx+=out.thrust[j]*std::max(0.,ux[j]);outz+=out.thrust[j]*uz[j];}
    out.ratio=(outx+outz)>1e-6?outx/(outx+outz):1.;
    return out;
  }
};
}} // namespace
