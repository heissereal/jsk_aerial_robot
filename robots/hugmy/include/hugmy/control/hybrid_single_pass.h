#pragma once
#include <functional>
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
// One continuous bend-coordinate reference is shared by Motion 2 and Motion
// 3.  A minimum-jerk profile avoids a thrust/pressure step at either end.  The
// clock is slowed (and can stop) when the measured arm cannot follow, so a
// pneumatic delay changes the execution time instead of silently changing the
// requested arm path.
inline double minimumJerkPosition(double phase) {
  const double u=clamp(phase,0.,1.);
  return u*u*u*(10.+u*(-15.+6.*u));
}
inline double minimumJerkVelocity(double phase) {
  const double u=clamp(phase,0.,1.);
  return 30.*u*u*(1.-u)*(1.-u);
}
inline double minimumJerkAcceleration(double phase) {
  const double u=clamp(phase,0.,1.);
  return 60.*u*(1.-u)*(1.-2.*u);
}
// The hardware bend angle is obtained from the relative gravity directions,
// while the fast bend rate is measured directly by the Neuron--Spinal gyro
// difference.  Keep those complementary roles: filter the gyro only enough to
// differentiate it into a bounded acceleration estimate, and never integrate
// it into the absolute bend coordinate.  This avoids the gyro-bias drift which
// previously looked like a 30--40-deg arm excursion.
struct BendKinematicsObserver {
  bool initialized=false;
  double rate=0.,acceleration=0.;
  void reset(double measured_rate=0.) {
    initialized=std::isfinite(measured_rate);
    rate=initialized?measured_rate:0.;
    acceleration=0.;
  }
  void update(double measured_rate,double dt,double rate_tau,
      double acceleration_tau,double acceleration_limit) {
    if(!std::isfinite(measured_rate) || !std::isfinite(dt) ||
       dt<=0. || dt>.1) {
      reset();return;
    }
    if(!initialized) {reset(measured_rate);return;}
    const double old_rate=rate;
    const double rate_gain=clamp(dt/(std::max(0.,rate_tau)+dt),0.,1.);
    rate+=rate_gain*(measured_rate-rate);
    const double raw_acceleration=(rate-old_rate)/dt;
    const double acceleration_gain=clamp(
        dt/(std::max(0.,acceleration_tau)+dt),0.,1.);
    acceleration=clamp(acceleration+
        acceleration_gain*(raw_acceleration-acceleration),
        -acceleration_limit,acceleration_limit);
  }
  double predictAngle(double angle,double horizon,double max_delta) const {
    if(!initialized || !std::isfinite(angle)) return angle;
    const double h=std::max(0.,horizon);
    return angle+clamp(rate*h+.5*acceleration*h*h,-max_delta,max_delta);
  }
  double predictRate(double horizon,double rate_limit) const {
    if(!initialized) return 0.;
    return clamp(rate+acceleration*std::max(0.,horizon),
        -rate_limit,rate_limit);
  }
};
struct BendTrajectory {
  bool active=false;
  Pair start{},goal{},target{},rate{},acceleration{};
  double elapsed=0.,duration=1.,phase=0.,clock_scale=1.,tracking_error=0.,
      observed_phase=0.,prediction_lag=0.;
  void reset() { *this=BendTrajectory(); }
  void configure(const Pair& from,const Pair& to,double seconds) {
    active=true;start=from;goal=to;target=from;rate.fill(0.);
    acceleration.fill(0.);
    elapsed=phase=0.;duration=std::max(.05,seconds);
    clock_scale=1.;tracking_error=0.;observed_phase=0.;prediction_lag=0.;
  }
  void sample(double time) {
    elapsed=clamp(time,0.,duration);phase=clamp(elapsed/duration,0.,1.);
    const double p=minimumJerkPosition(phase);
    const double v=minimumJerkVelocity(phase)/duration;
    const double a=minimumJerkAcceleration(phase)/(duration*duration);
    for(size_t j=0;j<2;++j) {
      target[j]=start[j]+(goal[j]-start[j])*p;
      rate[j]=(goal[j]-start[j])*v;
      acceleration[j]=(goal[j]-start[j])*a;
    }
  }
  void advance(double dt,const Pair& measured,double slow_error,
      double pause_error) {
    Pair zero{};
    advancePredictive(dt,measured,zero,zero,slow_error,pause_error,
        0.,0.,0.,1.,std::numeric_limits<double>::infinity());
  }
  void advancePredictive(double dt,const Pair& measured,
      const Pair& measured_rate,const Pair& measured_acceleration,
      double slow_error,double pause_error,double prediction_horizon,
      double rate_lag_horizon,double phase_correction_gain,
      double clock_max_scale,double prediction_max_delta) {
    if(!active) return;
    tracking_error=std::max(std::abs(target[0]-measured[0]),
        std::abs(target[1]-measured[1]));
    const double horizon=std::max(0.,prediction_horizon);
    const double future_time=std::min(duration,elapsed+horizon);
    const double future_phase=clamp(future_time/duration,0.,1.);
    const double future_position=minimumJerkPosition(future_phase);
    const double future_velocity=minimumJerkVelocity(future_phase)/duration;
    double lag=0.;
    observed_phase=1.;
    for(size_t j=0;j<2;++j) {
      const double direction=goal[j]>=start[j]?1.:-1.;
      const double desired_angle=start[j]+(goal[j]-start[j])*future_position;
      const double desired_rate=(goal[j]-start[j])*future_velocity;
      const double predicted_angle=measured[j]+clamp(
          measured_rate[j]*horizon+
              .5*measured_acceleration[j]*horizon*horizon,
          -prediction_max_delta,prediction_max_delta);
      const double predicted_rate=measured_rate[j]+
          measured_acceleration[j]*horizon;
      const double angle_lag=std::max(0.,
          direction*(desired_angle-predicted_angle));
      const double rate_lag=std::max(0.,
          direction*(desired_rate-predicted_rate))*rate_lag_horizon;
      lag=std::max(lag,std::max(angle_lag,rate_lag));
      const double span=goal[j]-start[j];
      if(std::abs(span)>1.e-6) {
        const double normalized=clamp(
            (predicted_angle-start[j])/span,0.,1.);
        // Invert the monotonic minimum-jerk position with a bounded search.
        double lower=0.,upper=1.;
        for(unsigned n=0;n<12;++n) {
          const double middle=.5*(lower+upper);
          if(minimumJerkPosition(middle)<normalized) lower=middle;
          else upper=middle;
        }
        observed_phase=std::min(observed_phase,.5*(lower+upper));
      }
    }
    prediction_lag=lag;
    // Slow only for an actuator which is behind the reference.  If momentum
    // or pressure makes an arm move ahead, allow the reference to catch up;
    // freezing it there creates an artificial pressure/thrust limit cycle.
    if(lag<=slow_error) clock_scale=1.;
    else if(lag>=pause_error) clock_scale=0.;
    else clock_scale=(pause_error-lag)/
        std::max(1.e-6,pause_error-slow_error);
    // If measured position and rate show that both arms are genuinely ahead,
    // let the reference catch up smoothly.  This advances pressure scheduling
    // without allowing a single gyro sample to jump the phase boundary.
    const double phase_ahead=std::max(0.,observed_phase-future_phase);
    if(lag<=slow_error)
      clock_scale=std::max(clock_scale,std::min(clock_max_scale,
          1.+phase_correction_gain*phase_ahead));
    sample(elapsed+dt*clock_scale);
  }
  Pair predictedTarget(double lead) const {
    if(!active) return target;
    const double u=clamp((elapsed+std::max(0.,lead))/duration,0.,1.);
    const double p=minimumJerkPosition(u);
    Pair result{};
    for(size_t j=0;j<2;++j)
      result[j]=start[j]+(goal[j]-start[j])*p;
    return result;
  }
  Pair predictedRate(double lead) const {
    if(!active) return rate;
    const double u=clamp((elapsed+std::max(0.,lead))/duration,0.,1.);
    const double v=minimumJerkVelocity(u)/duration;
    Pair result{};
    for(size_t j=0;j<2;++j) result[j]=(goal[j]-start[j])*v;
    return result;
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
  // Motion 3 has less than one second of useful unloaded motion.  Its inner
  // correction therefore needs more authority than the slow outer posture
  // loop, while remaining bounded to a modest pair moment.
  const double target=clamp(-.30*roll-.08*rate,-.05,.05);
  double missing=target-(thrust[0]*moment[0]+thrust[1]*moment[1]);
  // When the contact-conditioned feed-forward pair is asymmetric, a small
  // moment correction alone may still leave the restoring rotor far below
  // the adverse rotor.  During an actually worsening roll, first raise the
  // helpful (lower) side to the existing pair maximum.  This preserves both
  // extension demands and follows the hardware safety preference of adding
  // the smaller thrust instead of chopping the arm which is already moving.
  const double pair_max=std::max(thrust[0],thrust[1]);
  for(size_t j=0;j<2;++j)
    if(std::abs(moment[j])>.005 && missing*moment[j]>0. &&
       thrust[j]<pair_max) {
      const double addition=std::min(limit-thrust[j],pair_max-thrust[j]);
      thrust[j]+=addition;missing-=addition*moment[j];
    }
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
inline Pair verifiedPairRollMoments(const Pair& estimated,
    bool reverse_geometry=false) {
  if(reverse_geometry) {
    // The MuJoCo force-axis response is the opposite of the hardware mapping:
    // negative measured roll is restored by the arm-3 side of the front pair,
    // positive roll by arm 0.  Keep this empirically verified SIGN fixed and
    // use geometry only for magnitude.  Negating the estimated sign was not
    // equivalent: the analytical contact lever itself changes sign near a
    // 90--105 deg bend and selected arm 0 while negative roll was worsening.
    return {{-std::max(.005,std::abs(estimated[0])),
              std::max(.005,std::abs(estimated[1]))}};
  }
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
  const double pair_max=std::max(thrust[0],thrust[1]);
  for(size_t j=0;j<2;++j)
    if(std::abs(moment[j])>.0049 && missing*moment[j]>0. &&
       thrust[j]<pair_max) {
      const double addition=std::min(limit-thrust[j],pair_max-thrust[j]);
      thrust[j]+=addition;missing-=addition*moment[j];
    }
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 && missing*moment[j]>0.) {
    const double addition=std::min(std::max(0.,limit-thrust[j]),missing/moment[j]);
    thrust[j]+=addition;missing-=addition*moment[j];
  }
  return thrust;
}
// Locomotion phases need to stop a growing roll, not drive a static contact
// offset all the way to zero while contacts are being exchanged.  The loop is
// therefore enabled only for a measured outward trend.  Once enabled, use a
// small angle term as well as rate damping: pressure-driven flexion can create
// a nearly constant roll moment while the measured rate is still small, and a
// rate-only target reacted after the 10-deg support-transfer boundary.
inline Pair dampLocomotionRoll(Pair thrust,const Pair& moment,
    double roll,double rate,double limit) {
  const bool worsening=std::abs(roll)>.0174533 &&
      std::abs(rate)>.02 && roll*rate>0.;
  const double target=worsening?
      clamp(-.08*roll-.04*rate,-.020,.020):0.;
  const double pair_max=std::max(thrust[0],thrust[1]);
  // The moment-arm magnitude is only a local model and can be wrong on soft
  // contact.  When measured roll is already worsening, first remove the
  // commanded pair imbalance in the empirically restoring direction without
  // trusting that magnitude.  The actuator slew below keeps this continuous;
  // the model-based allocation then supplies only the remaining correction.
  if(worsening)
    for(size_t j=0;j<2;++j)
      if(roll*moment[j]<0. && thrust[j]<pair_max)
        thrust[j]=std::min(limit,pair_max);
  double missing=target-(thrust[0]*moment[0]+thrust[1]*moment[1]);
  // First remove a harmful pair imbalance by raising the helpful, smaller
  // rotor as far as the current common demand.  This is not an attitude
  // command: it only prevents the extension synchronizer from creating roll,
  // and it preserves both arms' extension authority.  A fixed small delta was
  // insufficient when one zero-pressure arm broke away before the other.
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 &&
      missing*moment[j]>0. && thrust[j]<pair_max) {
    const double addition=std::min({std::max(0.,limit-thrust[j]),
        pair_max-thrust[j],std::abs(missing/moment[j])});
    thrust[j]+=addition;missing-=addition*moment[j];
  }
  constexpr double maximum_residual_delta=.75;
  // Only the residual angular-rate damping is tightly bounded.  It must not
  // turn the locomotion inner loop into a zero-attitude recovery controller.
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 &&
      missing*moment[j]>0.) {
    const double addition=std::min({std::max(0.,limit-thrust[j]),
        std::abs(missing/moment[j]),maximum_residual_delta});
    thrust[j]+=addition;missing-=addition*moment[j];
  }
  // If addition alone cannot neutralize an adverse learned pair, remove only
  // the remaining adverse differential, with the same bounded authority.
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 &&
      missing*moment[j]<0.) {
    const double reduction=std::min({thrust[j],
        std::abs(missing/moment[j]),maximum_residual_delta});
    thrust[j]-=reduction;missing+=reduction*moment[j];
  }
  return thrust;
}
// Motion 2 is an identification/reach phase, not an attitude-recovery
// manoeuvre.  Adding thrust to one released front arm can turn a small roll
// error into an earlier breakaway, a much longer lever arm, and then a full
// one-arm launch.  Correct only the part of the current pair moment which can
// be removed by lowering an adverse rotor.  The common reach command remains
// the upper envelope and the shared posture controller owns any correction
// which would require additional authority.
inline Pair dampLiftRollReductionOnly(Pair thrust,const Pair& moment,
    double roll,double rate) {
  const bool worsening=std::abs(roll)>.0174533 &&
      std::abs(rate)>.02 && roll*rate>0.;
  const double target=worsening?clamp(-.025*rate,-.010,.010):0.;
  double missing=target-(thrust[0]*moment[0]+thrust[1]*moment[1]);
  for(size_t j=0;j<2;++j) if(std::abs(moment[j])>.0049 &&
      missing*moment[j]<0.) {
    // This is a target allocator; the actuator-side fall slew below remains
    // the physical rate limit.  Capping the target correction at 0.75 N left
    // a learned [2.8,1.7]-N pair permanently asymmetric: the ordinary model
    // recreated the same excess every cycle while roll continued outward.
    // Remove the full harmful component here and let the common slew produce
    // a smooth command.  No helpful rotor is increased in free extension.
    const double reduction=std::min(thrust[j],
        std::abs(missing/moment[j]));
    thrust[j]-=reduction;
    missing+=reduction*moment[j];
  }
  return thrust;
}
// One identification pass per operator task. Subsequent strokes reuse the
// measured envelope; they are locomotion, not repeated calibration trials.
struct SinglePassConfig {
  // MuJoCo has exact joint geometry but its applied force axis is reversed.
  // Hardware keeps empirically verified signs because its pivot is unknown.
  bool reverse_geometry_roll_moment=false;
  double slip_seconds=20., lift_seconds=20., pull_seconds=4., front_wait_seconds=8.;
  // End the one-pass diagnostic when deliberate rotor force has produced no
  // new axial position for this interval.  Continuing a static-friction
  // sweep after progress has stopped only consumes posture margin.
  double slip_no_progress_seconds=3.;
  // Sliding needs both net travel and velocity evidence.  The leaky time
  // confirmation accepts intermittent stick-slip motion, while the separate
  // displacement condition prevents stationary mocap noise from passing.
  double slip_confirm_seconds=.3, slip_min_progress=.002, slip_min_speed=.0015;
  // A compliant skin/fixture can move while force is applied and spring back
  // as soon as the probe stops. Verify that displacement remains after the
  // front thrust is removed and all four arms have regrasped before selecting
  // continuous sliding as the locomotion mode.
  double slip_settle_seconds=.50, slip_settle_timeout=2.0;
  double slip_retained_ratio=.75, slip_peak_retained_ratio=.50;
  // Motion observed while bottom/rear pressure is merely settling is not a
  // rotor-driven sliding result.  Start measuring at actual rear release and
  // require a small, deliberate pair thrust before accepting the probe.
  double slip_min_thrust=.75;
  // A single angular-rate sample may be an impact or IMU spike.  Confirm the
  // soft roll brake briefly; absolute roll and the outer hard guard remain
  // independent safety limits.
  double soft_roll_confirm=.10;
  // The slip probe is diagnostic, but a tapered elliptic forearm can support
  // a repeatable non-zero roll while still producing useful axial motion.
  // Do not apply the old circular-fixture 2.5-deg rejection to that useful
  // motion.  This threshold rejects only a sustained, non-progressing roll;
  // the adapter's absolute roll/rate guard remains the safety authority.
  double slip_roll_abort_angle=.26179939; // 15 deg relative to probe start
  double slip_roll_abort_rate=.02;       // rad/s, rejects stationary bias
  // StopForLift is a zero-thrust support-transfer boundary.  Bottom
  // inflation and restoration of the rear pair can cause a short roll
  // transient there; do not turn that transient into an endless
  // gait--recovery loop.  A soft-risk trend must persist for this longer
  // interval before ownership is handed to the shared recovery controller.
  // The adapter's independent hard roll/rate guards remain immediate.
  double stop_roll_recovery_confirm=.75;
  // A stationary contact offset is acceptable during a stroke, but it also
  // consumes the roll margin available to the next free-extension transient.
  // At the quiet, empty-bottom, four-arm boundary only, recover before the
  // next stroke when this smaller budget is exhausted.  This is deliberately
  // separate from the ordinary/hard recovery thresholds and therefore never
  // interrupts a useful pulse merely because its roll is transient.
  double pre_stroke_roll_limit=.10471976; // 6 deg
  double pull_min_seconds=.5, front_catch_tolerance=.06981317;
  // Stable stroke-to-stroke adaptation strengthens only the horizontal pull.
  // Lift/extension force remains the measured contact-specific value.
  double pull_progress_threshold=.002, pull_adaptation_step=.5;
  double pull_adaptation_max=6., slip_seed_pull_min=2.5,
      slip_seed_pull_max=2.55;
  // Goal conditioning initially uses the configured nominal stride and is
  // replaced by a comparable retained measurement when available. Keep the
  // breakaway force intact, shorten the motion-2 reach, then taper the
  // motion-3 impulse while pressure completes the regrasp.
  double goal_scale_min=.30, goal_conditioning_start_ratio=.55;
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
  // Pull must hand support back to the rear pair as soon as both released
  // front arms have begun the pressure-driven return.  Requiring the exact
  // pre-release bend to be reproduced left the rear pair at 0 kPa for the
  // whole fixed pull timeout on a contact-limited cylinder.  The normalized
  // flex threshold is deliberately small: it proves that both arms have
  // reversed from extension, while the pressure/contact checks prove that the
  // return is loaded rather than an inertial rebound.
  double pull_regrip_min_flex=.08, pull_regrip_confirm=.20;
  // Residual thrust keeps the unloaded body moving while chamber pressure
  // begins flexion.  Full pull is then admitted in proportion to measured and
  // delay-predicted flexion.  This prevents extension thrust from overpowering
  // a slower chamber while preserving the intended lift--flex--pull overlap.
  double pull_residual_thrust_ratio=.35;
  // Horizontal traction must be established inside the short load-light
  // interval, not after the chambers have already rebuilt full contact.
  double pull_full_thrust_flex_ratio=.35;
  // Golden-time force trajectory.  The measured unloading force is retained
  // while the front pair begins its pneumatic return.  Horizontal traction is
  // released with measured/predicted flexion; upward support is held through
  // the first part of the return and then removed before landing.
  double pull_up_hold_flex_ratio=.60;
  double pull_forward_start_flex_ratio=.05;
  // From measured load relief through early Motion 3, drive the compliant
  // rear pair in the same (positive) direction as the front pair.  The
  // delivered-time budget is consumed only while force is actually applied;
  // pneumatic delay or a temporary gate must not silently spend the useful
  // window. Rear pressure fades the assist continuously and measured flexion
  // ends it before the newly placed front contact lands.
  double pull_rear_positive_thrust=1.2;
  double pull_rear_positive_duration=1.5;
  double pull_rear_positive_front_start=.8;
  double pull_rear_positive_front_full=3.0;
  double pull_rear_positive_rise=4.0;
  double pull_rear_positive_fall=6.0;
  double pull_rear_positive_max_pressure=12.;
  double pull_rear_positive_pressure_fade=8.;
  double pull_rear_positive_flex_end=.60;
  // After the newly placed front pair has reached the full grasp pressure,
  // use it as a pneumatic anchor and briefly lift the still-compliant rear
  // pair.  This is a separate terminal event: increasing it cannot distort
  // the measured unloading/pull waveform above.  Regrip is delayed until the
  // pulse has also ramped back to zero.
  bool pull_terminal_rear_lift_enabled=true;
  double pull_terminal_rear_lift_thrust=1.2;
  double pull_terminal_rear_lift_duration=.40;
  double pull_terminal_rear_lift_rise=8.;
  double pull_terminal_rear_lift_fall=12.;
  double pull_terminal_front_pressure_tolerance=.75;
  double pull_terminal_rear_pressure_tolerance=2.;
  double pull_terminal_anchor_wait_timeout=2.;
  double pull_terminal_front_pressure_stall_seconds=.25;
  double pull_terminal_front_pressure_progress_kpa=.20;
  // Do not let the ordinary soft roll brake terminate the short unloaded
  // interval.  Roll allocation remains active, while this separate boundary
  // starts support transfer before the independent hard guard is reached.
  double pull_golden_roll_limit=.20943951; // 12 deg stroke-relative roll
  // Rear support is transferred continuously instead of switching from full
  // grasp to exhaust at one bend threshold.  Motion 2 reduces the rear pair
  // only after BOTH front arms have produced measured extension.  Motion 3
  // then uses the slower front arm's measured flexion, so an asymmetric pair
  // cannot release the last rear anchor prematurely.
  double lift_rear_release_start_ratio=.10;
  double lift_rear_pressure_floor=0.;
  // Bounds used by the signed rear-support coordinate.  -1 maps to the
  // minimum pressure and positive rear thrust; +1 maps to the maximum
  // pressure and (only while pressure is late) reverse thrust.  The slip and
  // first unloading probes establish the prior, then the outer search applies
  // bounded offsets about it.
  bool rear_pressure_adaptation_enabled=true;
  double rear_pressure_adaptation_min=0.;
  double rear_pressure_adaptation_max=30.;
  double rear_pressure_adaptation_up_step=5.;
  double rear_pressure_adaptation_down_step=2.5;
  double rear_pressure_adaptation_roll_limit=.20943951; // 12 deg
  double rear_pressure_adaptation_progress=.002;
  double rear_support_probe_sliding=-1.;
  double rear_support_probe_stable=-.75;
  double rear_support_probe_unstable=.25;
  double rear_support_roll_start=.06981317; // 4 deg
  double rear_support_roll_full=.17453293;  // 10 deg
  // Motion 2 and Motion 3 intentionally keep the rear pair compliant.  The
  // outer all-arm regrip is the only normal path which may exceed this cap.
  // This prevents a soft roll correction from recreating the old rear
  // contact during the short lift/flex/pull interval.
  double rear_motion_pressure_cap=7.5;
  // The signed rear-support coordinate is a feed-forward request.  Close an
  // inner loop around the two measured rear bends so the same pressure means
  // the same physical release on a rigid fixture and on compliant skin.  A
  // negative support command maps continuously to a relative extension
  // target; pressure and positive thrust only trim the remaining error.
  double rear_release_angle_min=.05235988; // require at least 3 deg
  double rear_release_angle_max=.08726646; // at most 5 deg
  double rear_release_angle_filter_tau=.06;
  double rear_release_assist_deadband=.00872665; // 0.5 deg
  double rear_release_assist_confirm=.10;
  double rear_release_pressure_kp=40.;
  double rear_release_pressure_kd=2.;
  double rear_release_pressure_trim_max=5.;
  double rear_release_thrust_kp=3.;
  double rear_release_thrust_kd=.25;
  double rear_release_thrust_trim_max=.5;
  // A scalar command cannot repair a large rear-pair asymmetry.  Fade only
  // the aggressive (more release/more positive thrust) correction first;
  // pressure restoration and the independent roll guard remain available.
  double rear_release_mismatch_soft=.10471976; // 6 deg
  double rear_release_mismatch_hard=.20943951; // 12 deg
  double rear_release_realization_ratio=.5;
  // Once the front pair has caught, retain only a compliant rear support
  // while rotor thrust is being removed.  Full grasp pressure is restored by
  // the outer regrip state only after every rotor command reaches zero.
  double regrip_rear_hold_pressure=10.;
  double pull_rear_release_start_ratio=.05;
  double pull_rear_release_end_ratio=.65;
  // During Motion 3, pressure supplies both normal grip and flexion torque.
  // If flexion is allowed before normal grip has built, a compliant/high-
  // friction surface simply carries the distant contact point back toward
  // its old location.  Hold the measured extended pose with rotor feedback,
  // then release a smooth bend trajectory as measured pressure approaches
  // this capture threshold.
  double pull_capture_pressure=15.;
  double pull_angle_kp=4.,pull_angle_kd=.8,pull_angle_hold_max=.5;
  // At the Lift -> Pull boundary the chamber has not yet reversed the arm.
  // Use measured bend rate to remove only the remaining extension command;
  // otherwise the event thrust can carry one arm another 20--30 deg straight
  // while pressure is already trying to flex it.
  double pull_extension_brake_start=.0872665; // 5 deg/s
  double pull_extension_brake_full=.261799;   // 15 deg/s
  double pull_extension_brake_fall=8.;
  // Pull normally returns only to the measured grasp pressure. During Pull,
  // temporarily reduce only the pressure of the arm whose normalized flexion
  // leads; FrontCatch always restores the full grasp target.
  double pull_pressure_margin=0.;
  double pull_pressure_sync_gain=60.;
  double pull_pressure_sync_max_drop=20.;
  // Beginning rear refill does not remove front support; it only starts the
  // slow pneumatic transfer.  Waiting for the front pair to reach almost the
  // full grasp pressure kept the rear at 0 kPa for another second after both
  // arms had already flexed/contacted, allowing roll to grow past 20 deg.
  double pull_support_transfer_pressure=5.;
  // Normal rear refill is not allowed to overtake the pressure-driven pull.
  // The emergency roll branch below remains independent and may refill it
  // sooner when posture is actually deteriorating.
  double pull_support_transfer_min_flex_ratio=.55;
  // A newly placed cylinder contact normally stops at a different bend from
  // the old grasp. Once both fronts have flexed under a measured
  // load-bearing pressure, end the useful pull after a short impulse instead
  // of dragging the contact backwards until the fixed timeout.
  double pull_loaded_catch_pressure=18.;
  // Pneumatic prediction may make the front chambers load bearing very early,
  // but that is only a contact event--not proof that the horizontal pull has
  // delivered useful impulse.  Keep a short, fixed inner-loop traction dwell
  // after unloading; otherwise the better pressure predictor paradoxically
  // shortened repeated pulls to 0.8--1.0 s, while retained-forward strokes
  // consistently used about 1.7 s.
  // The front contact becomes pressure-loaded before pressure-driven flexion
  // has delivered its useful horizontal impulse.  MuJoCo endpoint tests on
  // the high-friction 100-mm fixture retained about 21 mm with a 3-s overlap,
  // whereas the former 1.5-s handoff merely recovered the displacement lost
  // while venting Motion 2.  Hard roll/rate guards may still end it earlier.
  double pull_loaded_catch_min_seconds=3.0;
  double catch_roll_angle=.17453293;
  double vent_seconds=30., vent_tolerance=5., tilt_target=.05236;
  // Vent both front chambers through one common pressure trajectory.  The
  // zero target remains active as Motion 2 starts; measured pressure may still
  // be decaying, so thrust/roll authority begins before either contact becomes
  // the sole anchor.
  double vent_pressure_fall=15.;
  // VentFront is preparation rather than part of the extension search:
  // command both front chambers to zero together, keep rotor thrust at zero,
  // and begin Motion 2 only after both measured pressures are low.  Allowing
  // learned thrust or unequal holding pressure in this phase made the next
  // stroke extend one arm while the other was still pneumatically loaded.
  double rear_low_seconds=2.0, rear_grasp_seconds=1.0;
  double rear_low_kpa=0., rear_preload_kpa=15., rear_vent_lead_margin=.10;
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
  double identification_extension_limit=1.7278760;
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
  // Motion 2 ends on a measured unloading event, not on an arm-angle target.
  // Both front arms must have genuinely extended, and either the body must
  // pitch in the commanded direction or its mocap height must rise.  The
  // absolute identification bend remains only a bounded search/safety limit.
  double unload_min_extension=.05235988; // 3 deg on each front arm
  // A body-unloading event may occur while only one compliant arm has made
  // most of the reach.  Keep the rear anchor until the second arm catches up
  // to this relative-travel corridor.  This does not force equal absolute
  // bend angles; each extension is measured from its own stroke start.
  double unload_pair_mismatch=.17453293; // 10 deg
  // The two compliant front arms need not reach the extension threshold in
  // the same IMU sample.  Remember each arm's qualified powered extension
  // briefly and accept the pair only when both observations belong to the
  // same physical unloading event.  The current mismatch and the independent
  // roll/rate guards remain active, so this is not a one-arm escape path.
  double unload_pair_memory=.15;
  // Either a pitch-led event (>=4 deg without sinking) or a height-led event
  // (>=3 mm with >=2 deg of forward pitch) is required. Signed pitch is
  // deliberate: tipping in the opposite direction is never load relief.
  double unload_pitch=.06981317;          // 4 deg in the travel direction
  double unload_height=.003;              // 3 mm body rise
  double unload_min_pitch=.03490659;      // 2 deg for height-led evidence
  double unload_height_floor=-.0005;      // at most 0.5 mm downward motion
  // Retained for force-planning diagnostics and parameter compatibility.
  // Phase completion is deliberately based on measured signed body motion,
  // because the geometric force estimate is unreliable at soft contact.
  double unload_min_up_force=.20;         // N, summed over the front pair
  // The useful unloaded interval is short.  Three 50-Hz samples are enough
  // once both powered arm onsets, relative extension and a body/contact cue
  // agree; longer confirmation lets the body settle onto the opposite
  // support before pressure-driven flexion can start.
  double unload_confirm=.06;
  // Arm IMUs, mocap and pressure respond at different instants.  Remember a
  // powered body/contact unloading cue briefly so a second arm which reaches
  // its relative-extension threshold a few samples later can complete the
  // same physical event without demanding another lift impulse.
  double unload_event_fusion=.75;
  // After the first measured event, use the learned pneumatic delay to start
  // a small pilot fill before the predicted next event.  Full flexion pressure
  // still belongs to Pull, so prefill cannot cancel the free-extension phase.
  double unload_prefill_start_ratio=.70;
  double unload_prefill_kpa=3.;
  double unload_prefill_max_prediction_sec=.60;
  double unload_prefill_max_prediction_lead_ratio=.15;
  double horizontal_bend=1.570796, pressure_gain=20.;
  double lift_pressure_gain=20.;
  double lift_rise=.7, reused_lift_rise=8.;
  // The first Motion-2 reach is a short two-arm impulse, not a slow force
  // sweep. Both released fronts rise toward the same bounded target before
  // any per-arm onset model exists. Once both powered onsets are measured,
  // their own angle references and rate feedback keep both arms on the short
  // outward trajectory until the unloading event starts pneumatic flexion.
  double initial_pulse_thrust=2.;
  double initial_pulse_max=4.;
  double initial_pulse_step=.5;
  double initial_pulse_step_seconds=1.;
  double initial_pulse_rise=8.;
  double initial_pulse_extension_rate=.70;
  double initial_pulse_min_duration=.12;
  // Unified Motion-2/3 bend trajectory.  Motion 2 must execute a meaningful
  // two-arm reach before a body unloading cue may end it; the old 3-deg arm-
  // only shortcut is deliberately not a completion condition.  Angle/rate
  // feedback corrects the nominal rotor force and its bounded integral trim is
  // retained across strokes to learn repeatable contact/model bias.
  double trajectory_min_extension=.26179939; // 15 deg per front arm
  double trajectory_extension_rate=.70;
  double trajectory_extension_min_duration=.30;
  double trajectory_extension_max_duration=3.0;
  double trajectory_tracking_slow_error=.08726646; // 5 deg
  double trajectory_tracking_pause_error=.26179939; // 15 deg
  // Fast state prediction shared by Motion 2 and Motion 3.  The absolute
  // angle remains accelerometer based; the filtered relative gyro supplies
  // rate and a bounded differentiated acceleration.  These terms change the
  // trajectory clock and actuator lead, but never complete Motion 2 alone.
  double trajectory_rate_filter_tau=.04;
  double trajectory_acceleration_filter_tau=.08;
  double trajectory_acceleration_limit=20.;
  double trajectory_prediction_horizon=.08;
  double trajectory_prediction_max_delta=.26179939; // 15 deg
  double trajectory_rate_lag_horizon=.08;
  double trajectory_phase_correction_gain=1.5;
  double trajectory_clock_max_scale=1.5;
  double trajectory_extension_kp=5.;
  double trajectory_extension_kd=.8;
  double trajectory_extension_ki=.35;
  double trajectory_extension_trim_max=1.25;
  // A small pitch/height response only says that the front contact has begun
  // to unload.  In the deep-windup gait, extend a further bounded distance,
  // then apply a partial pneumatic brake.  The brake is strong enough to
  // arrest the fast compliant-arm motion but remains below grasp pressure, so
  // rotor thrust can keep rotating the body about the rear anchor.  Full
  // pressure begins only at the requested pitch or the independent bend
  // safety boundary; that pressure rise is also the start of the forward
  // flexion/swing.  A value no larger than unload_pitch retains the legacy
  // immediate handoff for isolated tests.
  // Compatibility default for direct unit construction. ROS launch/YAML set
  // the operational deep-windup target to 17 deg below the 20-deg guard.
  double trajectory_windup_pitch=.06981317;
  // The body load-relief event already contains paired arm travel and a
  // measured pitch/height/contact cue.  In the operational gait, hand this
  // event directly to the pressure-driven flex trajectory while the body is
  // light, rather than extending toward a second fixed pitch threshold.
  bool trajectory_golden_handoff=true;
  double trajectory_windup_pitch_margin=.05235988; // 3 deg before hard guard
  double trajectory_windup_confirm=.04;
  double trajectory_windup_thrust_max=6.;
  double trajectory_windup_thrust_rise=2.;
  double trajectory_windup_pitch_kp=3.; // common front thrust N/rad
  double trajectory_windup_pitch_kd=1.; // common front thrust N/(rad/s)
  double trajectory_windup_pitch_rate_max=.25;
  // Reduce only the broad bottom support after measured load relief so the
  // rear grasp becomes the pitch pivot. Pull restores bottom_kpa while front
  // pressure bends the arms and converts the stored pose into translation.
  double trajectory_windup_bottom_kpa=30.;
  double trajectory_windup_extra_extension=1.04719755; // seek up to 60 deg beyond measured load relief
  double trajectory_windup_hold_pressure_max=30.;
  double trajectory_windup_brake_pressure_min=5.;
  double trajectory_windup_brake_pressure_max=12.;
  double trajectory_windup_brake_angle_kp=15.; // kPa/rad beyond planned reach
  double trajectory_windup_brake_rate_kd=2.; // kPa/(rad/s) above capture rate
  double trajectory_windup_capture_extension=.05235988; // full catch 3 deg before the predicted hard bend boundary
  double trajectory_windup_capture_rate=.17453293; // 10 deg/s
  double trajectory_windup_capture_confirm=.04;
  // Once the fast pneumatic catch has arrested extension, pressure must not
  // jump between the inverse-model limits.  Such jumps excite the compliant
  // arm faster than its valve/chamber response and destroy the long moment
  // arm which the pitch windup needs.  The initial catch itself remains a
  // full-pressure command; this slew applies only to the subsequent hold.
  double trajectory_windup_pressure_slew=12.; // kPa/s
  double trajectory_windup_min_reachable_pitch=.06981317; // 4 deg legacy default
  double trajectory_windup_stall_seconds=1.25;
  // A small reverse force on the still-grasping rear pair closes the pitch
  // moment couple when front lift alone translates the body without rotating
  // it.  It is ramped only after measured load relief and remains well below
  // the front-pair authority.
  double trajectory_windup_rear_reverse_max=1.5;
  double trajectory_windup_rear_reverse_rise=.75;
  // Motion 3 follows the return bend reference generated by the same path.
  // Learned pneumatic delay advances pressure feed-forward; measured angle
  // and angular rate close the remaining loop.
  double trajectory_flex_rate=1.2;
  double trajectory_flex_min_duration=.65;
  double trajectory_flex_max_duration=2.0;
  double trajectory_pressure_kp=28.;
  double trajectory_pressure_kd=3.;
  double trajectory_pressure_ki=4.;
  double trajectory_pressure_trim_max=8.;
  double trajectory_pressure_lead_max=.60;
  double trajectory_pull_thrust_kp=2.;
  double trajectory_pull_thrust_kd=.35;
  double trajectory_pull_thrust_trim_max=.75;
  // The last measured unloading pose is a fast-approach checkpoint, not a
  // phase-completion condition. If current contact has not unloaded there,
  // continue with a much slower force search instead of racing toward the
  // independent 45-deg safety boundary.
  double reused_event_checkpoint_margin=.06981317;
  double reused_event_search_rise=.25;
  // After calibration, permit the measured pair feed-forward to establish
  // rotor authority before a soft roll trend hands the stroke to the shared
  // recovery. The independent hard-roll guard remains active in the adapter.
  double reused_lift_roll_grace=.5;
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
        slip_confirm_seconds,slip_min_progress,slip_min_speed,
        slip_no_progress_seconds,
        slip_settle_seconds,slip_settle_timeout,slip_retained_ratio,
        slip_peak_retained_ratio,slip_min_thrust,
        soft_roll_confirm,slip_roll_abort_angle,slip_roll_abort_rate,
        stop_roll_recovery_confirm,pre_stroke_roll_limit,
        pull_min_seconds,
        front_catch_tolerance,pull_progress_threshold,pull_adaptation_step,
        pull_adaptation_max,slip_seed_pull_min,slip_seed_pull_max,
        goal_scale_min,
        goal_conditioning_start_ratio,goal_progress_max_scale,
        goal_preferred_stride,
        goal_extension_min_scale,goal_pull_taper_fraction,
        catch_stall_confirm,catch_stall_rate,catch_safe_margin,
        catch_pressure_tolerance,catch_roll_rate,catch_roll_prediction_tolerance,
        pull_regrip_min_flex,pull_regrip_confirm,
        pull_residual_thrust_ratio,pull_full_thrust_flex_ratio,
        pull_up_hold_flex_ratio,pull_forward_start_flex_ratio,
        pull_rear_positive_thrust,pull_rear_positive_duration,
        pull_rear_positive_front_start,pull_rear_positive_front_full,
        pull_rear_positive_rise,pull_rear_positive_fall,
        pull_rear_positive_max_pressure,pull_rear_positive_pressure_fade,
        pull_rear_positive_flex_end,
        pull_terminal_rear_lift_thrust,pull_terminal_rear_lift_duration,
        pull_terminal_rear_lift_rise,pull_terminal_rear_lift_fall,
        pull_terminal_front_pressure_tolerance,
        pull_terminal_rear_pressure_tolerance,
        pull_terminal_anchor_wait_timeout,
        pull_terminal_front_pressure_stall_seconds,
        pull_terminal_front_pressure_progress_kpa,
        pull_golden_roll_limit,
        lift_rear_release_start_ratio,lift_rear_pressure_floor,
        rear_pressure_adaptation_min,rear_pressure_adaptation_max,
        rear_pressure_adaptation_up_step,rear_pressure_adaptation_down_step,
        rear_pressure_adaptation_roll_limit,
        rear_pressure_adaptation_progress,
        rear_support_roll_start,rear_support_roll_full,
        rear_motion_pressure_cap,
        rear_release_angle_min,rear_release_angle_max,
        rear_release_angle_filter_tau,
        rear_release_assist_deadband,rear_release_assist_confirm,
        rear_release_pressure_kp,rear_release_pressure_kd,
        rear_release_pressure_trim_max,
        rear_release_thrust_kp,rear_release_thrust_kd,
        rear_release_thrust_trim_max,
        rear_release_mismatch_soft,rear_release_mismatch_hard,
        rear_release_realization_ratio,
        regrip_rear_hold_pressure,
        pull_rear_release_start_ratio,pull_rear_release_end_ratio,
        pull_capture_pressure,pull_angle_kp,pull_angle_kd,pull_angle_hold_max,
        pull_extension_brake_start,pull_extension_brake_full,
        pull_extension_brake_fall,
        pull_pressure_margin,pull_pressure_sync_gain,
        pull_pressure_sync_max_drop,pull_support_transfer_pressure,
        pull_support_transfer_min_flex_ratio,
        pull_loaded_catch_pressure,pull_loaded_catch_min_seconds,
        catch_roll_angle,
        vent_seconds,vent_tolerance,tilt_target,vent_pressure_fall,
        rear_low_seconds,rear_grasp_seconds,rear_low_kpa,rear_preload_kpa,
        rear_vent_lead_margin,
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
        unload_min_extension,unload_pair_mismatch,unload_pair_memory,
        unload_pitch,unload_height,unload_min_pitch,
        unload_min_up_force,
        unload_confirm,unload_event_fusion,
        unload_prefill_start_ratio,unload_prefill_kpa,
        unload_prefill_max_prediction_sec,
        unload_prefill_max_prediction_lead_ratio,
        horizontal_bend,pressure_gain,lift_pressure_gain,lift_rise,reused_lift_rise,
        initial_pulse_thrust,initial_pulse_max,initial_pulse_step,
        initial_pulse_step_seconds,initial_pulse_rise,
        initial_pulse_extension_rate,initial_pulse_min_duration,
        trajectory_min_extension,trajectory_extension_rate,
        trajectory_extension_min_duration,trajectory_extension_max_duration,
        trajectory_tracking_slow_error,trajectory_tracking_pause_error,
        trajectory_rate_filter_tau,trajectory_acceleration_filter_tau,
        trajectory_acceleration_limit,trajectory_prediction_horizon,
        trajectory_prediction_max_delta,trajectory_rate_lag_horizon,
        trajectory_phase_correction_gain,trajectory_clock_max_scale,
        trajectory_extension_kp,trajectory_extension_kd,
        trajectory_extension_ki,trajectory_extension_trim_max,
        trajectory_windup_pitch,trajectory_windup_pitch_margin,
        trajectory_windup_confirm,trajectory_windup_thrust_max,
        trajectory_windup_thrust_rise,
        trajectory_windup_pitch_kp,trajectory_windup_pitch_kd,
        trajectory_windup_pitch_rate_max,
        trajectory_windup_bottom_kpa,
        trajectory_windup_extra_extension,
        trajectory_windup_hold_pressure_max,
        trajectory_windup_brake_pressure_min,
        trajectory_windup_brake_pressure_max,
        trajectory_windup_brake_angle_kp,
        trajectory_windup_brake_rate_kd,
        trajectory_windup_capture_extension,
        trajectory_windup_capture_rate,
        trajectory_windup_capture_confirm,
        trajectory_windup_pressure_slew,
        trajectory_windup_min_reachable_pitch,
        trajectory_windup_stall_seconds,
        trajectory_windup_rear_reverse_max,
        trajectory_windup_rear_reverse_rise,
        trajectory_flex_rate,trajectory_flex_min_duration,
        trajectory_flex_max_duration,trajectory_pressure_kp,
        trajectory_pressure_kd,trajectory_pressure_ki,
        trajectory_pressure_trim_max,trajectory_pressure_lead_max,
        trajectory_pull_thrust_kp,trajectory_pull_thrust_kd,
        trajectory_pull_thrust_trim_max,
        reused_event_checkpoint_margin,reused_event_search_rise,
        reused_lift_roll_grace,
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
      slip_no_progress_seconds>=1. && slip_no_progress_seconds<=8. &&
      slip_min_progress>=.001 && slip_min_progress<=.02 &&
      slip_min_speed>=.0005 && slip_min_speed<=.02 &&
      slip_settle_seconds>=.2 && slip_settle_seconds<=2. &&
      slip_settle_timeout>=slip_settle_seconds &&
      slip_settle_timeout<=5. &&
      slip_retained_ratio>=.5 && slip_retained_ratio<=1. &&
      slip_peak_retained_ratio>=.25 && slip_peak_retained_ratio<=1. &&
      slip_min_thrust>=.3 && slip_min_thrust<=2. &&
      soft_roll_confirm>=.04 && soft_roll_confirm<=.5 &&
      slip_roll_abort_angle>=.0349066 &&
      slip_roll_abort_angle<=.349066 &&
      slip_roll_abort_rate>=.005 && slip_roll_abort_rate<=.1 &&
      stop_roll_recovery_confirm>=soft_roll_confirm &&
      stop_roll_recovery_confirm<=2.0 &&
      pre_stroke_roll_limit>=.0523599 &&
      pre_stroke_roll_limit<=.174533 &&
      pull_min_seconds>=.5 && pull_min_seconds<=pull_seconds &&
      front_catch_tolerance>=.017453 && front_catch_tolerance<=.174533 &&
      pull_progress_threshold>=.001 && pull_progress_threshold<=.01 &&
      pull_adaptation_step>0. && pull_adaptation_step<=1. &&
      pull_adaptation_max>=1. && pull_adaptation_max<=12. &&
      slip_seed_pull_min>=1. && slip_seed_pull_min<=slip_seed_pull_max &&
      slip_seed_pull_max<=pull_adaptation_max &&
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
      pull_regrip_min_flex>=.02 && pull_regrip_min_flex<=.4 &&
      pull_regrip_confirm>=.08 && pull_regrip_confirm<=1. &&
      pull_residual_thrust_ratio>=.1 && pull_residual_thrust_ratio<=.8 &&
      pull_full_thrust_flex_ratio>=.2 &&
      pull_full_thrust_flex_ratio<=1. &&
      pull_up_hold_flex_ratio>=.2 &&
      pull_up_hold_flex_ratio<=1. &&
      pull_forward_start_flex_ratio>=0. &&
      pull_forward_start_flex_ratio<pull_full_thrust_flex_ratio &&
      pull_rear_positive_thrust>=0. && pull_rear_positive_thrust<=3. &&
      pull_rear_positive_duration>=.1 &&
      pull_rear_positive_duration<=3. &&
      pull_rear_positive_front_start>=.2 &&
      pull_rear_positive_front_start<pull_rear_positive_front_full &&
      pull_rear_positive_front_full<=6. &&
      pull_rear_positive_rise>=.5 && pull_rear_positive_rise<=10. &&
      pull_rear_positive_fall>=1. && pull_rear_positive_fall<=20. &&
      pull_rear_positive_max_pressure>=rear_low_kpa &&
      pull_rear_positive_max_pressure<=20. &&
      pull_rear_positive_pressure_fade>=1. &&
      pull_rear_positive_pressure_fade<=20. &&
      pull_rear_positive_max_pressure+pull_rear_positive_pressure_fade<=30. &&
      pull_rear_positive_flex_end>=pull_full_thrust_flex_ratio &&
      pull_rear_positive_flex_end<=pull_up_hold_flex_ratio &&
      pull_terminal_rear_lift_thrust>=0. &&
      pull_terminal_rear_lift_thrust<=3. &&
      pull_terminal_rear_lift_duration>=.1 &&
      pull_terminal_rear_lift_duration<=1. &&
      pull_terminal_rear_lift_rise>=1. &&
      pull_terminal_rear_lift_rise<=20. &&
      pull_terminal_rear_lift_fall>=2. &&
      pull_terminal_rear_lift_fall<=30. &&
      pull_terminal_rear_pressure_tolerance>=.5 &&
      pull_terminal_rear_pressure_tolerance<=5. &&
      pull_terminal_anchor_wait_timeout>=.5 &&
      pull_terminal_anchor_wait_timeout<=5. &&
      pull_terminal_front_pressure_stall_seconds>=.10 &&
      pull_terminal_front_pressure_stall_seconds<=2. &&
      pull_terminal_front_pressure_progress_kpa>=.05 &&
      pull_terminal_front_pressure_progress_kpa<=2. &&
      pull_terminal_front_pressure_tolerance>=.5 &&
      pull_terminal_front_pressure_tolerance<=5. &&
      pull_golden_roll_limit>=soft_roll &&
      pull_golden_roll_limit<=.349066 &&
      lift_rear_release_start_ratio>=0. &&
      lift_rear_release_start_ratio<=.8 &&
      lift_rear_pressure_floor>=rear_low_kpa &&
      lift_rear_pressure_floor<=30. &&
      rear_pressure_adaptation_min>=rear_low_kpa &&
      rear_pressure_adaptation_min<=rear_pressure_adaptation_max &&
      rear_pressure_adaptation_max<=30. &&
      rear_pressure_adaptation_up_step>0. &&
      rear_pressure_adaptation_up_step<=10. &&
      rear_pressure_adaptation_down_step>0. &&
      rear_pressure_adaptation_down_step<=10. &&
      rear_pressure_adaptation_roll_limit>=soft_roll &&
      rear_pressure_adaptation_roll_limit<=.349066 &&
      rear_pressure_adaptation_progress>=.0005 &&
      rear_pressure_adaptation_progress<=.02 &&
      std::isfinite(rear_support_probe_sliding) &&
      std::isfinite(rear_support_probe_stable) &&
      std::isfinite(rear_support_probe_unstable) &&
      rear_support_probe_sliding>=-1. &&
      rear_support_probe_sliding<=rear_support_probe_stable &&
      rear_support_probe_stable<=0. &&
      rear_support_probe_unstable>=0. &&
      rear_support_probe_unstable<=1. &&
      rear_support_roll_start>=.0174533 &&
      rear_support_roll_start<rear_support_roll_full &&
      rear_support_roll_full<=rear_pressure_adaptation_roll_limit &&
      rear_motion_pressure_cap>=rear_low_kpa &&
      rear_motion_pressure_cap<=15. &&
      rear_release_angle_min>=.0349066 &&
      rear_release_angle_min<=.0872665 &&
      rear_release_angle_max>=.0349066 &&
      rear_release_angle_max<=.261799 &&
      rear_release_angle_min<=rear_release_angle_max &&
      rear_release_angle_filter_tau>=.01 &&
      rear_release_angle_filter_tau<=.3 &&
      rear_release_assist_deadband>=0. &&
      rear_release_assist_deadband<=.0349066 &&
      rear_release_assist_deadband<rear_release_angle_min &&
      rear_release_assist_confirm>=.04 &&
      rear_release_assist_confirm<=.5 &&
      rear_release_pressure_kp>=0. && rear_release_pressure_kp<=100. &&
      rear_release_pressure_kd>=0. && rear_release_pressure_kd<=10. &&
      rear_release_pressure_trim_max>=0. &&
      rear_release_pressure_trim_max<=10. &&
      rear_release_thrust_kp>=0. && rear_release_thrust_kp<=10. &&
      rear_release_thrust_kd>=0. && rear_release_thrust_kd<=2. &&
      rear_release_thrust_trim_max>=0. &&
      rear_release_thrust_trim_max<=1.5 &&
      rear_release_mismatch_soft>=.0174533 &&
      rear_release_mismatch_soft<rear_release_mismatch_hard &&
      rear_release_mismatch_hard<=.349066 &&
      rear_release_realization_ratio>=.2 &&
      rear_release_realization_ratio<=1. &&
      regrip_rear_hold_pressure>=rear_low_kpa &&
      regrip_rear_hold_pressure<=30. &&
      pull_rear_release_start_ratio>=0. &&
      pull_rear_release_start_ratio< pull_rear_release_end_ratio &&
      pull_rear_release_end_ratio<=1. &&
      pull_capture_pressure>=pull_support_transfer_pressure &&
      pull_capture_pressure<=40. &&
      pull_angle_kp>=0. && pull_angle_kp<=12. &&
      pull_angle_kd>=0. && pull_angle_kd<=3. &&
      pull_angle_hold_max>=0. && pull_angle_hold_max<=2. &&
      pull_extension_brake_start>=0. &&
      pull_extension_brake_full>pull_extension_brake_start &&
      pull_extension_brake_full<=1.0 &&
      pull_extension_brake_fall>=1. &&
      pull_extension_brake_fall<=20. &&
      pull_pressure_margin>=0. && pull_pressure_margin<=10. &&
      pull_pressure_sync_gain>=0. && pull_pressure_sync_gain<=60. &&
      pull_pressure_sync_max_drop>=0. &&
      pull_pressure_sync_max_drop<=20. &&
      pull_support_transfer_pressure>=3. &&
      pull_support_transfer_pressure<=20. &&
      pull_support_transfer_min_flex_ratio>=
          pull_full_thrust_flex_ratio &&
      pull_support_transfer_min_flex_ratio<=1. &&
      pull_loaded_catch_pressure>=pull_support_transfer_pressure &&
      pull_loaded_catch_pressure<=40. &&
      pull_loaded_catch_min_seconds>=.5 &&
      pull_loaded_catch_min_seconds<=pull_seconds &&
      catch_roll_angle>=soft_roll && catch_roll_angle<=.349066 &&
      vent_seconds>=2. && vent_seconds<=60. && vent_tolerance>0. && vent_tolerance<=5. &&
      vent_pressure_fall>=2. && vent_pressure_fall<=60. &&
      reused_lift_roll_grace>=0. && reused_lift_roll_grace<=1. &&
      tilt_target>=.017453 && tilt_target<=.10472 &&
      rear_low_seconds>=.2 && rear_low_seconds<=10 && rear_grasp_seconds>=.2 && rear_grasp_seconds<=10 &&
      rear_low_kpa<=6 && rear_preload_kpa>=rear_low_kpa &&
      rear_preload_kpa<=30. && rear_vent_lead_margin<=1. &&
      rear_vent_start_margin>=.017453 && rear_vent_start_margin<=.2618 &&
      bottom_kpa<=30 && extension_rad>=.035 && extension_rad<=.262 &&
      identification_bend>=0. && identification_bend<=1.047198 &&
      unload_min_up_force<=2. &&
      identification_tolerance>0. && identification_tolerance<=.087267 &&
      identification_confirm>=.04 && identification_confirm<=.25 &&
      identification_extension_limit>=.349066 && identification_extension_limit<=2.094396 &&
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
      unload_min_extension>=.017453 && unload_min_extension<=.174533 &&
      unload_pair_mismatch>=identification_sync_deadband &&
      unload_pair_mismatch<=.349066 &&
      unload_pair_memory>=unload_confirm && unload_pair_memory<=.5 &&
      unload_pitch>=.0087266 && unload_pitch<=.174533 &&
      unload_height>=.0005 && unload_height<=.01 &&
      unload_min_pitch>=.0087266 && unload_min_pitch<=unload_pitch &&
      unload_height_floor>=-.003 && unload_height_floor<=0. &&
      unload_confirm>=.06 && unload_confirm<=.5 &&
      unload_event_fusion>=unload_confirm &&
      unload_event_fusion<=1.5 &&
      unload_prefill_start_ratio>=.4 && unload_prefill_start_ratio<=.95 &&
      unload_prefill_kpa>=1. && unload_prefill_kpa<=10. &&
      unload_prefill_max_prediction_sec>=.1 &&
      unload_prefill_max_prediction_sec<=1.2 &&
      unload_prefill_max_prediction_lead_ratio>=.05 &&
      unload_prefill_max_prediction_lead_ratio<=.3 &&
      horizontal_bend>=1.2 && horizontal_bend<=1.9 && pressure_gain>0 &&
      lift_pressure_gain>0 && lift_pressure_gain<=150. &&
      lift_rise>0 && lift_rise<=1 &&
      reused_lift_rise>=lift_rise && reused_lift_rise<=8. &&
      initial_pulse_thrust>=.5 && initial_pulse_thrust<=4. &&
      initial_pulse_max>=initial_pulse_thrust &&
      initial_pulse_max<=6. && initial_pulse_step>=.1 &&
      initial_pulse_step<=1. && initial_pulse_step_seconds>=.5 &&
      initial_pulse_step_seconds<=3. &&
      initial_pulse_rise>=2. && initial_pulse_rise<=12. &&
      initial_pulse_extension_rate>=.2 &&
      initial_pulse_extension_rate<=1.5 &&
      initial_pulse_min_duration>=.06 &&
      initial_pulse_min_duration<=.5 &&
      trajectory_min_extension>=.087266 &&
      trajectory_min_extension<=.785399 &&
      trajectory_extension_rate>=.2 && trajectory_extension_rate<=2. &&
      trajectory_extension_min_duration>=.1 &&
      trajectory_extension_max_duration>=trajectory_extension_min_duration &&
      trajectory_extension_max_duration<=5. &&
      trajectory_tracking_slow_error>=.017453 &&
      trajectory_tracking_pause_error>trajectory_tracking_slow_error &&
      trajectory_tracking_pause_error<=.523599 &&
      trajectory_rate_filter_tau>=.01 && trajectory_rate_filter_tau<=.2 &&
      trajectory_acceleration_filter_tau>=trajectory_rate_filter_tau &&
      trajectory_acceleration_filter_tau<=.5 &&
      trajectory_acceleration_limit>=2. && trajectory_acceleration_limit<=50. &&
      trajectory_prediction_horizon>=.02 &&
      trajectory_prediction_horizon<=.25 &&
      trajectory_prediction_max_delta>=.0349066 &&
      trajectory_prediction_max_delta<=.523599 &&
      trajectory_rate_lag_horizon>=.02 &&
      trajectory_rate_lag_horizon<=.25 &&
      trajectory_phase_correction_gain>=0. &&
      trajectory_phase_correction_gain<=5. &&
      trajectory_clock_max_scale>=1. && trajectory_clock_max_scale<=2. &&
      trajectory_extension_kp<=20. && trajectory_extension_kd<=5. &&
      trajectory_extension_ki<=5. && trajectory_extension_trim_max<=3. &&
      trajectory_windup_pitch>=unload_pitch &&
      trajectory_windup_pitch<=.436333 &&
      trajectory_windup_pitch_margin>=.017453 &&
      trajectory_windup_pitch_margin<=.174533 &&
      trajectory_windup_confirm>=.02 &&
      trajectory_windup_confirm<=.2 &&
      trajectory_windup_thrust_max>=initial_pulse_thrust &&
      trajectory_windup_thrust_max<=6. &&
      trajectory_windup_thrust_rise>=.25 &&
      trajectory_windup_thrust_rise<=8. &&
      trajectory_windup_pitch_kp>=0. &&
      trajectory_windup_pitch_kp<=20. &&
      trajectory_windup_pitch_kd>=0. &&
      trajectory_windup_pitch_kd<=10. &&
      trajectory_windup_pitch_rate_max>=.05 &&
      trajectory_windup_pitch_rate_max<=.5 &&
      trajectory_windup_bottom_kpa<=bottom_kpa &&
      trajectory_windup_extra_extension>=.0872665 &&
      trajectory_windup_extra_extension<=1.396264 &&
      trajectory_windup_hold_pressure_max>=3. &&
      trajectory_windup_hold_pressure_max<=50. &&
      trajectory_windup_brake_pressure_min>=0. &&
      trajectory_windup_brake_pressure_min<=trajectory_windup_brake_pressure_max &&
      trajectory_windup_brake_pressure_max<=trajectory_windup_hold_pressure_max &&
      trajectory_windup_brake_angle_kp>=0. &&
      trajectory_windup_brake_angle_kp<=100. &&
      trajectory_windup_brake_rate_kd>=0. &&
      trajectory_windup_brake_rate_kd<=20. &&
      trajectory_windup_capture_extension>=.0349066 &&
      trajectory_windup_capture_extension<=.523599 &&
      trajectory_windup_capture_rate>=.0349066 &&
      trajectory_windup_capture_rate<=.523599 &&
      trajectory_windup_capture_confirm>=.02 &&
      trajectory_windup_capture_confirm<=.25 &&
      trajectory_windup_pressure_slew>=2. &&
      trajectory_windup_pressure_slew<=40. &&
      trajectory_windup_min_reachable_pitch>=unload_pitch &&
      trajectory_windup_min_reachable_pitch<=trajectory_windup_pitch &&
      trajectory_windup_stall_seconds>=.2 &&
      trajectory_windup_stall_seconds<=2. &&
      trajectory_windup_rear_reverse_max>=0. &&
      trajectory_windup_rear_reverse_max<=3. &&
      trajectory_windup_rear_reverse_rise>=.1 &&
      trajectory_windup_rear_reverse_rise<=4. &&
      trajectory_flex_rate>=.2 && trajectory_flex_rate<=4. &&
      trajectory_flex_min_duration>=.2 &&
      trajectory_flex_max_duration>=trajectory_flex_min_duration &&
      trajectory_flex_max_duration<=5. &&
      trajectory_pressure_kp<=100. && trajectory_pressure_kd<=20. &&
      trajectory_pressure_ki<=20. && trajectory_pressure_trim_max<=20. &&
      trajectory_pressure_lead_max>=.05 &&
      trajectory_pressure_lead_max<=1.2 &&
      trajectory_pull_thrust_kp<=10. &&
      trajectory_pull_thrust_kd<=3. &&
      trajectory_pull_thrust_trim_max<=2. &&
      reused_event_checkpoint_margin>=.0174533 &&
      reused_event_checkpoint_margin<=.174533 &&
      reused_event_search_rise>=.05 &&
      reused_event_search_rise<=lift_rise &&
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
  double goal_scale_min=.30, goal_progress_max_scale=1.5;
  double goal_preferred_stride=.010;
  // Positive means that the side facing the requested travel direction has
  // risen about the rear contact. Standard RPY pitch has the opposite sign
  // for +x travel, so the adapter performs that coordinate conversion.
  double directional_pitch=0.;
  // Maximum of the two measured rear pressures.  Using the slower side makes
  // the Lift -> Pull handoff wait until both rear supports are released.
  double rear_pressure=0.;
  // Absolute bend and relative bend rate of the two rear arms.  As for the
  // front pair, zero bend is straight and a positive rate is flexion.  The
  // controller captures a per-arm grasp reference, so unequal initial bends
  // and object diameter do not bias the release measurement.
  Pair rear_bend{},rear_rate{};
  bool rear_kinematics_valid=false;
  // front_released means that at least one of the two front contacts has
  // unloaded.  Motion 2 already requires measured extension of BOTH arms, so
  // waiting for both binary contacts to open only detects the subsequent
  // snap-through, not the beginning of useful load relief.
  bool speed_valid=false, front_contact=true, front_released=false,
      contact_available=false;
  bool rear_released=false, rear_grasped=false;
};
struct SinglePassOutput {
  Pair thrust{}, pressure{}, angle_target{};
  double rear_pressure=0., rear_reverse_thrust=0.,
      rear_positive_thrust=0., bottom=30.;
  double desired_up=0., desired_forward=0., ratio=1.;
  bool regrip=false, contact_limited_regrip=false;
  bool fault=false, recover_roll=false, rear_low=true;
  // A strongly asymmetric first unloading event is used to identify the next
  // reach, then all four arms are restored before Motion 3 is attempted.
  bool calibration_regrip=false;
};
class SinglePass {
 public:
  // Motion 2 can extend an arm within a few controller samples.  Keep a short
  // history so an onset which is confirmed later can still recover the pose
  // immediately before powered motion.  This prevents bottom inflation,
  // passive pressure settling and mocap drift from becoming apparent lift.
  struct LiftHistorySample {
    double time=std::numeric_limits<double>::quiet_NaN();
    Pair bend{};
    double height=0.,pitch=0.;
  };
  static constexpr size_t lift_history_capacity=16;
  enum Phase { SlipProbe=0, Slide=1, Lift=2, Pull=3, FrontCatch=4,
      StopForLift=5, VentFront=6, SlipSettle=7 };
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
  std::array<LiftHistorySample,lift_history_capacity> lift_history{};
  size_t lift_history_head=0,lift_history_count=0;
  std::array<LiftHistorySample,2> onset_baseline_candidate{};
  std::array<bool,2> onset_baseline_candidate_valid{{false,false}};
  Pair powered_extension_reference{};
  std::array<bool,2> powered_extension_reference_valid{{false,false}};
  double powered_start_height=0.,powered_start_pitch=0.;
  double powered_body_reference_time=std::numeric_limits<double>::infinity();
  bool powered_body_reference_valid=false;
  double slide_best_x=std::numeric_limits<double>::quiet_NaN();
  double slip_candidate_peak_x=std::numeric_limits<double>::quiet_NaN();
  double slip_settle_mean_x=0.,slip_retained_progress=0.;
  Pair slip_settle_pressure{};
  // A confirmed outward roll during the diagnostic probe means that the
  // current contact cannot use continuous sliding safely.  Still pass
  // through SlipSettle so that the decision uses a zero-thrust retained
  // endpoint, but never promote that transient displacement to Slide.
  bool slip_probe_unstable=false;
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
  double vent_common_pressure=0.;
  Pair lift_prefill_command{};
  Pair lift_angle_pressure_command{};
  Pair lift_target_hold_thrust{};
  Pair overlap_peak_extension{};
  // Measured body-unloading event and its reusable local prediction.  The
  // event is reset every Lift; the learned bend/thrust remain for this task.
  double unload_evidence_time=0.,unload_body_candidate_time=0.,
      unload_body_memory_time=0.,
      unload_alignment_time=0.;
  // Per-arm dwell memory for an asynchronous two-arm unloading event.  It is
  // diagnostic state as well as a gate: both timers must overlap before
  // Motion 2 can hand off to pressure-driven flexion.
  Pair unload_extension_memory{};
  double unload_pair_mismatch_measured=0.;
  // Load relief is the start of windup, not necessarily the Lift->Pull
  // boundary.  Deep-windup operation latches this early cue and continues the
  // zero-pressure reach until predicted body pitch reaches the bounded target.
  // A candidate is an early, deliberately weaker version of the full load-
  // relief event.  It may start the rear assist/valve lead, but it can never
  // complete Motion 2; only load_relief_detected owns that transition.
  bool load_relief_candidate=false,load_relief_detected=false;
  double load_relief_elapsed=std::numeric_limits<double>::quiet_NaN();
  Pair load_relief_thrust{},load_relief_bend{},windup_brake_start_bend{},
      windup_reference_bend{},windup_hold_bend{},windup_hold_pressure{};
  double windup_pitch_target=0.,windup_pitch_progress=0.;
  double windup_pitch_direction=-1.;
  double windup_evidence_time=0.,windup_rear_reverse_thrust=0.;
  double windup_capture_evidence_time=0.,windup_best_pitch=0.;
  double windup_pitch_stall_time=0.;
  std::array<bool,2> windup_brake_started_by_arm{{false,false}};
  std::array<bool,2> windup_target_reached_by_arm{{false,false}};
  bool windup_extension_reached=false,windup_target_reached=false,
      windup_pose_captured=false,windup_reachable_peak=false;
  bool unload_event_detected=false,unload_capture_started=false;
  // True when Motion 2 was ended by the short, powered two-arm extension
  // trajectory rather than by a slower mocap/contact cue.  This is retained
  // only for diagnostics; both paths enter the same guarded capture and Pull.
  bool unload_from_arm_trajectory=false;
  // A front contact which is already open after the deliberate zero-pressure
  // preparation is not a new Motion-2 unloading observation.  Arm the direct
  // contact cue only after both fronts have first been observed supported in
  // the powered phase; pitch/height evidence remains available throughout.
  bool front_release_event_armed=false;
  double unload_pitch_delta=0.,unload_height_delta=0.;
  double unload_event_elapsed=std::numeric_limits<double>::quiet_NaN();
  // Fixed pair target chosen once at the measured unloading event.  Keeping
  // this target stationary prevents the extension leader/lagger roles from
  // swapping while chamber pressure and rotor thrust have different delays.
  double unload_alignment_extension_target=0.;
  // Contact-conditioned duration of the measured Motion-2 unloading reach.
  // Unlike unload_event_elapsed, this survives enter(Lift) and is used only
  // to schedule rear exhaust on later strokes; it never completes a phase.
  double learned_unload_elapsed=std::numeric_limits<double>::quiet_NaN();
  Pair unload_event_bend{},unload_event_thrust{};
  Pair unload_capture_bend{},unload_capture_thrust{};
  Pair predicted_unload_bend{{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<bool,2> reused_event_checkpoint_reached{{false,false}};
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
  // Monotonic common travel reached by both arms during the first Motion-2
  // test.  It is intentionally independent of the instantaneous
  // leader/lagger label: pneumatic delay can swap that label before either
  // chamber responds, whereas this common target must never move backward.
  double identification_pair_target_extension=0.;
  // The first test records two distinct quantities per arm: breakaway force
  // in onset[] and the largest force required before the absolute bend target
  // was measured. Later strokes may approach the latter quickly but still
  // retain measured-angle feedback and all safety guards.
  Pair identification_peak_thrust{};
  Pair identified_extension_thrust{{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  bool rear_low=true, rear_preload=false;
  // Continuous rear-support schedule shared by Motion 2 and Motion 3.
  // These values are public diagnostics as well as controller state.
  double rear_pressure_command=0.;
  double lift_rear_release_progress=0.;
  double rear_release_progress=0.;
  // A single signed rear-support coordinate unifies rear pressure and thrust.
  // The prior is identified by the slip/first-unloading probes.  The outer
  // choice adds one bounded offset and the inner roll loop may only increase
  // support; it never makes an unstable stroke more aggressive.
  double rear_support_prior=std::numeric_limits<double>::quiet_NaN();
  double rear_support_nominal=0.,rear_support_command=0.;
  double rear_support_safety_override=0.;
  Pair rear_grasp_reference{},rear_release_by_arm{};
  bool rear_grasp_reference_valid=false;
  double rear_release_target=0.,rear_release_pair=0.,
      rear_release_pair_rate=0.,rear_release_mismatch=0.;
  double rear_release_deficit_time=0.;
  bool rear_release_angle_achieved=false;
  double rear_release_balance_envelope=1.;
  double rear_angle_pressure_trim=0.,rear_angle_thrust_trim=0.;
  bool rear_angle_feedback_active=false;
  bool rear_support_slip_identified=false,rear_support_identified=false;
  // An outer rear-support candidate is scored only after the requested
  // pressure/thrust has actually been delivered without an inner roll
  // override replacing it.  Otherwise the resulting displacement belongs to
  // a different physical input and is not an optimizer observation.
  bool rear_candidate_requested=false,rear_candidate_distinct=false,
      rear_candidate_realized=false;
  double rear_candidate_realization_time=0.;
  double stroke_peak_abs_gait_roll=0.;
  double stroke_min_rear_pressure=std::numeric_limits<double>::infinity();
  double golden_flex_progress=0.;
  double golden_desired_up=0.,golden_desired_forward=0.;
  bool golden_window_active=false;
  // Rear positive assist is one continuous event shared by Lift and Pull.
  // It is not reset by enter(Pull), otherwise the phase transition would
  // create a second pulse instead of continuing the Motion-2 impulse.
  double rear_positive_elapsed=0.,rear_positive_thrust_command=0.;
  double rear_positive_front_envelope=0.;
  double rear_positive_pressure_envelope=0.;
  bool rear_positive_started=false;
  // One-shot rear lift after the front pair has become a full-pressure
  // anchor.  It is independent of rear_positive_elapsed, which belongs to
  // the earlier load-relief/golden-time assist.
  double terminal_rear_lift_elapsed=0.;
  double terminal_rear_lift_command=0.;
  double terminal_anchor_wait_elapsed=0.;
  double terminal_front_anchor_stall_time=0.;
  double terminal_front_anchor_best_pressure=0.;
  bool terminal_rear_lift_started=false;
  bool terminal_rear_lift_cancelled=false;
  bool terminal_rear_lift_complete=false;
  bool terminal_front_anchor_wait_started=false;
  bool release_seen=false,slip_measurement_started=false;
  // Rear refill is deliberately decoupled from stroke completion.  It may
  // overlap the latter part of front flexion, while Pull continues until a
  // real front catch (or a posture guard) is observed.
  bool support_transfer_started=false;
  // Normal rear refill requires a sustained, measured front support event.
  // Keep this timer separate from the catch/regrip timer so one pressure or
  // IMU sample cannot permanently end the optimized low-pressure interval.
  double support_transfer_ready_time=0.;
  // Motion 3 first builds measurable front pneumatic support, then releases
  // the rear anchor.  This command latch is separate from the measured
  // release below because the exhaust response has a substantial delay.
  bool rear_release_commanded=false;
  double rear_release_pressure_threshold=
      std::numeric_limits<double>::quiet_NaN();
  bool calibration_only_regrip_pending=false;
  // A support transfer is useful only after the old rear anchors have
  // actually released.  Commanding zero pressure is not evidence of release:
  // the pneumatic delay can leave 10--30 kPa in the chambers while the front
  // pair is already flexing.  Latch the existing measured/hysteretic release
  // observation once per Motion-2/3 stroke and do not refill the rear before
  // it has been seen.
  bool rear_release_observed=false;
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
  // Motion 2 and Motion 3 are two halves of one bend path.  These references
  // are also published for bag-level verification; unlike active_bend_target,
  // they are the instantaneous set-points actually followed by the feedback
  // loops.
  BendTrajectory extension_trajectory;
  BendTrajectory flex_trajectory;
  std::array<BendKinematicsObserver,2> trajectory_observer;
  Pair extension_thrust_trim{},flex_pressure_trim{};
  Pair trajectory_angle_error{},trajectory_rate_error{},
      trajectory_rate_filtered{},trajectory_acceleration{},
      trajectory_predicted_bend{};
  double trajectory_phase=0.,trajectory_clock_scale=1.;
  TrajectoryAdaptation trajectory;
  // One bounded inner pull calibration is allowed before outer trajectory
  // samples exist. Retain the previous command and its measured endpoint so
  // the next settled result can undo the change if it made progress worse.
  bool pull_calibration_pending=false;
  bool pull_calibration_complete=false;
  double pull_calibration_previous=0.;
  double pull_calibration_baseline_progress=0.;
  // -1: restored previous pull, 0: no pending comparison, +1: retained it.
  int finishPullCalibration(double progress) {
    if(!pull_calibration_pending || !std::isfinite(progress)) return 0;
    const bool worsened=progress+.0005<pull_calibration_baseline_progress;
    if(worsened) {
      pull_forward=pull_calibration_previous;
      // One measured increase has bracketed the useful local command.  Keep
      // the better lower value and hand later changes to the bounded outer
      // mix/force search instead of retrying the same harmful increase.
    }
    // Exactly one measured A/B comparison belongs to the inner calibration.
    // If it improved, keep the higher value; if it worsened, restore the
    // previous one.  Further changes are made by the context-conditioned
    // outer mix/force search, not by chaining another blind +dF step in this
    // same result callback.
    pull_calibration_complete=true;
    pull_calibration_pending=false;
    return worsened?-1:1;
  }
  bool adaptPullAfterMeasuredStroke(double progress,const SinglePassConfig& k) {
    if(!stroke_lifted || !std::isfinite(progress) ||
       progress>=k.pull_progress_threshold ||
       pull_forward>=k.pull_adaptation_max || pull_calibration_pending ||
       pull_calibration_complete)
      return false;
    const double previous=pull_forward;
    pull_forward=std::min(k.pull_adaptation_max,
        pull_forward+k.pull_adaptation_step);
    if(pull_forward>previous+1e-9) {
      pull_calibration_pending=true;
      pull_calibration_previous=previous;
      pull_calibration_baseline_progress=progress;
    }
    return pull_forward>previous+1e-9;
  }
  static double supportFromPressure(double pressure,
      const SinglePassConfig& k) {
    const double low=std::max(k.rear_low_kpa,
        k.rear_pressure_adaptation_min);
    const double high=std::max(low+1.,k.rear_pressure_adaptation_max);
    const double mid=.5*(low+high);
    const double p=clamp(pressure,low,high);
    return p<=mid?-(mid-p)/std::max(1.,mid-low):
        (p-mid)/std::max(1.,high-mid);
  }
  static double pressureFromSupport(double support,
      const SinglePassConfig& k) {
    const double low=std::max(k.rear_low_kpa,
        k.rear_pressure_adaptation_min);
    const double high=std::max(low,k.rear_pressure_adaptation_max);
    const double mid=.5*(low+high);
    const double u=clamp(support,-1.,1.);
    return u<0.?mid+(mid-low)*u:mid+(high-mid)*u;
  }
  double selectedRearSupport() const {
    const double prior=std::isfinite(rear_support_prior)?rear_support_prior:0.;
    return clamp(prior+trajectory.selected().rear_support_offset,-1.,1.);
  }
  bool selectedRearCandidateDistinct() const {
    const double prior=std::isfinite(rear_support_prior)?rear_support_prior:0.;
    return std::abs(trajectory.selected().rear_support_offset)>1.e-6 &&
        std::abs(selectedRearSupport()-prior)>.05;
  }
  bool selectedRearCandidateIdentifiable() const {
    return !rear_candidate_requested ||
        (rear_candidate_distinct && rear_candidate_realized);
  }
  double rearPressureFloor(const SinglePassConfig& k) const {
    return pressureFromSupport(rear_support_command,k);
  }
  // The slip probe gives a coarse prior, not an optimizer reward.  Retained
  // sliding favours release; a stable but stuck contact keeps a moderate
  // release for Motion 2; an outward-roll rejection starts on the support
  // side of the coordinate.
  void identifyRearSupportFromSlip(const SinglePassConfig& k) {
    if(!k.rear_pressure_adaptation_enabled) return;
    rear_support_prior=clamp(slip_probe_unstable?
        k.rear_support_probe_unstable:
        (sliding_effective?k.rear_support_probe_sliding:
         k.rear_support_probe_stable),-1.,1.);
    rear_support_nominal=rear_support_command=rear_support_prior;
    rear_support_slip_identified=true;
  }
  // Finish the coarse prior after the first actual unloading stroke.  A safe
  // forward stroke validates the support used by that stroke; it is not
  // evidence that still more rear release is safe.  Only a stable but
  // ineffective unloading stroke moves one small step toward release, while
  // excessive roll moves decisively toward support.  Later changes belong
  // exclusively to TrajectoryAdaptation, preventing a rear-pressure update
  // and a front-trajectory update from contaminating the same result.
  int finishRearSupportIdentification(double progress,bool safe,
      const SinglePassConfig& k) {
    if(!k.rear_pressure_adaptation_enabled || rear_support_identified ||
       !stroke_lifted || !std::isfinite(progress) ||
       !std::isfinite(stroke_peak_abs_gait_roll)) return 0;
    const double previous=std::isfinite(rear_support_prior)?
        rear_support_prior:supportFromPressure(k.lift_rear_pressure_floor,k);
    double next=previous;
    if(!safe ||
       stroke_peak_abs_gait_roll>=k.rear_pressure_adaptation_roll_limit)
      next=std::max(next,k.rear_support_probe_unstable);
    else if(load_relief_detected &&
            progress<k.rear_pressure_adaptation_progress)
      next=std::max(-1.,next-.25);
    rear_support_prior=clamp(next,-1.,1.);
    rear_support_nominal=rear_support_command=rear_support_prior;
    rear_support_identified=true;
    if(rear_support_prior>previous+1e-9) return 1;
    if(rear_support_prior<previous-1e-9) return -1;
    return 0;
  }
  void clearLiftHistory() {
    lift_history_head=lift_history_count=0;
    onset_baseline_candidate_valid.fill(false);
    powered_extension_reference_valid.fill(false);
    powered_body_reference_valid=false;
    powered_body_reference_time=std::numeric_limits<double>::infinity();
  }
  void recordLiftHistory(const SinglePassInput& in) {
    LiftHistorySample& sample=lift_history[lift_history_head];
    sample.time=phase_time;
    sample.bend=in.s.bend;
    sample.height=in.height;
    sample.pitch=in.directional_pitch;
    lift_history_head=(lift_history_head+1)%lift_history_capacity;
    lift_history_count=std::min(lift_history_count+1,lift_history_capacity);
  }
  LiftHistorySample liftHistoryBefore(double requested_time,
      const SinglePassInput& fallback) const {
    LiftHistorySample result;
    result.time=phase_time;
    result.bend=fallback.s.bend;
    result.height=fallback.height;
    result.pitch=fallback.directional_pitch;
    if(!lift_history_count) return result;
    // Walk newest to oldest and take the newest sample which precedes the
    // requested pre-trigger instant.  If the history is not yet that long,
    // use its oldest sample rather than the current, already moving pose.
    for(size_t n=0;n<lift_history_count;++n) {
      const size_t index=(lift_history_head+lift_history_capacity-1-n)%
          lift_history_capacity;
      result=lift_history[index];
      if(result.time<=requested_time) return result;
    }
    return result;
  }
  void reset() {*this=SinglePass();}
  void enter(Phase p,const SinglePassInput& in) {
    phase=p;phase_time=0.;ready_time=0.;motion_time=0.;stall_time=0.;roll_risk_time=0.;
    if(p==Pull || p==FrontCatch) catch_stall_time.fill(0.);
    if(p==Pull) {
      support_transfer_started=false;
      support_transfer_ready_time=0.;
      // Preserve a release which already began in Motion 2.  Its measured
      // pressure threshold is latched at that instant; a later roll-safety
      // request may raise the target again but must not retroactively call a
      // still-high rear chamber "released".
      rear_release_commanded=rear_release_commanded ||
          rear_pressure_command<in.base_pressure-.5;
      rear_release_observed=rear_release_observed || in.rear_released;
      rear_preload=false;
      // Motion 2 has already reduced rear support.  Preserve its last command
      // at the phase boundary; raising it back to full grasp would destroy the
      // short load-relief interval before pressure-driven flexion begins.
      rear_release_progress=0.;
    }
    if(p!=Lift) fast_approach.fill(false);
    if(p==StopForLift) {
      grasp_reference=in.s.bend;rear_low=false;rear_preload=false;
      stroke_lifted=false;
      rear_pressure_command=in.base_pressure;
      lift_rear_release_progress=0.;
    }
    if(p==VentFront) {
      rear_low=false;rear_preload=false;grasp_reference=in.s.bend;
      vent_common_pressure=std::min(in.base_pressure,
          std::max(in.s.command[0],in.s.command[1]));
      rear_pressure_command=in.base_pressure;
      lift_rear_release_progress=0.;
      // VentFront starts from a quiet four-arm grasp.  Rebase each rear arm
      // here rather than assuming a common absolute angle: object diameter,
      // taper and local skin deformation can make the two bends different.
      rear_grasp_reference_valid=in.rear_kinematics_valid;
      if(rear_grasp_reference_valid)
        rear_grasp_reference=in.rear_bend;
      rear_release_by_arm.fill(0.);
      rear_release_target=rear_release_pair=rear_release_pair_rate=0.;
      rear_release_mismatch=0.;rear_release_balance_envelope=1.;
      rear_release_deficit_time=0.;
      rear_release_angle_achieved=false;
      rear_angle_pressure_trim=rear_angle_thrust_trim=0.;
      rear_angle_feedback_active=false;
    }
    if(p==Slide) {slide_best_x=in.x;stall_time=0.;}
    if(p==SlipSettle) {
      slip_candidate_peak_x=std::isfinite(slide_best_x)
          ?std::max(slide_best_x,in.x):in.x;
      slip_settle_mean_x=in.x;
      slip_settle_pressure=in.s.command;
      slip_retained_progress=0.;
      rear_low=false;rear_preload=false;
    }
    if(p==Lift) {reference=in.s.bend;minimum=in.s.bend;start_height=in.height;
      start_pitch=in.directional_pitch;
      // `support_transfer_started` belongs to the preceding Pull/FrontCatch
      // only.  Leaving it latched across regrasp made the final output mapper
      // overwrite the next Motion-2 rear schedule with base pressure: the
      // internal command fell from 30 to 10 kPa while arms 1/2 were still
      // actually commanded to 30 kPa.  A new Lift starts from a confirmed
      // all-arm grasp, so its rear-support transfer must start afresh.
      support_transfer_started=false;
      extension_trajectory.reset();
      flex_trajectory.reset();
      trajectory_angle_error.fill(0.);trajectory_rate_error.fill(0.);
      trajectory_acceleration.fill(0.);
      for(size_t j=0;j<2;++j) {
        trajectory_observer[j].reset(in.s.rate[j]);
        trajectory_rate_filtered[j]=in.s.rate[j];
        trajectory_predicted_bend[j]=in.s.bend[j];
      }
      trajectory_phase=0.;trajectory_clock_scale=1.;
      clearLiftHistory();
      // Preserve the phase-entry sample as the oldest fallback.  If a real
      // arm extends within one 20-ms sensor interval, its first post-motion
      // sample must still be comparable with the pose before breakaway.
      recordLiftHistory(in);
      active_extension_target.fill(0.);active_bend_target=in.s.bend;
      identifying_straight=!calibrated;
      front_target_latched=false;
      lift_target_reached.fill(false);
      slope_overlap_active=false;overlap_extension_latched=false;
      lift_prefill_command.fill(0.);
      lift_angle_pressure_command.fill(0.);
      lift_target_hold_thrust.fill(0.);
      overlap_peak_extension.fill(0.);
      unload_evidence_time=0.;unload_body_candidate_time=0.;
      unload_body_memory_time=0.;
      unload_alignment_time=0.;
      unload_extension_memory.fill(0.);
      unload_pair_mismatch_measured=0.;
      load_relief_candidate=false;
      load_relief_detected=false;
      load_relief_elapsed=std::numeric_limits<double>::quiet_NaN();
      load_relief_thrust.fill(0.);
      load_relief_bend.fill(0.);
      windup_brake_start_bend.fill(0.);
      windup_reference_bend.fill(0.);
      windup_hold_bend.fill(0.);
      windup_hold_pressure.fill(0.);
      windup_pitch_target=windup_pitch_progress=0.;
      windup_pitch_direction=-1.;
      windup_evidence_time=0.;windup_rear_reverse_thrust=0.;
      windup_capture_evidence_time=windup_best_pitch=0.;
      windup_pitch_stall_time=0.;
      windup_brake_started_by_arm.fill(false);
      windup_target_reached_by_arm.fill(false);
      windup_extension_reached=false;windup_target_reached=false;
      windup_pose_captured=false;
      windup_reachable_peak=false;
      unload_event_detected=false;unload_capture_started=false;
      unload_from_arm_trajectory=false;
      front_release_event_armed=!in.front_released;
      calibration_only_regrip_pending=false;
      unload_pitch_delta=unload_height_delta=0.;
      unload_event_elapsed=std::numeric_limits<double>::quiet_NaN();
      unload_alignment_extension_target=0.;
      unload_event_bend.fill(0.);unload_event_thrust.fill(0.);
      unload_capture_bend.fill(0.);unload_capture_thrust.fill(0.);
      predicted_unload_bend.fill(std::numeric_limits<double>::quiet_NaN());
      reused_event_checkpoint_reached.fill(false);
      pressure_lead_seconds=0.;
      identification_ready_time=0.;
      ramp=std::max(in.s.thrust[0],in.s.thrust[1]);pressure_target=in.s.command;
      lift_ramp=in.s.thrust;onset_time.fill(0.);onset_this_stroke.fill(false);
      fast_approach.fill(false);onset_approach_ready.fill(false);
      identification_tracking.fill(false);identification_hold_thrust.fill(0.);
      identification_extension_rate_filtered.fill(0.);
      identification_fast_rate_time.fill(0.);
      identification_rate_command_slope.fill(0.);
      identification_pair_target_extension=0.;
      identification_peak_thrust.fill(0.);
      for(size_t j=0;j<2;++j) passive_extension_rate[j]=std::max(0.,-in.s.rate[j]);
      for(size_t j=0;j<2;++j) if(!onset[j].compatible(in.s.pressure[j],reference[j])) onset_this_stroke[j]=false;
      powered_time=0.;
      predicted_lift_completion_seconds=std::numeric_limits<double>::infinity();
      rear_vent_lead_seconds=0.;
      rear_release_observed=false;
      rear_release_commanded=false;
      rear_release_deficit_time=0.;
      rear_release_angle_achieved=false;
      rear_release_pressure_threshold=
          std::numeric_limits<double>::quiet_NaN();
      // VentFront has already used the measured valve model to lead the rear
      // exhaust toward a compliant hold. Preserve both its command and the
      // normalized progress at Motion-2 entry instead of stepping back to
      // full grasp and spending the short reach waiting for exhaust.
      rear_release_progress=0.;
      rear_positive_elapsed=0.;
      rear_positive_thrust_command=0.;
      rear_positive_front_envelope=0.;
      rear_positive_pressure_envelope=0.;
      rear_positive_started=false;
      terminal_rear_lift_elapsed=0.;
      terminal_rear_lift_command=0.;
      terminal_anchor_wait_elapsed=0.;
      terminal_front_anchor_stall_time=0.;
      terminal_front_anchor_best_pressure=0.;
      terminal_rear_lift_started=false;
      terminal_rear_lift_cancelled=false;
      terminal_rear_lift_complete=false;
      terminal_front_anchor_wait_started=false;
      if(!lift_tested) {++lift_probe_entries;lift_tested=true;}
      // A reused Action-C stroke rises much faster than the first sweep, but
      // motion 2 still needs the rear pair as its anchor.  Do not release the
      // rear merely because a calibration exists: that removed support at
      // exactly the fast-ramp edge and converted unequal front breakaway into
      // body roll.  The measured completion-time predictor below starts rear
      // exhaust only after front extension is visible and close enough to the
      // target for pressure to be low when motion 3 begins.
      rear_low=false;rear_preload=false;rear_time=0.;}
    if(p==Pull) {
      pull_start=in.s.bend;minimum=in.s.bend;rear_low=false;
      flex_trajectory.reset();
      trajectory_angle_error.fill(0.);trajectory_rate_error.fill(0.);
      trajectory_phase=0.;trajectory_clock_scale=1.;
    }
  }
  void begin(const SinglePassInput& in) {
    // A recovery/short goal must not silently restart an already attempted
    // identification pass. Keep its bounded observations for the next stroke.
    if(slip_entries>0) slip_tested=true;
    if(lift_probe_entries>0) lift_tested=true;
    initialized=true;++strokes;reference=in.s.bend;grasp_reference=reference;minimum=reference;start_pitch=in.s.pitch;
    rear_grasp_reference_valid=in.rear_kinematics_valid;
    if(rear_grasp_reference_valid)
      rear_grasp_reference=in.rear_bend;
    rear_release_by_arm.fill(0.);
    rear_release_target=rear_release_pair=rear_release_pair_rate=0.;
    rear_release_mismatch=0.;rear_release_balance_envelope=1.;
    rear_release_deficit_time=0.;
    rear_release_angle_achieved=false;
    rear_angle_pressure_trim=rear_angle_thrust_trim=0.;
    rear_angle_feedback_active=false;
    stroke_lifted=false;
    stroke_peak_abs_gait_roll=0.;
    stroke_min_rear_pressure=in.rear_pressure;
    golden_flex_progress=0.;
    golden_desired_up=golden_desired_forward=0.;
    golden_window_active=false;
    context_bend=in.s.bend;context_roll=in.s.roll;context_pitch=in.s.pitch;
    goal_conditioned=false;goal_scale=goal_extension_scale=goal_pull_envelope=1.;
    predicted_stride=std::numeric_limits<double>::quiet_NaN();
    desired_stride=std::numeric_limits<double>::quiet_NaN();
    stride_estimate_measured=false;
    if(calibrated) {
      predicted_stride=trajectory.estimatedProgress(
          context_bend,context_roll,context_pitch);
      stride_estimate_measured=std::isfinite(predicted_stride);
      if(!stride_estimate_measured) {
        const double provisional=trajectory.provisionalProgressForReduction(
            context_bend,context_roll,context_pitch);
        // One observation may only weaken an oversized final stroke. It must
        // never be used to strengthen a command or to seed long-range tiling.
        if(std::isfinite(provisional) && std::isfinite(in.remaining) &&
           in.remaining<.9*provisional) {
          predicted_stride=provisional;
          stride_estimate_measured=true;
        }
      }
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
    // Goal scaling answers "how far remains"; the outer candidate answers
    // "which contact-conditioned waveform is efficient". They are factored
    // independently and the exact command scale is stored with every result,
    // so long-distance commands must not freeze the optimizer at candidate 0.
    // Candidate changes still occur only at this settled stroke boundary.
    // The slip probe and the first powered unloading stroke provide the
    // coarse front-force operating point.  From the first measured repeated
    // stroke onward, TrajectoryAdaptation owns deliberate force changes.  Do
    // not wait for the legacy blind +dF calibration to finish: goal-distance
    // scaling can keep that flag false and would otherwise pin every stroke
    // to candidate zero.
    if(lift_probe_entries>0 && !trajectory.samples.empty()) {
      trajectory.select(context_bend,context_roll,context_pitch);
      // An offset clipped by the identified support prior produces exactly
      // the baseline command.  Skip that arm of the finite search instead of
      // spending real strokes on an experiment which cannot be distinguished.
      for(unsigned attempt=0;attempt<TrajectoryAdaptation::choice_count;
          ++attempt) {
        if(std::abs(trajectory.selected().rear_support_offset)<=1.e-6 ||
           selectedRearCandidateDistinct()) break;
        trajectory.markUnavailable(trajectory.current);
        trajectory.select(context_bend,context_roll,context_pitch);
      }
    }
    rear_candidate_requested=
        std::abs(trajectory.selected().rear_support_offset)>1.e-6;
    rear_candidate_distinct=selectedRearCandidateDistinct();
    rear_candidate_realized=false;
    rear_candidate_realization_time=0.;
    pressure_target=in.s.command;initial_x=in.x;rear_time=0.;rear_low=true;
    release_seen=false;slip_measurement_started=false;
    onset_time.fill(0.);onset_this_stroke.fill(false);fast_approach.fill(false);
    ramp=std::max(in.s.thrust[0],in.s.thrust[1]);
    if(!slip_tested) {
      slip_probe_unstable=false;
      ++slip_entries;enter(SlipProbe,in);
    }
    else if(sliding_effective) enter(Slide,in);
    else enter(StopForLift,in);
  }
  template<class Geometry>
  SinglePassOutput update(const SinglePassInput& in,const Config& c,
      const SinglePassConfig& k,const Model& model,Geometry geometry,
      const std::function<double(size_t,double,double)>& bend_equilibrium={}) {
    SinglePassOutput out;out.bottom=k.bottom_kpa;
    if(!initialized) begin(in);
    const auto& s=in.s;const double dt=in.dt;
    if(!k.valid() || !s.fresh || !std::isfinite(dt) || dt<=0 || dt>.1 ||
       !std::isfinite(in.height) || !std::isfinite(in.rear_pressure) ||
       !std::isfinite(in.directional_pitch))
      {out.fault=true;return out;}
    if(!std::isfinite(rear_support_prior)) {
      rear_support_prior=supportFromPressure(k.lift_rear_pressure_floor,k);
      rear_support_nominal=rear_support_command=rear_support_prior;
    }
    phase_time+=dt;
    // Preserve continuity at the windup-to-flex handoff.  Dropping the rear
    // pitch-couple command in one sample excites the compliant arms just as
    // front pressure begins the swing.  Outside active windup, remove it with
    // the same bounded slew and let the ordinary Pull command take over.
    const bool active_windup_couple=phase==Lift && load_relief_detected &&
        !unload_event_detected;
    if(!active_windup_couple) {
      windup_rear_reverse_thrust=slew(windup_rear_reverse_thrust,0.,
          k.trajectory_windup_rear_reverse_rise,
          2.*k.trajectory_windup_rear_reverse_rise,dt);
      out.rear_reverse_thrust=windup_rear_reverse_thrust;
    }
    for(size_t j=0;j<2;++j) {
      trajectory_observer[j].update(s.rate[j],dt,
          k.trajectory_rate_filter_tau,
          k.trajectory_acceleration_filter_tau,
          k.trajectory_acceleration_limit);
      trajectory_rate_filtered[j]=trajectory_observer[j].rate;
      trajectory_acceleration[j]=trajectory_observer[j].acceleration;
      trajectory_predicted_bend[j]=trajectory_observer[j].predictAngle(
          s.bend[j],k.trajectory_prediction_horizon,
          k.trajectory_prediction_max_delta);
    }
    if(phase==Lift) recordLiftHistory(in);
    // A compliant/tapered contact can have a repeatable non-zero roll at the
    // four-arm boundary.  The gait controller regulates the incremental roll
    // created by THIS stroke; the adapter independently retains the absolute
    // 20-deg/90-deg safety guards.  Using absolute roll here made a stable
    // -9-deg grasp trigger recovery as soon as front vent added only -1 deg.
    const double gait_roll=s.roll-context_roll;
    if(phase==Lift || phase==Pull || phase==FrontCatch) {
      stroke_peak_abs_gait_roll=std::max(stroke_peak_abs_gait_roll,
          std::abs(gait_roll));
      stroke_min_rear_pressure=std::min(stroke_min_rear_pressure,
          in.rear_pressure);
    }
    Pair ux{},uz{},moment{},pitch_moment{},raw{},rise_limit{{c.thrust_rise,c.thrust_rise}},
        fall_limit{{c.thrust_fall,c.thrust_fall}};
    std::array<bool,2> lift_terminal_braking{{false,false}};
    std::array<bool,2> lift_extension_guard{{false,false}};
    double fx=0.,up=0.,extension=0.;
    for(size_t j=0;j<2;++j) {
      const auto f=geometry(j,s.bend[j],s.roll,s.pitch);
      ux[j]=f.forward;uz[j]=std::max(0.,f.up);
      moment[j]=f.roll_moment;
      pitch_moment[j]=f.pitch_moment;
      if(!std::isfinite(f.forward) || !std::isfinite(f.up) || !std::isfinite(f.roll_moment) ||
         !std::isfinite(f.pitch_moment)) {out.fault=true;return out;}
      fx+=s.thrust[j]*std::max(0.,ux[j]);up+=s.thrust[j]*uz[j];
      minimum[j]=std::min(minimum[j],s.bend[j]);
      extension=std::max(extension,reference[j]-s.bend[j]);
    }
    const Pair verified_roll_moment=verifiedPairRollMoments(
        moment,k.reverse_geometry_roll_moment);
    const double predicted_roll=gait_roll+.3*s.roll_rate;
    // Slip has its own measured-progress decision.  The normal soft gait
    // threshold is intentionally not folded into it: doing so rejected a
    // retained 15-mm slide on the AIST-shaped forearm solely because the
    // body settled at a different, but still supported, roll angle.
    const double active_roll_threshold = phase==SlipProbe
        ? k.slip_roll_abort_angle : k.soft_roll;
    const double active_roll_rate_threshold = phase==SlipProbe
        ? k.slip_roll_abort_rate : .05;
    const bool roll_risk_sample=
      std::abs(predicted_roll)>active_roll_threshold &&
      std::abs(s.roll_rate)>active_roll_rate_threshold &&
      gait_roll*s.roll_rate>0.;
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
    const bool reused_lift_roll_grace = calibrated && phase==Lift &&
        phase_time<k.reused_lift_roll_grace;
    const bool posture_brake=roll_brake && !reused_lift_roll_grace;
    // Angle synchronization and body-roll correction can request opposite
    // rotors.  Once the incremental gait roll reaches the soft boundary,
    // body posture owns the differential allocation even if angular velocity
    // happens to be momentarily zero.  This is an inner-loop balance action,
    // not a request for shared posture recovery; a stable absolute grasp bias
    // therefore remains permissible.
    const bool gait_roll_worsening=std::abs(gait_roll)>.0174533 &&
        std::abs(s.roll_rate)>.02 && gait_roll*s.roll_rate>0.;
    // The outer candidate chooses the nominal rear role.  The inner safety
    // loop is one-sided: worsening roll may add support, but can never release
    // the rear anchor beyond the measured candidate.  This keeps front
    // progression and rear stability independently interpretable.
    rear_support_nominal=selectedRearSupport();
    rear_support_safety_override=gait_roll_worsening?
        clamp((std::abs(predicted_roll)-k.rear_support_roll_start)/
            (k.rear_support_roll_full-k.rear_support_roll_start),0.,1.):0.;
    // Blend continuously from the selected support toward full grasp.  Using
    // max(nominal, override) created a discontinuity at the first non-zero
    // correction: e.g. -0.75 release jumped straight to +0.02 support and the
    // rear pressure target rose from 3.75 to 15.3 kPa.  The blend below keeps
    // the candidate identifiable for small corrections while still reaching
    // +1 at the full safety boundary.
    rear_support_command=gait_roll_worsening?
        rear_support_nominal+rear_support_safety_override*
            (1.-rear_support_nominal):rear_support_nominal;
    // Pressure is only an actuator input, not proof that the rear anchor has
    // physically relaxed.  Motion 2/3 therefore require a small measured
    // 3--5-deg extension of BOTH rear arms.  The optimizer's nominal rear
    // release selects a target inside that corridor, but the fast roll safety
    // blend is deliberately not allowed to erase the geometric target: it
    // may stop thrust through the posture gates below, while the rear pair
    // remains compliant until the explicit all-arm regrip.
    const bool rear_release_phase=
        phase==Lift || phase==Pull || phase==FrontCatch;
    const double nominal_release_fraction=clamp(
        std::max(0.,-rear_support_nominal),0.,1.);
    rear_release_target=rear_release_phase?
        k.rear_release_angle_min+nominal_release_fraction*
            (k.rear_release_angle_max-k.rear_release_angle_min):0.;
    rear_angle_feedback_active=false;
    rear_angle_pressure_trim=rear_angle_thrust_trim=0.;
    rear_release_balance_envelope=1.;
    if(rear_grasp_reference_valid && in.rear_kinematics_valid) {
      const double filter_gain=clamp(
          dt/(k.rear_release_angle_filter_tau+dt),0.,1.);
      for(size_t j=0;j<2;++j) {
        const double measured_release=clamp(
            rear_grasp_reference[j]-in.rear_bend[j],
            -k.rear_release_angle_max,
            std::max(2.*k.rear_release_angle_max,
                k.rear_release_mismatch_hard+
                    k.rear_release_assist_deadband));
        rear_release_by_arm[j]+=filter_gain*
            (measured_release-rear_release_by_arm[j]);
      }
      const size_t limiting_arm=rear_release_by_arm[0]<=
          rear_release_by_arm[1]?0:1;
      rear_release_pair=rear_release_by_arm[limiting_arm];
      // Positive means that the limiting rear arm is extending.
      rear_release_pair_rate=-in.rear_rate[limiting_arm];
      rear_release_mismatch=std::abs(
          rear_release_by_arm[0]-rear_release_by_arm[1]);
      rear_release_balance_envelope=clamp(
          (k.rear_release_mismatch_hard-rear_release_mismatch)/
              (k.rear_release_mismatch_hard-
                  k.rear_release_mismatch_soft),0.,1.);
      rear_angle_feedback_active=
          phase==Lift || phase==Pull || phase==FrontCatch;
      if(rear_angle_feedback_active) {
        const double error=rear_release_target-rear_release_pair;
        rear_angle_pressure_trim=clamp(
            -k.rear_release_pressure_kp*error+
                k.rear_release_pressure_kd*rear_release_pair_rate,
            -k.rear_release_pressure_trim_max,
            k.rear_release_pressure_trim_max);
        rear_angle_thrust_trim=clamp(
            k.rear_release_thrust_kp*error-
                k.rear_release_thrust_kd*rear_release_pair_rate,
            -k.rear_release_thrust_trim_max,
            k.rear_release_thrust_trim_max);
        // With a large rear-pair mismatch, do not further release both arms
        // merely because the lagging one has not moved. Restorative pressure
        // and thrust reduction remain unscaled.
        if(rear_angle_pressure_trim<0.)
          rear_angle_pressure_trim*=rear_release_balance_envelope;
        if(rear_angle_thrust_trim>0.)
          rear_angle_thrust_trim*=rear_release_balance_envelope;
      }
    }
    // Venting is given a short measured opportunity before rotor assistance
    // is admitted.  Once both rear angles reach the target corridor, the
    // positive command is removed; if contact loads bend either arm back, the
    // same sustained-deficit test may re-enable it.  This is an angle event,
    // not a pressure-threshold proxy.
    rear_release_angle_achieved=rear_angle_feedback_active &&
        rear_release_pair>=
            rear_release_target-k.rear_release_assist_deadband;
    const double rear_motion_pressure_limit=std::min(
        in.base_pressure,k.rear_motion_pressure_cap);
    const bool rear_pressure_compliant=
        in.rear_pressure<=rear_motion_pressure_limit+
            k.catch_pressure_tolerance;
    const bool rear_release_deficit=rear_angle_feedback_active &&
        !rear_release_angle_achieved && rear_pressure_compliant;
    rear_release_deficit_time=rear_release_deficit?
        std::min(k.rear_release_assist_confirm,
            rear_release_deficit_time+dt):
        std::max(0.,rear_release_deficit_time-2.*dt);
    // Worsening measured roll gets immediate balance authority, including
    // the first 0.5 s of a reused reach.  The grace period suppresses only a
    // static/one-sample soft-boundary response; it must not leave a genuine
    // outward rotation open-loop while asymmetric arm breakaway develops.
    const bool inner_roll_control=gait_roll_worsening ||
        (roll_brake && !reused_lift_roll_grace);
    // Keep this across the Lift -> Pull transition.  It represents a measured
    // contact event, whereas rear_low below is only a valve command.
    if(phase==Lift || phase==Pull || phase==FrontCatch)
      rear_release_observed=rear_release_observed || in.rear_released;
    // Sliding pressure and its absolute 90-deg target are a different load
    // case.  Learn the reusable free-extension threshold only in Lift, after
    // both front chambers have been exhausted.  This prevents one arm's
    // pressure-assisted slip onset from enabling a one-sided shortcut.
    if(phase==Lift) for(size_t j=0;j<2;++j) {
      const double measured_extension=reference[j]-s.bend[j];
      const bool moving=measured_extension>=k.onset_angle &&
          -s.rate[j]>=k.onset_rate;
      // A real breakaway can finish between two Neuron messages.  Once a
      // candidate was triggered by measured motion, retain it while the
      // displacement and powered command persist; a one-sample IMU spike
      // returns inside the threshold and therefore clears the candidate.
      const bool continuing_candidate=onset_time[j]>0. &&
          measured_extension>=k.onset_angle && s.thrust[j]>.3;
      const bool learn=(moving || continuing_candidate) &&
          s.thrust[j]>.3 && !posture_brake &&
          std::abs(s.roll_rate)<.1 && std::abs(in.pitch_rate)<.1;
      if(!onset_this_stroke[j]) {
        if(learn) {
          if(onset_time[j]==0.) {
            onset_candidate_thrust[j]=s.thrust[j];onset_candidate_pressure[j]=s.pressure[j];
            onset_candidate_bend[j]=s.bend[j];
            // `moving` is first asserted after a small measured excursion.
            // Recover the body pose from two controller samples earlier and
            // retain the most-flexed per-arm reference accumulated before
            // powered breakaway.
            onset_baseline_candidate[j]=liftHistoryBefore(
                phase_time-std::max(.04,2.*dt),in);
            onset_baseline_candidate_valid[j]=true;
          }
          onset_time[j]+=dt;
          if(onset_time[j]+1e-9>=k.onset_confirm &&
              onset_candidate_bend[j]-s.bend[j]>=
                  .5*k.onset_rate*k.onset_confirm) {
            onset[j].observe(onset_candidate_thrust[j],onset_candidate_pressure[j],reference[j]);
            onset_this_stroke[j]=true;
            lift_onset_seen[j]=true;
            const LiftHistorySample baseline=onset_baseline_candidate_valid[j]?
                onset_baseline_candidate[j]:liftHistoryBefore(phase_time,in);
            powered_extension_reference[j]=baseline.bend[j];
            powered_extension_reference_valid[j]=true;
          }
        } else {
          onset_time[j]=0.;
          onset_baseline_candidate_valid[j]=false;
        }
      }
    }
    // Arm bend is measured from each arm's own breakaway pose, but body
    // unloading cannot be evaluated until BOTH front arms are participating.
    // Freeze height and pitch at the later of the two powered-onset poses.
    // Using the first arm's onset included the second arm's pneumatic/contact
    // transient in the body displacement and allowed Motion 2 to finish on a
    // small pressure change rather than on the intended two-arm lift event.
    if(phase==Lift && !powered_body_reference_valid &&
       onset_this_stroke[0] && onset_this_stroke[1] &&
       powered_extension_reference_valid[0] &&
       powered_extension_reference_valid[1] &&
       onset_baseline_candidate_valid[0] &&
       onset_baseline_candidate_valid[1]) {
      const size_t later=onset_baseline_candidate[0].time>=
          onset_baseline_candidate[1].time?0:1;
      const LiftHistorySample& baseline=onset_baseline_candidate[later];
      powered_start_height=baseline.height;
      powered_start_pitch=baseline.pitch;
      powered_body_reference_time=baseline.time;
      powered_body_reference_valid=true;
    }
    onset_identified=onset_identified ||
        (lift_onset_seen[0] && lift_onset_seen[1]);
    if(phase==SlipSettle) {
      // Remove the diagnostic input before deciding whether the observed
      // motion was rigid-body slip or elastic deformation of the contacted
      // object. Bottom pressure is intentionally unchanged, so it cannot
      // manufacture the retained displacement used by this decision.
      rear_low=false;
      raw.fill(0.);
      // Hold the SAME front pressure used by the probe. Raising it to grasp
      // pressure here can itself bend the arms and move the body, which is
      // neither rotor-driven slip nor retained evidence from the probe.
      for(size_t j=0;j<2;++j)
        pressure_target[j]=clamp(slip_settle_pressure[j],0.,c.pressure_max);
      out.angle_target=reference;
      slip_candidate_peak_x=std::max(slip_candidate_peak_x,in.x);
      const double average_alpha=dt/(.30+dt);
      slip_settle_mean_x+=average_alpha*(in.x-slip_settle_mean_x);
      const bool front_settled=
          std::abs(s.pressure[0]-slip_settle_pressure[0])<=3. &&
          std::abs(s.pressure[1]-slip_settle_pressure[1])<=3.;
      const bool stopped=std::max(s.thrust[0],s.thrust[1])<=.05 &&
          (!in.speed_valid || std::abs(s.speed)<=.5*k.slip_min_speed);
      const bool settled=front_settled && in.rear_grasped && stopped;
      ready_time=settled?ready_time+dt:0.;
      const bool settle_complete=ready_time>=k.slip_settle_seconds ||
          (phase_time>=k.slip_settle_timeout && front_settled &&
           in.rear_grasped && std::max(s.thrust[0],s.thrust[1])<=.05);
      if(settle_complete) {
        slip_retained_progress=slip_settle_mean_x-initial_x;
        const double peak_progress=std::max(k.slip_min_progress,
            slip_candidate_peak_x-initial_x);
        slip_tested=true;
        if(!slip_probe_unstable && slip_retained_progress>=
               k.slip_retained_ratio*k.slip_min_progress &&
           slip_retained_progress>=
               k.slip_peak_retained_ratio*peak_progress) {
          sliding_effective=true;
          identifyRearSupportFromSlip(k);
          enter(Slide,in);
        } else {
          sliding_effective=false;
          identifyRearSupportFromSlip(k);
          pull_forward=clamp(fx,k.slip_seed_pull_min,k.slip_seed_pull_max);
          enter(StopForLift,in);
        }
      }
    } else if(phase==SlipProbe || phase==Slide) {
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
        initial_x=in.x;motion_time=0.;stall_time=0.;slide_best_x=in.x;
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
      const bool slip_candidate=phase==SlipProbe && moving;
      // Do not keep cycling the rear support until the independent hard-roll
      // guard has to take ownership.  `posture_brake` is already a filtered
      // condition (angle/rate direction plus confirmation time), so a single
      // accelerometer/gyro spike cannot reject the probe.  A real worsening
      // roll, however, is evidence that this contact is unsuitable for the
      // pure-slip branch and should proceed to the load-relief gait.
      // Forward displacement plus signed velocity is direct evidence that
      // the probe is doing its job.  Let that candidate reach the zero-force
      // retained-position test even if roll changed at the same time.  Only
      // a non-progressing outward rotation rejects the sliding branch; the
      // independent absolute hard guard can still interrupt any phase.
      const bool unstable_probe=phase==SlipProbe && posture_brake && !moving;
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
      } else if(phase==SlipProbe && slip_measurement_started) {
        // This is a classifier, not an endurance pulse.  Only a genuine new
        // best position resets the timer, so contact oscillation cannot keep
        // an ineffective test alive until the force ceiling.
        const double progress_epsilon=std::max(.0005,.25*k.slip_min_progress);
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
      if(!slip_candidate && !unstable_probe &&
         ((phase==SlipProbe &&
           (probe_time>=k.slip_seconds ||
            stall_time>=k.slip_no_progress_seconds ||
            phase_time>=3.*k.slip_seconds)) ||
          (phase==Slide && stall_time>=3.))) {
        slip_tested=true;sliding_effective=false;
        // Whether the short probe never reached the displacement threshold or
        // a retained candidate later stalled, the final measured outcome is
        // "no sustained slip".  Reclassify in both cases.  Previously only
        // the second path updated the prior, so an ordinary high-friction
        // timeout silently inherited lift_rear_pressure_floor=0 kPa and made
        // the first unloading test maximally release its rear anchor.
        identifyRearSupportFromSlip(k);
        // No sustained slide was identified.  The last instantaneous force
        // is a lower bound on static friction, not a safe pressure-assisted
        // pull command.  Seed a moderate pull and let completed, retained
        // strokes adapt it; do not inherit the 5-N probe endpoint directly.
        pull_forward=clamp(fx,k.slip_seed_pull_min,k.slip_seed_pull_max);
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
      if(slip_candidate || unstable_probe) {
        // Keep the force which produced this candidate for a possible Slide
        // restart, but first verify its zero-thrust retained endpoint.
        slip_thrust=std::max(s.thrust[0],s.thrust[1]);
        slide_best_x=in.x;
        slip_probe_unstable=unstable_probe;
        enter(SlipSettle,in);
        raw.fill(0.);
        slip_settle_pressure=pressure_target;
        for(size_t j=0;j<2;++j)
          pressure_target[j]=clamp(slip_settle_pressure[j],0.,c.pressure_max);
        out.angle_target=reference;
      }
    }
    if(phase==StopForLift || phase==VentFront) {
      rear_low=false;
      raw.fill(0.);out.angle_target=grasp_reference;
      if(phase==StopForLift) {
        // First remove the slip-test thrust.  A preceding posture recovery
        // may deliberately retain a contact-conditioned diagonal pressure
        // (for example [30,30,16,16] kPa); forcing all four chambers to 30 kPa
        // here destroys that measured equilibrium before Motion 2 starts.
        // Once thrust is stopped, VentFront performs one coordinated support
        // transfer: both front chambers go to zero while the measured valve
        // model begins a bounded rear pre-release toward the compliant hold.
        // Motion 2 retains the remaining optimizer-selected release. This
        // avoids an unnecessary all-30 intermediate pose without abruptly
        // removing the rear anchor.
        pressure_target=s.command;
        const bool stopped_and_supported=
            std::max(s.thrust[0],s.thrust[1])<=.05;
        if(stopped_and_supported && roll_brake) {
          // The slip probe can leave a residual roll although it did not
          // produce useful forward motion. Hand over before removing front
          // support only when that roll is still moving outward for the
          // configured confirmation time. A static contact offset may be
          // larger than the soft gait threshold without being unstable; the
          // adapter's independent hard-roll limit remains immediate. Using
          // magnitude alone here caused a stable -10-deg pose to alternate
          // indefinitely between the gait and a no-op recovery.
          ready_time+=dt;
          if(ready_time>=k.stop_roll_recovery_confirm)
            out.recover_roll=true;
        } else if(stopped_and_supported && !roll_risk_sample) {
          enter(VentFront,in);
        } else ready_time=0.;
      }
      if(phase==VentFront) {
        // Front vent removes two contact supports before any rotor has enough
        // authority to oppose roll.  A static contact offset is allowed, but
        // once the measured roll is still moving outward for the normal soft
        // confirmation time, do not cross the zero-pressure boundary into a
        // powered Lift.  Let the shared roll-recovery controller restore the
        // pose first, then restart this preparation from a measured grasp.
        // This closes the run-74 failure where VentFront continued from
        // -9 deg to Lift at -10.3 deg and the following unload event occurred
        // only after roll had already become unrecoverable.
        // A short rate transient is expected while one front contact unloads.
        // Handing off on predicted roll alone made VentFront oscillate with
        // posture recovery forever.  Require the measured angle itself to
        // cross the normal catch boundary; below it, the synchronization loop
        // continues and the powered Lift will soon regain roll authority.
        // A tapered/elliptic forearm can have a repeatable non-zero roll
        // equilibrium.  Do not convert that quiet offset into a mandatory
        // recovery before every pulse.  VentFront owns only the incremental,
        // worsening roll created by this support exchange; the adapter keeps
        // the independent absolute hard-angle/rate guard.  This distinction
        // permits the next pulse after a safely settled 10--15-deg contact
        // pose, while a sustained outward trend still restores four-arm
        // support before Lift.
        if(roll_brake) {
          pressure_target.fill(in.base_pressure);
          raw.fill(0.);
          // A worsening support exchange cancels every anticipatory exhaust.
          // Shared posture recovery must begin from a real rear anchor.
          rear_pressure_command=in.base_pressure;
          lift_rear_release_progress=0.;
          ready_time=0.;
          out.recover_roll=true;
        } else {
        // Motion 2 starts with the rear pair still gripping at base pressure.
        // Only the front pair is exhausted in this preparation phase. Compare
        // extension relative to each arm's phase-entry bend (not the two
        // absolute bends), and temporarily hold only the leading arm. This
        // prevents unequal pneumatic/contact response from creating roll
        // before any rotor authority exists.
        rear_low=false;
        // Depressurize both fronts with exactly the same command. Holding
        // pressure on only the faster-opening arm left that contact anchored
        // while its partner detached and therefore created roll.  Relative
        // extension is controlled by the powered Lift loop after this common
        // support boundary, where rotor authority is available.  Keep rotor
        // thrust strictly zero in this preparation phase: an earlier 0.75-N
        // one-sided correction prevented the `thrust <= 0.05 N` empty gate
        // from ever completing, so the controller waited here until shared
        // posture recovery was triggered instead of entering Motion 2.
        if(!roll_brake)
          vent_common_pressure=std::max(0.,
              vent_common_pressure-k.vent_pressure_fall*dt);
        pressure_target.fill(vent_common_pressure);
        raw.fill(0.);
        // Rear pressure has a much slower measured response than the short
        // powered reach.  Waiting until Motion 2 to open its exhaust left a
        // 3--4-kPa command at 9--16 kPa measured pressure when Motion 3
        // began.  Lead only toward the compliant regrip hold while the two
        // front chambers are being vented; the optimized floor remains owned
        // by Motion 2/3.  The start time follows the same measured pneumatic
        // delay/rate model as the later release, so this is not a fixed early
        // valve pulse.
        const double selected_rear_floor=std::min(
            std::min(in.base_pressure,k.rear_motion_pressure_cap),
            rearPressureFloor(k));
        const double pre_release_target=std::min(
            std::min(in.base_pressure,k.rear_motion_pressure_cap),
            std::max(selected_rear_floor,k.rear_low_kpa));
        const double front_time_to_empty=std::max(0.,
            vent_common_pressure-k.vent_tolerance)/
                std::max(1.,k.vent_pressure_fall);
        rear_vent_lead_seconds=c.pressure_delay+
            std::max(0.,in.rear_pressure-pre_release_target)/
                std::max(1.,c.exhaust_rate)+k.rear_vent_lead_margin;
        const double pre_release_phase=rear_vent_lead_seconds>1.e-3?
            clamp(1.-front_time_to_empty/rear_vent_lead_seconds,0.,1.):1.;
        const double full_release_span=std::max(1.e-6,
            in.base_pressure-selected_rear_floor);
        const double pre_release_fraction=clamp(
            (in.base_pressure-pre_release_target)/full_release_span,0.,1.);
        lift_rear_release_progress=std::max(lift_rear_release_progress,
            pre_release_fraction*minimumJerkPosition(pre_release_phase));
        rear_pressure_command=in.base_pressure-
            lift_rear_release_progress*full_release_span;
        // Keep this boundary strictly pneumatic.  Waiting for passive arm
        // motion to settle lets the two unloaded contacts drift to very
        // different poses, while adding a one-sided rotor command turns that
        // preparation into an unmeasured extension test.  Motion 2 already
        // estimates the passive rate of each arm and ramps both rotors from
        // the measured zero-pressure pose, so hand over as soon as the common
        // pressure is empty and the old thrust is gone.
        const bool empty=vent_common_pressure<=k.vent_tolerance &&
            std::max(s.pressure[0],s.pressure[1])<=2.*k.vent_tolerance;
        const bool thrust_stopped=
            std::max(s.thrust[0],s.thrust[1])<=.05;
        ready_time=empty && thrust_stopped && !roll_risk_sample ?
            ready_time+dt:0.;
        if(ready_time>=.2) enter(Lift,in);
        }
      }
      if(!out.recover_roll && phase_time>=k.vent_seconds) out.fault=true;
      // A phase entry cannot also consume stale extension from the old
      // reference in this tick. Start the powered search on the next tick.
    } else if(phase==Lift) {
      const bool deep_windup_enabled=!k.trajectory_golden_handoff &&
          k.trajectory_windup_pitch>k.unload_pitch+1.e-6;
      // VentFront may already have led rear exhaust toward the compliant
      // hold. A measured-time prediction below continues the remaining
      // optimizer-selected exhaust so the pair is physically near its target
      // when Motion 3 begins.
      // A timeout/brake with negligible extension is not an identified lift.
      // Otherwise it teaches a 2-deg target which rear release alone can meet,
      // producing repeated unpowered "completed" strokes on a sticky fixture.
      identifying_straight=!calibrated;
      // Final rear release is owned exclusively by the measured angle/rate
      // and previous-event prediction below. VentFront may only reach the
      // compliant hold, never the lower optimized floor. A former nominal
      // `probe travel / angular rate` shortcut was always shorter than the
      // pneumatic exhaust lead, so it commanded rear=0 at Lift entry and
      // removed the anchor before either front arm had generated authority.
      // Unloading is now one event-driven trajectory for level and sloped
      // contacts.  Keep the former slope-overlap flag false so its independent
      // angle handoff cannot compete with the measured body event below.
      slope_overlap_active=false;
      extension=0.;
      identification_rate_command_slope.fill(0.);
      powered_time+=dt;
      // Update the passive reference first for BOTH arms.  Identification
      // tracking is pair-gated below; deciding it inside the per-arm loop let
      // the first arm enter absolute-angle feedback one control sample before
      // the second arm was even evaluated.
      for(size_t j=0;j<2;++j) {
        if(!unload_event_detected && !onset_this_stroke[j] &&
           s.bend[j]>reference[j]) {
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
      // Reuse accelerates only the known, sub-onset part of Motion 2 (the
      // onset_pair_approach_pending branch below).  Once both rotors have
      // reached that contact-conditioned feed-forward force, the remaining
      // interval is unknown again: the cylinder/skin contact may have changed
      // since the preceding stroke.  Continuing at reused_lift_rise here made
      // the second MuJoCo reach jump from the learned 1--2 N neighbourhood to
      // 3--5 N before a body-unloading sample could be confirmed, producing a
      // 20-deg roll excursion.  Search the post-onset interval at the same
      // bounded rate used for initial identification.  A stalled arm can
      // still reach the full physical thrust limit; it simply cannot get
      // there in one open-loop burst.
      const double lift_search_rise=k.lift_rise;
      // Reuse the two measured breakaway forces as a PAIR.  A partial
      // one-arm observation is deliberately insufficient: accelerating only
      // that arm on the following stroke creates roll before the other arm
      // starts to extend.  Both arms first approach their own sub-onset force,
      // the early arm waits there, and only then do both resume the measured
      // extension search together.  This state is independent of the final
      // absolute-bend trajectory calibration below.
      Pair onset_approach_target{};
      const bool learned_event_pair_available=calibrated &&
          std::isfinite(identified_extension_thrust[0]) &&
          std::isfinite(identified_extension_thrust[1]);
      const bool measured_onset_pair=onset[0].samples>0 &&
          onset[1].samples>0 &&
          onset[0].compatible(s.pressure[0],reference[0]) &&
          onset[1].compatible(s.pressure[1],reference[1]);
      // The unloading-event forces are completion forces, not breakaway
      // forces.  Replaying an asymmetric event pair as a fast approach made
      // the next reach begin at [2.8,0.6] N and converted that difference
      // directly into roll.  Use per-arm fast feed-forward only when BOTH
      // onset forces were measured in a compatible contact pose.  If passive
      // venting hid one onset, approach both arms to the smaller event force;
      // the bounded measured search below then identifies what each arm needs.
      const double conservative_event_reach=learned_event_pair_available?
          std::min(identified_extension_thrust[0],
                   identified_extension_thrust[1]):0.;
      bool onset_pair_context=measured_onset_pair ||
          learned_event_pair_available;
      for(size_t j=0;j<2;++j) {
        const double learned_reach=measured_onset_pair?
            onset[j].thrust:conservative_event_reach;
        onset_approach_target[j]=std::max(0.,
            learned_reach-k.onset_margin_n);
        // A fresh measured onset is stronger evidence than reaching the old
        // feed-forward force.  Without this latch, passive release could
        // produce onset_this_stroke while thrust was still below the stored
        // target; fast_approach then became false (because onset was seen),
        // onset_approach_ready stayed false, and Motion 2 deadlocked forever
        // at 0.3--0.5 N.  Mark this arm ready and let the pair resume the
        // bounded measured search once BOTH current-stroke onsets are known.
        if(onset_this_stroke[j]) onset_approach_ready[j]=true;
        if(onset_pair_context &&
           s.thrust[j]+.02>=onset_approach_target[j])
          onset_approach_ready[j]=true;
      }
      if(!onset_pair_context) onset_approach_ready.fill(false);
      const bool onset_pair_approach_pending=onset_pair_context &&
          !(onset_approach_ready[0] && onset_approach_ready[1]);
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
        // The old implementation made the learned absolute bend the completion
        // condition. On soft tissue that does not prove that normal load has
        // actually decreased. Keep 45 deg (or the independent travel cap) only
        // as the deepest safe search pose. The bend measured at the previous
        // unloading event predicts where the next event should occur, but can
        // neither complete the reach nor stop a cautious continued search.
        double safety_bend_target=std::max(.035,std::max(
            k.identification_bend,
            reference[j]-k.identification_extension_limit));
        safety_bend_target=std::min(safety_bend_target,
            reference[j]-k.onset_angle);
        active_bend_target[j]=safety_bend_target;
        active_extension_target[j]=clamp(reference[j]-safety_bend_target,
            k.onset_angle,k.identification_extension_limit);
        // The absolute bend measured at the preceding event is useful only
        // while the new grasp begins from the same geometry.  Along a tapered
        // or compliant arm the start bend can change by tens of degrees; an
        // old absolute target then becomes either already reached or is
        // rejected all the way back to the 45-deg safety boundary.  Reapply
        // the measured per-arm EVENT TRAVEL to the current reference for
        // pneumatic timing.  This target predicts rear exhaust and pilot
        // fill only--fresh body/contact evidence below still owns completion.
        double event_bend_target=identifying_straight?safety_bend_target:
            identified_bend_target[j];
        if(!identifying_straight && calibrated &&
           observed_extension_by_arm[j]>=k.onset_angle)
          event_bend_target=reference[j]-observed_extension_by_arm[j];
        if(!std::isfinite(event_bend_target) || event_bend_target<0. ||
           event_bend_target>=reference[j]-k.onset_angle)
          event_bend_target=safety_bend_target;
        predicted_unload_bend[j]=clamp(event_bend_target,
            safety_bend_target,reference[j]-k.onset_angle);
        reused_event_checkpoint_reached[j]=!identifying_straight && calibrated &&
            s.bend[j]<=predicted_unload_bend[j]+
                k.reused_event_checkpoint_margin;
        const double expected_event_extension=clamp(
            reference[j]-predicted_unload_bend[j],k.onset_angle,
            active_extension_target[j]);
        // Distance conditioning changes only the pressure-driven pull. Every
        // stroke still has to establish a real unloading event before it can
        // use that pull, including a short terminal correction.
        goal_extension_scale=1.;
        const auto& reach_pneumatic=model.pneumatic[j][Model::load(s)];
        // Predict only the small pilot pressure required to make flexion begin
        // at the learned event. Full holding/regrasp pressure is commanded in
        // Pull after the event has actually been measured.
        const double reach_pressure=std::min(
            k.unload_prefill_kpa,c.pressure_max);
        const double reach_pressure_lead=clamp(reach_pneumatic.delay+
            (std::max(0.,reach_pressure-s.pressure[j])+reach_pneumatic.error)/
                std::max(1.,reach_pneumatic.fill),
            0.,k.unload_prefill_max_prediction_sec);
        // Use the measured pneumatic delay to start braking before the angle
        // crosses the target.  Bound the extrapolation to the same four-degree
        // terminal region so a breakaway-rate spike cannot assert completion.
        const double predicted_extension=ext+std::min(
            std::max(0.,-s.rate[j])*reach_pressure_lead,
            k.reused_lift_brake_margin);
        const double predicted_remaining=
            active_extension_target[j]-predicted_extension;
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
              identification_peak_thrust[j],s.thrust[j]);
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
              identification_peak_thrust[j],s.thrust[j]);
        // Initial identification retains a bounded per-arm reach event. The
        // pair handoff below requires both latches, rather than an unrelated
        // body-height/pitch shortcut or an unregulated historical minimum.
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
        const double approach_target=onset_approach_target[j];
        // Venting alone can pass the small onset-angle threshold before any
        // rotor force is applied.  That passive travel is neither a measured
        // breakaway nor a reason to lock the reused stroke at zero thrust.
        // Approach the learned sub-onset force while the measured extension
        // rate remains inside the normal Motion-2 envelope; the reduction-only
        // roll allocator below may still trim an adverse side.
        fast_approach[j]=onset_pair_approach_pending &&
            !onset_approach_ready[j] && !onset_this_stroke[j] &&
            extending<k.extension_rate;
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
        } else {
          // Soft roll braking must not disable the common Motion-2 search.
          // Doing so left both rotors at 0 N while unequal passive venting
          // kept increasing roll.  Generate the bounded search target here;
          // the reduction-only allocator later removes only the component
          // which worsens measured roll.  Independent hard roll/rate guards
          // retain authority to abort/regrip.
          if(ext>=active_extension_target[j]) {
            // Action C waits for BOTH measured front-arm bends.  Hold a
            // leading arm instead of continuing to increase it while the
            // other arm reaches its independently identified absolute target.
            lift_ramp[j]=std::min(lift_ramp[j],s.thrust[j]);
          } else if(extending>k.extension_rate) {
            lift_ramp[j]=std::max(0.,s.thrust[j]-3.*(extending-k.extension_rate)*dt);
          } else {
            // Reuse the previous unloading pose to reach the useful region
            // quickly, then identify the current contact with a gentle creep.
            // The checkpoint never completes Motion 2: fresh body/contact
            // evidence below is still mandatory.
            const double measured_search_rise=
                reused_event_checkpoint_reached[j]?
                    k.reused_event_search_rise:lift_search_rise;
            lift_ramp[j]+=measured_search_rise*
                clamp(1.-extending/k.extension_rate,0.,1.)*dt;
            rise_limit[j]=std::max(rise_limit[j],measured_search_rise);
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
        if(!lift_target_reached[j]) {
          lift_angle_pressure_command[j]=0.;
          if(reused_event_checkpoint_reached[j]) {
            // A leading released arm can continue travelling under a few
            // tenths of a newton while its partner catches up.  The learned
            // event pose therefore becomes a compliant angle-hold region:
            // pressure grows from the pilot value at the outer margin to the
            // bounded 10-kPa hold at the old event pose.  Rotor thrust still
            // creeps upward, so changed contact can produce a fresh body-
            // unloading event; pressure does not declare phase completion.
            const double checkpoint_penetration=clamp(
                predicted_unload_bend[j]+k.reused_event_checkpoint_margin-
                    s.bend[j],0.,k.reused_event_checkpoint_margin);
            const double checkpoint_fraction=checkpoint_penetration/
                k.reused_event_checkpoint_margin;
            const double pilot=std::min(k.unload_prefill_kpa,
                k.reused_lift_hold_pressure_max);
            lift_angle_pressure_command[j]=pilot+
                (k.reused_lift_hold_pressure_max-pilot)*checkpoint_fraction;
          }
        }
        if(calibrated && !deep_windup_enabled && !unload_event_detected &&
           predicted_extension>=k.onset_angle) {
          // The previous measured event supplies only a timing prediction.
          // Extrapolate by the learned pneumatic delay, but bound the look-ahead
          // in trajectory coordinates so a breakaway-rate spike cannot fill the
          // chamber early and suppress unloading.  This path is common to level
          // and sloped contacts; there is no second slope-specific owner.
          pressure_lead_seconds=std::max(
              pressure_lead_seconds,reach_pressure_lead);
          const double expected_event_extension=std::max(k.onset_angle,
              reference[j]-predicted_unload_bend[j]);
          const double measured_ratio=clamp(
              ext/expected_event_extension,0.,1.);
          const double predicted_event_extension=ext+std::min(
              std::max(0.,-s.rate[j])*reach_pressure_lead,
              expected_event_extension*
                  k.unload_prefill_max_prediction_lead_ratio);
          const double predicted_ratio=std::min(
              clamp(predicted_event_extension/expected_event_extension,0.,1.),
              measured_ratio+k.unload_prefill_max_prediction_lead_ratio);
          const double fill_fraction=clamp((predicted_ratio-
              k.unload_prefill_start_ratio)/
              std::max(.05,1.-k.unload_prefill_start_ratio),0.,1.);
          const double pilot_pressure=std::min(in.base_pressure,
              k.unload_prefill_kpa);
          // Prediction schedules only the small pilot fill.  Blending this
          // command to the 30-kPa grasp pressure *before* an unloading event
          // made the released fronts re-flex against the cylinder while the
          // force search was still active.  The controller then interpreted
          // the resulting stall as a need for more thrust and climbed to the
          // 6-N limit without ever unloading both contacts.  Full pressure is
          // owned exclusively by Pull after the measured event; its faster
          // return slew and three-second overlap account for the delay.
          const double request=pilot_pressure*fill_fraction;
          // Monotonic during one reach prevents valve chatter. Pull replaces
          // this pilot with the full pressure-driven flexion command.
          lift_prefill_command[j]=std::max(
              lift_prefill_command[j],request);
        }
        pressure_target[j]=std::max(lift_prefill_command[j],
            lift_angle_pressure_command[j]);
        raw[j]=lift_ramp[j];
      }
      const bool powered_pair_trajectory_ready=onset_this_stroke[0] &&
          onset_this_stroke[1] && powered_extension_reference_valid[0] &&
          powered_extension_reference_valid[1] &&
          powered_body_reference_valid;
      if(powered_pair_trajectory_ready) {
        if(!extension_trajectory.active) {
          Pair trajectory_goal=active_bend_target;
          double largest_travel=0.;
          for(size_t j=0;j<2;++j) {
            // Every reach asks for at least a meaningful 15-deg-class motion;
            // the deeper absolute target remains the independent search bound.
            trajectory_goal[j]=std::min(
                powered_extension_reference[j]-k.trajectory_min_extension,
                active_bend_target[j]);
            trajectory_goal[j]=std::max(trajectory_goal[j],
                powered_extension_reference[j]-
                    k.identification_extension_limit);
            largest_travel=std::max(largest_travel,
                powered_extension_reference[j]-trajectory_goal[j]);
          }
          const double duration=clamp(
              largest_travel/std::max(.05,k.trajectory_extension_rate),
              k.trajectory_extension_min_duration,
              k.trajectory_extension_max_duration);
          extension_trajectory.configure(powered_extension_reference,
              trajectory_goal,duration);
        }
        extension_trajectory.advancePredictive(dt,s.bend,
            trajectory_rate_filtered,trajectory_acceleration,
            k.trajectory_tracking_slow_error,
            k.trajectory_tracking_pause_error,
            k.trajectory_prediction_horizon,
            k.trajectory_rate_lag_horizon,
            k.trajectory_phase_correction_gain,
            k.trajectory_clock_max_scale,
            k.trajectory_prediction_max_delta);
        trajectory_phase=extension_trajectory.phase;
        trajectory_clock_scale=extension_trajectory.clock_scale;
        out.angle_target=extension_trajectory.target;
        for(size_t j=0;j<2;++j) {
          trajectory_angle_error[j]=extension_trajectory.target[j]-s.bend[j];
          trajectory_rate_error[j]=extension_trajectory.rate[j]-
              trajectory_rate_filtered[j];
          // A positive actual-minus-target error means the arm is lagging in
          // extension and needs more rotor force.  Update only while the
          // trajectory clock is active; a paused/stalled path must not wind up
          // to the actuator limit indefinitely.
          if(extension_trajectory.clock_scale>.05) {
            extension_thrust_trim[j]=clamp(extension_thrust_trim[j]+
                k.trajectory_extension_ki*
                    (s.bend[j]-extension_trajectory.target[j])*dt,
                -k.trajectory_extension_trim_max,
                k.trajectory_extension_trim_max);
          }
        }
      }
      if(calibrated && !unload_event_detected) {
        // Motion 2 is a free-extension experiment.  An older checkpoint
        // controller applied as much as 10 kPa to only the leading arm; that
        // arm then re-flexed while its partner kept extending and the pressure
        // transient became a roll actuator.  Before the measured unloading
        // event allow only the small, PAIR-SYNCHRONOUS pneumatic pilot used to
        // compensate valve delay.  Individual angle-pressure feedback belongs
        // to Pull, after front load relief has actually been observed.
        const double common_prefill=std::min(
            lift_prefill_command[0],lift_prefill_command[1]);
        lift_prefill_command.fill(common_prefill);
        lift_angle_pressure_command.fill(0.);
        pressure_target.fill(common_prefill);
      }
      ramp=std::max(lift_ramp[0],lift_ramp[1]);
      const bool powered=std::max(s.thrust[0],s.thrust[1])>.3;
      // Detect actual unloading relative to powered breakaway, not relative
      // to phase entry.  This removes bottom inflation, residual front
      // exhaust and mocap drift from the event measurement.  Each arm owns an
      // independent pre-onset angle reference because their pneumatic delays
      // and initial bends need not be equal.
      unload_pitch_delta=in.directional_pitch-
          (powered_body_reference_valid?powered_start_pitch:start_pitch);
      unload_height_delta=in.height-
          (powered_body_reference_valid?powered_start_height:start_height);
      double minimum_front_extension=std::numeric_limits<double>::infinity();
      Pair powered_front_extension{};
      Pair peak_powered_front_extension{};
      for(size_t j=0;j<2;++j) {
        const double extension_reference=powered_extension_reference_valid[j]?
            powered_extension_reference[j]:reference[j];
        powered_front_extension[j]=std::max(0.,
            extension_reference-s.bend[j]);
        // The useful unloaded interval can finish between two IMU samples.
        // Preserve each arm's largest powered excursion so two asynchronous
        // breakaways still form one pair event instead of waiting for an
        // artificial instant at which both peaks coincide.
        peak_powered_front_extension[j]=std::max(0.,
            extension_reference-minimum[j]);
        minimum_front_extension=std::min(minimum_front_extension,
            powered_front_extension[j]);
      }
      // Arm travel corroborates that both released fronts are responding, but
      // the phase boundary is the measured body/contact unloading event.  Do
      // not silently reintroduce the old fixed 8-deg probe target here: on a
      // soft contact useful load relief can occur earlier, and continuing the
      // search merely amplifies left/right breakaway differences.
      const auto trajectory_choice=trajectory.selected();
      // A few degrees of arm motion only identifies breakaway.  It cannot end
      // Motion 2.  Require a meaningful portion of the explicit bend path in
      // addition to measured body/contact unloading.
      const double required_pair_extension=std::max(
          k.unload_min_extension*trajectory_choice.unload_scale,
          k.trajectory_min_extension*trajectory_choice.unload_scale);
      const bool powered_pair_onset=onset_this_stroke[0] &&
          onset_this_stroke[1] && powered_extension_reference_valid[0] &&
          powered_extension_reference_valid[1] &&
          powered_body_reference_valid;
      // Preserve a rear pivot until both front rotors are genuinely powered,
      // then lead the rear exhaust by the measured valve delay.  Waiting for
      // measured front extension before commanding the valve made the short
      // Motion-2 pulse finish while the rear chambers were still near grasp
      // pressure.  The arm-travel schedule remains as corroboration, but the
      // pneumatic lead may start earlier because pressure is the slow plant.
      const double lift_release_phase=powered_pair_onset?
          clamp((minimum_front_extension/
                    std::max(.017453,required_pair_extension)-
                 k.lift_rear_release_start_ratio)/
                std::max(.05,1.-k.lift_rear_release_start_ratio),0.,1.):0.;
      const double lift_rear_floor=std::min(
          std::min(in.base_pressure,k.rear_motion_pressure_cap),
          rearPressureFloor(k));
      rear_vent_lead_seconds=c.pressure_delay+
          std::max(0.,in.rear_pressure-lift_rear_floor)/
              std::max(1.,c.exhaust_rate)+k.rear_vent_lead_margin;
      const bool front_pair_powered=
          std::min(s.thrust[0],s.thrust[1])>.3;
      double rear_exhaust_lead_phase=0.;
      if(front_pair_powered && rear_vent_lead_seconds>1.e-3) {
        // Reused strokes have a measured Lift-entry-to-unload time.  On the
        // first stroke use the explicit 15-deg-class trajectory duration;
        // this is a timing prior only and cannot complete Motion 2.
        const double nominal_event_time=
            calibrated && std::isfinite(learned_unload_elapsed)?
                learned_unload_elapsed:
                std::max(k.trajectory_extension_min_duration,
                    required_pair_extension/
                        std::max(.05,k.trajectory_extension_rate));
        const double time_to_event=std::max(0.,nominal_event_time-phase_time);
        rear_exhaust_lead_phase=clamp(
            1.-time_to_event/rear_vent_lead_seconds,0.,1.);
      }
      // Rear exhaust is a one-way transfer within one Motion-2 reach.  IMU
      // rebound used to make `minimum_front_extension` shrink again, which
      // raised the target from (for example) 29.6 back to 30 kPa and wasted
      // most of the pneumatic response time.  Never re-grasp here; only the
      // explicit Pull safety transfer below may do that.
      lift_rear_release_progress=std::max(lift_rear_release_progress,
          std::max(minimumJerkPosition(lift_release_phase),
                   minimumJerkPosition(rear_exhaust_lead_phase)));
      rear_pressure_command=in.base_pressure-
          lift_rear_release_progress*(in.base_pressure-lift_rear_floor);
      if(!rear_release_commanded &&
         rear_pressure_command<in.base_pressure-.5) {
        rear_release_commanded=true;
        rear_release_pressure_threshold=std::min(in.base_pressure-.5,
            lift_rear_floor+k.catch_pressure_tolerance);
      }
      const double powered_pair_mismatch=std::abs(
          powered_front_extension[0]-powered_front_extension[1]);
      unload_pair_mismatch_measured=powered_pair_mismatch;
      // The useful unloaded interval is shorter than the pneumatic/IMU
      // skew.  Qualify each front arm from its own powered reference and keep
      // that evidence for a bounded time.  A whole-phase peak is too lenient,
      // while requiring both instantaneous peaks is too strict.  Neither arm
      // can complete Motion 2 alone: both memories, the current 10-deg
      // corridor and signed body unloading are still required below.
      for(size_t j=0;j<2;++j) {
        const bool arm_extension_sample=powered &&
            onset_this_stroke[j] &&
            powered_extension_reference_valid[j] &&
            powered_front_extension[j]>=required_pair_extension;
        unload_extension_memory[j]=arm_extension_sample?
            k.unload_pair_memory:
            std::max(0.,unload_extension_memory[j]-dt);
      }
      const bool front_extension_history_evidence=powered_pair_onset &&
          unload_extension_memory[0]>0. && unload_extension_memory[1]>0.;
      // A useful unloading event must move the body in the commanded pitch
      // direction and must not be an apparent lift caused by bottom inflation
      // or mocap noise. There are two physically meaningful routes:
      //   1) pitch-led: at least the requested pitch without sinking; or
      //   2) height-led: a clear rise accompanied by forward pitch.
      // The reference is captured only after both front rotors break away, so
      // the preceding bottom inflation is excluded from both deltas.
      const double requested_pitch=
          k.unload_pitch*trajectory_choice.unload_scale;
      const double requested_height=
          k.unload_height*trajectory_choice.unload_scale;
      const double corroborating_pitch=
          k.unload_min_pitch*trajectory_choice.unload_scale;
      const bool pitch_led_unload=
          unload_pitch_delta>=requested_pitch &&
          unload_height_delta>=k.unload_height_floor;
      const bool height_led_unload=
          unload_height_delta>=requested_height &&
          unload_pitch_delta>=corroborating_pitch;
      const bool full_kinematic_unload_evidence=
          pitch_led_unload || height_led_unload;
      if(!in.front_released) front_release_event_armed=true;
      const bool new_front_release=
          front_release_event_armed && in.front_released;
      const bool kinematic_unload_evidence=
          full_kinematic_unload_evidence;
      // Accept the event anywhere inside the independent hard boundary;
      // Pull immediately removes the free-extension search and begins the
      // pressure-driven catch.
      const bool unload_roll_safe=std::abs(s.roll)<
          std::max(k.catch_roll_angle,c.roll_limit-0.5*M_PI/180.);
      // Start preparing rear positive assistance at the beginning of the
      // physical unloading event instead of waiting until the full Motion-2
      // completion dwell.  This precursor still requires both front arms,
      // a signed body/contact cue and the normal hard roll corridor.  It may
      // start a pressure/thrust envelope, but cannot transition to Pull.
      const bool candidate_pair_extension=powered_pair_onset &&
          powered_front_extension[0]>=.5*required_pair_extension &&
          powered_front_extension[1]>=.5*required_pair_extension &&
          powered_pair_mismatch<=1.5*k.unload_pair_mismatch;
      const bool candidate_pitch_cue=
          unload_pitch_delta>=.5*requested_pitch &&
          unload_height_delta>=k.unload_height_floor;
      const bool candidate_height_cue=
          unload_height_delta>=.5*requested_height &&
          unload_pitch_delta>=.5*corroborating_pitch;
      const bool load_relief_candidate_sample=powered &&
          extension_trajectory.active && candidate_pair_extension &&
          (candidate_pitch_cue || candidate_height_cue ||
           new_front_release) && unload_roll_safe;
      load_relief_candidate=load_relief_candidate ||
          load_relief_candidate_sample;
      // Explicit contact release is sufficient but not necessary.  The
      // binary MuJoCo contact often stays closed while its normal load has
      // already fallen enough for the pressure-driven pull.  Requiring twice
      // the hardware pitch/height event in that case delayed the handoff
      // until one arm had travelled more than 20 deg.  Use the same measured
      // event on simulation and hardware.  A vibration alone still cannot
      // complete Motion 2 because BOTH-arm powered travel, a measured body or
      // contact cue, roll safety and the confirmation dwell remain required.
      // Contact release and the force model remain useful timing diagnostics,
      // but neither may replace signed body motion. This rejects the real-arm
      // failure in which both fronts extended while pitch moved -0.89 deg and
      // body height fell 1.26 mm.
      const bool body_unload_evidence=kinematic_unload_evidence;
      const bool body_unload_sample=powered && powered_pair_onset &&
          body_unload_evidence && unload_roll_safe;
      const double body_cue_confirmation=new_front_release?
          std::min(.02,k.unload_confirm):std::min(.04,k.unload_confirm);
      unload_body_candidate_time=body_unload_sample?
          std::min(body_cue_confirmation,
              unload_body_candidate_time+dt):
          std::max(0.,unload_body_candidate_time-2.*dt);
      if(unload_body_candidate_time+1.e-9>=body_cue_confirmation)
        unload_body_memory_time=k.unload_event_fusion;
      else
        unload_body_memory_time=std::max(0.,unload_body_memory_time-dt);
      // Detect the unloading event from the measured arm/body response.  Do
      // not require the 0.3-s predicted roll to remain inside the tighter
      // soft-brake boundary: in the repeated MuJoCo reach both arms had
      // extended >7 deg and body height had changed 0.9 mm at only 6 deg
      // measured roll, yet the prediction rejected the event and kept raising
      // thrust until roll exceeded 20 deg.  The event handoff itself removes
      // that extension search and Pull has a guarded early-catch path.  Keep
      // the measured catch-angle boundary here; the independent adapter still
      // owns the hard roll/rate limit.
      // A measured unloading event must END the free-extension search when
      // roll is deteriorating, not be discarded merely because the ordinary
      // 10-deg catch corridor has already been crossed.  Discarding the
      // event left pressure near zero and kept increasing thrust until the
      // independent 20-deg guard fired.  Accept the measured pitch/height or
      // contact event anywhere inside that independent hard boundary; Pull's
      // guarded catch then restores rear support.  Planner/adapter safety
      // still prevents this update at or beyond roll_limit.
      // The phase boundary is a measured unloading event, not a force-model
      // threshold.  Requiring the projected upward force as an additional
      // hard gate kept the zero-pressure search active after the body had
      // already moved in pitch/height; the pair then exchanged leader roles
      // and excited roll.  The force projection remains useful for planning
      // and diagnostics, while BOTH-arm travel and measured body/contact
      // response establish that load relief actually occurred.
      // Powered relative arm travel proves that this is a rotor-driven reach,
      // but it does not prove load relief.  In MuJoCo a [3.8, 3.2]-deg reach
      // was previously promoted while body height had actually fallen
      // 1.4 mm.  Require the paired arm response together with measured body
      // pitch/height or direct contact release, so bottom motion and mocap
      // noise cannot complete Motion 2 and arm motion alone cannot claim that
      // the body is floating.
      const bool measured_body_unload_sample=powered &&
          extension_trajectory.active &&
          front_extension_history_evidence &&
          powered_pair_mismatch<=k.unload_pair_mismatch &&
          (body_unload_evidence || unload_body_memory_time>0.) &&
          unload_roll_safe;
      const bool unload_sample=measured_body_unload_sample;
      // MuJoCo contact state is direct physical evidence and needs only a
      // short debounce.  Hardware has no such signal and therefore retains
      // the longer pitch/height confirmation dwell.
      // The paired pulse event is already time-qualified twice: each arm has
      // a confirmed powered onset and the common impulse has exceeded its
      // minimum ON duration.  Requiring the legacy body-cue dwell once more
      // left the zero-pressure rotors accelerating for another 60 ms; in the
      // repeated MuJoCo stroke the valid [5.3, 4.1]-deg pair then ran on to
      // [19.2, 22.1] deg at 126/248 deg/s before pneumatic flexion began.
      // Hand an aligned two-arm pulse to pressure control in this cycle.
      // Body/contact-only evidence retains its debounce, so mocap noise,
      // passive venting and a one-arm IMU spike cannot use this fast path.
      const double unload_confirmation=in.contact_available &&
          new_front_release?std::min(k.unload_confirm,.04):k.unload_confirm;
      // Pitch/height relief can be a short peak: once normal load starts to
      // leave the front contact, continuing the thrust search makes the peak
      // disappear by moving the body onto the opposite support. Accumulate
      // consistent event samples over a short window and decay them slowly,
      // instead of demanding one uninterrupted plateau. A vibration cannot
      // complete this by itself because powered paired extension and the
      // independent roll corridor are still required on every positive
      // sample.
      unload_evidence_time=unload_sample?
          std::min(unload_confirmation,unload_evidence_time+dt):
          std::max(0.,unload_evidence_time-.25*dt);
      if(!load_relief_detected &&
         unload_evidence_time+1e-9>=unload_confirmation) {
        load_relief_detected=true;
        load_relief_elapsed=phase_time;
        load_relief_thrust=s.thrust;
        load_relief_bend=s.bend;
        for(size_t j=0;j<2;++j) {
          // Load relief is only the beginning of the windup.  The final bend
          // target is deliberately placed just inside the independent hard
          // limit, so the rotor keeps its long moment arm until body pitch is
          // generated.  Start pneumatic braking `extra_extension` before that
          // target; unlike the old implementation this first braking point is
          // not mistaken for the end of arm extension.
          windup_hold_bend[j]=std::min(load_relief_bend[j],std::max(
              active_bend_target[j]+k.trajectory_windup_capture_extension,
              load_relief_bend[j]-k.trajectory_windup_extra_extension));
          windup_brake_start_bend[j]=load_relief_bend[j];
          windup_reference_bend[j]=load_relief_bend[j];
          windup_brake_started_by_arm[j]=true;
        }
        const double available_pitch_moment=
            pitch_moment[0]+pitch_moment[1];
        windup_pitch_direction=available_pitch_moment<0.?-1.:1.;
        if(k.trajectory_golden_handoff) {
          // The paired arm/body event is the opening of the useful unloaded
          // interval.  Begin the pressure-driven return now; waiting for a
          // second fixed pitch target lets the compliant arms rebound or
          // recontact before horizontal traction is available.
          windup_pose_captured=true;
          windup_evidence_time=k.trajectory_windup_confirm;
          const double capture_pressure=std::min(
              in.base_pressure,k.pull_capture_pressure);
          for(size_t j=0;j<2;++j) {
            windup_hold_bend[j]=std::max(active_bend_target[j],s.bend[j]);
            windup_hold_pressure[j]=capture_pressure;
            lift_prefill_command[j]=capture_pressure;
            lift_angle_pressure_command[j]=capture_pressure;
            pressure_target[j]=capture_pressure;
          }
          // Motion 2 must never increase rear grasp after unloading.  Start
          // Motion 3 from its compliant support floor and let measured front
          // flexion schedule the remaining release.
          rear_pressure_command=lift_rear_floor;
        }
      }
      // The old body/contact threshold remains the measured start of load
      // relief.  In deep-windup mode it no longer starts flexion immediately:
      // keep extending around the rear anchor until the measured/predicted
      // body pitch reaches the bounded windup target.  Prediction compensates
      // the rotor and valve delay so full pressure begins before the hard
      // pitch guard, rather than after the body has already crossed it.
      const double safe_windup_ceiling=std::max(k.unload_pitch,
          c.pitch_change_limit-k.trajectory_windup_pitch_margin);
      windup_pitch_target=deep_windup_enabled?
          std::min(k.trajectory_windup_pitch*trajectory_choice.unload_scale,
                   safe_windup_ceiling):
          k.unload_pitch*trajectory_choice.unload_scale;
      const double pitch_braking_horizon=std::min(
          k.trajectory_pressure_lead_max,
          k.trajectory_prediction_horizon+.15);
      const double predicted_pitch_delta=unload_pitch_delta+clamp(
          in.pitch_rate*pitch_braking_horizon,
          -k.trajectory_prediction_max_delta,
          k.trajectory_prediction_max_delta);
      // Only pitch in the commanded travel direction is useful windup.  An
      // absolute value here could accept a backward tip-over as a successful
      // reach and immediately start the swing in the wrong direction.
      windup_pitch_progress=std::max(0.,std::max(
          windup_pitch_direction*unload_pitch_delta,
          windup_pitch_direction*predicted_pitch_delta));
      if(load_relief_detected && deep_windup_enabled &&
         !unload_event_detected) {
        // Keep the front chambers released during the initial impulse. Near
        // the planned deep reach, start a partial pneumatic brake using the
        // predicted angle and rate. Rotor reduction alone cannot arrest a
        // zero-pressure arm moving at 60--140 deg/s before the mechanical
        // limit, whereas immediately restoring 30 kPa creates a second
        // high-friction anchor and prevents body pitch. The bounded partial
        // pressure occupies the useful interval between those two failures.
        Pair brake_pressure{{0.,0.}};
        bool measured_hard_bend_limit=true;
        for(size_t j=0;j<2;++j) {
          // Advance one continuous bend reference from the measured
          // load-relief pose to the deep windup pose.  This avoids the former
          // step from about 95 deg directly to 40 deg, which made angle
          // feedback add thrust while the physical arm was already
          // accelerating.  Angle/rate/acceleration prediction now compares
          // against this moving reference.
          const double windup_elapsed=std::max(0.,phase_time-load_relief_elapsed);
          windup_reference_bend[j]=std::max(windup_hold_bend[j],
              load_relief_bend[j]-
                  k.trajectory_extension_rate*windup_elapsed);
          const auto& pneumatic=model.pneumatic[j][Model::load(s)];
          const double valve_lead=clamp(pneumatic.delay+
              pneumatic.error/std::max(1.,pneumatic.fill),
              .02,k.trajectory_pressure_lead_max);
          const double predicted_bend=trajectory_observer[j].predictAngle(
              s.bend[j],valve_lead,k.trajectory_prediction_max_delta);
          const double predicted_rate=trajectory_observer[j].predictRate(
              valve_lead,k.identification_bend_rate_limit);
          // This is a current-state condition, not a one-way latch.  The two
          // compliant arms can cross the target at different instants and
          // rebound while their chambers are still filling.  Latching each
          // crossing independently made the pair look complete even when one
          // arm had already flexed back by 30--50 deg; the resulting swing
          // started from a large roll-producing mismatch.  Require both arms
          // to be inside their final predicted corridor at the same time.
          windup_target_reached_by_arm[j]=
              std::abs(s.bend[j]-windup_hold_bend[j])<=
                  k.identification_tolerance ||
              (std::abs(predicted_bend-windup_hold_bend[j])<=
                   k.identification_tolerance &&
               std::abs(s.bend[j]-windup_hold_bend[j])<=
                   k.trajectory_prediction_max_delta);
          const bool braking_active=windup_brake_started_by_arm[j];
          if(braking_active) {
            const double brake_span=std::max(1.*M_PI/180.,
                windup_brake_start_bend[j]-windup_hold_bend[j]);
            const double brake_progress=clamp(
                (windup_brake_start_bend[j]-predicted_bend)/brake_span,
                0.,1.);
            // Continue a controlled extension while braking its inertia.  The
            // minimum pressure is phased in only near the final bend target;
            // applying it at the first brake point re-anchored the front pair
            // and limited pitch to roughly two degrees in MuJoCo.
            const double desired_rate=windup_target_reached_by_arm[j]?
                0.:-k.trajectory_extension_rate;
            const double angle_error=
                windup_reference_bend[j]-predicted_bend;
            double pressure=
                k.trajectory_windup_brake_pressure_min*brake_progress+
                k.trajectory_windup_brake_angle_kp*std::max(
                    0.,angle_error)+
                k.trajectory_windup_brake_rate_kd*std::max(
                    0.,desired_rate-predicted_rate);
            // Use the measured 2-D bend model as feed-forward instead of
            // asking a small PD term to discover the force/pressure balance
            // during a sub-second reach.  Search pressure for the current
            // moving bend reference at the measured rotor force, then phase
            // it in quadratically over the extension.  Early Motion 2 stays
            // almost pressure free; near the windup pose the model holds a
            // long moment arm while pitch feedback acts on the body.  Angle,
            // rate and bounded acceleration prediction remain the feedback
            // correction when model/contact mismatch is present.
            if(bend_equilibrium) {
              double model_pressure=0.;
              double model_error=std::numeric_limits<double>::infinity();
              const double pressure_limit=std::min(c.pressure_max,
                  k.trajectory_windup_hold_pressure_max);
              constexpr int pressure_samples=30;
              for(int sample=0;sample<=pressure_samples;++sample) {
                const double candidate=pressure_limit*sample/
                    pressure_samples;
                const double predicted_equilibrium=bend_equilibrium(
                    j,candidate,std::max(s.thrust[j],load_relief_thrust[j]));
                if(!std::isfinite(predicted_equilibrium)) continue;
                const double error=std::abs(
                    predicted_equilibrium-windup_reference_bend[j]);
                if(error<model_error) {
                  model_error=error;
                  model_pressure=candidate;
                }
              }
              if(std::isfinite(model_error))
                pressure=std::max(pressure,
                    model_pressure*brake_progress*brake_progress);
            }
            brake_pressure[j]=clamp(pressure,0.,std::min(c.pressure_max,
                k.trajectory_windup_hold_pressure_max));
          }
          // Prediction starts the individual arm catch.  The pair-wide
          // emergency catch is reserved for an actual measured hard-boundary
          // crossing, otherwise the early arm would force the lagging arm out
          // of Motion 2 before it had completed the same windup trajectory.
          measured_hard_bend_limit=measured_hard_bend_limit &&
              s.bend[j]<=active_bend_target[j];
        }
        const bool predicted_pitch_reached=
            windup_pitch_progress>=windup_pitch_target;
        windup_extension_reached=windup_brake_started_by_arm[0] &&
            windup_brake_started_by_arm[1];
        windup_target_reached=windup_target_reached_by_arm[0] &&
            windup_target_reached_by_arm[1];
        // Reachability is based on measured pitch, not the short-horizon
        // prediction.  Prediction is intentionally conservative and follows
        // gyro spikes; using it as the running maximum kept resetting the
        // stall timer while the compliant arms merely oscillated in place.
        const double measured_pitch_progress=std::max(
            0.,windup_pitch_direction*unload_pitch_delta);
        if(measured_pitch_progress>windup_best_pitch+.25*M_PI/180.) {
          windup_best_pitch=measured_pitch_progress;
          windup_pitch_stall_time=0.;
        } else {
          windup_pitch_stall_time+=dt;
        }
        // A compliant contact may never reach the requested nominal pitch:
        // after the front pair has unloaded, the largest safely reachable
        // pitch is the useful windup boundary. Detect that measured peak
        // directly instead of waiting for an arm-angle target or a fixed
        // timeout. A second dwell is the bounded fallback when this contact
        // produces mostly height and very little pitch.
        const bool reached_useful_pitch=
            windup_best_pitch>=k.trajectory_windup_min_reachable_pitch;
        const bool hold_stalled=
            windup_pitch_stall_time>=k.trajectory_windup_stall_seconds &&
            (reached_useful_pitch ||
             windup_pitch_stall_time>=2.*k.trajectory_windup_stall_seconds);
        // Do not spend the remaining roll margin trying to gain another
        // fraction of a degree of pitch. Start the pressure-driven swing
        // before the independent 10-deg posture-recovery boundary.
        const bool roll_margin_reached=std::abs(gait_roll)>=6.*M_PI/180. &&
            std::abs(s.roll_rate)>.02 && gait_roll*s.roll_rate>0.;
        const bool pneumatic_catch_due=predicted_pitch_reached ||
            hold_stalled || roll_margin_reached || measured_hard_bend_limit;
        if(pneumatic_catch_due && !windup_pose_captured) {
          windup_pose_captured=true;
          windup_reachable_peak=(windup_target_reached || hold_stalled ||
              roll_margin_reached || measured_hard_bend_limit) &&
              !predicted_pitch_reached;
          for(size_t j=0;j<2;++j) {
            windup_hold_bend[j]=std::max(active_bend_target[j],s.bend[j]);
            trajectory_observer[j].reset(s.rate[j]);
          }
        }
        for(size_t j=0;j<2;++j) {
          const double hold=windup_pose_captured?
              std::min(c.pressure_max,k.trajectory_windup_hold_pressure_max):
              brake_pressure[j];
          windup_hold_pressure[j]=hold;
          lift_prefill_command[j]=hold;
          lift_angle_pressure_command[j]=hold;
          pressure_target[j]=hold;
        }
        out.angle_target=windup_hold_bend;
        const double rear_support_strength=std::max(0.,rear_support_command);
        const double rear_pressure_shortfall=clamp(
            (rear_pressure_command-in.rear_pressure)/
                std::max(1.,k.catch_pressure_tolerance),0.,1.);
        const double rear_reverse_need=std::max(
            rear_support_safety_override,rear_pressure_shortfall);
        windup_rear_reverse_thrust=slew(windup_rear_reverse_thrust,
            k.trajectory_windup_rear_reverse_max*
                rear_support_strength*rear_reverse_need,
            k.trajectory_windup_rear_reverse_rise,
            2.*k.trajectory_windup_rear_reverse_rise,dt);
        out.rear_reverse_thrust=windup_rear_reverse_thrust;
      }
      const bool windup_pitch_ready=!deep_windup_enabled ||
          windup_pitch_progress>=windup_pitch_target ||
          windup_reachable_peak || windup_pose_captured;
      windup_evidence_time=load_relief_detected && windup_pitch_ready?
          std::min(k.trajectory_windup_confirm,
                   windup_evidence_time+dt):
          0.;
      if(!unload_event_detected && load_relief_detected &&
         windup_evidence_time+1e-9>=k.trajectory_windup_confirm) {
        unload_event_detected=true;
        unload_from_arm_trajectory=false;
        unload_event_elapsed=phase_time;
        learned_unload_elapsed=std::isfinite(learned_unload_elapsed)?
            .7*learned_unload_elapsed+.3*unload_event_elapsed:
            unload_event_elapsed;
        unload_event_bend=s.bend;
        unload_event_thrust=s.thrust;
        const double extension0=powered_front_extension[0];
        const double extension1=powered_front_extension[1];
        // The event gate has already proved that both arms crossed the
        // required relative extension and that their mismatch is inside the
        // safe corridor.  Use the shallower measured travel as the handoff
        // target. Asking that arm to extend halfway across the remaining
        // mismatch kept Motion 2 alive after load relief and missed the short
        // interval in which pressure must flex the pair and pull the body.
        unload_alignment_extension_target=clamp(
            std::min(extension0,extension1),
            k.unload_min_extension,k.identification_extension_limit);
        // The first measured unloading event is an event boundary, not a
        // reason to abandon the stroke.  Contact and pneumatic delay commonly
        // make one front arm arrive first.  Keep the rear pair grasped, brake
        // that leader with pressure/thrust, and let the lagging arm reach the
        // fixed common-travel target below.  Only the aligned capture is used
        // as the reusable calibration.  Ending the trial here made Motion 2
        // repeatedly identify an asymmetric event without ever executing the
        // pressure-driven Motion-3 pull.
      }
      // Predict when both front bends will reach the previously measured
      // unloading pose. Rear exhaust is scheduled against that event rather
      // than against the deepest allowed arm angle.
      // Rear exhaust time is estimated conservatively from the larger measured
      // rear pressure, the configured pneumatic delay and exhaust rate.  Do not
      // pre-vent before actual extension motion is visible: the rear pair must
      // remain the anchor during the force search.
      predicted_lift_completion_seconds=0.;
      bool completion_prediction_valid=powered;
      bool extension_motion_seen=true;
      for(size_t j=0;j<2;++j) {
        const double remaining=std::max(0.,
            s.bend[j]-predicted_unload_bend[j]);
        const double extending=std::max(0.,-s.rate[j]);
        extension_motion_seen=extension_motion_seen &&
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
      // Once BOTH front rotors have produced measured extension, use the
      // learned pneumatic lead time rather than waiting until the unloading
      // event to start rear exhaust.  The pressure plant is much slower than
      // the 1--1.5-s Motion-2 reach: commanding zero only at handoff leaves
      // substantial pressure physically in the rear chambers.  The predicted
      // term rises smoothly as the remaining reach time enters the exhaust
      // horizon and is merged monotonically with the measured-extension
      // schedule above.
      if(powered_pair_onset &&
         std::isfinite(predicted_lift_completion_seconds) &&
         rear_vent_lead_seconds>1e-3) {
        const double predicted_release_phase=clamp(
            1.-predicted_lift_completion_seconds/rear_vent_lead_seconds,
            0.,1.);
        lift_rear_release_progress=std::max(lift_rear_release_progress,
            minimumJerkPosition(predicted_release_phase));
        rear_pressure_command=in.base_pressure-
            lift_rear_release_progress*(in.base_pressure-lift_rear_floor);
      }
      // Before paired onset the rear pair remains the physical anchor.  The
      // predictor is enabled only after both released fronts are demonstrably
      // moving, so a one-arm IMU spike cannot pre-vent the established support.
      rear_preload=false;
      // The measured arm/body event owns the Motion-2 -> Motion-3 handoff, but it
      // does not prove that the two newly placed front contacts can carry the
      // body together.  In high-friction validation the event appeared at
      // [9.3, 28.0] deg extension; releasing the rear anchor immediately
      // converted that difference into roll and erased the retained travel.
      // Keep rear grasp while the existing addition-only synchronizer lets
      // the lagging arm catch up.  The short dwell rejects one-sample IMU
      // agreement.  Absolute bend angles may still differ because the gate
      // compares travel from each arm's own reference.
      const double event_pair_mismatch=std::abs(
          powered_front_extension[0]-powered_front_extension[1]);
      const double current_minimum_event_extension=std::min(
          powered_front_extension[0],powered_front_extension[1]);
      // Unequal travel is not itself a failure, but a large hidden mismatch
      // becomes roll as soon as the rear anchor is released.  Retain rear
      // support and drive both arms toward the event-conditioned fixed target
      // above.  Exact equality is neither requested nor needed.
      // The event proves body unloading but not a balanced support-transfer
      // pose. Allow the configured compliant-contact corridor and use the
      // bounded, rear-anchored alignment below outside it; exact equality is
      // neither observable during the short event nor required for Pull.
      const double handoff_pair_mismatch=k.unload_pair_mismatch;
      const bool event_pair_ready=unload_event_detected &&
          event_pair_mismatch<=handoff_pair_mismatch &&
          current_minimum_event_extension>=
              unload_alignment_extension_target-
                  k.identification_sync_deadband;
      // Reaching the corridor can occur at high speed. Latch a short capture
      // interval while rear support is still intact and start the common
      // pressure-driven flexion immediately. This is an event-driven boundary,
      // not an extra wait for the brief extension impulse to disappear.
      if(unload_event_detected) {
        const double event_pressure=event_pair_ready?
            std::min(in.base_pressure,k.pull_capture_pressure):
            std::min(in.base_pressure,k.unload_prefill_kpa);
        // The useful interval is brief. Once the two powered trajectories are
        // inside the deliberately broad 8-deg corridor, start the SAME common
        // pressure-driven flexion which Pull will continue.  Waiting for IMU
        // rate to settle first allowed the pair to exchange leader roles and
        // wasted the unloaded interval.  A more asymmetric event keeps only
        // the small common pilot while thrust alignment remains rear-anchored.
        for(size_t j=0;j<2;++j)
          lift_prefill_command[j]=std::max(
              lift_prefill_command[j],event_pressure);
        pressure_target=lift_prefill_command;
      }
      if(unload_event_detected && !unload_capture_started &&
         event_pair_ready) {
        // The load-relief event can be detected while both released arms are
        // still extending rapidly.  Do not wait at the 3-kPa timing pilot for
        // the rate gate to settle by itself: pressure is the actuator which
        // creates the intended flexion and brakes that extension.  Apply the
        // same common support-building pressure used at Pull entry while the
        // rear pair is still fully grasped.  Once rates settle, capture below
        // transfers this command continuously into Motion 3.
        const double capture_pressure=std::min(
            in.base_pressure,k.pull_capture_pressure);
        lift_prefill_command.fill(capture_pressure);
        pressure_target.fill(capture_pressure);
      }
      if(unload_event_detected && !unload_capture_started &&
         event_pair_mismatch>handoff_pair_mismatch) {
        // Rear grasp still carries the body in this short alignment window.
        // Keep pressure pair-synchronous: one-sided 15-kPa braking exchanged
        // leader roles and produced a 20-deg mismatch in simulation.  The
        // fixed bend/rate thrust objective below aligns the pair; pressure
        // remains only the small delay-compensation pilot until it is safe to
        // start the common flexion trajectory.
        const double pilot=std::min(in.base_pressure,k.unload_prefill_kpa);
        lift_prefill_command.fill(pilot);
        pressure_target.fill(pilot);
      }
      if(!unload_capture_started && event_pair_ready) {
        unload_capture_started=true;
        unload_capture_bend=s.bend;
        for(size_t j=0;j<2;++j)
          unload_capture_thrust[j]=std::max(.30,s.thrust[j]);
      }
      const bool event_capture_ready=unload_capture_started &&
          event_pair_mismatch<=handoff_pair_mismatch &&
          current_minimum_event_extension>=
              unload_alignment_extension_target-
                  k.identification_sync_deadband;
      // `unload_event_detected` already includes the configured temporal
      // confirmation, and event_pair_ready proves that both powered arms are
      // inside the spatial corridor.  A second dwell here missed the useful
      // unloaded interval: the faster zero-pressure arm could travel tens of
      // degrees before pressure-driven flexion began.  Latch in this control
      // cycle so Pull starts the pneumatic catch immediately.
      unload_alignment_time=event_capture_ready?
          k.identification_confirm:0.;
      if(unload_event_detected && event_capture_ready)
        front_target_latched=true;
      if(front_target_latched) {
        rear_preload=false;
        // Motion 3 first raises both front chambers to a load-bearing level.
        // Pull owns the subsequent rear release; releasing here exposes the
        // pneumatic fill delay as an unsupported roll impulse.
        rear_low=false;
      }
      if(calibration_only_regrip_pending) {
        out.calibration_regrip=true;
        rear_low=false;
      }
      const bool handoff_due=front_target_latched &&
          !calibration_only_regrip_pending;
      if(handoff_due) {
        stroke_lifted=true;
        // Learn the pair-aligned support-transfer pose, not the earlier
        // instant at which unloading was first detected.  The earlier sample
        // remains available in unload_event_bend for diagnostics and timing.
        Pair event_extension_hint{};
        for(size_t j=0;j<2;++j)
          event_extension_hint[j]=clamp(
              powered_front_extension[j],k.onset_angle,
              k.identification_extension_limit);
        const double minimum_event_extension=std::min(
            event_extension_hint[0],event_extension_hint[1]);
        const double maximum_event_hint=minimum_event_extension+
            2.*k.identification_sync_deadband;
        for(double& value:event_extension_hint)
          value=std::min(value,maximum_event_hint);
        if(!calibrated) {
          for(size_t j=0;j<2;++j) {
            // Save the bounded event travel, not the deepest permitted angle.
            const double safe_bend_target=active_bend_target[j];
            const double extension_reference=
                powered_extension_reference_valid[j]?
                    powered_extension_reference[j]:reference[j];
            identified_bend_target[j]=clamp(
                std::max(extension_reference-event_extension_hint[j],
                         safe_bend_target),.035,
                reference[j]-k.onset_angle);
            identified_extension_thrust[j]=clamp(
                std::max(identification_peak_thrust[j],
                    s.thrust[j]),0.,c.thrust_max);
            observed_extension_by_arm[j]=event_extension_hint[j];
          }
          observed_extension=std::min(observed_extension_by_arm[0],
              observed_extension_by_arm[1]);
          learned_lift_thrust=std::max(identified_extension_thrust[0],
              identified_extension_thrust[1]);
          learned_ratio=clamp(fx/std::max(.01,fx+up),0.,1.);
          calibrated=true;
        } else {
          // Contact slowly changes while moving along an arm. Adapt only from
          // another positively measured unloading event, never from a timeout
          // or from the fixed angle safety boundary.
          for(size_t j=0;j<2;++j) {
            const double extension_reference=
                powered_extension_reference_valid[j]?
                    powered_extension_reference[j]:reference[j];
            identified_bend_target[j]=.75*identified_bend_target[j]+
                .25*(extension_reference-event_extension_hint[j]);
            identified_extension_thrust[j]=clamp(
                .75*identified_extension_thrust[j]+.25*std::max(
                    unload_event_thrust[j],s.thrust[j]),0.,c.thrust_max);
          }
        }
        lift_up=std::min(c.up_force_max*.7,std::max(.2,up));
        lift_force=std::max(s.thrust[0],s.thrust[1]);
        // Motion 2 identifies the force required to unload the contact. Its
        // instantaneous horizontal projection is not a safe Motion-3 pull
        // command: asymmetric/high lift events previously overwrote the
        // bounded 2.5--3.0 N slip-test seed with 4.4--4.6 N. Keep unloading
        // force and horizontal traction independently identifiable; only the
        // measured pull calibration/outer trajectory search may change the
        // latter after a settled retained endpoint exists.
        pull_forward=std::max(pull_forward,
            std::min(k.slip_seed_pull_max,std::max(1.,fx)));
        enter(Pull,in);
      } else if(powered_time>=k.lift_seconds || phase_time>=k.lift_seconds+k.vent_seconds) {
        // During Lift, the inner allocator above owns transient roll by
        // reshaping the two front thrusts. Do not hand a temporary gait
        // excursion to shared posture recovery before the unloading event:
        // that repeatedly interrupted Motion 2 at 9--15 deg and released a
        // second support pair. The adapter independently hands over at the
        // measured 20-deg/rate safety limit, and residual roll is evaluated
        // after regrasp at the empty-bottom boundary.
        // An unachieved lift at the deadline is not a normal pull/sample.
        out.fault=true;
      }
    }
    bool pneumatic_pair_catch_active=false;
    if(phase==Pull || phase==FrontCatch) {
      rear_low=rear_release_commanded && !support_transfer_started;
      bool angle_caught=true, stalled_caught=phase==FrontCatch;
      bool pressure_loaded_pair=true, minimum_support_loaded_pair=true,
          loaded_catch_pressure_pair=true, flex_return_pair=true;
      planned_mix=0.;pressure_lead_seconds=0.;
      Pair measured_flex{},predicted_flex{},fill_target{};
      const auto selected=trajectory.selected();
      if(phase==Pull && !flex_trajectory.active) {
        double pneumatic_time=0.,largest_return=0.;
        for(size_t j=0;j<2;++j) {
          const auto& pneumatic=model.pneumatic[j][Model::load(s)];
          const double target_pressure=std::min(c.pressure_max,
              in.base_pressure+k.pull_pressure_margin);
          const double fill_time=pneumatic.delay+
              (std::max(0.,target_pressure-s.pressure[j])+pneumatic.error)/
                  std::max(1.,pneumatic.fill);
          pneumatic_time=std::max(pneumatic_time,fill_time);
          largest_return=std::max(largest_return,
              std::max(0.,grasp_reference[j]-pull_start[j]));
        }
        // The same duration schedules angle, pressure and pull thrust.  It is
        // long enough for the learned chamber response, but never the old
        // instantaneous 0 -> 30-kPa switch.
        const double duration=clamp(std::max(
            largest_return/std::max(.05,k.trajectory_flex_rate),
            pneumatic_time+.10),k.trajectory_flex_min_duration,
            k.trajectory_flex_max_duration);
        flex_trajectory.configure(pull_start,grasp_reference,duration);
      }
      if(phase==Pull && flex_trajectory.active) {
        flex_trajectory.advancePredictive(dt,s.bend,
            trajectory_rate_filtered,trajectory_acceleration,
            k.trajectory_tracking_slow_error,
            k.trajectory_tracking_pause_error,
            k.trajectory_prediction_horizon,
            k.trajectory_rate_lag_horizon,
            k.trajectory_phase_correction_gain,
            k.trajectory_clock_max_scale,
            k.trajectory_prediction_max_delta);
        trajectory_phase=flex_trajectory.phase;
        trajectory_clock_scale=flex_trajectory.clock_scale;
      }
      for(size_t j=0;j<2;++j) {
        const double span=std::max(.017453,grasp_reference[j]-pull_start[j]);
        measured_flex[j]=clamp((s.bend[j]-pull_start[j])/span,0.,1.);
        const auto& pneumatic=model.pneumatic[j][Model::load(s)];
        fill_target[j]=std::min(c.pressure_max,
            in.base_pressure+k.pull_pressure_margin);
        const double fill_time=pneumatic.delay+
            (std::max(0.,fill_target[j]-s.pressure[j])+pneumatic.error)/
                std::max(1.,pneumatic.fill);
        pressure_lead_seconds=std::max(pressure_lead_seconds,fill_time);
        const double horizon=clamp(fill_time,.05,.5);
        predicted_flex[j]=clamp(measured_flex[j]+
            clamp(s.rate[j]*horizon/span,0.,.1),0.,1.);
      }
      const double pair_pressure=std::min(s.pressure[0],s.pressure[1]);
      const double front_flex_mismatch=std::abs(
          measured_flex[0]-measured_flex[1]);
      const double pair_flex=std::min(measured_flex[0],measured_flex[1]);
      const double common_flex=.5*(measured_flex[0]+measured_flex[1]);
      // Continue the Motion-2 support transfer without a pressure step.  The
      // slower front arm determines the rear exhaust, while measured front
      // pressure proves that the newly placed pair can begin carrying load.
      // The final floor comes from the signed rear-support coordinate learned
      // around the probe-derived prior.
      const double pull_release_phase=clamp(
          (pair_flex-k.pull_rear_release_start_ratio)/
              (k.pull_rear_release_end_ratio-
                  k.pull_rear_release_start_ratio),0.,1.);
      const double front_support_fraction=clamp(
          (pair_pressure-k.pull_support_transfer_pressure)/
              std::max(1.,k.pull_capture_pressure-
                  k.pull_support_transfer_pressure),0.,1.);
      rear_release_progress=minimumJerkPosition(pull_release_phase)*
          front_support_fraction;
      const double selected_rear_floor=std::min(
          std::min(in.base_pressure,k.rear_motion_pressure_cap),
          rearPressureFloor(k));
      const double continuous_release=std::max(
          lift_rear_release_progress,rear_release_progress);
      const double scheduled_rear_pressure=in.base_pressure-
          continuous_release*(in.base_pressure-selected_rear_floor);
      // Release is monotonic over one Motion-2/3 stroke.  A measured roll risk
      // may start support_transfer below, but flexion itself never re-grasps
      // the old rear point and destroys the useful unloaded interval.
      if(rear_support_safety_override>1.e-6)
        rear_pressure_command=std::max(rear_pressure_command,
            scheduled_rear_pressure);
      else
        rear_pressure_command=std::min(rear_pressure_command,
            scheduled_rear_pressure);
      if(!rear_release_commanded &&
         rear_pressure_command<in.base_pressure-.5) {
        rear_release_commanded=true;
        rear_release_pressure_threshold=std::min(in.base_pressure-.5,
            selected_rear_floor+k.catch_pressure_tolerance);
      }
      rear_low=rear_pressure_command<=selected_rear_floor+.5 &&
          !support_transfer_started;
      rear_release_observed=rear_release_observed ||
          (rear_release_commanded &&
           std::isfinite(rear_release_pressure_threshold) &&
           in.rear_pressure<=rear_release_pressure_threshold);
      const double measured_rear_release=clamp(
          (in.base_pressure-in.rear_pressure)/std::max(1.,
              in.base_pressure-selected_rear_floor),0.,1.);
      // Scheduled release is available immediately during the golden flexion
      // interval; measured pressure decay catches up through the pneumatic
      // loop.  Never reduce authority below what either observation proves.
      const double rear_release_fraction=std::max(
          measured_rear_release,rear_release_progress);
      goal_pull_envelope=1.;
      if(goal_conditioned && goal_scale<1.) {
        const double cutoff=clamp(goal_scale,k.goal_scale_min,1.);
        goal_pull_envelope=clamp((cutoff-common_flex)/
            k.goal_pull_taper_fraction,0.,1.);
      }
      // Golden-time progress follows the observed bend return.  A short
      // kinematic prediction compensates sensor/valve delay, but is limited to
      // the already bounded 10% prediction used above.  When roll is worsening
      // the slower arm owns the phase; otherwise a harmless left/right bend
      // difference is allowed and the common response advances the force path.
      Pair controlled_flex{};
      for(size_t j=0;j<2;++j)
        controlled_flex[j]=clamp(.7*measured_flex[j]+.3*predicted_flex[j],0.,1.);
      golden_flex_progress=gait_roll_worsening?
          std::min(controlled_flex[0],controlled_flex[1]):
          .5*(controlled_flex[0]+controlled_flex[1]);
      const double forward_phase=clamp(
          (golden_flex_progress-k.pull_forward_start_flex_ratio)/
              (k.pull_full_thrust_flex_ratio-
                  k.pull_forward_start_flex_ratio),0.,1.);
      const double forward_envelope=minimumJerkPosition(forward_phase);
      const double up_fade_phase=clamp(
          (golden_flex_progress-k.pull_up_hold_flex_ratio)/
              std::max(.05,1.-k.pull_up_hold_flex_ratio),0.,1.);
      const double up_envelope=1.-minimumJerkPosition(up_fade_phase);
      const double minimum_mix=calibrated?
          clamp(learned_ratio+selected.mix_offset,.1,.95):.5;
      const double nominal_force=std::max(.2,lift_up+pull_forward);
      // delta-m changes the requested horizontal/upward split around the
      // measured contact-specific baseline. k_F remains the outer magnitude
      // variable; Motion 2 itself is unchanged.
      const double peak_forward=std::max(.2,
          pull_forward+selected.mix_offset*nominal_force);
      const double peak_up=std::max(.2,
          lift_up-selected.mix_offset*nominal_force);
      const double distance_force_scale=goal_conditioned?
          std::sqrt(goal_scale):1.;
      const double common_force_scale=selected.force_scale*
          distance_force_scale*goal_pull_envelope*rear_release_fraction;
      golden_desired_up=peak_up*common_force_scale*up_envelope;
      golden_desired_forward=peak_forward*common_force_scale*
          forward_envelope;
      planned_mix=(golden_desired_up+golden_desired_forward)>1.e-6?
          golden_desired_forward/
              (golden_desired_up+golden_desired_forward):minimum_mix;
      golden_window_active=phase==Pull && !support_transfer_started &&
          golden_flex_progress<1.;
      for(size_t j=0;j<2;++j) {
        // Return to the measured feasible grasp, not an imposed 90-deg pose.
        // Motion 2 and 3 now share one time-parametrized bend path: Pull starts
        // exactly at its measured handoff angle and continues with a smooth
        // minimum-jerk return to the pre-release grasp.
        const double target=grasp_reference[j];
        const double scheduled_bend=phase==Pull && flex_trajectory.active?
            flex_trajectory.target[j]:target;
        const double scheduled_rate=phase==Pull && flex_trajectory.active?
            flex_trajectory.rate[j]:0.;
        out.angle_target[j]=scheduled_bend;
        const double span=std::max(.017453,target-pull_start[j]);
        const double actual=measured_flex[j];
        // Prediction helps schedule pressure but cannot assert an unobserved
        // catch. Reduce pressure only on the flexion leader; once the pair is
        // aligned (or FrontCatch begins), both targets return to full grasp.
        const double progress=predicted_flex[j];
        const double flex_lead=std::max(0.,
            predicted_flex[j]-predicted_flex[1-j]);
        // Do not chase every IMU flexion difference with chamber pressure.
        // The bend estimate is intentionally fast and can exchange the
        // leader every few samples; the old unrestricted 30/10-kPa switching
        // then excited roll instead of suppressing it.  Equal fill is the
        // nominal trajectory. Use a small one-sided pneumatic trim only when
        // a large flexion mismatch and measured outward roll agree that the
        // asymmetry is physically affecting the body. Thrust allocation owns
        // smaller/shorter roll errors.
        const bool pressure_sync_needed=phase==Pull &&
            front_flex_mismatch>.25 && gait_roll_worsening;
        const double sync_drop=pressure_sync_needed?std::min(
            std::min(k.pull_pressure_sync_max_drop,5.),
            k.pull_pressure_sync_gain*flex_lead):0.;
        // Pressure direction does not share the verified rotor-moment sign:
        // increasing pressure flexes an arm, whereas positive rotor thrust
        // extends it. Applying the rotor sign to pressure reduced the lagging
        // chamber as well as the flexion leader, leaving both at 18 kPa and
        // allowing one front arm to remain straight while roll diverged.
        // Synchronize the measured flexion response here; the independent
        // thrust allocator below owns body-roll correction.
        const auto& pneumatic=model.pneumatic[j][Model::load(s)];
        const double pressure_lead=clamp(pneumatic.delay+
            pneumatic.error/std::max(1.,pneumatic.fill),0.,
            k.trajectory_pressure_lead_max);
        const double state_prediction_lead=std::min(
            pressure_lead,k.trajectory_prediction_horizon);
        const Pair pressure_angle_target=flex_trajectory.active?
            flex_trajectory.predictedTarget(pressure_lead):grasp_reference;
        const Pair pressure_rate_target=flex_trajectory.active?
            flex_trajectory.predictedRate(state_prediction_lead):Pair{};
        const double pressure_phase=flex_trajectory.active?
            clamp((flex_trajectory.elapsed+pressure_lead)/
                flex_trajectory.duration,0.,1.):1.;
        const double pressure_ff=fill_target[j]*
            minimumJerkPosition(pressure_phase);
        // The chamber target is advanced by the full learned pneumatic delay,
        // but a constant-acceleration arm prediction is trustworthy only over
        // the short gyro horizon.  Extrapolating a breakaway acceleration for
        // the full 0.3--0.6 s pressure delay creates a false bend-rate command.
        const double pressure_predicted_bend=
            trajectory_observer[j].predictAngle(s.bend[j],
                state_prediction_lead,
                k.trajectory_prediction_max_delta);
        const double pressure_predicted_rate=
            trajectory_observer[j].predictRate(state_prediction_lead,
                k.identification_return_bend_rate_limit);
        const double pressure_angle_error=
            pressure_angle_target[j]-pressure_predicted_bend;
        const double pressure_rate_error=
            pressure_rate_target[j]-pressure_predicted_rate;
        if(phase==Pull && flex_trajectory.active &&
           flex_trajectory.clock_scale>.05) {
          flex_pressure_trim[j]=clamp(flex_pressure_trim[j]+
              k.trajectory_pressure_ki*pressure_angle_error*dt,
              -k.trajectory_pressure_trim_max,
              k.trajectory_pressure_trim_max);
        }
        pressure_target[j]=clamp(pressure_ff+
            k.trajectory_pressure_kp*pressure_angle_error+
            k.trajectory_pressure_kd*pressure_rate_error+
            flex_pressure_trim[j]-sync_drop,0.,fill_target[j]);
        trajectory_angle_error[j]=scheduled_bend-s.bend[j];
        trajectory_rate_error[j]=scheduled_rate-
            trajectory_rate_filtered[j];
        // Both front rotors share one envelope to avoid introducing roll.
        // Pressure continues to the full grasp even after the final-stroke
        // thrust impulse has ended.
        // Keep only the measured event-hold force while the rear anchor is
        // loaded.  Release the optimized lift/traction vector continuously
        // with the measured rear-pressure decay, not with the valve command.
        const double desired_up=.5*golden_desired_up;
        const double desired_x=.5*golden_desired_forward;
        // Project the desired force onto the measured rotor direction. Do
        // not divide by cos(bend) near horizontal and request infinite lift.
        const double norm=ux[j]*ux[j]+uz[j]*uz[j];
        const double projected=norm>.05?(desired_x*ux[j]+desired_up*uz[j])/norm:0.;
        // Pressure must first reverse this arm from extension into flexion.
        // Retain only a bounded fraction of the event thrust at zero progress,
        // then release the requested horizontal pull as measured/predicted
        // flexion advances.  Each arm gets its own envelope, so a lagging arm
        // is not held straight merely because its partner already flexed.
        // Do not unload the newly advanced contact at the exact Lift->Pull
        // transition.  The measured event thrust is the smallest value which
        // just produced body unloading for THIS arm/contact.  Dropping from
        // [3.09,2.05] N to [2.37,1.33] N while pressure started to flex the
        // arms let the MuJoCo front contact slide 27 mm backward.  Hold that
        // per-arm event value, then increase only as measured flexion releases
        // the requested horizontal pull.  When no event sample exists (a
        // direct unit-test/legacy entry), retain the bounded old fallback.
        const double event_hold=std::isfinite(unload_event_thrust[j]) &&
            unload_event_thrust[j]>.05 ? unload_event_thrust[j] :
            k.pull_residual_thrust_ratio*lift_force;
        // The event force belongs to load reduction, not to the entire pull.
        // Retain it while front pressure is too small to anchor the newly
        // advanced contact, then hand support continuously to pneumatic
        // flexion.  Keeping the full event force after both chambers reached
        // 15 kPa stored a large elastic shear in compliant tissue and the
        // body sprang backward when the rotors stopped.
        // Hold the measured unloading thrust through the load-light portion,
        // then remove it with the same upward-force envelope.  The earlier
        // linear decay began immediately at u=0 and recreated contact before
        // horizontal traction had risen; this explicit overlap is the
        // lift--flex--pull golden interval.
        const double residual=.30+
            std::max(0.,event_hold-.30)*up_envelope;
        const double pull_target=std::max(projected,residual);
        raw[j]=pull_target;
        // Positive thrust extends the arm.  Add only the authority needed to
        // keep pressure-driven flexion on the scheduled trajectory; never
        // command negative thrust to force a faster bend.  Once pressure is
        // load bearing, scheduled_bend reaches `target` and this term fades
        // naturally while the horizontal force objective remains active.
        const double premature_flex=std::max(0.,
            s.bend[j]-scheduled_bend);
        const double premature_flex_rate=std::max(0.,s.rate[j]);
        const double angle_hold=clamp(
            k.pull_angle_kp*premature_flex+
                k.pull_angle_kd*premature_flex_rate,
            0.,k.pull_angle_hold_max);
        raw[j]=std::max(raw[j],residual+angle_hold);
        // Pressure supplies positive bend torque and rotor thrust opposes it.
        // Apply a bounded signed correction around the planned pull: an arm
        // which flexes ahead receives more holding thrust, while an arm still
        // too extended releases thrust so chamber pressure can catch up.
        const double pull_angle_correction=clamp(
            k.trajectory_pull_thrust_kp*(
                trajectory_predicted_bend[j]-
                    flex_trajectory.predictedTarget(
                        k.trajectory_prediction_horizon)[j])+
            k.trajectory_pull_thrust_kd*(
                trajectory_observer[j].predictRate(
                    k.trajectory_prediction_horizon,
                    k.identification_return_bend_rate_limit)-
                flex_trajectory.predictedRate(
                    k.trajectory_prediction_horizon)[j]),
            -k.trajectory_pull_thrust_trim_max,
            k.trajectory_pull_thrust_trim_max);
        raw[j]=clamp(raw[j]+pull_angle_correction,0.,c.thrust_max);
        // The unloading-event force is valid only until the arm starts its
        // pressure-driven return.  If it is still extending rapidly at the
        // phase boundary, continuing that full force defeats the chamber and
        // turns left/right rate mismatch directly into body roll.  Brake as a
        // continuous function of measured extension rate, then let measured
        // flexion release the optimized horizontal pull in the usual way.
        const double extension_rate=std::max(0.,-s.rate[j]);
        const double extension_brake=clamp(
            (extension_rate-k.pull_extension_brake_start)/
                (k.pull_extension_brake_full-
                    k.pull_extension_brake_start),0.,1.);
        if(extension_brake>0.) {
          // Braking one still-extending arm is necessary, but copying its
          // almost-zero result to the pair removed all front support exactly
          // at Motion-3 entry. Keep a bounded fraction of the force which
          // actually produced the measured unloading event. Pressure can
          // therefore reverse both arms while horizontal traction starts,
          // without continuing the full extension impulse.
          const double transition_floor=.30+
              std::max(0.,event_hold-.30)*
                  k.pull_residual_thrust_ratio*up_envelope;
          const double braked=.30+
              std::max(0.,raw[j]-.30)*(1.-extension_brake);
          raw[j]=std::min(raw[j],std::max(braked,transition_floor));
          fall_limit[j]=k.pull_extension_brake_fall;
        }
        out.desired_up+=desired_up;out.desired_forward+=desired_x;
        // Reproducing the old grasp geometry means being close to it from
        // either direction.  The former one-sided comparison also accepted a
        // grossly over-flexed arm (for example 121 deg for a 107-deg target),
        // which bypassed the loaded-contact traction dwell and ended Motion 3
        // after only the short minimum.  Contact-limited objects already have
        // the independent pressure/flex/stall paths below, so they do not
        // need this unsafe shortcut.
        const bool this_angle_caught=
            std::abs(s.bend[j]-target)<=k.front_catch_tolerance &&
            s.pressure[j]>=in.base_pressure-3. && std::abs(s.rate[j])<.2;
        angle_caught=angle_caught && this_angle_caught;
        const bool pressure_loaded=s.pressure[j]>=
            fill_target[j]-k.catch_pressure_tolerance;
        pressure_loaded_pair=pressure_loaded_pair && pressure_loaded;
        // During left/right flexion synchronization, the leader is
        // intentionally commanded below the full grasp pressure.  Requiring
        // it to reach base_pressure-3 kPa before an emergency support
        // transfer made the catch condition unreachable and left the rear
        // pair exhausted while roll grew from 6 to 20 deg.  The reduced
        // synchronization target is nevertheless a real load-bearing
        // pressure and is sufficient to end the unloaded interval.
        const double minimum_support_pressure=std::min(
            std::max(0.,fill_target[j]-k.pull_pressure_sync_max_drop-
                k.catch_pressure_tolerance),
            k.pull_support_transfer_pressure);
        minimum_support_loaded_pair=minimum_support_loaded_pair &&
            s.pressure[j]>=minimum_support_pressure;
        // The pneumatic loop is considered on target within
        // catch_pressure_tolerance.  Use the same tolerance for the
        // load-bearing catch: a stable 17.95-kPa chamber commanded to the
        // 18-kPa threshold must not spend the entire FrontCatch timeout in a
        // numerical deadband.  Keep the independent support-transfer floor,
        // so a merely prefilled 3--5-kPa chamber cannot complete Motion 3.
        const double loaded_catch_threshold=std::max(
            k.pull_support_transfer_pressure,
            k.pull_loaded_catch_pressure-k.catch_pressure_tolerance);
        loaded_catch_pressure_pair=loaded_catch_pressure_pair &&
            s.pressure[j]>=loaded_catch_threshold;
        flex_return_pair=flex_return_pair &&
            actual>=k.pull_regrip_min_flex;
        const bool safely_stalled=s.bend[j]>=s.floor[j]+k.catch_safe_margin &&
            std::abs(s.rate[j])<=k.catch_stall_rate;
        catch_stall_time[j]=(phase==FrontCatch && pressure_loaded && safely_stalled)?
            catch_stall_time[j]+dt:0.;
        stalled_caught=stalled_caught &&
            catch_stall_time[j]+1e-9>=k.catch_stall_confirm;
      }
      // Immediately after the extension impulse, pressure -- not a rotor
      // imbalance -- must reverse both arms into flexion.  Per-arm rate
      // braking is still used above to detect the safer of the two demands,
      // but apply that smaller demand to BOTH rotors until both measured arms
      // have started returning.  Keeping the two individual brake outputs
      // produced a 2.38/0.79-N pair in MuJoCo even though the pulse ended with
      // only 0.5 deg of extension mismatch; that differential became roll
      // before pneumatic flexion could take control.
      const bool pneumatic_pair_catch=phase==Pull &&
          std::min(measured_flex[0],measured_flex[1])<
              k.pull_regrip_min_flex;
      pneumatic_pair_catch_active=pneumatic_pair_catch;
      if(pneumatic_pair_catch) {
        const double common_catch_thrust=std::min(raw[0],raw[1]);
        raw.fill(common_catch_thrust);
        fall_limit.fill(std::max(k.pull_extension_brake_fall,
            c.thrust_fall));
      }
      // Once both front chambers are load bearing and both released arms have
      // actually reversed into flexion, the traction impulse is a common
      // front-pair command.  Per-arm extension-rate braking is essential at
      // the Lift->Pull boundary, but retaining its resulting 1--2 N mismatch
      // through rear release turns that mismatch directly into roll. Raise
      // only the smaller target to the larger measured-safe demand; the roll
      // allocator below may then introduce the differential required by the
      // measured body response.
      const bool common_pull_ready=phase==Pull &&
          pair_pressure>=k.pull_capture_pressure &&
          std::min(measured_flex[0],measured_flex[1])>=
              k.pull_regrip_min_flex;
      if(common_pull_ready) {
        const double common_pull=std::max(raw[0],raw[1]);
        for(size_t j=0;j<2;++j)
          if(raw[j]<common_pull) {
            raw[j]=common_pull;
            rise_limit[j]=std::max(rise_limit[j],
                k.identification_tracking_rise);
          }
      }
      // Positive signed rear support first raises chamber pressure. Reverse
      // thrust is used only while that pneumatic support is late (or the
      // inner roll override is active), so it cannot fight the negative-side
      // forward-assist command.
      const double rear_support_strength=std::max(0.,rear_support_command);
      const double rear_pressure_shortfall=clamp(
          (rear_pressure_command-in.rear_pressure)/
              std::max(1.,k.catch_pressure_tolerance),0.,1.);
      const double rear_reverse_need=std::max(
          rear_support_safety_override,rear_pressure_shortfall);
      out.rear_reverse_thrust=std::max(out.rear_reverse_thrust,
          k.trajectory_windup_rear_reverse_max*rear_support_strength*
              rear_reverse_need*common_flex*goal_pull_envelope);
      const double predicted_roll=gait_roll+.3*s.roll_rate;
      const bool roll_stable=std::abs(s.roll_rate)<=k.catch_roll_rate &&
          std::abs(predicted_roll)<=std::abs(gait_roll)+k.catch_roll_prediction_tolerance;
      stalled_caught=stalled_caught && roll_stable;
      const bool contact_ok=!in.contact_available || in.front_contact;
      // MuJoCo provides explicit front contact.  The real robot does not, so
      // use the same observable event there: both chambers are loaded and
      // both arms have measurably reversed into flexion.  This is the start of
      // a load-bearing catch, not a requirement to reproduce the old free
      // angle.  Reject it while roll is already running away.
      const bool pressure_flex_caught=pressure_loaded_pair &&
          flex_return_pair && contact_ok && roll_stable &&
          std::abs(gait_roll)<k.catch_roll_angle;
      const bool loaded_contact_caught=loaded_catch_pressure_pair &&
          flex_return_pair && contact_ok && roll_stable &&
          std::abs(gait_roll)<k.catch_roll_angle;
      // A worsening-roll guard is allowed to cancel rear exhaust before the
      // measured rear-release threshold is crossed.  In that case the
      // ordinary `rear_release_observed` completion path is intentionally
      // unreachable: the old rear contact was never fully removed.  Once
      // both pressure-flexed front arms and the restored rear pair are
      // carrying load, nevertheless, all four supports have formed a valid
      // safe boundary for the next short extension impulse.  Recognize that
      // measured boundary instead of waiting in FrontCatch until timeout.
      // This does not weaken an ordinary anchor-transfer stroke; only a
      // support transfer which has already been started by the roll guard
      // may use this path, and explicit front contact remains mandatory when
      // the simulator provides it.
      const bool guarded_four_arm_caught=support_transfer_started &&
          in.rear_grasped && loaded_catch_pressure_pair &&
          flex_return_pair && contact_ok && roll_stable &&
          std::abs(gait_roll)<k.catch_roll_angle;
      // Transfer the anchor only after BOTH newly placed front chambers have
      // reached the measured load-bearing pressure and have reversed into
      // flexion.  Starting rear refill at the former 5-kPa pilot threshold
      // recreated the old rear anchors during the useful horizontal pull:
      // Motion 3 moved 11--14 mm, but nearly all of it relaxed at the settled
      // endpoint.  The pressure threshold is measured rather than timed, so
      // it follows the real pneumatic delay.  A separate roll-risk branch
      // below still restores rear support immediately when posture requires
      // it.
      // Use the slower arm's MEASURED flexion for anchor transfer.  The
      // golden force scheduler may use a bounded prediction, but that
      // prediction is not evidence that both new contacts are already load
      // bearing.  Require the pressure/flex event to persist as well; a
      // single IMU or pressure sample must not latch rear refill for the rest
      // of Motion 3.
      const bool traction_released=
          pair_flex>=k.pull_support_transfer_min_flex_ratio;
      const bool support_transfer_sample=rear_release_observed && contact_ok &&
          loaded_catch_pressure_pair && flex_return_pair &&
          traction_released && roll_stable;
      support_transfer_ready_time=support_transfer_sample?
          std::min(k.pull_regrip_confirm,
              support_transfer_ready_time+dt):
          std::max(0.,support_transfer_ready_time-2.*dt);
      // Keep the optimized low-pressure interval through the useful traction
      // dwell.  FrontCatch is already beyond that bounded Pull interval, so
      // it may transfer as soon as the sustained measured event is present.
      const bool traction_dwell_complete=phase==FrontCatch ||
          phase_time>=std::max(k.pull_min_seconds,
              k.pull_loaded_catch_min_seconds);
      const bool support_transfer_ready=
          support_transfer_ready_time+1.e-9>=k.pull_regrip_confirm &&
          traction_dwell_complete;
      // If the released interval develops a confirmed worsening roll, restore
      // the rear anchor immediately even when one front arm has not yet
      // flexed. Waiting for the ordinary catch-angle boundary spends the
      // pneumatic refill delay after the body is already rotating and was too
      // late on the tapered forearm.
      // Requiring the normal two-arm flex condition in this case creates a
      // deadlock: the stuck arm prevents rear refill while the unsupported
      // body continues toward the independent hard-roll guard.
      // Pneumatic refill has a finite delay.  Waiting until the rear pressure
      // has already crossed the measured-release threshold spends that delay
      // while the body is unsupported.  Once rear exhaust has been commanded,
      // a confirmed worsening roll therefore cancels the release immediately;
      // the settled endpoint still decides whether useful progress remained.
      const bool golden_roll_limit_reached=
          std::abs(predicted_roll)>=k.pull_golden_roll_limit;
      const bool pull_support_brake=posture_brake &&
          (!golden_window_active || golden_roll_limit_reached);
      const bool emergency_support_transfer=
          (rear_release_commanded || rear_release_observed) &&
          pull_support_brake;
      if(support_transfer_ready || emergency_support_transfer)
        support_transfer_started=true;
      rear_low=rear_release_commanded && !support_transfer_started;
      // If roll is already running outward, do not wait for a normal settled
      // contact after the rear refill has begun.  Otherwise continue Pull so
      // pressure-driven flexion can actually place and pull the front pair.
      // `posture_brake` starts the fast inner roll allocator at only a few
      // degrees.  It must not also terminate Motion 3: doing so classified the
      // normal lift/flex transient as residual posture error and regripped
      // before the pressure synchronizer could bend the lagging arm.  Keep
      // correcting inside the gait and request an early support transfer only
      // at the separate catch boundary.  The adapter retains its harder
      // absolute roll/rate safety guard.
      // `posture_brake` already requires a sustained outward roll trend, not
      // merely a static contact offset.  Waiting for the separate 10-deg
      // catch angle after that confirmation allowed angular momentum to grow
      // throughout the rear-chamber refill delay.  End the short unsupported
      // pull as soon as the filtered brake is confirmed; stable gait roll is
      // still ignored and does not cause this transition.
      const bool posture_guarded_catch=pull_support_brake &&
          support_transfer_started;
      // Explicit front contact remains mandatory for a normal completed
      // catch.  It is deliberately not mandatory for the posture-guarded
      // branch: that branch is a safety support transfer, not a claim that
      // the new front contact point is already established.  Regrip restores
      // all four chambers and the stable endpoint check decides whether the
      // stroke completed safely.
      const bool ordinary_caught=angle_caught || stalled_caught ||
          pressure_flex_caught;
      const bool caught=(rear_release_observed &&
          (ordinary_caught || loaded_contact_caught) && contact_ok) ||
          posture_guarded_catch || guarded_four_arm_caught;
      ready_time=caught?ready_time+dt:0.;
      // A pressure/flex or stall catch means that the new point is carrying
      // load, not that useful horizontal pulling is already complete.  Keep
      // the bounded pull long enough to produce retained motion.  Only a
      // genuine return to the measured angle may use the short minimum.
      const bool contact_limited_catch=!angle_caught &&
          (stalled_caught || pressure_flex_caught || loaded_contact_caught ||
              guarded_four_arm_caught);
      // A measured outward-roll guard is a safety support transfer and must
      // not wait for the useful-traction dwell.  Ordinary pressure/stall
      // catches use the dwell so fast pressure prediction cannot end Motion 3
      // before horizontal impulse has been delivered.
      const double required_pull_seconds=posture_guarded_catch?
          k.pull_min_seconds:(contact_limited_catch?
          std::max(k.pull_min_seconds,k.pull_loaded_catch_min_seconds):
          k.pull_min_seconds);
      // A terminal rear lift is permitted only after BOTH front chambers are
      // physically at the full grasp pressure.  A predicted/commanded 30 kPa
      // is not sufficient.  The front pair then acts as the anchor while the
      // rear pair receives one short positive impulse.  Do not use this
      // optional progress event on a posture-guarded catch.
      // The terminal rear lift is owned by the outer regrip controller.  At
      // that boundary all four measured arm angles are available, so the
      // bidirectional rotor sign can be selected from the actual upward-force
      // component.  A scalar positive command here can point downward after
      // a rear arm flexes beyond 90 degrees.
      const bool terminal_front_anchor_ready=false;
      const bool terminal_front_anchor_wait_safe=false;
      if(ready_time>=k.pull_regrip_confirm &&
         phase_time>=required_pull_seconds) {
        // A normal catch may be recognized at a lower load-bearing pressure
        // than the full front anchor needed by the terminal rear lift.  Wait
        // while the measured pressure is still rising, but never wait merely
        // because a 30-kPa command exists: if neither chamber makes measurable
        // progress for the bounded stall interval, use the ordinary safe
        // regrip path.  This separates pneumatic delay from a disconnected or
        // saturated pressure sensor/valve.
        if(terminal_front_anchor_wait_safe &&
           !terminal_front_anchor_ready) {
          terminal_anchor_wait_elapsed+=dt;
          if(!terminal_front_anchor_wait_started) {
            terminal_front_anchor_wait_started=true;
            terminal_front_anchor_best_pressure=pair_pressure;
            terminal_front_anchor_stall_time=0.;
          } else if(pair_pressure>=terminal_front_anchor_best_pressure+
                    k.pull_terminal_front_pressure_progress_kpa) {
            terminal_front_anchor_best_pressure=pair_pressure;
            terminal_front_anchor_stall_time=0.;
          } else {
            terminal_front_anchor_stall_time+=dt;
          }
        }
        const bool terminal_front_anchor_pending=
            terminal_front_anchor_wait_safe &&
            !terminal_front_anchor_ready &&
            terminal_anchor_wait_elapsed<
                k.pull_terminal_anchor_wait_timeout &&
            (pair_pressure>=in.base_pressure-
                 k.pull_terminal_front_pressure_tolerance ||
             terminal_front_anchor_stall_time<
                 k.pull_terminal_front_pressure_stall_seconds);
        if(!terminal_rear_lift_started && !terminal_rear_lift_complete &&
           !terminal_rear_lift_cancelled && terminal_front_anchor_ready) {
          terminal_rear_lift_started=true;
          terminal_rear_lift_elapsed=0.;
          terminal_rear_lift_command=0.;
        }
        // Once started, complete the pulse and its falling edge before the
        // outer all-arm regrip takes ownership.  If it cannot be started
        // safely, retain the ordinary regrip path without waiting.
        // A normal catch can be recognized at the lower load-bearing pressure
        // used for support transfer.  When terminal lift is enabled, remain in
        // Motion 3 until the measured front pressure reaches the full-grasp
        // threshold instead of handing ownership to the outer regrip first.
        // A posture-guarded catch never waits for this optional event.
        if((!terminal_rear_lift_started &&
            !terminal_front_anchor_pending) ||
           terminal_rear_lift_complete) {
          out.regrip=true;
          out.contact_limited_regrip=posture_guarded_catch ||
              (!angle_caught && (stalled_caught || pressure_flex_caught ||
                  loaded_contact_caught || guarded_four_arm_caught));
        }
      }
      if(terminal_rear_lift_started && !terminal_rear_lift_complete) {
        // The front anchor is pneumatic during the terminal event.  Remove
        // the remaining front rotor force while its 30-kPa grasp is retained;
        // otherwise front and rear lift each other instead of recovering the
        // rear contact point.
        raw.fill(0.);
        fall_limit.fill(std::max(c.thrust_fall,
            k.pull_terminal_rear_lift_fall));
      }
      if(phase==Pull && phase_time>=k.pull_seconds) enter(FrontCatch,in);
      if(phase==FrontCatch) {
        // Stop spending thrust on an unproductive lift; leave the rear LOW
        // until front catch, or hand over to explicit fault regrasp at timeout.
        for(double& t:raw) t=std::min(t,.5);
        if(phase_time>=k.front_wait_seconds && !out.regrip &&
           (!terminal_rear_lift_started || terminal_rear_lift_complete))
          out.fault=true;
      }
    }
    // Keep the first unloading identification envelope through its whole
    // pressure-driven Pull/FrontCatch.  Re-enabling the ordinary pulse floor
    // as soon as the bend crossed it once made a later 1--2 degree settling
    // drift abort an otherwise successful regrip.  The next Lift sets this
    // flag from !calibrated, so ordinary strokes still regain the normal
    // Action-C floor.  Absolute bend/rate and body roll guards remain active.
    out.rear_low=rear_low;
    // Motion 2/3 keep the rear pair compliant.  A soft roll correction may
    // reshape rotor thrust or end the stroke, but must not recreate the old
    // high-pressure rear contact during the useful unloaded interval.  Full
    // rear grasp belongs exclusively to the outer regrip state after the
    // front catch has been accepted.
    if(phase==VentFront)
      out.rear_pressure=clamp(rear_pressure_command,
          k.rear_low_kpa,in.base_pressure);
    else if(phase==Lift || phase==Pull || phase==FrontCatch) {
      const double motion_pressure_limit=std::min(
          in.base_pressure,k.rear_motion_pressure_cap);
      rear_pressure_command=std::min(rear_pressure_command,
          motion_pressure_limit);
      out.rear_pressure=clamp(rear_pressure_command,
          k.rear_low_kpa,motion_pressure_limit);
    }
    else
      out.rear_pressure=rear_low?k.rear_low_kpa:
          (rear_preload?k.rear_preload_kpa:in.base_pressure);
    if(rear_angle_feedback_active) {
      const double pressure_limit=(phase==Lift || phase==Pull ||
          phase==FrontCatch)?
          std::min(in.base_pressure,k.rear_motion_pressure_cap):
          in.base_pressure;
      out.rear_pressure=clamp(
          out.rear_pressure+rear_angle_pressure_trim,
          k.rear_low_kpa,pressure_limit);
    }
    const bool has_identified_absolute_target=calibrated &&
        std::isfinite(identified_bend_target[0]) &&
        std::isfinite(identified_bend_target[1]);
    const bool identified_absolute_envelope=
        (identifying_straight || has_identified_absolute_target) &&
        (phase==Lift || phase==Pull || phase==FrontCatch);
    double proposed_up=0.;
    for(size_t j=0;j<2;++j) {
      raw[j]=clamp(raw[j],0.,c.thrust_max);
      const bool golden_soft_roll_allowed=phase==Pull &&
          golden_window_active &&
          std::abs(gait_roll+.3*s.roll_rate)<k.pull_golden_roll_limit;
      if(posture_brake && !golden_soft_roll_allowed &&
         (phase==Pull || phase==FrontCatch)) {
        // During pressure-driven flexion the front chambers are already
        // rebuilding support. Remove residual pull promptly when roll keeps
        // growing; intended pitch is handled by the independent hard guard.
        // Lift has a separate addition-only extension synchronizer below.
        // Reducing both free-extension rotors here drove the lagging rotor to
        // zero while the leading arm kept moving, so roll worsened until the
        // hard guard. Pull/FrontCatch already rebuild pneumatic support and
        // may safely remove their residual pull.
        raw[j]=std::min(raw[j],std::max(0.,s.thrust[j]-c.thrust_fall*dt));
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
          phase!=SlipProbe && phase!=Slide && phase!=SlipSettle &&
          ext>extension_limit-.035;
      const bool extension_guard=
          (phase==Lift || phase==Pull || phase==SlipProbe || phase==Slide ||
          phase==FrontCatch || phase==SlipSettle) &&
          (relative_extension_exceeded || anticipated<=bend_floor+.035);
      lift_extension_guard[j]=phase==Lift && extension_guard;
      if(extension_guard) {
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
      // Keep one unambiguous owner for Lift pressure.  It is zero during free
      // extension except for the pair-synchronous pilot scheduled above.
      // Per-arm pressure feedback starts only after entering Pull.
      if(phase==Lift) pressure_target[j]=lift_prefill_command[j];
      // Unequal arm angles are allowed. Individual extension/floor guards
      // remain; body roll feedback, not angle matching, owns differential thrust.
    }
    bool identification_sync_active=false;
    size_t identification_leading=0,identification_lagging=1;
    const double measured_extension_mismatch=std::abs(
        (reference[0]-s.bend[0])-(reference[1]-s.bend[1]));
    // Reused strokes normally allow unequal arm motion and let body-roll
    // feedback decide whether it matters.  A large pair divergence, however,
    // precedes measurable roll on the high-friction contact: the third
    // validation stroke reached 13--25 deg mismatch and crossed the hard roll
    // boundary within the next half second.  Start addition-only lagging-arm
    // assistance at twice the ordinary identification deadband (6 deg by
    // default), while still leaving smaller harmless asymmetry untouched.
    const bool reused_pair_diverging=!identifying_straight &&
        measured_extension_mismatch>2.*k.identification_sync_deadband;
    if(phase==Lift && !inner_roll_control &&
       (identifying_straight || roll_brake || reused_pair_diverging) &&
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
      // Reused Action-C strokes normally permit unequal extension.  When
      // measured roll is actually worsening, however, apply the same
      // addition-only synchronizer: hold the leading arm and raise only the
      // lagging arm.  This preserves the learned common waveform in stable
      // contact while preventing a one-arm reach from becoming body roll.
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
      const bool large_pair_divergence=
          mismatch>k.unload_pair_mismatch;
      const bool roll_requires_pair_sync=gait_roll_worsening ||
          roll_brake;
      if(mismatch>k.identification_sync_deadband &&
         (large_pair_divergence || roll_requires_pair_sync)) {
        const size_t leading=
            predicted_extension[0]>predicted_extension[1]?0:1;
        const size_t lagging=1-leading;
        // Angle matching is useful only when it does not push the body
        // farther in its current roll direction.  On a compliant contact the
        // lagging extension arm can be exactly the rotor whose roll moment is
        // adverse; blindly assisting it caused the high-friction validation
        // to fall after an otherwise successful 8.6-mm stroke.  In that case
        // retain the ordinary common search and let body-roll allocation own
        // the differential demand.
        const bool assist_worsens_roll=std::abs(s.roll)>.0174533 &&
            s.roll*verified_roll_moment[lagging]>0.;
        identification_sync_active=true;
        identification_leading=leading;
        identification_lagging=lagging;
        const double assist=std::min(k.identification_sync_assist_max,
            k.identification_sync_gain*
                (mismatch-k.identification_sync_deadband));
        // A leading arm which is still extending must be allowed to coast all
        // the way to zero thrust.  Merely freezing its measured force left a
        // 0.3--0.4 N command in MuJoCo; at zero chamber pressure that was
        // enough for the arm to gain another 10 deg while its partner caught
        // up.  If it has already stopped (or started flexing back), retain the
        // force instead so the pair does not enter a cut/rebound oscillation.
        // Pair safety depends on the total measured motion, regardless of
        // whether that motion is attributed to passive decompression or to
        // rotor force.  Subtracting the passive-rate estimate hid exactly the
        // continuing motion which must be stopped while the other arm waits.
        const double leading_extension_rate=std::max(0.,-s.rate[leading]);
        const bool leading_still_extending=
            leading_extension_rate>.5*M_PI/180. ||
            identification_extension_rate_filtered[leading]>.5*M_PI/180.;
        raw[identification_leading]=std::min(raw[identification_leading],
            leading_still_extending?0.:s.thrust[identification_leading]);
        // Once both arms are in first-pass rate control, synchronization may
        // accelerate only an actually slow lagging arm. Otherwise it would
        // overwrite the speed controller and recreate the overspeed which the
        // controller just removed. Before pair tracking, retain the original
        // catch-up assist so the second breakaway can still be acquired.
        const bool lagging_rate_allows_assist=
            !reused_event_checkpoint_reached[lagging] &&
            (!identifying_straight ||
            !(identification_tracking[0] && identification_tracking[1]) ||
            (identification_extension_rate_filtered[lagging]<
                 k.extension_rate-k.identification_rate_deadband &&
             std::max(0.,-s.rate[lagging]-passive_extension_rate[lagging])<=
                 k.extension_rate+k.identification_rate_deadband));
        if(!assist_worsens_roll && lagging_rate_allows_assist) {
          raw[lagging]=std::min(c.thrust_max,raw[lagging]+assist);
          rise_limit[lagging]=std::max(rise_limit[lagging],
              k.identification_tracking_rise);
        }
      }
    }
    // Start every load-relief test with one common short pulse.  Once
    // the measured extension of the pair separates, however, retain the
    // addition-only synchronizer above: freeze the leading arm and keep only
    // the lagging arm on the same gentle search slew.  The previous max/fill
    // here erased that correction on every cycle.  Equal thrust was then
    // applied to unequal contacts and the first unloading event was reached
    // with about 10 versus 28 deg of extension.  A lower common command is
    // used only while no synchronization is active, so an overspeed hold from
    // either arm cannot be overwritten by its partner's larger demand.
    const double initial_pulse_level=std::min(k.initial_pulse_max,
        k.initial_pulse_thrust+k.initial_pulse_step*std::floor(
            powered_time/std::max(.1,k.initial_pulse_step_seconds)));
    if(phase==Lift && !unload_event_detected) {
      const bool powered_pair_onset_ready=onset_this_stroke[0] &&
          onset_this_stroke[1] && powered_extension_reference_valid[0] &&
          powered_extension_reference_valid[1];
      if(!identification_sync_active) {
        const double common_search=std::min(raw[0],raw[1]);
        raw.fill(common_search);
      }
      // Before the paired powered onsets, there is no valid relative-angle
      // reference for a differential command. Use the same rapidly rising,
      // bounded target on both rotors on every stroke.
      rise_limit.fill(k.initial_pulse_rise);
      if(!powered_pair_onset_ready) {
        // Fire one simultaneous, bounded extension impulse.  The previous
        // 0.7-N/s identification sweep allowed one released arm to move for
        // about a second before the other reached breakaway; by then the
        // command had already become a differential roll input.  Before both
        // onset references exist there is nothing meaningful to synchronize,
        // so keep exactly the same target and rise rate on both rotors.
        raw.fill(std::min(c.thrust_max,initial_pulse_level));
        rise_limit.fill(k.initial_pulse_rise);
        identification_sync_active=false;
      }
    }
    std::array<bool,2> identification_needs_extension{{false,false}};
    if(phase==Lift && !unload_event_detected) {
      const bool powered_pair_tracking=onset_this_stroke[0] &&
          onset_this_stroke[1] && powered_extension_reference_valid[0] &&
          powered_extension_reference_valid[1];
      const Pair measured_extension{{
          std::max(0.,(powered_pair_tracking?
              powered_extension_reference[0]:reference[0])-s.bend[0]),
          std::max(0.,(powered_pair_tracking?
              powered_extension_reference[1]:reference[1])-s.bend[1])}};
      // The instantaneous common target comes from the minimum-jerk bend
      // trajectory advanced above.  Keep the legacy scalar only as a bag/log
      // diagnostic; it no longer owns a second, linear Motion-2 reference.
      if(powered_pair_tracking && extension_trajectory.active)
        identification_pair_target_extension=std::min(
            powered_extension_reference[0]-extension_trajectory.target[0],
            powered_extension_reference[1]-extension_trajectory.target[1]);
      else
        identification_pair_target_extension=std::max(
            identification_pair_target_extension,
            std::min(measured_extension[0],measured_extension[1]));
      for(size_t j=0;j<2;++j) {
        const double desired_extension=powered_pair_tracking &&
            extension_trajectory.active?
            powered_extension_reference[j]-extension_trajectory.target[j]:
            identification_pair_target_extension;
        const double error=measured_extension[j]-desired_extension;
        // At or behind the common trajectory, the arm is still responsible
        // for advancing that trajectory.  Protect its current force as well
        // as the clearly lagging case.  Waiting until it fell a full
        // half-deadband behind allowed the roll allocator to remove 1.7 N in
        // roughly 0.15 s; pneumatic delay then made the opposite arm run away
        // while this side rebounded.  An arm ahead of the corridor is the
        // only side whose rotor may be reduced here.
        identification_needs_extension[j]=
            error<=.5*k.identification_sync_deadband;
      }
      // Before load relief Motion 2 is a true zero-pressure extension pulse.
      // Once deep windup has latched load relief, however, the model-based
      // common pressure calculated above must remain authoritative: it holds
      // the extended moment arm while the body pitches.  Clearing it here was
      // a late command-stack overwrite; diagnostics showed 29 kPa requested
      // but 0 kPa sent, so every trial still ran to the bend/rate guard.
      const bool deep_windup_hold=
          k.trajectory_windup_pitch>k.unload_pitch+1.e-6 &&
          load_relief_detected && !unload_event_detected;
      if(!deep_windup_hold) {
        lift_prefill_command.fill(0.);
        pressure_target.fill(0.);
      }
    }
    // A small, stationary roll bias is part of the contact condition, not a
    // reason to fight the extension search. Only a measured outward trend
    // activates the inner roll allocator. Use the hardware-verified pair sign
    // rather than the contact-lever sign, which can reverse near a 104-deg
    // bend.
    const double requested_roll_moment=
        raw[0]*verified_roll_moment[0]+raw[1]*verified_roll_moment[1];
    const bool differential_worsens_roll=std::abs(s.roll)>.0174533 &&
        s.roll*requested_roll_moment>0.;
    // During the first Motion-2 identification, a static roll bias and an
    // estimated unequal lever are not sufficient evidence for differential
    // actuation.  The user-visible requirement is a common gentle search; a
    // correction is introduced only after measured roll is actually growing.
    // Otherwise the allocator continually trims one common target and the
    // compliant pair reaches the unloading event at very different angles.
    const bool lift_identification_roll_control=
        phase==Lift && identifying_straight;
    if((phase==SlipProbe || phase==Slide) &&
       (inner_roll_control || differential_worsens_roll)) {
      // Keep the diagnostic input pair-symmetric.  The old circular-fixture
      // roll allocator could drive one front rotor to zero while leaving the
      // other near the probe force; on the tapered forearm that differential
      // was itself the dominant roll excitation.  Slip is accepted or
      // rejected from retained x, while the outer absolute posture guard
      // remains available.  A common reduction preserves the force direction
      // without creating an artificial one-arm gait.
      raw.fill(std::min(raw[0],raw[1]));
    } else if(inner_roll_control ||
       (differential_worsens_roll && !lift_identification_roll_control)) {
      // Once the requested differential itself points farther into the
      // measured roll, angle synchronization no longer owns either actuator.
      // This also catches a slowly growing contact bias before angular-rate
      // confirmation becomes reliable.
      if(differential_worsens_roll) identification_sync_active=false;
      if(phase==Lift) {
        if(identifying_straight) {
          // The first measured reach must not lose the force that is carrying
          // an arm along the common extension trajectory.  A reduction-only
          // allocator removed both fronts in 0.15 s when roll crossed zero;
          // the pneumatically released leader then continued extending while
          // its partner rebounded.  Use the existing addition-only allocator
          // here: it raises only the restoring side, first to the current pair
          // maximum and then by a bounded residual.  The common gentle slew,
          // pneumatic leader brake, total-up-force cap and hard posture guards
          // remain authoritative.
          raw=addRestoringRollSupport(raw,verified_roll_moment,
              s.roll,s.roll_rate,c.thrust_max);
          rise_limit[0]=rise_limit[1]=std::max(
              k.lift_rise,std::min(k.identification_tracking_rise,1.5));
        } else {
          // Reused reaches already have a measured extension feed-forward.
          // Preserve their reduction-only fast brake.
          raw=dampLiftRollReductionOnly(raw,verified_roll_moment,
              s.roll,s.roll_rate);
        }
      } else if(phase==Pull || phase==FrontCatch) {
        const Pair before_roll_allocation=raw;
        raw=dampLocomotionRoll(raw,verified_roll_moment,
            s.roll,s.roll_rate,c.thrust_max);
        // Equal raw targets do not remove an already existing actuator
        // difference when both sides retain the ordinary 1-N/s rise limit:
        // they simply rise in parallel and keep producing the same roll
        // moment.  Give only the rotor increased by the roll allocator the
        // measured identification catch-up slew.  Reductions already use the
        // faster normal fall limit.  This makes the commanded balance reach
        // the actuators before a 2--3 deg gait bias grows into a hard recovery.
        for(size_t j=0;j<2;++j)
          if(raw[j]>before_roll_allocation[j]+1.e-6)
            rise_limit[j]=std::max(rise_limit[j],
                k.identification_tracking_rise);
      } else
        raw=reduceAdverseRoll(raw,verified_roll_moment,gait_roll,s.roll_rate);
    }
    if(identification_sync_active) {
      const double leading_extension_rate=std::max(
          0.,-s.rate[identification_leading]);
      const bool leading_still_extending=
          leading_extension_rate>.5*M_PI/180. ||
          identification_extension_rate_filtered[identification_leading]>
              .5*M_PI/180.;
      raw[identification_leading]=std::min(raw[identification_leading],
          leading_still_extending?0.:
              s.thrust[identification_leading]);
    }
    if(identification_sync_active) {
      // A speed hold or roll allocator must not undo the catch-up request by
      // ramping the lagging arm down.  Keeping its present measured thrust is
      // the minimum; any increase is still limited by the ordinary gentle
      // first-pass slew above.  Hard bend/rate/roll guards remain outside this
      // synchronizer and can still terminate the reach.
      raw[identification_lagging]=std::max(
          raw[identification_lagging],s.thrust[identification_lagging]);
    }
    if(phase==Lift && !unload_event_detected) {
      // Do not let the fast roll allocator or a stale filtered-rate hold
      // unload an arm which pneumatic delay has moved below the monotonic
      // common target.  Its increase remains limited to lift_rise.  The
      // independent hard roll/rate/bend guards can still end the reach.
      for(size_t j=0;j<2;++j)
        if(identification_needs_extension[j])
          raw[j]=std::max(raw[j],s.thrust[j]);
    }
    if(phase==Lift && !unload_event_detected &&
       onset_this_stroke[0] &&
       onset_this_stroke[1] && powered_extension_reference_valid[0] &&
       powered_extension_reference_valid[1]) {
      // Track the same monotonically advancing relative-angle trajectory from
      // each arm's own powered-onset pose.  The measured onset thrust is the
      // contact-conditioned feed-forward term; angle and rate feedback supply
      // only the difference needed to keep the two physical responses on the
      // same path.  The small onset threshold is only breakaway evidence; the
      // explicit minimum trajectory travel and measured body/contact load
      // relief are both required before Pull starts pressure-driven flexion.
      const Pair predicted_extension_target=
          extension_trajectory.predictedTarget(
              k.trajectory_prediction_horizon);
      const Pair predicted_extension_rate=
          extension_trajectory.predictedRate(
              k.trajectory_prediction_horizon);
      const bool holding_windup=
          k.trajectory_windup_pitch>k.unload_pitch+1.e-6 &&
          load_relief_detected && !unload_event_detected;
      for(size_t j=0;j<2;++j) {
        const double target_bend=holding_windup?
            windup_reference_bend[j]:(extension_trajectory.active?
                predicted_extension_target[j]:s.bend[j]);
        const double target_rate=holding_windup?
            ((windup_target_reached_by_arm[j] || windup_pose_captured)?0.:
                -k.trajectory_extension_rate):
            (extension_trajectory.active?predicted_extension_rate[j]:0.);
        double predicted_bend=trajectory_observer[j].predictAngle(
            s.bend[j],k.trajectory_prediction_horizon,
            k.trajectory_prediction_max_delta);
        double predicted_rate=trajectory_observer[j].predictRate(
            k.trajectory_prediction_horizon,
            k.identification_bend_rate_limit);
        // A genuine sharp breakaway must brake in this controller cycle, not
        // after the low-pass observer catches up.  Retain the established
        // hardware rule: only above the 45-deg bend boundary may the raw gyro
        // make the prediction more conservative. It can reduce/freeze thrust,
        // but the separately time-qualified angle/body cues still own phase
        // completion, so a one-sample spike cannot enter Motion 3.
        if(s.bend[j]>=k.identification_rate_feedback_min_bend) {
          predicted_rate=std::min(predicted_rate,s.rate[j]);
          predicted_bend=std::min(predicted_bend,s.bend[j]+clamp(
              s.rate[j]*k.trajectory_prediction_horizon,
              -k.trajectory_prediction_max_delta,
              k.trajectory_prediction_max_delta));
        }
        // Positive bend error means the arm is still too flexed and requires
        // more extension force. Compare the delayed future state with the
        // future path, so a fast arm is braked before its angle has overshot.
        // The acceleration term is observer-filtered and bounded; it can move
        // timing but can never declare an unloading event by itself.
        const double bend_error=predicted_bend-target_bend;
        const double rate_error=predicted_rate-target_rate;
        // Retain the simultaneous pulse level after both onsets. Per-arm
        // feedback may raise or lower it only according to each arm's own
        // relative travel; it must not collapse the early arm back to its
        // smaller measured breakaway force while the other is being launched.
        double onset_ff=std::max(initial_pulse_level,
            onset[j].samples>0?onset[j].thrust:std::max(.3,s.thrust[j]));
        if(holding_windup &&
           !unload_event_detected && std::isfinite(load_relief_elapsed)) {
          // Once actual load relief has begun, use the remaining safe pitch
          // corridor instead of the old 4-N pulse ceiling.  This is still a
          // slew-limited, per-arm angle/rate-controlled reach; it merely lets
          // the measured body rotate toward the windup target when 4 N is not
          // enough on a high-friction compliant contact.
          // Pitch and arm-bend control act in orthogonal command channels:
          // this common force closes body pitch using Spinal pitch/rate,
          // while the per-arm angle/rate terms below only shape the two-arm
          // windup trajectory.  Previously thrust merely ramped upward and
          // never used the large configured pitch error.
          const double pitch_error=std::max(0.,
              windup_pitch_target-windup_pitch_progress);
          const double desired_pitch_rate=std::min(
              k.trajectory_windup_pitch_rate_max,pitch_error);
          const double measured_pitch_rate=
              windup_pitch_direction*in.pitch_rate;
          const double common_load_relief=.5*(load_relief_thrust[0]+
              load_relief_thrust[1]);
          // Preserve pitch feedback only while there is useful roll margin.
          // This is a continuous authority reduction, not a posture switch:
          // at 4 deg the full pitch request is available, and by 6 deg only
          // the force already proven at load relief remains. The measured
          // peak detector above then hands immediately to flexion.
          const double pitch_authority=clamp(
              (6.*M_PI/180.-std::abs(gait_roll))/(2.*M_PI/180.),0.,1.);
          const double pitch_force_target=clamp(common_load_relief+
              pitch_authority*(k.trajectory_windup_pitch_kp*pitch_error+
                  k.trajectory_windup_pitch_kd*
                      (desired_pitch_rate-measured_pitch_rate)),
              0.,std::min(c.thrust_max,
                  k.trajectory_windup_thrust_max));
          const double windup_force=pitch_force_target>=s.thrust[j]?
              std::min(pitch_force_target,s.thrust[j]+
                  k.trajectory_windup_thrust_rise*dt):
              std::max(pitch_force_target,s.thrust[j]-
                  k.identification_rate_control_fall*dt);
          onset_ff=std::max(onset_ff,windup_force);
          rise_limit[j]=k.trajectory_windup_thrust_rise;
        }
        if(holding_windup) {
          // While chambers are still open, follow the planned deep reach with
          // measured bend rate and predicted angular acceleration.  A fixed
          // 6-N target drove the arm at 150--300 deg/s, so even an immediate
          // valve command crossed the safe bend before pressure arrived.
          // After the predicted catch boundary, freeze the current rotor
          // level for the few samples before Pull takes ownership; pressure
          // then flexes the arm instead of fighting another thrust increase.
          raw[j]=windup_pose_captured?s.thrust[j]:clamp(
              onset_ff+extension_thrust_trim[j]+
              k.trajectory_extension_kp*bend_error+
              k.trajectory_extension_kd*rate_error,
              0.,std::min(c.thrust_max,k.trajectory_windup_thrust_max));
          rise_limit[j]=k.trajectory_windup_thrust_rise;
        } else {
          raw[j]=clamp(onset_ff+extension_thrust_trim[j]+
              k.trajectory_extension_kp*bend_error+
              k.trajectory_extension_kd*rate_error,
              0.,c.thrust_max);
          rise_limit[j]=std::max(rise_limit[j],k.initial_pulse_rise);
        }
        fall_limit[j]=std::max(fall_limit[j],
            k.identification_rate_control_fall);
      }
      identification_sync_active=false;
    }
    if(phase==Lift && unload_event_detected &&
       !unload_capture_started) {
      // Freeze the alignment objective at the measured unloading event.
      // Pressure supplies the slow flexion/return torque above, while this
      // bounded bend/rate controller prevents the shallow arm from rebounding
      // and the deep arm from being launched through the corridor.  A moving
      // leader/lagger target caused exactly that role reversal in simulation.
      for(size_t j=0;j<2;++j) {
        const double target_bend=
            reference[j]-unload_alignment_extension_target;
        raw[j]=clamp(unload_event_thrust[j]+
            k.identification_angle_kp*(s.bend[j]-target_bend)+
            k.identification_rate_kd*s.rate[j],0.,
            std::min(c.thrust_max,
                unload_event_thrust[j]+k.identification_sync_assist_max));
        rise_limit[j]=std::max(rise_limit[j],
            k.identification_tracking_rise);
        fall_limit[j]=std::max(fall_limit[j],
            k.identification_rate_control_fall);
      }
      // The fixed bend objective owns pair synchronization, but it does not
      // disable the measured body-roll brake. It remains reduction-only in
      // Motion 2, so no extra extension authority is created here.
      raw=addRestoringRollSupport(raw,verified_roll_moment,
          s.roll,s.roll_rate,c.thrust_max);
      identification_sync_active=false;
    }
    if(phase==Lift && unload_capture_started) {
      // Pair alignment without rate capture is unsafe: run 191 crossed the
      // angle corridor with one front arm still extending at 100 deg/s.  A
      // simple thrust rundown then made that arm rebound at 240 deg/s while
      // its partner continued extending.  Hold each measured capture bend
      // with a bounded angle/rate damper while the rear pair still anchors
      // the body.  Positive bend/rate means pneumatic flexion and therefore
      // requests more opposing (extension) thrust; continued extension asks
      // for less.  Pull takes over continuously once both rates settle.
      for(size_t j=0;j<2;++j) {
        raw[j]=clamp(unload_capture_thrust[j]+
            k.identification_angle_kp*(s.bend[j]-unload_capture_bend[j])+
            k.identification_rate_kd*s.rate[j],0.,
            std::min(c.thrust_max,unload_capture_thrust[j]+1.));
        rise_limit[j]=std::max(rise_limit[j],
            k.identification_tracking_rise);
        fall_limit[j]=std::max(fall_limit[j],
            k.pull_extension_brake_fall);
      }
      identification_sync_active=false;
    }
    // The generic roll allocator runs after the Pull planner and may
    // otherwise re-introduce a differential command during this short
    // pneumatic catch.  At this boundary neither arm has established the new
    // loaded contact, so its instantaneous lever sign is not trustworthy.
    // Preserve the reduction chosen by the safer side and keep the pair
    // common until both measured bends reverse; the ordinary roll allocator
    // resumes immediately afterwards.
    if(pneumatic_pair_catch_active) {
      const double common_catch_thrust=std::min(raw[0],raw[1]);
      raw.fill(common_catch_thrust);
      fall_limit.fill(std::max(k.pull_extension_brake_fall,
          c.thrust_fall));
    }
    if(phase==Lift &&
       (lift_extension_guard[0] || lift_extension_guard[1])) {
      // The pulse generator is deliberately late in the command stack, but
      // it must never overwrite an absolute bend/extension safety guard.
      // Stop the pair together so the guarded side is not replaced by a
      // one-arm impulse while the other rotor continues to rise.
      double guarded_pair_limit=std::numeric_limits<double>::infinity();
      for(size_t j=0;j<2;++j)
        if(lift_extension_guard[j])
          guarded_pair_limit=std::min(guarded_pair_limit,
              std::max(0.,s.thrust[j]-c.thrust_fall*dt));
      for(size_t j=0;j<2;++j) {
        raw[j]=std::min(raw[j],guarded_pair_limit);
        fall_limit[j]=std::max(fall_limit[j],c.thrust_fall);
      }
    }
    // lift_ramp already contains each arm's measured target/rate braking
    // before the roll allocator runs.  Do not clamp it a second time here:
    // that old post-allocation clamp erased the restoring side whenever it
    // happened to be the first arm at its bend target, leaving the opposite
    // rotor to roll the body out of contact.  The allocator may add only the
    // bounded force required to cancel that differential; the absolute bend,
    // rate and total-up-force guards below remain authoritative.
    for(size_t j=0;j<2;++j) {
      out.thrust[j]=slew(s.thrust[j],raw[j],rise_limit[j],fall_limit[j],dt);
      const double pressure_rise=(identifying_straight &&
          (phase==Pull || phase==FrontCatch))?
          k.identification_return_pressure_slew:
          ((phase==Lift && load_relief_detected &&
            windup_brake_started_by_arm[j] && !unload_event_detected)?
              std::max(c.pressure_slew,k.trajectory_windup_pressure_slew):
              c.pressure_slew);
      out.pressure[j]=slew(s.command[j],pressure_target[j],pressure_rise,c.pressure_slew,dt);
      proposed_up+=out.thrust[j]*uz[j];
    }
    if(pneumatic_pair_catch_active) {
      // A common raw target with identical fall slopes preserves any thrust
      // difference inherited from the pulse for several control samples.
      // End the impulse as one pair command instead: the lower already-safe
      // measured output becomes the common catch force.  This is the intended
      // pulse OFF edge, while pressure is simultaneously rising to flex both
      // arms; subsequent traction still uses the normal slew limits.
      const double common_catch_output=std::min(
          out.thrust[0],out.thrust[1]);
      out.thrust.fill(common_catch_output);
      proposed_up=common_catch_output*(uz[0]+uz[1]);
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
    // Pressure relief alone did not physically open the rear pair on the
    // compliant arm: both chambers reached 6--12 kPa while the measured rear
    // bends moved farther into flexion.  Use the bounded common front-pair
    // level and add a small positive rear command only after a sustained
    // measured 3--5-deg release deficit.  The command disappears as soon as
    // the limiting rear arm enters the target corridor.
    const double front_pair_min=std::min(out.thrust[0],out.thrust[1]);
    const double front_pair_level=.5*(out.thrust[0]+out.thrust[1]);
    const double rear_front_phase=clamp(
        (front_pair_level-k.pull_rear_positive_front_start)/
            (k.pull_rear_positive_front_full-
                k.pull_rear_positive_front_start),0.,1.);
    double rear_front_envelope=minimumJerkPosition(rear_front_phase);
    const double rear_pressure_off=k.pull_rear_positive_max_pressure+
        k.pull_rear_positive_pressure_fade;
    const double rear_pressure_envelope=clamp(
        (rear_pressure_off-in.rear_pressure)/
            k.pull_rear_positive_pressure_fade,0.,1.);
    rear_positive_front_envelope=rear_front_envelope;
    rear_positive_pressure_envelope=rear_pressure_envelope;
    const bool rear_positive_phase=
        phase==Lift || phase==Pull || phase==FrontCatch;
    const bool rear_positive_posture_safe=!posture_brake &&
        std::abs(predicted_roll)<k.pull_golden_roll_limit;
    const bool rear_positive_start_request=rear_positive_phase &&
        rear_release_deficit_time+1.e-9>=
            k.rear_release_assist_confirm;
    if(!rear_positive_started && rear_positive_start_request) {
      rear_positive_started=true;
      rear_positive_elapsed=0.;
    }
    const bool rear_positive_window=rear_positive_started &&
        rear_positive_phase &&
        rear_positive_elapsed<k.pull_rear_positive_duration;
    const bool rear_positive_hard_safe=rear_positive_posture_safe &&
        front_pair_min>.25;
    if((phase==Pull || phase==FrontCatch) && rear_positive_window)
      rear_front_envelope=std::max(rear_front_envelope,
          k.pull_residual_thrust_ratio);
    rear_positive_front_envelope=rear_front_envelope;
    const double rear_release_error=std::max(0.,
        rear_release_target-rear_release_pair-
            k.rear_release_assist_deadband);
    const double rear_release_deficit_envelope=clamp(
        rear_release_error/std::max(k.rear_release_assist_deadband,
            rear_release_target-k.rear_release_assist_deadband),0.,1.);
    const double rear_assist_envelope=rear_front_envelope*
        rear_pressure_envelope*rear_release_deficit_envelope*
        rear_release_balance_envelope;
    const double rear_positive_target=
        rear_positive_window && rear_positive_hard_safe?
        clamp((k.pull_rear_positive_thrust+
               (rear_angle_feedback_active?
                    std::max(0.,rear_angle_thrust_trim):0.))*
                  rear_assist_envelope,
            0.,k.pull_rear_positive_thrust+
                k.rear_release_thrust_trim_max):0.;
    // Roll/support-transfer gates are hard safety stops. Pressure and normal
    // event completion are smooth envelopes, avoiding the previous 12-kPa
    // on/off edge. Only genuinely delivered positive force consumes the
    // finite budget, so a delayed rear exhaust cannot expire it unseen.
    rear_positive_thrust_command=rear_positive_hard_safe?
        slew(rear_positive_thrust_command,rear_positive_target,
            k.pull_rear_positive_rise,k.pull_rear_positive_fall,dt):0.;
    if(rear_positive_target>.05 && rear_positive_thrust_command>.05)
      rear_positive_elapsed=std::min(k.pull_rear_positive_duration,
          rear_positive_elapsed+dt);
    // Once the front pair is a measured full-pressure anchor, give the rear
    // pair one short positive lift before the all-arm regrip.  Keep the rear
    // chambers at the low-pressure command for the duration; refilling them
    // here would mechanically cancel the requested lift.  Loss of front
    // pressure/contact or an adverse roll trend cancels the event, after
    // which its falling edge is still completed before regrip.
    const bool terminal_rear_lift_active=terminal_rear_lift_started &&
        !terminal_rear_lift_complete;
    const double terminal_front_pressure=std::min(
        s.pressure[0],s.pressure[1]);
    const bool terminal_front_contact=!in.contact_available ||
        in.front_contact;
    const bool terminal_rear_lift_safe=
        (phase==Pull || phase==FrontCatch) &&
        terminal_front_pressure>=in.base_pressure-
            k.pull_terminal_front_pressure_tolerance &&
        in.rear_pressure<=k.rear_motion_pressure_cap+
            k.pull_terminal_rear_pressure_tolerance &&
        terminal_front_contact && !posture_brake &&
        std::abs(gait_roll+.2*s.roll_rate)<k.catch_roll_angle &&
        std::abs(s.roll_rate)<=2.*k.catch_roll_rate;
    if(terminal_rear_lift_active && !terminal_rear_lift_safe)
      terminal_rear_lift_cancelled=true;
    const bool terminal_rear_lift_drive=terminal_rear_lift_active &&
        !terminal_rear_lift_cancelled &&
        terminal_rear_lift_elapsed<k.pull_terminal_rear_lift_duration;
    const double terminal_rear_lift_target=terminal_rear_lift_drive?
        k.pull_terminal_rear_lift_thrust:0.;
    terminal_rear_lift_command=slew(terminal_rear_lift_command,
        terminal_rear_lift_target,k.pull_terminal_rear_lift_rise,
        k.pull_terminal_rear_lift_fall,dt);
    if(terminal_rear_lift_drive && terminal_rear_lift_command>.05)
      terminal_rear_lift_elapsed=std::min(
          k.pull_terminal_rear_lift_duration,
          terminal_rear_lift_elapsed+dt);
    if(terminal_rear_lift_active && !terminal_rear_lift_drive &&
       terminal_rear_lift_command<=.05) {
      terminal_rear_lift_command=0.;
      terminal_rear_lift_complete=true;
    }
    if(terminal_rear_lift_active)
      out.rear_pressure=k.rear_low_kpa;
    out.rear_positive_thrust=std::max(rear_positive_thrust_command,
        terminal_rear_lift_command);
    if(out.rear_positive_thrust>1.e-6)
      out.rear_reverse_thrust=0.;
    if(rear_candidate_requested && rear_candidate_distinct &&
       (phase==Lift || phase==Pull || phase==FrontCatch)) {
      const double candidate_pressure=pressureFromSupport(
          rear_support_nominal,k);
      const double pressure_tolerance=std::max(2.,k.catch_pressure_tolerance);
      const bool candidate_not_overridden=
          rear_support_safety_override<.10 &&
          std::abs(rear_support_command-rear_support_nominal)<.10;
      const bool candidate_commanded=
          std::abs(out.rear_pressure-candidate_pressure)<=pressure_tolerance;
      const bool candidate_measured=
          std::abs(in.rear_pressure-candidate_pressure)<=pressure_tolerance;
      const bool candidate_thrust_delivered=
          trajectory.selected().rear_support_offset<0.?
              rear_positive_thrust_command>.05:
              out.rear_reverse_thrust>.05;
      const double candidate_release_target=std::max(
          0.,-rear_support_nominal)*k.rear_release_angle_max;
      const bool candidate_angle_realized=
          trajectory.selected().rear_support_offset>=0. ||
          (rear_grasp_reference_valid && in.rear_kinematics_valid &&
           rear_release_pair>=k.rear_release_realization_ratio*
               candidate_release_target &&
           rear_release_mismatch<=k.rear_release_mismatch_hard);
      const bool realized_sample=candidate_not_overridden &&
          candidate_angle_realized &&
          ((candidate_commanded && candidate_measured) ||
           candidate_thrust_delivered);
      rear_candidate_realization_time=realized_sample?
          std::min(.10,rear_candidate_realization_time+dt):
          std::max(0.,rear_candidate_realization_time-2.*dt);
      if(rear_candidate_realization_time+1.e-9>=.10)
        rear_candidate_realized=true;
    }
    // Regrasp has been confirmed at the front pair, but the rear pair is
    // still vented.  Preserve the final bounded pull through this handoff so
    // the body cannot slide back before rear pressure becomes load bearing.
    // The outer regrip phase fades it only after measured rear pressure has
    // recovered; hard roll/rate guards remain active during that transfer.
    double outx=0.,outz=0.;for(size_t j=0;j<2;++j) {outx+=out.thrust[j]*std::max(0.,ux[j]);outz+=out.thrust[j]*uz[j];}
    out.ratio=(outx+outz)>1e-6?outx/(outx+outz):1.;
    // The output owns this phase change so the valve command is synchronized
    // with the same measured load-relief event as the thrust trajectory.
    // Motion 3 immediately restores the nominal inflated support.
    if(phase==Lift && load_relief_detected && !unload_event_detected)
      out.bottom=k.trajectory_windup_bottom_kpa;
    return out;
  }
};
}} // namespace
