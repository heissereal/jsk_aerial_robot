#!/usr/bin/env python3
"""Offline tests of the production geometry-feedforward / roll-rate PI law."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
output_start = source.index('void AttitudePressureController::publishCorrectionThrust(')
output_end = source.index('double AttitudePressureController::pwmModelForce(',
                          output_start)
output_safety = source[output_start:output_end]
assert 'std::isfinite(value)' in output_safety
assert 'correction_output_fault_latched_' in output_safety
assert 'publishNeutralPwmTest()' in output_safety
assert 'publishPwmTest(pwm_test_safe_thrust_)' in output_safety
assert 'pwm_test_hard_forward_thrust_n_' in output_safety
assert 'pwm_test_output_rise_n_s_' in output_safety
assert 'publishPwmTest(thrust);' not in output_safety
assert 'const bool c_preload_roll = continuous_c_recovery &&' in source
assert 'action_c_preload_recovery_trigger_rad_' in source
start = source.index('case PREPARE:')
branch = source[start:source.index('publishCorrectionThrust(fixed_arm_test_enabled_', start)]
assert 'continuous_roll_recovery_.update(' in branch
assert 'enterState(THRUST_CORRECTION' not in branch
assert 'active_pair_pressure_ready' not in branch
assert 'prepare_arm_active_[arm] ? thrust : 0.0' in branch
assert 'publishBottomTarget(0.0)' in branch
assert 'continuous_recovery_blocked_ = true' in branch
assert 'measuredRecoveryModel()' in branch
assert 'crossed_recovery_target' not in branch
assert 'roll_recovery_response_[correction_axis_[0] > 0.0 ? 0 : 1]' in branch
assert '(now - state_start_).toSec() >= prepare_timeout_' in branch
assert 'state_start_ =' not in branch # progress must not restart the overall deadline
handoff_start = branch.index('const double pressure_brake_horizon')
pressure_handoff = branch[handoff_start:
                          branch.index('// The pair sign is fixed by the verified',
                                       handoff_start)]
assert 'pneumatic_capture_imminent' in pressure_handoff
assert 'recovery_predictive_config_.pressure_tau' in pressure_handoff
assert 'continuous_roll_recovery_.progress_observed ||' in pressure_handoff
assert 'continuous_roll_recovery_.speed_braking ||' in pressure_handoff
assert 'continuous_roll_recovery_.predicted_braking' in pressure_handoff
assert '0.25 * roll_recovery_config_.limit' in pressure_handoff
assert 'thrust <= pressure_brake_thrust_gate' in pressure_handoff
assert 'recovery_pressure_brake_started_ = true' in pressure_handoff
assert 'std::abs(signed_error) <= correction_slowdown_angle_' not in pressure_handoff

code = r'''
#include <hugmy/control/continuous_roll_recovery.h>
#include <cassert>
#include <cstdio>
using namespace hugmy;
const double deg=M_PI/180.;
int main() {
  RollRecoveryConfig cfg; RollRecoveryModel m; RollRecoveryResponse response;
  RollRecoveryConfig no_pair_retry=cfg;no_pair_retry.pair_retry_sec=20.;
  m.valid=true; m.moment=.2; m.lever=.1;
  ContinuousRollRecovery c;c.reset(25*deg,0.);
  // Braking flags can assert far from level and must never initiate pneumatic
  // regrasp. Terminal angle capture may start a minimum-pressure pneumatic
  // brake only after correction thrust has run down.
  assert(!c.pressureHandoffReady());
  c.speed_braking=c.predicted_braking=true;assert(!c.pressureHandoffReady());
  c.speed_braking=c.predicted_braking=false;
  c.capture_active=true;c.thrust=.2;assert(!c.pressureHandoffReady());
  c.thrust=.05;assert(c.pressureHandoffReady());
  c.capture_active=false;c.thrust=0.;assert(!c.pressureHandoffReady());
  c.finished=true;assert(c.pressureHandoffReady());c.finished=false;
  c.timed_out=true;assert(c.pressureHandoffReady());c.timed_out=false;
  c.fault=true;assert(c.pressureHandoffReady());c.fault=false;
  double last=0;
  for(int i=1;i<=100;++i) {
    double v=c.update(25*deg,0.,i*.02,.02,m,cfg,response);
    assert(v>=last && v-last<=.030001 && v<=4.);last=v;
  }
  assert(c.model_valid && std::abs(c.feedforward-2.)<1e-8);
  assert(c.thrust>2.4 && c.thrust<3.0); // model FF + rate FB + bounded plateau authority
  assert(c.boost>0. && c.boost<.6); // no old fixed search/boost in parallel
  // True motion, then a faster-than-requested roll: reduce, not hold breakaway.
  for(int i=101;i<=125;++i)c.update((25-(i-100)*.08)*deg,.12,i*.02,.02,m,cfg,response);
  assert(c.moving && c.hold>0.);
  last=c.thrust;
  for(int i=126;i<=145;++i)c.update(22*deg,.3,i*.02,.02,m,cfg,response);
  assert(c.thrust<last && !c.fault);
  // Plateau: signed rate error integrates at no more than 0.3 N/s.
  // Strong overspeed braking can remove all thrust. Allow the 1.5-N/s
  // actuator ramp to catch up again before checking steady integration.
  for(int i=146;i<=245;++i)c.update(22*deg,0.,i*.02,.02,m,cfg,response);
  double integral=c.boost;
  for(int i=246;i<=295;++i) {
    double prev=c.boost;c.update(22*deg,0.,i*.02,.02,m,cfg,response);
    assert(c.boost-prev<=.006001);
  }
  // Part of this interval still recovers from the preceding rate brake;
  // slew anti-windup correctly suspends integration until thrust catches up.
  assert(c.boost>integral+.15);
  // Reaching the angular/rate band while a loaded correction command remains
  // is not completion.  The actuator must run down first; otherwise regrasp
  // changes the contact stiffness at the same instant as support disappears.
  ContinuousRollRecovery loaded_band;loaded_band.reset(4*deg,0.);
  loaded_band.thrust=1.2;loaded_band.rate_initialized=true;
  loaded_band.filtered_rate=0.;
  loaded_band.update(4*deg,0.,.02,.02,m,cfg,response);
  assert(!loaded_band.finished && loaded_band.thrust>0.);
  for(int i=2;i<30 && !loaded_band.finished;++i)
    loaded_band.update(4*deg,0.,i*.02,.02,m,cfg,response);
  assert(loaded_band.finished && loaded_band.thrust<=.05);
  // A small rebound across the nominal stop angle during actuator rundown
  // remains in capture hysteresis; it must not restart PI.
  ContinuousRollRecovery capture_hysteresis;
  capture_hysteresis.reset(cfg.stop-.1*deg,0.);
  capture_hysteresis.thrust=1.2;capture_hysteresis.rate_initialized=true;
  capture_hysteresis.filtered_rate=0.;
  capture_hysteresis.update(cfg.stop-.1*deg,0.,.02,.02,m,cfg,response);
  const double captured_thrust=capture_hysteresis.thrust;
  capture_hysteresis.update(cfg.stop+.1*deg,0.,.04,.02,m,cfg,response);
  assert(capture_hysteresis.target_thrust==0. &&
         capture_hysteresis.thrust<captured_thrust &&
         !capture_hysteresis.finished);
  // A flexed contact may enter the rotor handoff at 5 deg and rebound to
  // 5--6 deg while thrust runs down.  Finish the rotor phase inside that
  // hysteresis so the caller's pressure-conditioned loop can close to the
  // strict 3-deg final target without restarting high-authority search.
  RollRecoveryConfig three_degree=cfg;
  three_degree.stop=3*deg;three_degree.stop_margin=1*deg;
  three_degree.capture_margin_factor=2.;three_degree.settle_sec=.08;
  ContinuousRollRecovery rotor_handoff;
  rotor_handoff.reset(4.9*deg,0.);rotor_handoff.progress_observed=true;
  rotor_handoff.thrust=.30;rotor_handoff.rate_initialized=true;
  rotor_handoff.filtered_rate=0.;
  for(int i=1;i<=30 && !rotor_handoff.finished;++i)
    rotor_handoff.update((i<3?4.9:5.4)*deg,0.,i*.02,.02,m,
                         three_degree,response);
  assert(rotor_handoff.capture_active && rotor_handoff.finished &&
         rotor_handoff.thrust<=.05);
  // Once a captured one-direction correction crosses the target with
  // retained velocity, request re-selection immediately.  Waiting for the
  // outer capture margin lets slow pneumatic dynamics consume the remaining
  // posture envelope.
  ContinuousRollRecovery crossing_capture;
  crossing_capture.reset(4*deg,0.);
  crossing_capture.capture_active=true;
  crossing_capture.thrust=.01;
  crossing_capture.rate_initialized=true;
  crossing_capture.filtered_rate=.08;
  crossing_capture.update(-.2*deg,.08,.02,.02,m,cfg,response);
  assert(crossing_capture.target_crossed && crossing_capture.timed_out &&
         crossing_capture.fault && crossing_capture.thrust==0.);
  // A newly selected opposite pair may inherit wrong-way velocity from the
  // pair which crossed the target.  Do not reject it at the ordinary small
  // motion-detection thrust; once it has reached the explicitly requested
  // braking authority, the same measured reversal test applies.
  ContinuousRollRecovery inherited_motion;
  inherited_motion.reset(6*deg,0.);
  inherited_motion.wrong_way_detection_thrust=2.0;
  inherited_motion.wrong_way_detection_until=1.6;
  for(int i=1;i<=80;++i) {
    inherited_motion.thrust=1.5;
    inherited_motion.update((6.+.03*i)*deg,-.04,i*.02,.02,m,cfg,response);
  }
  assert(!inherited_motion.wrong_way);
  for(int i=81;i<=140 && !inherited_motion.wrong_way;++i) {
    inherited_motion.thrust=2.1;
    inherited_motion.update((6.+.03*i)*deg,-.04,i*.02,.02,m,cfg,response);
  }
  assert(inherited_motion.wrong_way);
  // A contact-lever reversal can settle into a high-thrust oscillation.  Its
  // one-second progress changes sign each cycle, but it remains far from the
  // best attitude already achieved by this candidate.  Reject that retained
  // loss after the bounded dwell instead of holding the 5-N ceiling forever.
  ContinuousRollRecovery retained_reversal;
  retained_reversal.reset(12*deg,0.);
  retained_reversal.progress_observed=true;
  retained_reversal.best_error=5*deg;
  for(int i=1;i<=220 && !retained_reversal.wrong_way;++i) {
    retained_reversal.thrust=4.;
    retained_reversal.update((8.+.2*std::sin(i*.3))*deg,
        0.,i*.02,.02,m,cfg,response);
  }
  assert(retained_reversal.wrong_way && retained_reversal.timed_out);
  ContinuousRollRecovery steady;steady.reset(.5,0.);steady.thrust=2.48;
  for(int i=1;i<=50;++i)steady.update(.5,0.,i*.02,.02,m,cfg,response);
  assert(std::abs(steady.boost-.3)<1e-8); // exactly ONE 0.3-N/s cap, not two boosts
  last=c.thrust;
  c.update(6.1*deg,.2,5.92,.02,m,cfg,response);
  assert(c.thrust<last && c.boost>0.); // prediction must retain contact-support I
  for(int i=0;i<60;++i)c.update(4.9*deg,0.,5.94+i*.02,.02,m,cfg,response);
  assert(c.finished);
  c.update(20*deg,0.,7.16,.02,m,cfg,response);assert(c.finished); // no restart
  // Wrong-sign and tiny levers cannot invert into a maximum thrust command.
  for(double lever: {-1.,0.,.001}) {
    ContinuousRollRecovery a;a.reset(.5,0.);m.lever=lever;
    a.update(.5,0.,.02,.02,m,cfg,response);
    assert(!a.model_valid && a.feedforward==cfg.fallback_thrust);
    assert(a.target_thrust<1.1);
  }
  m.lever=.1;m.moment=100.;
  ContinuousRollRecovery cap;cap.reset(.5,0.);
  for(int i=1;i<=400;++i)cap.update(.5,0.,i*.02,.02,m,no_pair_retry,response);
  assert(cap.thrust<=4. && cap.thrust>3.99 && cap.boost==0.); // anti-windup
  // c13 regression: even a grossly high FF and retained I cannot continue
  // increasing thrust after sustained roll rate exceeds its target. A single
  // raw contact spike is deliberately rejected by the rate filter.
  for(double rate: {.121,.186,.384}) {
    ContinuousRollRecovery fast;fast.reset(25*deg,0.);
    fast.thrust=2.39;fast.boost=.72;fast.filtered_rate=rate;fast.rate_initialized=true;
    const double before=fast.thrust, old_i=fast.boost;
    fast.update(25*deg,rate,.02,.02,m,cfg,response);
    assert(fast.speed_braking && fast.thrust<before);
    assert(before-fast.thrust<=cfg.terminal_brake_fall*.02+1e-10 &&
           fast.boost<=old_i);
    assert(fast.target_thrust<=before && !fast.fault);
  }
  // Include command rundown: at 15 deg and 0.384 rad/s, a fixed 0.2-s
  // prediction would not brake, whereas 3.17 N takes another 0.53 s to remove.
  ContinuousRollRecovery early;early.reset(15*deg,0.);early.thrust=3.17;
  early.update(15*deg,.384,.02,.02,m,cfg,response);
  assert(15*deg-.384*cfg.horizon>cfg.stop);
  assert(early.predicted_braking && early.target_thrust==0. && early.thrust<3.17);
  assert(early.terminal_brake_latched); // no retained-progress latch required
  assert(std::abs(early.braking_horizon-(.2+3.17/6.))<1e-9);
  // A short stick-slip acceleration contributes only over its bounded
  // forecast horizon.  It may trigger an early terminal brake near the
  // target, but must not extrapolate over the complete actuator rundown and
  // stop a correction which is still more than 20 deg away.
  ContinuousRollRecovery far_acceleration;far_acceleration.reset(23.4*deg,0.);
  far_acceleration.thrust=3.;far_acceleration.filtered_rate=.08;
  far_acceleration.rate_initialized=true;
  far_acceleration.update(23.4*deg,.21,.02,.02,m,cfg,response);
  assert(far_acceleration.filtered_acceleration>0. &&
         !far_acceleration.predicted_braking &&
         !far_acceleration.terminal_brake_latched);
  ContinuousRollRecovery accelerating;accelerating.reset(8*deg,0.);
  accelerating.thrust=2.;accelerating.filtered_rate=.08;
  accelerating.rate_initialized=true;
  accelerating.update(8*deg,.65,.02,.02,m,cfg,response);
  assert(accelerating.filtered_acceleration>0. &&
         accelerating.predicted_braking &&
         accelerating.terminal_brake_latched && !accelerating.moving &&
         !accelerating.progress_observed);
  // An invalid down-rate cannot disable braking and leave the motors running.
  RollRecoveryConfig no_brake=cfg;no_brake.fall=0.;
  ContinuousRollRecovery invalid;invalid.reset(.5,0.);invalid.thrust=2.;
  invalid.update(.5,.15,.02,.02,m,no_brake,response);assert(invalid.fault && invalid.thrust==0.);
  m.moment=.2;
  // Predictive stop, target crossing, bad input, high angular rate, clock gap.
  for(int kind=0;kind<5;++kind) {
    ContinuousRollRecovery a;a.reset(.5,0.);
    double e=.5,w=0,t=.02;
    if(kind==0)e=-.01;
    if(kind==1)e=std::numeric_limits<double>::quiet_NaN();
    if(kind==2)w=.8;
    if(kind==3)t=.30;
    if(kind==4)t=-.01;
    a.update(e,w,t,.02,m,cfg,response);
    assert(a.thrust==0. && (kind==0 ? !a.finished && !a.fault : a.fault));
  }
  // Crossing beyond the target hysteresis cannot be corrected by the same
  // one-direction rotor pair.  Request an immediate zero-thrust pair retry
  // instead of waiting through the generic no-progress timeout.
  ContinuousRollRecovery crossed;crossed.reset(10*deg,0.);
  crossed.update(-8.1*deg,0.,.02,.02,m,cfg,response);
  assert(crossed.target_crossed && crossed.timed_out && crossed.fault &&
         !crossed.wrong_way && crossed.thrust==0.);
  ContinuousRollRecovery timeout;timeout.reset(.5,0.);
  for(int i=1;i<=500;++i)timeout.update(.5,0.,i*.02,.02,m,no_pair_retry,response);
  const double timeout_entry_thrust=timeout.thrust;
  double first_timeout=timeout.update(.5,0.,10.02,.02,m,no_pair_retry,response);
  assert(timeout.timed_out && !timeout.fault && first_timeout<timeout_entry_thrust && first_timeout>0.);
  for(int i=502;i<=600 && !timeout.fault;++i)timeout.update(.5,0.,i*.02,.02,m,no_pair_retry,response);
  assert(timeout.fault && timeout.timed_out && timeout.thrust==0.);
  // A pair which reaches meaningful thrust but produces no retained progress
  // requests one controlled opposite-pair retry before the global timeout.
  ContinuousRollRecovery unresponsive;unresponsive.reset(.5,0.);
  RollRecoveryConfig quick_retry=cfg;quick_retry.pair_retry_sec=1.0;
  for(int i=1;i<=300 && !unresponsive.timed_out;++i)
    unresponsive.update(.5,0.,i*.02,.02,m,quick_retry,response);
  assert(unresponsive.unresponsive && unresponsive.timed_out &&
         !unresponsive.wrong_way && !unresponsive.finished);
  // A low initial model command is not evidence that the pair is ineffective.
  // Retry timing begins only after the controller has built meaningful
  // breakaway authority, then requires a full dwell at that authority.
  ContinuousRollRecovery delayed_retry;delayed_retry.reset(.5,0.);
  RollRecoveryConfig delayed_cfg=cfg;delayed_cfg.pair_retry_sec=1.0;
  delayed_cfg.pair_retry_thrust_fraction=.75;
  for(int i=1;i<=100;++i) {
    delayed_retry.update(.5,0.,i*.02,.02,m,delayed_cfg,response);
    assert(!delayed_retry.unresponsive);
  }
  int first_loaded=-1, retry_tick=-1;
  for(int i=101;i<=400 && !delayed_retry.unresponsive;++i) {
    delayed_retry.update(.5,0.,i*.02,.02,m,delayed_cfg,response);
    if(first_loaded<0 && delayed_retry.thrust>=.75*delayed_cfg.limit)
      first_loaded=i;
    if(delayed_retry.unresponsive) retry_tick=i;
  }
  assert(first_loaded>0 && retry_tick-first_loaded>=49);
  // Alternating contact/gyro impulses during a genuine error plateau must not
  // cancel the breakaway integral. The actual command can still brake on the
  // positive-rate samples, but retained authority grows monotonically until
  // the bounded no-response decision is available.
  ContinuousRollRecovery noisy_plateau;noisy_plateau.reset(.5,0.);
  RollRecoveryConfig plateau_cfg=cfg;plateau_cfg.pair_retry_sec=8.;
  double previous_boost=0.;
  for(int i=1;i<=250;++i) {
    const double noisy_rate=(i%2?.18:-.18);
    noisy_plateau.update(.5,noisy_rate,i*.02,.02,m,plateau_cfg,response);
    if(noisy_plateau.stalled) {
      assert(noisy_plateau.boost+1e-12>=previous_boost);
      previous_boost=noisy_plateau.boost;
    }
  }
  assert(noisy_plateau.boost>1.0 && !noisy_plateau.wrong_way);
  // After genuine correction, a soft-contact local minimum inside the
  // configured envelope is a valid capture; do not re-accelerate through the
  // useful lever and then classify the resulting reversal as a new action.
  ContinuousRollRecovery contact_minimum;contact_minimum.reset(20*deg,0.);
  RollRecoveryConfig soft_contact=cfg;
  soft_contact.stop=8*deg;
  soft_contact.contact_minimum_limit=12*deg;
  contact_minimum.moving=true;contact_minimum.progress_observed=true;
  contact_minimum.thrust=.8;contact_minimum.rate_initialized=true;
  contact_minimum.filtered_rate=.02;contact_minimum.recent_progress=1*deg;
  contact_minimum.last_progress=.02;
  contact_minimum.update(11.5*deg,.02,.04,.02,m,soft_contact,response);
  assert(contact_minimum.capture_active &&
         contact_minimum.target_thrust==0. &&
         contact_minimum.thrust<.8 && !contact_minimum.wrong_way);
  // Released contact candidates keep their unambiguous moment arm nearly to
  // the true stop boundary (half a measurement margin); flexed candidates
  // may hand off earlier because their loaded lever can reverse.
  ContinuousRollRecovery flexed_handoff, released_handoff;
  flexed_handoff.reset(10*deg,0.);released_handoff.reset(10*deg,0.);
  RollRecoveryConfig flexed_cfg=cfg,released_cfg=cfg;
  flexed_cfg.stop=released_cfg.stop=3*deg;
  flexed_cfg.stop_margin=released_cfg.stop_margin=1*deg;
  released_cfg.capture_margin_factor=.5;
  for(auto* law:{&flexed_handoff,&released_handoff}) {
    law->moving=true;law->progress_observed=true;law->thrust=.8;
    law->rate_initialized=true;law->filtered_rate=.01;
    law->recent_progress=1*deg;law->last_progress=.02;
  }
  flexed_handoff.update(4.5*deg,.01,.04,.02,m,flexed_cfg,response);
  released_handoff.update(4.5*deg,.01,.04,.02,m,released_cfg,response);
  assert(flexed_handoff.capture_active && !released_handoff.capture_active);
  released_handoff.update(3.4*deg,.01,.06,.02,m,released_cfg,response);
  assert(released_handoff.capture_active);
  // c3 (2026-09-29): 15.5 -> 32 -> 26 deg is real recent improvement,
  // even though the initial/best-ever error has not been beaten in 10 s.
  ContinuousRollRecovery returning;returning.reset(15.5*deg,0.);
  for(int i=1;i<=650;++i) {
    const double t=i*.02;
    const double error=(t<=5.?15.5+3.3*t:32.-1.2*(t-5.))*deg;
    returning.update(error,0.,t,.02,m,no_pair_retry,response); // isolate progress-window behavior from pair retry
    assert(!returning.fault && !returning.finished);
  }
  assert(!returning.moving && returning.best_error<=15.5*deg);
  assert(returning.last_progress>12.9 && returning.recent_progress>cfg.progress_angle);
  // A subsequent true plateau still times out (window flush + 10 s).
  for(int i=651;i<=1300 && !returning.fault;++i)returning.update(22.4*deg,0.,i*.02,.02,m,no_pair_retry,response);
  assert(returning.fault && returning.timed_out && returning.thrust==0.);
  // Neither worsening posture nor sub-threshold IMU jitter renew progress.
  for(bool worsening:{false,true}) {
    ContinuousRollRecovery no_progress;no_progress.reset(25*deg,0.);
    for(int i=1;i<=501;++i) {
      const double error=(25.+(worsening?.01*i:.1*std::sin(i*.43)))*deg;
      no_progress.update(error,0.,i*.02,.02,m,cfg,response);
    }
    assert(no_progress.timed_out && no_progress.last_progress==0.);
  }
  returning.reset(30*deg,26.);
  assert(returning.progress_history.size()==1 && returning.recent_progress==0.);
  returning.update(30*deg,0.,26.02,.02,m,cfg,response);
  assert(returning.last_progress==26. && !returning.fault);
  // If a contact-pivot change reverses a correction which had already begun,
  // a full-window deterioration must ramp thrust down for regrasp instead of
  // winding the PI integral up until the robot falls.
  ContinuousRollRecovery reversed;reversed.reset(8*deg,0.);
  RollRecoveryResponse reverse_response;
  reversed.moving=true;reversed.thrust=1.5;reversed.hold=1.0;
  int reverse_tick=0;
  for(int i=1;i<=80 && !reversed.timed_out;++i) {
    reverse_tick=i;
    reversed.update((8.+.03*i)*deg,-.04,i*.02,.02,m,cfg,reverse_response);
  }
  const double reversal_entry_thrust=reversed.thrust;
  reversed.update(10*deg,-.04,(reverse_tick+1)*.02,.02,m,cfg,reverse_response);
  assert(reversed.wrong_way && reversed.timed_out &&
         reversed.target_thrust==0. &&
         reversed.thrust<reversal_entry_thrust && !reversed.finished);
  // A useful candidate may rebound while predictive braking removes rotor
  // force.  The retained error can worsen by more than the reversal-angle
  // threshold, but decreasing actuator authority proves this was not a new
  // wrong-sign command. Keep the candidate available for closed-loop catch.
  ContinuousRollRecovery rundown_rebound;rundown_rebound.reset(5*deg,0.);
  rundown_rebound.moving=true;rundown_rebound.progress_observed=true;
  for(int i=1;i<=70;++i) {
    rundown_rebound.thrust=2.0-.01*i;
    rundown_rebound.update((5.+.03*i)*deg,-.04,i*.02,.02,m,cfg,response);
    assert(!rundown_rebound.wrong_way);
  }
  assert(rundown_rebound.recent_progress<-2.*cfg.progress_angle &&
         rundown_rebound.recent_thrust_change<-.1);
  // A compliant contact can reverse before the stricter breakaway latch has
  // completed.  A full-window deterioration under real thrust must still
  // abort; otherwise the PI integrates harder into the wrong-way moment.
  ContinuousRollRecovery unlatch_reversed;unlatch_reversed.reset(8*deg,0.);
  RollRecoveryResponse unlatch_response;
  unlatch_reversed.thrust=1.5;
  // Establish measured progress without satisfying the breakaway-rate latch.
  for(int i=1;i<=60;++i) {
    unlatch_reversed.update((8.-.02*i)*deg,.01,i*.02,.02,m,cfg,unlatch_response);
  }
  assert(!unlatch_reversed.moving && unlatch_reversed.progress_observed);
  for(int i=61;i<=150 && !unlatch_reversed.timed_out;++i) {
    // Hold actual correction authority while the contact response reverses;
    // a simultaneous commanded rundown is intentionally not classified as
    // a contact-sign reversal by the new discriminator.
    unlatch_reversed.thrust=1.5;
    unlatch_reversed.update((6.8+.03*(i-60))*deg,-.01,i*.02,.02,m,cfg,unlatch_response);
  }
  assert(!unlatch_reversed.moving && unlatch_reversed.wrong_way &&
         unlatch_reversed.timed_out);
  // c14 regression: a brief speed/prediction excursion near the stop band
  // must not discard the 1.5-N integral and cause seconds of re-search.
  ContinuousRollRecovery retained;retained.reset(9*deg,0.);
  retained.thrust=2.1;retained.boost=1.5;retained.moving=true;
  RollRecoveryConfig three=cfg;three.stop=3*deg;
  m.moment=.01;m.lever=.05;
  // A slow measured state in the approach band may finish the rotor-only
  // phase.  Final +/-3-deg acceptance belongs to the subsequent closed-loop
  // pressure handoff, avoiding moment reversal of an over-extended arm.
  ContinuousRollRecovery margin;margin.reset(4.4*deg,0.);
  margin.thrust=1.5;margin.boost=.8;margin.moving=true;
  margin.progress_observed=true;
  margin.update(4.4*deg,0.,.02,.02,m,three,response);
  assert(margin.capture_active && margin.target_thrust==0. &&
         margin.thrust<1.5 && !margin.finished);
  int margin_step=2;
  while(margin.thrust>.05 && margin_step<50) {
    margin.update(4.4*deg,0.,margin_step*.02,.02,m,three,response);
    ++margin_step;
  }
  for(;margin_step<70 && !margin.finished;++margin_step)
    margin.update(4.4*deg,0.,margin_step*.02,.02,m,three,response);
  assert(margin.capture_active && margin.finished);
  retained.update(9*deg,.15,.02,.02,m,three,response);
  assert(retained.speed_braking && !retained.predicted_braking && retained.boost>1.49);
  assert(retained.thrust>1.9 && retained.thrust<2.1);
  for(int i=2;i<=8 && !retained.predicted_braking;++i)
    retained.update(8*deg,.4,i*.02,.02,m,three,response);
  assert(retained.predicted_braking && retained.boost>1.48 && retained.target_thrust==0.);
  retained.update(8*deg,0.,.18,.02,m,three,response);assert(retained.boost>1.47);
  for(int i=0;i<30;++i)retained.update(2.9*deg,0.,.20+i*.02,.02,m,three,response);
  assert(retained.finished && retained.boost==0.);
  // Repeated geometry changes after motion confirmation cannot modulate FF.
  ContinuousRollRecovery held_model;held_model.reset(.5,0.);
  m.moment=.02;m.lever=.05;
  held_model.update(.5,0.,.02,.02,m,cfg,response);
  const double model_baseline=held_model.feedforward;held_model.moving=true;
  for(int i=2;i<50;++i) {
    m.lever=i%2?.014:.016;m.moment=.1;
    held_model.update(.5,0.,i*.02,.02,m,cfg,response);
    assert(held_model.feedforward==model_baseline);
  }
  m.moment=.2;m.lever=.1;
  // Euler roll-rate consistency with nonzero pitch/roll and all gyro axes.
  assert(std::abs(recoveryRollRate(.2,.4,{{.1,.2,.3}})-
      (.1+std::tan(.4)*(std::sin(.2)*.2+std::cos(.2)*.3)))<1e-10);
  assert(!std::isfinite(recoveryRollRate(0,M_PI/2,{{0,0,0}})));
  // A synthetic lag + static-friction plant: approach must FINISH, not
  // asymptotically stop just outside the 5-degree band. Not a hardware model.
  for(double friction: {.3, 1.0, 1.5}) {
    ContinuousRollRecovery plant;plant.reset(20*deg,0.);
    RollRecoveryResponse memory;double e=20*deg,w=0;
    for(int i=1;i<2000 && !plant.finished && !plant.fault;++i) {
      double force=plant.update(e,w,i*.02,.02,m,cfg,memory);
      double rate=.08*std::max(0.,force-friction);
      w+=(rate-w)*.02/.15;e-=w*.02;
    }
    assert(plant.finished && !plant.fault && e>0.);
  }
  // Delayed linear synthetic response; independent pair memories, bounded updates.
  std::array<RollRecoveryResponse,2> pair;
  ContinuousRollRecovery learner;learner.reset(.5,0.);
  for(int i=0;i<=200;++i) {
    double t=i*.02;learner.thrust=t;
    learner.filtered_rate=.1*std::max(0.,t-cfg.response_delay);
    learner.observe(t,m,cfg,pair[0],true);
  }
  assert(pair[0].samples>=3 && pair[0].gain>.08 && pair[0].gain<.11);
  assert(pair[0].compatible(m.context,cfg) && pair[1].samples==0);
  unsigned n=pair[0].samples;
  for(int i=201;i<=300;++i) { // rapid exhaust: no identifying contact changes as thrust gain
    double t=i*.02;m.context[0]=5.*t;learner.thrust=t;learner.filtered_rate=.1*t;
    learner.observe(t,m,cfg,pair[0],true);
  }
  assert(pair[0].samples==n && !pair[0].compatible(m.context,cfg));
  m.context.fill(0.);
  for(int i=301;i<=400;++i)learner.observe(i*.02,m,cfg,pair[0],false);
  assert(pair[0].samples==n); // unconfirmed motion / braking not learned
  // Brief gyro twitch must not confirm whole-body movement.
  ContinuousRollRecovery twitch;twitch.reset(.5,0.);twitch.thrust=1.;
  twitch.update(.5,.12,.02,.02,m,cfg,response);
  twitch.update(.499,0.,.04,.02,m,cfg,response);assert(!twitch.moving);
  // Braking may reduce thrust below the detection threshold after a genuine
  // candidate starts; gyro direction + actual error reduction must still
  // complete the confirmation instead of resetting it.
  ContinuousRollRecovery braking_candidate;braking_candidate.reset(8*deg,0.);
  braking_candidate.thrust=.7;
  braking_candidate.update(7.8*deg,.08,.02,.02,m,cfg,response);
  for(int i=2;i<=11;++i) {
    braking_candidate.thrust=.1;
    braking_candidate.update((7.8-.1*i)*deg,.08,i*.02,.02,m,cfg,response);
  }
  assert(braking_candidate.moving);
  // Once stopping-distance prediction starts a terminal brake, a brief rate
  // reversal cannot restore the old integral command.  Force must reach zero
  // first, after which any remaining error uses a bounded fresh approach.
  ContinuousRollRecovery latched_brake;latched_brake.reset(20*deg,0.);
  latched_brake.progress_observed=true;latched_brake.moving=true;
  latched_brake.thrust=3.;latched_brake.boost=2.5;
  latched_brake.filtered_rate=.30;latched_brake.rate_initialized=true;
  latched_brake.update(12*deg,.30,.02,.02,m,cfg,response);
  assert(latched_brake.terminal_brake_latched &&
         latched_brake.thrust<3. && latched_brake.boost>2.49);
  double brake_last=latched_brake.thrust;
  for(int i=2;i<=30 && latched_brake.terminal_brake_latched;++i) {
    const double v=latched_brake.update(11*deg,-.02,i*.02,.02,m,cfg,response);
    assert(v<=brake_last+1e-12);brake_last=v;
  }
  assert(!latched_brake.terminal_brake_latched &&
         latched_brake.low_authority_reapproach &&
         latched_brake.thrust<=cfg.reapproach_limit &&
         latched_brake.target_thrust<=cfg.reapproach_limit &&
         latched_brake.boost<.02);
  // The gentle retry is an initial bound, not a permanent 1.5-N ceiling.
  // If retained roll does not improve, restore breakaway authority smoothly;
  // the normal rate/prediction guards remain active once motion resumes.
  const double retry_start=latched_brake.last_update;
  for(int i=1;i<=100;++i)
    latched_brake.update(11*deg,0.,retry_start+i*.02,.02,m,cfg,response,
                         std::numeric_limits<double>::quiet_NaN(),false);
  assert(latched_brake.reapproach_authority>cfg.reapproach_limit+.5 &&
         latched_brake.thrust>cfg.reapproach_limit &&
         latched_brake.thrust<=latched_brake.reapproach_authority+1e-9);
  // A released pair whose currently measured moment arm still has the wrong
  // sign receives shaping force only.  The cap is removed by the caller once
  // measured geometry—not merely the predicted final bend—is restorative.
  ContinuousRollRecovery geometry_gate;geometry_gate.reset(20*deg,0.);
  for(int i=1;i<=200;++i)
    geometry_gate.update(20*deg,0.,i*.02,.02,m,cfg,response,
                         std::numeric_limits<double>::quiet_NaN(),false,.6);
  assert(geometry_gate.thrust<=.600001 &&
         geometry_gate.target_thrust<=.600001);
  // Pressure-release support transfer is not evidence about rotor moment
  // sign. Clear the response window while the caller keeps the gate closed;
  // after pressure settles, the same loaded sustained response is detected.
  ContinuousRollRecovery release_gate;release_gate.reset(15*deg,0.);
  for(int i=1;i<=80;++i) {
    release_gate.thrust=2.;
    release_gate.update((15.+.04*i)*deg,-.04,i*.02,.02,m,cfg,response,
                        std::numeric_limits<double>::quiet_NaN(),false);
  }
  assert(!release_gate.wrong_way && !release_gate.unresponsive);
  for(int i=81;i<=170 && !release_gate.wrong_way;++i) {
    release_gate.thrust=2.;
    release_gate.update((15.+.04*i)*deg,-.04,i*.02,.02,m,cfg,response,
                        std::numeric_limits<double>::quiet_NaN(),true);
  }
  assert(release_gate.wrong_way);
  puts("PASS: model FF, signed rate PI, overspeed priority despite saturated FF/I, rundown prediction, 4N/slew limits, faults and learning gates");
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_model_recovery_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                    '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
    subprocess.run([binary], check=True)
