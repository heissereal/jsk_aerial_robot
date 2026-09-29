#!/usr/bin/env python3
"""Offline tests of the production geometry-feedforward / roll-rate PI law."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
start = source.index('case PREPARE:')
branch = source[start:source.index('publishCorrectionThrust(fixed_arm_test_enabled_', start)]
assert 'continuous_roll_recovery_.update(' in branch
assert 'enterState(THRUST_CORRECTION' not in branch
assert 'active_pair_pressure_ready' not in branch
assert 'prepare_arm_active_[arm] ? thrust : 0.0' in branch
assert 'publishBottomTarget(0.0)' in branch
assert 'continuous_recovery_blocked_ = true' in branch
assert 'measuredRecoveryModel()' in branch
assert 'roll_recovery_response_[correction_axis_[0] > 0.0 ? 0 : 1]' in branch
assert '(now - state_start_).toSec() >= prepare_timeout_' in branch
assert 'state_start_ =' not in branch # progress must not restart the overall deadline

code = r'''
#include <hugmy/control/continuous_roll_recovery.h>
#include <cassert>
#include <cstdio>
using namespace hugmy;
const double deg=M_PI/180.;
int main() {
  RollRecoveryConfig cfg; RollRecoveryModel m; RollRecoveryResponse response;
  m.valid=true; m.moment=.2; m.lever=.1;
  ContinuousRollRecovery c;c.reset(25*deg,0.);
  double last=0;
  for(int i=1;i<=100;++i) {
    double v=c.update(25*deg,0.,i*.02,.02,m,cfg,response);
    assert(v>=last && v-last<=.030001 && v<=4.);last=v;
  }
  assert(c.model_valid && std::abs(c.feedforward-2.)<1e-8);
  assert(c.thrust>2.4 && c.thrust<2.8); // aims at model FF + rate feedback
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
  ContinuousRollRecovery steady;steady.reset(.5,0.);steady.thrust=2.48;
  for(int i=1;i<=50;++i)steady.update(.5,0.,i*.02,.02,m,cfg,response);
  assert(std::abs(steady.boost-.3)<1e-8); // exactly ONE 0.3-N/s cap, not two boosts
  last=c.thrust;
  c.update(6*deg,.2,5.92,.02,m,cfg,response);
  assert(c.thrust<last && c.boost>0.); // prediction must retain contact-support I
  c.update(4.9*deg,0.,5.94,.02,m,cfg,response);assert(c.finished);
  c.update(20*deg,0.,5.96,.02,m,cfg,response);assert(c.finished); // no restart
  // Wrong-sign and tiny levers cannot invert into a maximum thrust command.
  for(double lever: {-1.,0.,.001}) {
    ContinuousRollRecovery a;a.reset(.5,0.);m.lever=lever;
    a.update(.5,0.,.02,.02,m,cfg,response);
    assert(!a.model_valid && a.feedforward==cfg.fallback_thrust);
    assert(a.target_thrust<1.1);
  }
  m.lever=.1;m.moment=100.;
  ContinuousRollRecovery cap;cap.reset(.5,0.);
  for(int i=1;i<=400;++i)cap.update(.5,0.,i*.02,.02,m,cfg,response);
  assert(cap.thrust<=4. && cap.thrust>3.99 && cap.boost==0.); // anti-windup
  // c13 regression: even a grossly high FF and retained I cannot continue
  // increasing thrust after measured roll rate exceeds its 0.12-rad/s target.
  for(double rate: {.121,.186,.384}) {
    ContinuousRollRecovery fast;fast.reset(25*deg,0.);
    fast.thrust=2.39;fast.boost=.72;fast.filtered_rate=.03;fast.rate_initialized=true;
    const double before=fast.thrust, old_i=fast.boost;
    fast.update(25*deg,rate,.02,.02,m,cfg,response);
    assert(fast.speed_braking && fast.thrust<before);
    assert(before-fast.thrust<=cfg.fall*.02+1e-10 && fast.boost<=old_i);
    assert(fast.target_thrust<=before && !fast.fault);
  }
  // Include command rundown: at 15 deg and 0.384 rad/s, a fixed 0.2-s
  // prediction would not brake, whereas 3.17 N takes another 0.53 s to remove.
  ContinuousRollRecovery early;early.reset(15*deg,0.);early.thrust=3.17;
  early.update(15*deg,.384,.02,.02,m,cfg,response);
  assert(15*deg-.384*cfg.horizon>cfg.stop);
  assert(early.predicted_braking && early.target_thrust==0. && early.thrust<3.17);
  assert(std::abs(early.braking_horizon-(.2+3.17/6.))<1e-9);
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
    assert(a.thrust==0. && (kind==0 ? a.finished : a.fault));
  }
  ContinuousRollRecovery timeout;timeout.reset(.5,0.);
  for(int i=1;i<=501;++i)timeout.update(.5,0.,i*.02,.02,m,cfg,response);
  assert(timeout.fault && timeout.timed_out && timeout.thrust==0.);
  // c3 (2026-09-29): 15.5 -> 32 -> 26 deg is real recent improvement,
  // even though the initial/best-ever error has not been beaten in 10 s.
  ContinuousRollRecovery returning;returning.reset(15.5*deg,0.);
  for(int i=1;i<=650;++i) {
    const double t=i*.02;
    const double error=(t<=5.?15.5+3.3*t:32.-1.2*(t-5.))*deg;
    returning.update(error,0.,t,.02,m,cfg,response); // do not rely on gyro motion latch
    assert(!returning.fault && !returning.finished);
  }
  assert(!returning.moving && returning.best_error<=15.5*deg);
  assert(returning.last_progress>12.9 && returning.recent_progress>cfg.progress_angle);
  // A subsequent true plateau still times out (window flush + 10 s).
  for(int i=651;i<=1250;++i)returning.update(22.4*deg,0.,i*.02,.02,m,cfg,response);
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
  // c14 regression: a brief speed/prediction excursion near the stop band
  // must not discard the 1.5-N integral and cause seconds of re-search.
  ContinuousRollRecovery retained;retained.reset(9*deg,0.);
  retained.thrust=2.1;retained.boost=1.5;retained.moving=true;
  RollRecoveryConfig three=cfg;three.stop=3*deg;
  m.moment=.01;m.lever=.05;
  retained.update(9*deg,.2,.02,.02,m,three,response);
  assert(retained.speed_braking && !retained.predicted_braking && retained.boost>1.49);
  assert(retained.thrust>1.9 && retained.thrust<2.1);
  retained.update(8*deg,.4,.04,.02,m,three,response);
  assert(retained.predicted_braking && retained.boost>1.48 && retained.target_thrust==0.);
  retained.update(8*deg,0.,.06,.02,m,three,response);assert(retained.boost>1.47);
  retained.update(2.9*deg,0.,.08,.02,m,three,response);assert(retained.finished && retained.boost==0.);
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
  puts("PASS: model FF, signed rate PI, overspeed priority despite saturated FF/I, rundown prediction, 4N/slew limits, faults and learning gates");
}
'''
with tempfile.TemporaryDirectory(prefix='hugmy_model_recovery_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                    '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
    subprocess.run([binary], check=True)
