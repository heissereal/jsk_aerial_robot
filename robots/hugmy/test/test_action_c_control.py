#!/usr/bin/env python3
"""Offline C++ regressions; no ROS master, bag playback or actuator outputs.

Run: python3 robots/hugmy/test/test_action_c_control.py
Requires g++ only. Compile the real selector body against a data-only stub,
and the real feedback header, so the tested logic is not a Python rewrite.
"""
import pathlib
import re
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    header = (root / 'include/hugmy/control/attitude_pressure_controller.h').read_text()
    start = source.index('void AttitudePressureController::selectNextActionCCandidate()')
    end = source.index('void AttitudePressureController::finishActionCOptimizerTrial(', start)
    selector = source[start:end]
    fields = sorted(set(re.findall(r'\baction_c_\w+_', selector)))
    declarations = []
    for field in fields:
        match = re.search(r'\b(bool|int|double|size_t|PressureArray)\s+[^;{}]*?\b' + re.escape(field) + r'\b', header)
        assert match, field
        declarations.append(f'{match[1]} {field}{{}};')
    # Exercise the production best-eligibility and progress-validity expressions.
    start = source.index('const bool eligible =', end)
    eligibility = source[start:source.index(';', start) + 1]
    start = source.index('const bool valid_progress =')
    validity = source[start:source.index(';', start) + 1]
    start = source.index('const bool reprobe_qualified =')
    reprobe_qualification = source[start:source.index(';', start) + 1]
    # Per-tick output must survive until the one-sided ramp limiter uses it.
    pull = source[source.index('case ACTION_C_HORIZONTAL_PULL:'):]
    assert 'if (!inchworm_front_[arm] || !action_c_pulse_started_)' in pull
    assert 'previous_thrust + action_c_thrust_ramp_n_s_ * std::max(0.0, dt)' in pull
    code = r'''
#include <hugmy/control/action_c_feedback.h>
#include <array>
#include <cassert>
#include <cstdio>
#include <limits>
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
class AttitudePressureController {
public:
using PressureArray=std::array<double,4>;
FIELDS
void selectNextActionCCandidate();
};
SELECTOR
AttitudePressureController initial() {
  AttitudePressureController c;
  c.action_c_adaptive_burst_enabled_=true;
  c.action_c_min_thrust_n_=3.; c.action_c_max_thrust_n_=6.;
  c.action_c_thrust_n_=3.; c.action_c_optimizer_safe_thrust_max_n_=6.;
  c.action_c_bottom_min_kpa_=20.; c.action_c_bottom_max_kpa_=30.;
  c.action_c_bottom_initial_kpa_=30.; c.action_c_bottom_hold_kpa_=30.;
  c.action_c_optimizer_thrust_step_n_=.25;
  c.action_c_optimizer_bottom_step_kpa_=5.;
  c.action_c_optimizer_best_score_=-std::numeric_limits<double>::infinity();
  c.action_c_optimizer_burst_pulses_=1;
  c.action_c_optimizer_min_burst_pulses_=1; c.action_c_optimizer_max_burst_pulses_=5;
  c.action_c_optimizer_valid_pulses_=1;
  return c;
}
bool eligible(int action_c_optimizer_valid_pulses_, double retained_dx,
              bool action_c_optimizer_trial_had_extension_guard_, double score=0.,
              int action_c_optimizer_successful_pulses_=1) {
  ELIGIBILITY
  return eligible;
}
bool pressure_ok;
bool pressureReached(const std::array<double,4>&) {return pressure_ok;}
bool valid(bool action_c_pulse_started_, bool action_c_pulse_recoil_complete_,
           bool action_c_pulse_overextended_, bool pressure) {
  std::array<double,4> inchworm_pressure_{};
  pressure_ok=pressure;
  VALIDITY
  return valid_progress;
}
bool qualified(bool recovery_required, bool action_c_optimizer_trial_had_extension_guard_,
               int action_c_optimizer_completed_pulses_, int action_c_optimizer_valid_pulses_,
               double retained_dx, double action_c_optimizer_peak_extension_mismatch_rad_,
               double action_c_optimizer_peak_tilt_rad_, double action_c_optimizer_peak_rate_rad_s_) {
  const double action_c_thrust_reprobe_max_mismatch_rad_=12*std::acos(-1.)/180.;
  const double action_c_posture_recovery_trigger_rad_=8*std::acos(-1.)/180.;
  const double action_c_posture_rate_trigger_rad_s_=.8;
  REPROBE_QUALIFICATION
  return reprobe_qualified;
}
int main() {
  // Actual timed-pulse selector: failed 8-deg tracking does not block amplitude
  // search. Increase only after complete safe regrip and inadequate progress.
  auto search=initial();
  search.action_c_reference_pulse_mode_=true;
  search.action_c_reference_single_pulse_=false;
  search.action_c_pulse_search_enabled_=true;
  search.action_c_pulse_search_step_n_=.5;
  search.action_c_pulse_search_progress_m_=.002;
  search.action_c_pulse_started_=true;
  search.action_c_optimizer_retained_progress_m_=.00015; // bag c13: 0.15 mm
  search.action_c_optimizer_tracking_pulses_=0;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==3.5);
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.);
  search.action_c_optimizer_retained_progress_m_=.002;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.);
  search.action_c_optimizer_retained_progress_m_=-.001;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.5);
  search.action_c_optimizer_retained_progress_m_=-.01;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==5.);
  search.action_c_optimizer_trial_had_extension_guard_=true;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.5);
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.);
  search.action_c_optimizer_trial_had_extension_guard_=false;
  search.action_c_optimizer_retained_progress_m_=0.;
  search.action_c_optimizer_valid_pulses_=0;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.);
  search.action_c_optimizer_trial_required_recovery_=true;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==3.5);
  search.action_c_optimizer_trial_required_recovery_=false;
  search.action_c_optimizer_trial_had_extension_guard_=true;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==3.);
  search.action_c_optimizer_trial_had_extension_guard_=false;
  search.action_c_optimizer_valid_pulses_=1;
  search.action_c_reference_single_pulse_=true;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==3.);
  search.action_c_reference_single_pulse_=false;
  search.action_c_pulse_search_enabled_=false;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==3.);
  search.action_c_pulse_search_enabled_=true;
  for (int i=0;i<20;++i) search.selectNextActionCCandidate();
  assert(search.action_c_thrust_n_==6. && search.action_c_bottom_hold_kpa_==30.);
  search.action_c_optimizer_safe_thrust_max_n_=4.;
  search.selectNextActionCCandidate(); assert(search.action_c_thrust_n_==4.);
  assert(hugmy::actionCSearchPulseDuration(.6,3.,8.)==.6);
  assert(std::abs(hugmy::actionCSearchPulseDuration(.6,6.,8.)-.9)<1e-9);
  assert(hugmy::actionCSearchPulseDuration(.6,6.,.1)==1.);
  assert(hugmy::actionCSearchPulseDuration(.6,6.,0.)==.6);
  // A pair guard stops exploration at 3 N, but three verified pulses unlock
  // exactly 3.25 N. Never skip directly to the configured 6-N maximum.
  hugmy::ActionCThrustSafety safety;
  safety.reset(6.);
  assert(!safety.observeTrial(3.,3.,.25,hugmy::ACTION_C_PAIR_MISMATCH,0,3));
  assert(safety.limit()==3. && safety.hard_max_n==6.);
  assert(!safety.observeTrial(3.,3.,.25,0,1,3));
  assert(!safety.observeTrial(3.,3.,.25,0,1,3));
  assert(safety.observeTrial(3.,3.,.25,0,1,3));
  assert(safety.limit()==3.25 && safety.safe_pulses==0);
  auto probe=initial(); probe.action_c_optimizer_reprobe_pending_=true;
  probe.action_c_optimizer_safe_thrust_max_n_=safety.limit();
  probe.action_c_optimizer_best_score_=-.01; // A finite old best cannot swallow the probe.
  probe.selectNextActionCCandidate();
  assert(probe.action_c_thrust_n_==3.25 && probe.action_c_optimizer_burst_pulses_==1);
  assert(probe.action_c_bottom_hold_kpa_==30. && !probe.action_c_optimizer_reprobe_pending_);
  // Lower-thrust samples cannot reopen a higher ceiling. Invalid samples
  // reset consecutive confirmation; a fresh guard relocks the temporary cap.
  assert(!safety.observeTrial(3.,3.,.25,0,3,3)); assert(safety.limit()==3.25);
  assert(!safety.observeTrial(3.25,3.,.25,0,2,3));
  assert(!safety.observeTrial(3.25,3.,.25,0,0,3)); assert(safety.safe_pulses==0);
  assert(!safety.observeTrial(3.25,3.,.25,0,1,3));
  assert(!safety.observeTrial(3.25,3.,.25,hugmy::ACTION_C_PAIR_MISMATCH,0,3));
  assert(safety.limit()==3. && safety.safe_pulses==0);
  // Each non-pair reason is a hard latch, including mixed hard+pair events.
  for(unsigned int bit=1;bit<=hugmy::ACTION_C_ANGLE_UNRELIABLE;bit<<=1) {
    if(bit==hugmy::ACTION_C_PAIR_MISMATCH) continue;
    safety.reset(6.);
    safety.observeTrial(3.,3.,.25,bit|hugmy::ACTION_C_PAIR_MISMATCH,0,3);
    for(int i=0;i<10;++i) assert(!safety.observeTrial(3.,3.,.25,0,3,3));
    assert(safety.limit()==3. && safety.hard_max_n==3.);
  }
  // Large configured increments are capped; repeated qualification never
  // crosses the hard/configured ceiling. A zero increment disables reopening.
  safety.reset(6.); safety.observeTrial(3.,3.,.25,hugmy::ACTION_C_PAIR_MISMATCH,0,3);
  assert(!safety.observeTrial(3.,3.,0.,0,3,3));
  for(int i=1;i<=12;++i) {
    assert(safety.observeTrial(safety.limit(),3.,10.,0,3,3));
    assert(safety.limit()==3.+.25*i);
  }
  assert(!safety.observeTrial(6.,3.,.25,0,3,3)); assert(safety.limit()==6.);
  const double d=std::acos(-1.)/180.;
  assert(qualified(false,false,1,1,.001,9*d,4*d,.3));
  assert(!qualified(true,false,1,1,.001,9*d,4*d,.3));
  assert(!qualified(false,true,1,1,.001,9*d,4*d,.3));
  assert(!qualified(false,false,2,1,.001,9*d,4*d,.3));
  assert(!qualified(false,false,0,0,.001,9*d,4*d,.3));
  assert(!qualified(false,false,1,1,-.001,9*d,4*d,.3));
  assert(!qualified(false,false,1,1,.001,13*d,4*d,.3));
  assert(!qualified(false,false,1,1,.001,9*d,8*d,.3));
  assert(!qualified(false,false,1,1,.001,9*d,4*d,.8));
  assert(!qualified(false,false,1,1,std::numeric_limits<double>::quiet_NaN(),9*d,4*d,.3));
  // No best: 30-kPa support, one pulse, monotone bounded discovery.
  auto c=initial();
  for(int step=1; step<=12; ++step) {
    c.selectNextActionCCandidate();
    assert(c.action_c_thrust_n_==3.+.25*step);
    assert(c.action_c_bottom_hold_kpa_==30. && c.action_c_optimizer_burst_pulses_==1);
  }
  for(int step=0; step<12; ++step) {
    c.selectNextActionCCandidate(); assert(c.action_c_thrust_n_==6.);
  }
  // Invalid/unsafe trials cannot increase thrust, pressure exploration or burst.
  for(int reason=0; reason<3; ++reason) {
    c=initial(); c.action_c_bottom_hold_kpa_=20.; c.action_c_optimizer_burst_pulses_=4;
    if(reason==0) c.action_c_optimizer_valid_pulses_=0;
    if(reason==1) c.action_c_optimizer_trial_required_recovery_=true;
    if(reason==2) c.action_c_optimizer_trial_had_extension_guard_=true;
    for(int i=0;i<10;++i) c.selectNextActionCCandidate();
    assert(c.action_c_thrust_n_==3. && c.action_c_bottom_hold_kpa_==30.);
    assert(c.action_c_optimizer_burst_pulses_==1);
  }
  c=initial(); c.action_c_optimizer_safe_thrust_max_n_=3.;
  c.selectNextActionCCandidate(); assert(c.action_c_thrust_n_==3.);
  c=initial(); c.action_c_adaptive_burst_enabled_=false;
  c.selectNextActionCCandidate(); assert(c.action_c_thrust_n_==3.);
  // Best candidate, not failed current candidate, anchors subsequent search.
  c=initial(); c.action_c_thrust_n_=5.; c.action_c_bottom_hold_kpa_=20.;
  c.action_c_optimizer_best_score_=-.01; c.action_c_optimizer_best_thrust_n_=3.25;
  c.action_c_optimizer_best_bottom_kpa_=30.; c.action_c_optimizer_best_burst_pulses_=1;
  c.selectNextActionCCandidate();
  assert(c.action_c_thrust_n_==3.5 && c.action_c_bottom_hold_kpa_==30.);
  for(int i=0;i<24;++i) {
    c.selectNextActionCCandidate(); assert(c.action_c_bottom_hold_kpa_==30.);
    assert(c.action_c_optimizer_burst_pulses_==1); // No successful stride: don't grow burst.
  }
  // Bottom search is opt-in and must be based on an established best.
  c.action_c_optimize_bottom_enabled_=true; c.action_c_optimizer_direction_index_=1;
  c.action_c_optimizer_bottom_step_kpa_=5.; c.selectNextActionCCandidate();
  assert(c.action_c_bottom_hold_kpa_==25.);
  c=initial(); c.action_c_optimize_bottom_enabled_=true; c.selectNextActionCCandidate();
  assert(c.action_c_bottom_hold_kpa_==30.);
  // Continuous progress preserves small valid forward motion, not unsafe drift.
  assert(eligible(1,.0003,false)); assert(!eligible(0,.01,false));
  assert(!eligible(1,.0003,false,0.,0)); // no extension: tiny drift is not best
  assert(!eligible(1,.01,false,0.,0)); // even large drift cannot replace success
  assert(!eligible(1,.01,true)); assert(!eligible(1,-.001,false));
  assert(!eligible(1,.01,false,std::numeric_limits<double>::quiet_NaN()));
  for(int mask=0;mask<16;++mask) {
    bool ok=valid(mask&1,mask&2,mask&4,mask&8);
    assert(ok==(mask==11));
    assert(hugmy::actionCMeasuredProgress(.0003,ok)==(ok?.0003:0.));
    assert(hugmy::actionCMeasuredProgress(-.001,ok)==-.001);
  }
  const double deg=std::acos(-1.)/180.;
  auto trim=[deg](double extension,double slowest,double rate) {
    return hugmy::actionCExtensionSyncReduction(extension*deg,slowest*deg,rate*deg,
        8*deg,.04,2*deg,8.,2.);
  };
  assert(trim(0,0,0)==0.); assert(trim(6,6,0)==0.);
  assert(trim(1,1,0)==0.); assert(trim(6,1,0)>.4);
  assert(trim(10,0,0)>1.); assert(trim(6,1,-40)>trim(6,1,0));
  assert(trim(0,-10,0)==0.); // Other arm re-bending alone must not brake this arm.
  assert(trim(100,0,-300)==2.);
  assert(trim(std::numeric_limits<double>::quiet_NaN(),0,0)==2.);
  // Symmetric extension from unequal initial angles is symmetric feedback.
  assert(trim(123-117,6,0)==trim(143-137,6,0));
  // Normal pair synchronization raises the slower arm, never cuts the leader.
  auto individual=[deg](double extension,double fastest,double rate,double ceiling=3.25) {
    return hugmy::actionCIndividualThrust(3.,ceiling,.5,8.,6*deg,
        extension*deg,fastest*deg,rate*deg,6.,.12,.04,2*deg,4*deg,8*deg);
  };
  assert(individual(6,6,0)==3.);
  assert(individual(0,6,0)==3.25);
  assert(individual(0,6,0,3.)==3.); // Hard ceiling includes the extra assistance.
  assert(individual(14,14,0)==0.);  // Overshoot protection is not disabled.
  assert(individual(8,8,-100)<3.); // Predicted overshoot brakes early.
  assert(individual(std::numeric_limits<double>::quiet_NaN(),0,0)==0.);
  assert(hugmy::actionCExtensionReference(0,.45,8*deg)==0.);
  assert(std::abs(hugmy::actionCExtensionReference(.225,.45,8*deg)-4*deg)<1e-12);
  assert(hugmy::actionCExtensionReference(.6,.45,8*deg)==8*deg);
  for(int e=-30;e<=30;++e) for(int peer=-30;peer<=30;++peer) {
    const double t=individual(e,peer,-20);
    assert(t>=0. && t<=3.25);
  }
  // Per-arm increase is based on measured response and saturation, not simply
  // a small progress reward. Reverse response, unsafe or incomplete trials fail.
  auto raise=[deg](bool valid,bool guard,bool posture,double mismatch,double terminal,
                   double peak,double thrust) {
    return hugmy::actionCCanRaiseArm(valid,guard,posture,mismatch*deg,12*deg,
        terminal*deg,peak*deg,8*deg,2*deg,thrust,3.25);
  };
  assert(raise(true,false,true,5,2,3,3.25));
  assert(!raise(false,false,true,5,2,3,3.25));
  assert(!raise(true,true,true,5,2,3,3.25));
  assert(!raise(true,false,false,5,2,3,3.25));
  assert(!raise(true,false,true,13,2,3,3.25));
  assert(!raise(true,false,true,5,-4,3,3.25));
  assert(!raise(true,false,true,5,2,8,3.25));
  assert(!raise(true,false,true,5,2,3,2.5));
  c=initial(); c.action_c_individual_extension_enabled_=true;
  c.action_c_arm_calibration_changed_=true; c.action_c_optimizer_best_score_=1.;
  c.action_c_arm_thrust_offset_n_[3]=.25; c.action_c_optimizer_burst_pulses_=3;
  c.selectNextActionCCandidate();
  assert(c.action_c_thrust_n_==3. && c.action_c_optimizer_burst_pulses_==1);
  assert(!c.action_c_arm_calibration_changed_ && !std::isfinite(c.action_c_optimizer_best_score_));
  assert(c.action_c_arm_thrust_offset_n_[3]==.25);
  c=initial(); c.action_c_individual_extension_enabled_=true;
  c.action_c_optimizer_tracking_pulses_=0;
  c.action_c_optimizer_reprobe_pending_=true;
  c.selectNextActionCCandidate();
  assert(c.action_c_thrust_n_==3. && c.action_c_optimizer_burst_pulses_==1);
  assert(!c.action_c_optimizer_reprobe_pending_);
  c.action_c_optimizer_tracking_pulses_=1;
  c.selectNextActionCCandidate(); assert(c.action_c_thrust_n_==3.25);
  puts("PASS: lagging-arm assist, shared trajectory, learning gates, overshoot protection, safety ceilings and optimizer regressions");
}
'''
    code = code.replace('FIELDS', '\n'.join(declarations)).replace('SELECTOR', selector)
    code = code.replace('ELIGIBILITY', eligibility).replace('VALIDITY', validity)
    code = code.replace('REPROBE_QUALIFICATION', reprobe_qualification)
    with tempfile.TemporaryDirectory(prefix='hugmy_action_c_test_') as directory:
        binary = str(pathlib.Path(directory) / 'regression')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
