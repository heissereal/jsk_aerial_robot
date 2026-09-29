#!/usr/bin/env python3
"""Compile production entry/resume expressions against an offline sensor stub."""
import math
import pathlib
import re
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    config = (root / 'config/attitude_pressure_controller.yaml').read_text()
    for key, value in [('correction_trigger_angle_rad', math.radians(8)),
                       ('action_c_posture_recovery_trigger_rad', math.radians(8)),
                       ('correction_stop_angle_rad', math.radians(5)),
                       ('action_c_posture_resume_rad', math.radians(3)),
                       ('correction_trigger_confirm_sec', .3)]:
        match = re.search(r'^' + key + r':\s*([0-9.eE+-]+)', config, re.M)
        assert match and abs(float(match[1]) - value) < 1e-10, key

    def statement(prefix):
        start = source.index(prefix)
        return source[start:source.index(';', start) + 1]

    start = source.index('if (action_c_adaptive_burst_enabled_ && safe_to_handoff &&')
    handoff = source[start:source.index('{', start)]
    assert source.count('attitude_error <= recovery_stop_angle') == 2
    assert 'angular_rate >= correction_max_angular_rate_' in source
    code = r'''
#include <hugmy/control/posture_recovery_gate.h>
#include <cassert>
#include <cstdio>
#include <limits>
struct Time {double seconds; double toSec() const {return seconds;}};
struct Controller {
  hugmy::PostureRecoveryGate correction_entry_gate_, action_c_recovery_gate_, action_c_resume_gate_;
  bool action_c_resume_pending_=false, target_received_=true;
  bool body_imu_fresh=true, pressure_valid=true, neurons_fresh=true, grasp_pressure_ready=true;
  bool action_c_adaptive_burst_enabled_=true;
  bool recovery_bottom_ready=true, recovery_settled=true;
  bool continuous_c_recovery=false, continuous_recovery_blocked_=false;
  bool continuous_recovery_waiting_for_settle_=false, model_c_recovery=false;
  bool continuous_recovery_started_=false, c_hard_roll=false;
  bool recovery_pressure_fresh=true;
  double angular_rate=0., correction_max_angular_rate_=.8;
  bool stable_c_roll=false, c_recovery_requested=true, hard_roll=false;
  struct {bool worsening=true;} action_c_roll_trend_; // Legacy/non-C test fixture.
  double correction_trigger_angle_=8*M_PI/180.;
  double action_c_posture_recovery_trigger_rad_=8*M_PI/180.;
  double action_c_posture_resume_rad_=3*M_PI/180., correction_stop_angle_=5*M_PI/180.;
  double correction_trigger_confirm_sec_=.3;
  double action_c_pre_roll_rate_tolerance_rad_s_=10*M_PI/180.;
  double action_c_posture_rate_trigger_rad_s_=.8;
  bool immediate(double attitude_error) {
    TRIGGER
    IMMEDIATE
    return continuous_recovery_requested;
  }
  bool entry(double attitude_error, Time now) {
    TRIGGER
    ENTRY
    return correction_confirmed;
  }
  bool resume(double attitude_error, double angular_rate, Time now) {
    TRIGGER
    RESUME
    return resume_ready;
  }
  bool handoff(double tilt_error, double body_rate, Time now, bool safe_to_handoff=true,
               bool boundary_settled=true) {
    POSTURE
    HANDOFF return true;
    return false;
  }
  bool finished(double attitude_error) {
    STOP
    return attitude_error <= recovery_stop_angle;
  }
};
int main() {
  const double deg=M_PI/180.;
  hugmy::PostureRecoveryGate gate;
  assert(!gate.update(true,0.,.3)); // Simulation time zero is valid.
  assert(!gate.update(true,.29,.3));
  assert(gate.update(true,.30,.3));
  assert(!gate.update(false,.31,.3));
  assert(!gate.update(true,.4,.3));
  assert(!gate.update(true,.1,.3)); // Clock reset restarts confirmation.
  assert(!gate.update(true,std::numeric_limits<double>::quiet_NaN(),.3));
  assert(!gate.update(true,1.,.3));
  gate.reset(); assert(!gate.update(true,2.,.3));
  for (bool pending: {false,true}) {
    Controller c; c.action_c_resume_pending_=pending;
    assert(!c.entry(6.9*deg,{0.}));
    assert(!c.entry(8*deg,{1.}));
    assert(!c.entry(9*deg,{2.}));
    assert(!c.entry(9*deg,{2.29}));
    assert(c.entry(9*deg,{2.31}));
    assert(!c.entry(7.9*deg,{2.32}));
    assert(!c.entry(9*deg,{3.}));
    assert(!c.entry(7*deg,{3.2}));
    assert(!c.entry(9*deg,{3.3})); // Short excursions never accumulate.
    c.body_imu_fresh=false; assert(!c.entry(9*deg,{4.}));
    c.body_imu_fresh=true; assert(!c.entry(9*deg,{5.}));
    assert(c.entry(9*deg,{5.31}));
    const double band=pending?c.action_c_posture_resume_rad_:c.correction_stop_angle_;
    assert(c.finished(band) && !c.finished(band+.1*deg));
  }
  Controller c;
  assert(!c.handoff(9*deg,0.,{0.}));
  assert(!c.handoff(9*deg,0.,{.29}));
  assert(!c.handoff(9*deg,0.,{.31},false,false)); // Pulse resets ordinary confirmation.
  assert(!c.handoff(9*deg,0.,{.32}));
  assert(c.handoff(9*deg,0.,{.63}));
  assert(!c.handoff(7*deg,0.,{.33}));
  assert(c.handoff(1*deg,.8,{.34})); // Rate path has NO confirmation delay.
  assert(!c.handoff(1*deg,.8,{.35},false));
  c.action_c_resume_pending_=true;
  c.recovery_bottom_ready=false; c.recovery_settled=false;
  assert(!c.entry(9*deg,{.7}));
  assert(!c.entry(9*deg,{1.1})); // No correction while bottom is exhausting.
  assert(!c.resume(5*deg,0.,{.7}));
  assert(!c.resume(5*deg,0.,{1.1})); // No early reinflation, either.
  c.recovery_bottom_ready=true; c.recovery_settled=true;
  assert(!c.resume(5.8*deg,0.,{1.}));
  assert(c.resume(5.8*deg,0.,{1.31})); // Regrasp rebound is not a deadlock.
  assert(!c.resume(8.1*deg,0.,{2.}));
  assert(!c.resume(5*deg,.2,{3.})); // Wait for settled gyro, too.
  assert(!c.resume(5*deg,0.,{4.}));
  c.grasp_pressure_ready=false; assert(!c.resume(5*deg,0.,{4.31}));
  c.grasp_pressure_ready=true; assert(!c.resume(5*deg,0.,{5.}));
  assert(c.resume(5*deg,0.,{5.31}));
  c.action_c_resume_pending_=false; assert(!c.resume(5*deg,0.,{6.}));
  // New C gate must be consistent at GRASP: stable >8 deg can resume,
  // whereas worsening roll still enters correction. A/B/D keep legacy mode.
  Controller stable;stable.action_c_resume_pending_=true;
  stable.stable_c_roll=true;stable.c_recovery_requested=false;
  assert(!stable.entry(11*deg,{0.}));assert(!stable.entry(11*deg,{1.}));
  assert(!stable.resume(11*deg,0.,{1.}));assert(stable.resume(11*deg,0.,{1.31}));
  stable.stable_c_roll=false;stable.c_recovery_requested=true;
  assert(!stable.resume(11*deg,0.,{2.}));
  assert(!stable.entry(11*deg,{2.}));assert(stable.entry(11*deg,{2.31}));
  // Continuous recovery does not wait for atmospheric pressure / regrasp,
  // but stale sensors, faults and high angular rates still inhibit it.
  Controller continuous; continuous.continuous_c_recovery=true;
  continuous.action_c_resume_pending_=true;
  continuous.recovery_bottom_ready=false; continuous.recovery_settled=false;
  continuous.grasp_pressure_ready=false;
  assert(continuous.immediate(12*deg)); // Already-confirmed handoff: no dwell.
  continuous.continuous_recovery_started_=true;
  assert(!continuous.immediate(12*deg)); // Stable regrasp rebound must not chatter.
  continuous.c_hard_roll=true; assert(continuous.immediate(16*deg));
  continuous.c_hard_roll=false;
  assert(!continuous.entry(12*deg,{0.})); assert(continuous.entry(12*deg,{.31}));
  assert(!continuous.resume(5*deg,0.,{.32}));
  continuous.recovery_pressure_fresh=false; assert(!continuous.entry(12*deg,{.4}));
  continuous.recovery_pressure_fresh=true; continuous.angular_rate=.8;
  assert(!continuous.entry(12*deg,{.8}));
  continuous.angular_rate=0.; continuous.continuous_recovery_blocked_=true;
  assert(!continuous.entry(12*deg,{1.2}));
  Controller assisted;
  assisted.action_c_resume_pending_=true;assisted.continuous_c_recovery=true;
  assisted.continuous_recovery_blocked_=true;assisted.continuous_recovery_waiting_for_settle_=true;
  assert(!assisted.immediate(20*deg)); // no repeated thrust search during manual/settle wait
  assert(!assisted.resume(3.1*deg,0.,{0.}));
  assert(!assisted.resume(2.9*deg,.06,{.1}));
  assert(!assisted.resume(2.9*deg,0.,{.2}));
  assert(!assisted.resume(2.9*deg,0.,{1.19}));
  assert(assisted.resume(2.9*deg,0.,{1.21}));
  assisted.recovery_pressure_fresh=false;assert(!assisted.resume(0.,0.,{1.3}));
  assisted.recovery_pressure_fresh=true;assisted.continuous_recovery_waiting_for_settle_=false;
  assert(!assisted.resume(0.,0.,{3.})); // hard fault remains blocked, not auto-rearmed
  std::puts("PASS: 8-deg entry, C 3-deg success, soft-timeout stable wait/resume, hard-fault block and fresh sensor gates");
}
'''
    for key, value in {
        'TRIGGER': statement('const double active_correction_trigger ='),
        'ENTRY': statement('const bool correction_confirmed ='),
        'IMMEDIATE': statement('const bool continuous_recovery_requested ='),
        'RESUME': statement('const bool resume_ready ='),
        'POSTURE': statement('const bool posture_confirmed ='),
        'HANDOFF': handoff,
        'STOP': statement('const double recovery_stop_angle ='),
    }.items():
        code = code.replace(key, value)
    with tempfile.TemporaryDirectory(prefix='hugmy_posture_confirmation_') as directory:
        cpp = pathlib.Path(directory) / 'test.cpp'
        binary = pathlib.Path(directory) / 'test'
        cpp.write_text(code)
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra',
                        '-I' + str(root / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
