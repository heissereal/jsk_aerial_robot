#!/usr/bin/env python3
"""Replay deterministic phase/pressure/roll sequences through production gates.

No ROS, simulated plant or actuator access. Compile the actual C++ boundary
logic rather than reimplementing its state-transition rules in Python.
"""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    start = source.index('const bool posture_boundary =')
    end = source.index('if (action_c_adaptive_burst_enabled_ && safe_to_handoff &&', start)
    conditions = source[start:end]
    entry = source[end:source.index('{', end)]
    # No zero-bottom command in the full normal C cycle or candidate boundary.
    for first, last in [('ACTION_C_PREPARE', 'ACTION_C_HORIZONTAL_PULL'),
                        ('ACTION_C_HORIZONTAL_PULL', 'ACTION_C_EVALUATE'),
                        ('ACTION_C_EVALUATE', 'ACTION_C_REGRIP'),
                        ('ACTION_C_REGRIP', 'ROCK_FRONT_UNLOAD')]:
        block = source[source.index('case ' + first + ':'):source.index('case ' + last + ':')]
        assert 'publishBottomTarget(actionCBottomTarget())' in block
        assert 'publishBottomTarget(0.0)' not in block
    settle = source[source.index('case SETTLE:'):]
    assert 'publishBottomTarget(actionCBottomTarget())' in settle
    assert 'action_c_boundary_ready_' in settle
    assert 'Action C boundary settling timeout' in settle
    recovery = source[source.index('void AttitudePressureController::requestActionCPostureRecovery('):]
    recovery = recovery[:recovery.index('bool AttitudePressureController::resumeActionCAfterRecovery(')]
    assert 'publishBottomTarget(0.0)' in recovery
    enter = source[source.index('void AttitudePressureController::enterState('):]
    assert 'action_c_boundary_settle_gate_.reset()' in enter[:enter.index('if (state_ == PULL)')]
    assert 'pneumatic_ready && roll_ready && action_c_boundary_ready_' in source
    code = r'''
#include <hugmy/control/posture_recovery_gate.h>
#include <hugmy/control/roll_trend_gate.h>
#include <array>
#include <cassert>
#include <cstdio>
#define ROS_INFO_THROTTLE(...) ((void)0)
struct Time { double seconds; double toSec() const {return seconds;} };
struct Controller {
  enum {ACTION_C_PREPARE, ACTION_C_HORIZONTAL_PULL, ACTION_C_EVALUATE,
        ACTION_C_REGRIP, SETTLE};
  int state_=ACTION_C_PREPARE;
  bool action_c_pulse_started_=false, action_c_boundary_ready_=false;
  bool action_c_bottom_sync_enabled_=false;
  struct {bool filling=false;} action_c_bottom_timing_;
  bool action_c_adaptive_burst_enabled_=true, arms_ready=true;
  double action_c_command_thrust_n_=0, bottom_pressure_kpa_=30;
  double action_c_bottom_hold_kpa_=30, pressure_tolerance_kpa_=1;
  double action_c_pre_roll_rate_tolerance_rad_s_=10*M_PI/180;
  double action_c_posture_settle_sec_=.5, correction_trigger_confirm_sec_=.3;
  double action_c_posture_recovery_trigger_rad_=8*M_PI/180;
  double action_c_posture_rate_trigger_rad_s_=.8;
  double action_c_posture_hard_roll_rad_=15*M_PI/180.;
  double roll_=0, target_roll_=0;
  std::array<double,3> body_angular_velocity_{};
  hugmy::RollTrendGate action_c_roll_trend_;
  std::array<double,4> inchworm_pressure_{}, inchworm_thrust_{};
  hugmy::PostureRecoveryGate action_c_boundary_settle_gate_, action_c_recovery_gate_;
  bool pressureReached(const std::array<double,4>&) {return arms_ready;}
  bool actionCPressureReady(const std::array<double,4>&, Time) {return arms_ready;}
  double actionCBottomTarget() const {return action_c_bottom_hold_kpa_;}
  static double wrapAngle(double x) {return x;}
  void phase(int state) {
    state_=state; action_c_boundary_ready_=false;
    action_c_boundary_settle_gate_.reset(); action_c_recovery_gate_.reset();
    action_c_roll_trend_.reset();
  }
  bool tick(double t, double degrees, double body_rate=0) {
    const Time now{t}; const double tilt_error=std::abs(degrees)*M_PI/180;
    roll_=degrees*M_PI/180.;body_angular_velocity_[0]=body_rate;body_rate=std::abs(body_rate);
    CONDITIONS
    HANDOFF return true;
    return false;
  }
  bool drive(double start,double seconds,double angle,double slope=0) {
    bool recovery=false;
    for(int i=0;i<=int(seconds/.02);++i)
      recovery=tick(start+.02*i,angle+.02*i*slope,slope*M_PI/180.) || recovery;
    return recovery;
  }
};
int main() {
  using C=Controller;
  C c;
  // Inflation, even if roll stays above 8 degrees for seconds, is not a
  // settled posture and cannot accumulate ordinary recovery confirmation.
  c.bottom_pressure_kpa_=10;
  assert(!c.tick(0,9)); assert(!c.tick(5,9));
  assert(!c.action_c_boundary_ready_);
  c.bottom_pressure_kpa_=30;
  assert(!c.tick(5.1,9)); assert(!c.tick(5.61,9));
  assert(!c.tick(5.8,7)); // Natural return cancels the excursion.
  assert(c.action_c_boundary_ready_);
  c.phase(C::ACTION_C_HORIZONTAL_PULL); c.action_c_pulse_started_=true;
  c.action_c_command_thrust_n_=3; c.inchworm_thrust_={{3,0,0,3}};
  assert(!c.tick(6,9)); assert(!c.tick(6.6,9));
  c.phase(C::ACTION_C_EVALUATE);
  c.action_c_command_thrust_n_=0; c.inchworm_thrust_.fill(0);
  assert(!c.tick(6.7,9)); assert(!c.tick(7.5,9)); // Let recoil complete.
  c.phase(C::ACTION_C_REGRIP);
  assert(!c.tick(7.6,9)); assert(!c.tick(8.5,9));
  c.phase(C::SETTLE);
  assert(!c.drive(8.6,.9,9)); // Stable 9 deg must not recover or deadlock.
  assert(c.action_c_boundary_ready_);
  // A returning transient completes normally, without invoking recovery.
  c.phase(C::SETTLE);
  assert(!c.tick(10,9)); assert(!c.tick(10.3,7));
  assert(!c.tick(10.51,7)); assert(c.action_c_boundary_ready_);
  // New candidate pressure must settle; invalid pressures never pass.
  c.phase(C::ACTION_C_PREPARE); c.action_c_bottom_hold_kpa_=25;
  assert(!c.tick(11,7)); assert(!c.tick(12,7));
  assert(!c.action_c_boundary_ready_);
  c.bottom_pressure_kpa_=25; assert(!c.tick(12.1,7));
  assert(!c.tick(12.61,7)); assert(c.action_c_boundary_ready_);
  c.arms_ready=false; assert(!c.tick(13,7)); assert(!c.action_c_boundary_ready_);
  c.arms_ready=true; c.bottom_pressure_kpa_=NAN;
  assert(!c.tick(14,7)); assert(!c.action_c_boundary_ready_);
  // Rate guard bypasses ordinary pressure/settle waits at thrust-off.
  assert(c.tick(14.1,1,.8));
  c.bottom_pressure_kpa_=25;
  assert(!c.tick(15,7)); assert(!c.tick(15.51,7));
  assert(c.action_c_boundary_ready_);
  assert(!c.tick(1,7)); assert(!c.action_c_boundary_ready_); // Clock reset.
  c.phase(C::SETTLE);assert(c.drive(20,1.,9,3));
  c.phase(C::SETTLE);assert(c.drive(22,1.,-9,-3)); // Same for negative roll.
  c.phase(C::SETTLE);assert(!c.drive(24,1.,12,-3));assert(c.action_c_boundary_ready_);
  c.phase(C::ACTION_C_PREPARE);assert(!c.drive(26,1.,-11));assert(c.action_c_boundary_ready_);
  c.phase(C::SETTLE);c.arms_ready=false;assert(c.tick(28,15)); // Hard limit bypasses settling.
  std::puts("PASS: stable +/-roll continues, outward growth confirms recovery, improvement, inflation/pulse isolation, hard/rate limits, clock reset");
}
'''.replace('CONDITIONS', conditions).replace('HANDOFF', entry)
    with tempfile.TemporaryDirectory(prefix='hugmy_phase_recovery_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
