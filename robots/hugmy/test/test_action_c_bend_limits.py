#!/usr/bin/env python3
"""Offline production-helper regression: shallow starts and frozen ON limits."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    src = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    prepare = src[src.index('case ACTION_C_PREPARE:'):src.index('case ACTION_C_HORIZONTAL_PULL:')]
    on = src[src.index('case ACTION_C_HORIZONTAL_PULL:'):src.index('case ACTION_C_EVALUATE:')]
    pull = src[src.index('void AttitudePressureController::updateActionCPressurePull('):
               src.index('double AttitudePressureController::actionCBottomTarget()')]
    assert 'actionCStartBendReady(' in prepare
    assert 'action_c_min_safe_angle_rad_' not in prepare
    assert on.index('actionCStartBendReady(') < on.index('action_c_pulse_started_ = true;')
    for code in (on, pull):
        assert 'actionCPulseBendFloor(' in code
        assert 'action_c_pulse_start_angle_rad_[arm]' in code
        for guard in ('OVER_EXTENSION', 'LOW_BEND', 'EXTENSION_RATE', 'PAIR_MISMATCH'):
            assert 'ACTION_C_' + guard in code
    code = r'''
#include <hugmy/control/action_c_feedback.h>
#include <cassert>
#include <cmath>
int main() {
  const double d = M_PI / 180.;
  auto ready = [=](double bend) {
    return hugmy::actionCStartBendReady(bend*d,40*d,30*d,8*d,2*d);
  };
  auto floor = [=](double start) {
    return hugmy::actionCPulseBendFloor(start*d,55*d,30*d,8*d,2*d)/d;
  };
  // Bag: initial/released front arm3 was 52, 45, and 41 deg.
  for(double start : {40.,41.,45.,52.,58.,68.,90.}) {
    assert(ready(start));
    assert(start - 8. > floor(start)); // goal can precede the bend stop
    assert(floor(start) >= 30. - 1e-10);
  }
  assert(!ready(39.9) && !ready(0.) && !ready(NAN) && !ready(INFINITY));
  assert(std::abs(floor(52.)-42.)<1e-10);
  assert(std::abs(floor(45.)-35.)<1e-10);
  assert(std::abs(floor(40.)-30.)<1e-10);
  assert(std::abs(floor(68.)-55.)<1e-10);
  assert(std::abs(floor(90.)-55.)<1e-10);
  // Increasing target must require more starting headroom, not relax hard floor.
  assert(!hugmy::actionCStartBendReady(41*d,40*d,30*d,12*d,2*d));
  assert(!hugmy::actionCStartBendReady(50*d,NAN,30*d,8*d,2*d));
  assert(!hugmy::actionCStartBendReady(50*d,40*d,30*d,8*d,0.));
  assert(std::isinf(hugmy::actionCPulseBendFloor(NAN,55*d,30*d,8*d,2*d)));
  assert(std::isinf(hugmy::actionCPulseBendFloor(52*d,20*d,30*d,8*d,2*d)));
  // Freeze per-arm limits at ON; neither follows subsequent extension.
  const double a_floor=floor(52.), b_floor=floor(68.);
  assert(44.>a_floor && 42.<=a_floor+1e-10);
  assert(60.>b_floor && 54.<b_floor);
  // Role reversal uses the new pair's individual starts, not old front limits.
  assert(floor(132.)==floor(128.));
}
'''
    with tempfile.TemporaryDirectory(prefix='hugmy_bend_limits_') as tmp:
        binary = str(pathlib.Path(tmp) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(root / 'include'), '-x', 'c++', '-', '-o', binary],
                       input=code, text=True, check=True)
        subprocess.run([binary], check=True)
    print('Action C separated start/pulse bend limits: PASS')


if __name__ == '__main__':
    main()
