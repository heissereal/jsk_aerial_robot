#!/usr/bin/env python3
"""Offline production-helper regression: shallow starts and frozen ON limits."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    src = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    adapter = (root / 'src/control/hybrid_locomotion.cpp').read_text()
    assert 'actionCPulseBendFloor(s.start_bend[j]' in adapter
    assert 'hybrid_start_bend_=s.bend' in adapter
    assert 'Planner::valid(safety_state,safety_config,pressure_pending)' in adapter
    assert 'hybrid_single_.phase==SinglePass::VentFront' in adapter
    assert 'hybrid_single_.phase==SinglePass::StopForLift' in adapter
    assert 'safety_state.floor.fill(std::max(hybrid_config_.bend_min,action_c_hard_min_bend_rad_))' in adapter
    assert 'if(returning_from_straight_identification)' in adapter
    assert 'safety_state.start_bend=s.bend;' in adapter
    assert 'hybrid_start_bend_=s.bend' in adapter
    code = r'''
#include <hugmy/control/arm_bend_limits.h>
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
