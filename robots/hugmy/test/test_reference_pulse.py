#!/usr/bin/env python3
"""Offline reference-pulse wiring and production gyro-observer regressions."""
import pathlib
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    config = (root / 'config/attitude_pressure_controller.yaml').read_text()
    for setting in ('action_c_reference_pulse_mode: false',
                    'action_c_reference_single_pulse: false',
                    'action_c_pulse_search_enabled: false',
                    'action_c_pressure_pull_enabled: true',
                    'action_c_reference_thrust_n: 3.0',
                    'action_c_front_pulse_pressure_kpa: 5.0',
                    'action_c_reference_off_sec: 1.0'):
        assert setting in config, setting
    for filename in ('bringup.launch', 'attitude_pressure_controller.launch'):
        launch = ET.parse(root / 'launch' / filename).getroot()
        args = {x.attrib['name']: x.attrib.get('default') for x in launch.findall('arg')}
        assert args['action_c_reference_pulse_mode'] == 'false'
        assert args['action_c_reference_single_pulse'] == 'false'
        assert args['action_c_pulse_search_enabled'] == 'false'
        assert args['action_c_reference_thrust_n'] == '3.0'
        assert '$(arg action_c_reference_thrust_n)' in (root / 'launch' / filename).read_text()
    # Integration contracts: reference mode must not silently chase 90 deg,
    # trim the paired waveform, optimize its amplitude, or retry after a guard.
    assert 'if (!action_c_front_release_enabled_ && !action_c_reference_pulse_mode_ && action_c_front_angle_rad_[arm] <' in source
    assert 'if (action_c_reference_pulse_mode_) pulse_roll_trim_n = 0.0;' in source
    assert 'if (action_c_reference_pulse_mode_) angle_thrust_reduction_n[arm] = 0.0;' in source
    assert '(!action_c_reference_pulse_mode_ || pressure_pull_mode) && both_extended' in source
    assert 'extension_guard || pulse_timeout)' in source
    assert 'action_c_pressure_pull_active_ = pressure_pull_mode && !extension_guard;' in source
    assert 'updateActionCPressurePull(now);' in source
    assert 'action_c_individual_extension_enabled_ = false;' in source
    assert 'action_c_online_pressure_adaptation_enabled_ = false;' in source
    assert '(action_c_reference_single_pulse_ || action_c_pulse_overextended_))' in source
    assert '(!action_c_pulse_search_enabled_ || action_c_reference_single_pulse_))) return;' in source
    assert '? action_c_reference_thrust_n_ : action_c_min_thrust_n_' in source
    assert '(action_c_reference_single_pulse_ || action_c_pulse_overextended_ ||' in source
    assert '!action_c_pulse_recoil_complete_ || !pressureReached(inchworm_pressure_))' in source
    assert 'elapsed >= action_c_reference_off_sec_ && pressureReached(inchworm_pressure_)' in source
    assert '? std::max(inchworm_anchor_timeout_sec_,' in source
    code = r'''
#include <hugmy/control/pulse_bend_observer.h>
#include <cassert>
#include <cstdio>
#include <limits>
int main() {
  const double pi=std::acos(-1.);
  // A rigid arm rotating with the body must have zero relative bend rate.
  for (double yaw: {-3*pi/4, -pi/4, pi/4, 3*pi/4}) {
    const double wx=.3, wy=-.2;
    const double local=-std::sin(yaw)*wx+std::cos(yaw)*wy;
    assert(std::abs(hugmy::relativeBendRate(local,wx,wy,yaw))<1e-12);
    assert(std::abs(hugmy::relativeBendRate(local+.4,wx,wy,yaw)-.4)<1e-12);
  }
  // Independent geometric oracle, not a duplicate of the gyro sign formula:
  // rotate a fixed gravity vector into a sensor at +Y bend q, then use the
  // production accelerometer atan2 convention. Its derivative must equal gyro.
  for (double q: {.2, .8, 1.5, 2.2}) for (double rate: {-.4, .4}) {
    const double dt=1.e-5, up_x=.2, up_z=.98;
    auto measured_bend=[&](double angle) {
      double ax=std::cos(angle)*up_x-std::sin(angle)*up_z;
      double az=std::sin(angle)*up_x+std::cos(angle)*up_z;
      return -std::atan2(up_z*ax-up_x*az, up_x*ax+up_z*az);
    };
    const double derivative=(measured_bend(q+rate*dt)-measured_bend(q))/dt;
    assert(std::abs(derivative-hugmy::relativeBendRate(rate,0.,0.,0.))<1.e-8);
  }
  hugmy::PulseBendObserver o;
  o.seed(pi/2, -.2, 0.);
  for (int i=1;i<=30;++i) o.update(-.2,i*.02);
  assert(o.valid && std::abs(o.bend-(pi/2-.12))<1e-12);
  // A repeated transport sample must not integrate twice.
  double bend=o.bend; o.update(-.2,.6); assert(o.valid && o.bend==bend);
  o.update(-.2,.5); assert(!o.valid); // Out of order; fail closed.
  o.seed(pi/2,0.,1.); o.update(0.,1.11); assert(!o.valid); // Gap.
  o.seed(pi/2,0.,2.); o.update(std::numeric_limits<double>::quiet_NaN(),2.02);
  assert(!o.valid);
  o.seed(std::numeric_limits<double>::quiet_NaN(),0.,2.); assert(!o.valid);
  // Trapezoidal integration, followed by flexion after extension.
  o.seed(1.,0.,0.); o.update(-1.,.02);
  assert(std::abs(o.bend-.99)<1e-12);
  o.update(1.,.04); o.update(1.,.06);
  assert(o.valid && std::abs(o.bend-1.01)<1e-12);
  // The observer has NO acceleration input: a 2g transient cannot be
  // interpreted as an instantaneous geometric bend change in this path.
  std::puts("PASS: reference wiring, relative gyro sign/body cancellation, sample faults and integration");
}
'''
    with tempfile.TemporaryDirectory(prefix='hugmy_reference_') as directory:
        cpp = pathlib.Path(directory) / 'test.cpp'
        cpp.write_text('#include <initializer_list>\n' + code)
        binary = pathlib.Path(directory) / 'test'
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra',
                        '-I' + str(root / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
