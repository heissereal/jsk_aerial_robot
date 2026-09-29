#!/usr/bin/env python3
"""Compile actual pressure gates; offline, no ROS/actuator connection."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    start = source.index('bool AttitudePressureController::pressureReached(')
    pressure = source[start:source.index('double AttitudePressureController::inchwormTravel()', start)]
    start = source.index('const bool recovery_bottom_ready =')
    bottom = source[start:source.index(';', start) + 1]
    code = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
class AttitudePressureController {
public:
 using PressureArray=std::array<double,4>;
 static constexpr size_t ARM_COUNT=4;
 enum {GRASP, ACTION_C_PREPARE, REGRIP};
 int state_=ACTION_C_PREPARE;
 bool action_c_front_release_enabled_=true, action_c_reference_pulse_mode_=false;
 double action_c_front_release_tolerance_kpa_=5, pressure_tolerance_kpa_=2;
 PressureArray measured_pressure_{{3.9,30.,30.,4.9}};
 std::array<bool,4> inchworm_front_{{true,false,false,true}};
 bool pressureReached(const PressureArray&) const;
};
PRESSURE
bool deflated(double bottom_pressure_kpa_, bool continuous_c_recovery=false,
              bool recovery_pressure_fresh=false) {
 bool action_c_resume_pending_=true;
 double action_c_bottom_deflation_tolerance_kpa_=5.;
 BOTTOM
 return recovery_bottom_ready;
}
int main() {
 AttitudePressureController c;
 assert(c.pressureReached({{0,30,30,0}}));
 c.measured_pressure_[0]=5.01; assert(!c.pressureReached({{0,30,30,0}}));
 c.measured_pressure_[0]=-5.01; assert(!c.pressureReached({{0,30,30,0}}));
 c.measured_pressure_[0]=NAN; assert(!c.pressureReached({{0,30,30,0}}));
 c.measured_pressure_[0]=3.9;
 c.measured_pressure_[1]=27.9; assert(!c.pressureReached({{0,30,30,0}}));
 c.measured_pressure_={{30.,3.9,4.9,30.}};c.inchworm_front_={{false,true,true,false}};
 assert(c.pressureReached({{30,0,0,30}})); // Reverse direction uses correct front pair.
 c.state_=c.REGRIP; assert(!c.pressureReached({{30,0,0,30}}));
 c.state_=c.ACTION_C_PREPARE;c.action_c_front_release_enabled_=false;
 assert(!c.pressureReached({{30,0,0,30}}));
 assert(deflated(3.9));assert(deflated(5.));assert(deflated(-5.));
 assert(!deflated(5.01));assert(!deflated(-5.01));assert(!deflated(NAN));
 assert(deflated(12.,true,true)); // Fresh bottom need not reach zero to resume C.
 assert(!deflated(12.,true,false));
 assert(!deflated(0.,true,false)); // Low but stale is not permission to resume.
 puts("PASS: front release arrival, rear/regrip tolerance, reverse pair, bottom 0 +/-5");
}
'''.replace('PRESSURE', pressure).replace('BOTTOM', bottom)
    with tempfile.TemporaryDirectory(prefix='hugmy_release_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-x', 'c++', '-', '-o', binary],
                       input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
