#!/usr/bin/env python3
"""Offline regression of production recovery functions; no actuator outputs.

Run: python3 robots/hugmy/test/test_roll_only_correction.py
Requires g++ only. Geometry and sensor inputs are deterministic test doubles.
"""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    config = (root / 'config/attitude_pressure_controller.yaml').read_text()
    assert 'correction_roll_only: true' in config
    assert 'pnh_.param("correction_roll_only", correction_roll_only_, true)' in source

    def function(name, result):
        start = source.index('AttitudePressureController::' + name + '(')
        opening = source.index('{', start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        return result + ' ' + source[start:end]

    functions = '\n'.join(function(name, result) for name, result in [
        ('correctionRollOnly', 'bool'),
        ('desiredThrustMoment', 'AttitudePressureController::Vector2'),
        ('allocateCorrectionThrustFromAngles', 'AttitudePressureController::PressureArray'),
        ('allocatePressure', 'AttitudePressureController::PressureArray'),
    ])
    start = source.index('const bool roll_only_recovery =')
    end = source.index('if (action_c_optimizer_trial_active_', start)
    metrics = source[start:end]
    # Both PREPARE and THRUST_CORRECTION project the gyro onto the axis
    # derived from the masked desired moment. Their stop gates use the same
    # masked attitude/rate metrics exercised below.
    assert source.count('body_angular_velocity_[1] * correction_axis_[1]') == 2
    assert 'correction_axis_ = {{desired_moment[0] / desired_moment_norm,' in source
    assert 'desired_moment[1] / desired_moment_norm' in source
    code = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <hugmy/control/continuous_roll_recovery.h>
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
class AttitudePressureController {
public:
  using Vector2=std::array<double,2>;
  using Vector3=std::array<double,3>;
  using PressureArray=std::array<double,4>;
  static constexpr size_t ARM_COUNT=4;
  bool correction_roll_only_=true, action_c_resume_pending_=false;
  bool gravity_compensation_=false;
  bool action_c_continuous_recovery_enabled_=false, inchworm_action_c_enabled_=false;
  bool inchworm_action_d_enabled_=false, fixed_arm_test_enabled_=false;
  double roll_=0., pitch_=0., target_roll_=0., target_pitch_=0.;
  double roll_moment_kp_=.1, pitch_moment_kp_=.1;
  double roll_moment_kd_=.015, pitch_moment_kd_=.015, mass_kg_=1.;
  double correction_min_thrust_n_=0., correction_max_thrust_n_=3.;
  double base_pressure_kpa_=0., maximum_pressure_kpa_=2.;
  double allocation_pressure_epsilon_kpa_=.25;
  Vector3 body_angular_velocity_{}, center_of_mass_{{.1,.2,0.}};
  Vector3 contact_position_{}, gravity_body_{{0.,0.,-9.8}};
  static double wrapAngle(double v) {return std::atan2(std::sin(v),std::cos(v));}
  static Vector3 cross(const Vector3& a,const Vector3& b) {
    return {{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}};
  }
  Vector3 rotorPositionFromAngle(size_t i,double) const {
    const std::array<Vector3,4> p{{{{.2,.1,0.}},{{-.1,.1,0.}},
                                  {{-.2,-.1,0.}},{{.1,-.1,0.}}}};
    return p[i];
  }
  Vector3 rotorDirectionFromAngle(size_t,double) const {return {{0.,0.,1.}};}
  Vector2 thrustMoment(const PressureArray& p) const {
    return {{p[0]+p[1]-p[2]-p[3], -2*p[0]+p[1]+2*p[2]-p[3]}};
  }
  bool correctionRollOnly() const;
  Vector2 desiredThrustMoment() const;
  PressureArray allocateCorrectionThrustFromAngles(const Vector2&,const PressureArray&) const;
  PressureArray allocatePressure(const Vector2&) const;
  Vector2 metrics() const {
    const double roll_error=wrapAngle(target_roll_-roll_);
    const double pitch_error=wrapAngle(target_pitch_-pitch_);
    METRICS
    return {{attitude_error,angular_rate}};
  }
};
FUNCTIONS
int main() {
  AttitudePressureController c;
  // Pitch alone cannot trigger correction, inhibit completion or trip the
  // recovery gyro threshold. Locomotion's separate tilt guard is unchanged.
  c.pitch_=1.; c.body_angular_velocity_[1]=2.;
  auto m=c.metrics(); assert(m[0]==0. && m[1]==0.);
  assert(c.desiredThrustMoment()[1]==0.);
  c.roll_=.02; assert(c.metrics()[0]<.04); // Finished despite pitch.
  c.roll_=.2; c.body_angular_velocity_[0]=.03;
  m=c.metrics(); assert(m[0]>.12 && m[1]==.03);
  auto desired=c.desiredThrustMoment();
  auto thrust=c.allocateCorrectionThrustFromAngles(desired,{});
  assert(thrust[0]==0. && thrust[1]==0. && thrust[2]>0. && thrust[2]==thrust[3]);
  // Large opposite pitch error/rate leaves the roll command unchanged.
  c.pitch_=-1.3; c.target_pitch_=.2; c.body_angular_velocity_[1]=-3.;
  assert(c.metrics()==m && c.desiredThrustMoment()==desired);
  assert(c.allocateCorrectionThrustFromAngles(c.desiredThrustMoment(),{})==thrust);
  auto contaminated=desired; contaminated[1]=100.;
  assert(c.allocateCorrectionThrustFromAngles(contaminated,{})==thrust);
  assert(c.allocatePressure(desired)==c.allocatePressure(contaminated));
  c.roll_=-.2;
  thrust=c.allocateCorrectionThrustFromAngles(c.desiredThrustMoment(),{});
  assert(thrust[0]>0. && thrust[0]==thrust[1] && thrust[2]==0. && thrust[3]==0.);
  // Gravity compensation must not reintroduce a pitch command on reallocation.
  c.gravity_compensation_=true;
  assert(c.desiredThrustMoment()[1]==0.);
  c.correction_roll_only_=false;
  assert(std::abs(c.desiredThrustMoment()[1])>.01);
  assert(c.metrics()[0]>.2 && c.metrics()[1]>2.);
  c.gravity_compensation_=false;
  thrust=c.allocateCorrectionThrustFromAngles(c.desiredThrustMoment(),{});
  assert(thrust[0]==0. && thrust[1]>0. && thrust[1]==thrust[2] && thrust[3]==0.);
  // Optional two-axis mode still preserves Action-C's roll-only handoff.
  c.action_c_resume_pending_=true;
  assert(c.correctionRollOnly() && c.desiredThrustMoment()[1]==0.);
  thrust=c.allocateCorrectionThrustFromAngles(c.desiredThrustMoment(),{});
  assert(thrust[0]>0. && thrust[0]==thrust[1] && thrust[2]==0. && thrust[3]==0.);
  // New continuous C uses Euler roll derivative, NOT a pitch-error target.
  c.action_c_continuous_recovery_enabled_=true;c.inchworm_action_c_enabled_=true;
  c.roll_=.2;c.pitch_=.4;c.body_angular_velocity_={{.1,.2,.3}};
  m=c.metrics();
  assert(std::abs(m[1]-std::abs(hugmy::recoveryRollRate(.2,.4,c.body_angular_velocity_)))<1e-10);
  c.target_pitch_=1.2;assert(c.metrics()==m);
  c.inchworm_action_d_enabled_=true;assert(c.metrics()[1]==.1); // D untouched
  std::puts("PASS: roll-only recovery metrics, gravity mask, rotor/pressure allocation and two-axis fallback");
}
'''.replace('METRICS', metrics).replace('FUNCTIONS', functions)
    with tempfile.TemporaryDirectory(prefix='hugmy_roll_only_') as directory:
        cpp = pathlib.Path(directory) / 'test.cpp'
        cpp.write_text(code)
        binary = pathlib.Path(directory) / 'test'
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
