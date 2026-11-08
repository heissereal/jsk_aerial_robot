#!/usr/bin/env python3
"""Compile the real ROS adapter body with lightweight mocks, without ROS."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'src/control/roll_recovery_planner_adapter.cpp').read_text()
method = source[source.index('hugmy::recovery::Selection\nAttitudePressureController::evaluateUnifiedRecoveryCandidates('):]

code = r'''
#include <hugmy/control/roll_recovery_planner.h>
#include <hugmy/control/continuous_roll_recovery.h>
#include <hugmy/control/circular_grasp_geometry.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#define ROS_INFO(...) ((void)0)
class AttitudePressureController {
public:
  using Vector3 = std::array<double, 3>;
  using PressureArray = std::array<double, 4>;
  std::array<double,4> measured_pressure_{{30.,30.,30.,30.}};
  std::array<double,4> arm_yaw_{{-3*M_PI/4,-M_PI/4,M_PI/4,3*M_PI/4}};
  hugmy::CircularGraspEstimate recovery_circle_event_estimate_;
  hugmy::PressureThrustBendModel bend_model_;
  hugmy::RollRecoveryConfig roll_recovery_config_;
  double recovery_circle_min_signed_lever_m_=.005;
  double recovery_circle_release_penalty_n_=.1;
  double recovery_circle_contact_pressure_kpa_=10.;
  double recovery_extension_prediction_sec_=.3;
  double recovery_rotor_cant_rad_=6.5*M_PI/180.;
  double recovery_release_pressure_kpa_=0.;
  double bend_model_thrust_offset_n_=1.8;
  double recovery_unsafe_roll_rad_=45.*M_PI/180.;
  double recovery_extension_response_sec_=.5;
  double roll_=-.2,pitch_=0.;
  Vector3 body_angular_velocity_{{0.,0.,0.}};
  double measuredArmBendAngle(size_t)const{return 1.8;}
  Vector3 armContactPointFromAngle(size_t arm,double)const{
    return {{0.,arm<2?.035:-.035,-.07}};
  }
  Vector3 rotorPositionFromAngle(size_t arm,double bend)const{
    return {{0.,arm<2?.08:-.08,.02*std::cos(bend)}};
  }
  static Vector3 rotateAboutLocalY(const Vector3& a,double theta) {
    return {{std::cos(theta)*a[0]-std::sin(theta)*a[2], a[1],
             std::sin(theta)*a[0]+std::cos(theta)*a[2]}};
  }
  static Vector3 rotateLocalToBody(const Vector3& a,double theta) {
    return {{std::cos(theta)*a[0]-std::sin(theta)*a[1],
             std::sin(theta)*a[0]+std::cos(theta)*a[1],a[2]}};
  }
  hugmy::recovery::Selection evaluateUnifiedRecoveryCandidates(
      double,double,const hugmy::recovery::Vec3&,bool);
};
METHOD
int main() {
  AttitudePressureController c;
  c.recovery_circle_event_estimate_.valid = true;
  c.recovery_circle_event_estimate_.radius = 0.04;
  const auto shadow = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,0.,-.07}},false);
  assert(shadow.valid);
  assert(shadow.candidates[shadow.index].contact_feasible);
  c.measured_pressure_[0] = 0.;
  c.evaluateUnifiedRecoveryCandidates(.2,.02,{{0.,0.,-.07}},true);
  const auto invalid = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,NAN,-.07}},false);
  assert(!invalid.valid);
  std::puts("PASS: ROS-independent compile/smoke of actual recovery planner adapter");
}
'''.replace('METHOD', method)

with tempfile.TemporaryDirectory(prefix='hugmy_recovery_adapter_') as tmp:
    binary = str(Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra',
                    '-I', str(root / 'include'), '-x', 'c++', '-', '-o', binary],
                   input=code, text=True, check=True)
    subprocess.run([binary], check=True)
