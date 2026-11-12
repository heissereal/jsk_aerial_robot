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
  std::array<double,4> last_pressure_{{30.,30.,30.,30.}};
  std::array<double,4> arm_yaw_{{-3*M_PI/4,-M_PI/4,M_PI/4,3*M_PI/4}};
  hugmy::CircularGraspEstimate recovery_circle_event_estimate_;
  hugmy::PressureThrustBendModel bend_model_;
  hugmy::RollRecoveryConfig roll_recovery_config_;
  double recovery_circle_min_signed_lever_m_=.005;
  double recovery_geometry_max_support_rotor_distance_m_=.5;
  double recovery_circle_release_penalty_n_=.1;
  double recovery_circle_contact_pressure_kpa_=10.;
  double pressure_tolerance_kpa_=1.;
  double recovery_extension_prediction_sec_=.3;
  double recovery_rotor_cant_rad_=6.5*M_PI/180.;
  double recovery_release_pressure_kpa_=0.;
  double bend_model_thrust_offset_n_=1.8;
  double recovery_unsafe_roll_rad_=45.*M_PI/180.;
  double recovery_extension_response_sec_=.5;
  double recovery_unified_extension_target_bend_rad_=0.;
  double recovery_unified_thrust_step_n_=.1;
  double recovery_unified_pair_switch_penalty_n_=.2;
  double recovery_grasp_probe_initial_thrust_n_=.8;
  double recovery_circle_side_margin_rad_=5.*M_PI/180.;
  double mock_rotor_depth_=2.0;
  double roll_=-.2,pitch_=0.;
  Vector3 body_angular_velocity_{{0.,0.,0.}};
  double measuredArmBendAngle(size_t)const{return 1.8;}
  Vector3 armContactPointFromAngle(size_t arm,double)const{
    return {{0.,arm<2?.035:-.035,-.07}};
  }
  Vector3 rotorPositionFromAngle(size_t arm,double bend)const{
    return {{0.,arm<2?.08:-.08,.02*std::cos(bend)}};
  }
  double rotorDepthAngleAbout(size_t,double,const Vector3&)const{
    return mock_rotor_depth_;
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
      double,double,const hugmy::recovery::Vec3&,int);
};
METHOD
int main() {
  AttitudePressureController c;
  c.roll_recovery_config_.limit = 5.5;
  c.recovery_circle_event_estimate_.valid = true;
  c.recovery_circle_event_estimate_.radius = 0.04;
  const auto active = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,0.,-.07}},-1);
  assert(active.valid);
  assert(active.candidates[active.index].contact_feasible);
  assert(active.candidates[0].support_vertex_count == 4);
  assert(active.candidates[1].support_vertex_count == 2);
  // Both pairs are geometrically below the side, but their actual thrust
  // vectors turn the object in opposite directions. Only the pair whose
  // centre-based cross product has the requested sign may run a probe.
  assert(active.candidates[0].below_object_side);
  assert(active.candidates[2].below_object_side);
  assert(active.candidates[0].signed_center_lever < 0.0);
  assert(active.candidates[2].signed_center_lever > 0.0);
  assert(!active.candidates[0].conditional_grasp_probe);
  assert(active.candidates[2].conditional_grasp_probe);
  assert(active.alternatives[active.index].support_vertex_count >= 2);
  assert(std::isfinite(active.alternatives[active.index].worst_support_body[1]));
  assert(std::isfinite(active.candidates[1].extension_min_thrust));
  assert(active.candidates[1].extension_min_thrust > 0.0);
  assert(active.candidates[1].extension_min_thrust ==
         active.candidates[3].extension_min_thrust);
  c.measured_pressure_[0] = 0.;
  const auto reduced_support = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,0.,-.07}},active.index);
  assert(!reduced_support.candidates[0].contact_feasible);
  c.measured_pressure_[0] = 30.;
  c.mock_rotor_depth_ = 1.0;
  const auto above_side = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,0.,-.07}},-1);
  assert(!above_side.candidates[0].conditional_grasp_probe);
  assert(!above_side.candidates[2].conditional_grasp_probe);
  c.mock_rotor_depth_ = 2.0;
  const auto reverse_moment = c.evaluateUnifiedRecoveryCandidates(
      -.2,-.02,{{0.,0.,-.07}},-1);
  assert(reverse_moment.candidates[0].signed_center_lever > 0.0);
  assert(reverse_moment.candidates[2].signed_center_lever < 0.0);
  assert(reverse_moment.candidates[0].conditional_grasp_probe);
  assert(!reverse_moment.candidates[2].conditional_grasp_probe);
  const auto invalid = c.evaluateUnifiedRecoveryCandidates(
      .2,.02,{{0.,NAN,-.07}},-1);
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
