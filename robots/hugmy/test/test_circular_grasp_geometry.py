#!/usr/bin/env python3
"""Compile and exercise the circle-constrained grasp estimator."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
code = r'''
#include <hugmy/control/circular_grasp_geometry.h>
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
  constexpr double pi = 3.14159265358979323846;
  const double radius = 0.050;
  const std::array<double,2> surface{{0.0, -0.070}};
  const std::array<double,2> gravity{{0.0, -9.81}};
  std::array<std::array<double,2>,4> points{};
  // Two samples on each side.  60 deg lies above the side; 120 below it.
  const std::array<double,4> angle{{60*pi/180, 120*pi/180,
                                   120*pi/180, 60*pi/180}};
  for (size_t i=0;i<4;++i) {
    const double side = i<2 ? 1.0 : -1.0;
    points[i][0] = side * radius * std::sin(angle[i]);
    points[i][1] = surface[1] - radius + radius * std::cos(angle[i]);
  }
  std::array<bool,4> active{{true,true,true,true}};
  const auto result = hugmy::estimateCircularGrasp(
      points, active, surface, gravity, 0.025, 0.15, 0.015);
  assert(result.valid && result.samples==4);
  assert(std::abs(result.radius-radius)<1e-12);
  assert(std::abs(result.center_y)<1e-12);
  assert(std::abs(result.center_z+0.120)<1e-12);
  assert(result.rms<1e-12 && result.confidence>.99);
  for(size_t i=0;i<4;++i)
    assert(std::abs(result.depth_angle[i]-angle[i])<1e-12);

  const std::array<double,4> no_projection{{NAN,NAN,NAN,NAN}};
  // One pair below the side: use it.
  auto choice=hugmy::chooseCircularRecoveryPair(
      {{100*pi/180,100*pi/180,70*pi/180,70*pi/180}},
      no_projection,1);
  assert(choice.valid && choice.pair_index==0 &&
         !choice.used_gravity_or_roll_fallback);
  // Both above the side: use the deeper pair.
  choice=hugmy::chooseCircularRecoveryPair(
      {{70*pi/180,72*pi/180,80*pi/180,82*pi/180}},
      no_projection,0);
  assert(choice.valid && choice.pair_index==1);
  // Nearly equal reconstructed depths are not enough to override the
  // independently measured body-roll side. This reproduces the +21-deg
  // hardware case which had only a 0.8-deg pair-depth separation.
  choice=hugmy::chooseCircularRecoveryPair(
      {{69.4*pi/180,69.6*pi/180,68.6*pi/180,68.8*pi/180}},
      no_projection,1,0.5*pi,5*pi/180);
  assert(choice.valid && choice.pair_index==1 &&
         choice.used_gravity_or_roll_fallback);
  // Both below the side: keep the deeper pair as anchor and actuate the
  // shallower pair.
  choice=hugmy::chooseCircularRecoveryPair(
      {{115*pi/180,117*pi/180,100*pi/180,102*pi/180}},
      no_projection,0);
  assert(choice.valid && choice.pair_index==1);
  // A single arm angle never decides the pair. Complete contact/gravity
  // evidence wins; without it, the roll-derived lower pair is used.
  choice=hugmy::chooseCircularRecoveryPair(
      {{100*pi/180,NAN,70*pi/180,70*pi/180}},
      {{.8,.7,-.2,-.1}},1);
  assert(choice.valid && choice.pair_index==0 &&
         choice.used_gravity_or_roll_fallback);
  choice=hugmy::chooseCircularRecoveryPair(
      {{100*pi/180,NAN,70*pi/180,70*pi/180}},
      no_projection,1);
  assert(choice.valid && choice.pair_index==1 &&
         choice.used_gravity_or_roll_fallback);

  // Samples from one side only cannot identify diameter in this constrained
  // two-sided gripper geometry.
  active={{true,true,false,false}};
  assert(!hugmy::estimateCircularGrasp(
      points,active,surface,gravity,0.025,0.15,0.015).valid);

  // The same geometry remains valid after rotating the gravity/contact frame.
  const double roll=20*pi/180;
  const std::array<double,2> rotated_gravity{{-std::sin(roll),-std::cos(roll)}};
  const std::array<double,2> tangent{{std::cos(roll),-std::sin(roll)}};
  const std::array<double,2> down{{-std::sin(roll),-std::cos(roll)}};
  for(size_t i=0;i<4;++i) {
    const double side=i<2?1.0:-1.0;
    points[i][0]=surface[0]+radius*down[0]+radius*(
        side*std::sin(angle[i])*tangent[0]-std::cos(angle[i])*down[0]);
    points[i][1]=surface[1]+radius*down[1]+radius*(
        side*std::sin(angle[i])*tangent[1]-std::cos(angle[i])*down[1]);
  }
  active={{true,true,true,true}};
  const auto rotated=hugmy::estimateCircularGrasp(
      points,active,surface,rotated_gravity,0.025,0.15,0.015);
  assert(rotated.valid && std::abs(rotated.radius-radius)<1e-12);
  std::puts("PASS: constrained circle radius, depth side, coverage and roll frame");
}
'''

with tempfile.TemporaryDirectory(prefix='hugmy_circle_geometry_') as tmp:
    binary = str(pathlib.Path(tmp) / 'test')
    subprocess.run([
        'g++', '-std=c++14', '-Wall', '-Wextra',
        '-I', str(root / 'include'), '-x', 'c++', '-', '-o', binary
    ], input=code, text=True, check=True)
    subprocess.run([binary], check=True)
