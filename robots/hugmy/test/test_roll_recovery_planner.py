#!/usr/bin/env python3
"""Exercise the standalone four-candidate roll planner without ROS or hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
code = r'''
#include <hugmy/control/roll_recovery_planner.h>
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace hugmy::recovery;

static bool near(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) < tol;
}

static Candidate candidate(std::size_t pair, bool release, double arm_y,
                           double extension_thrust = 0.0) {
  Candidate c;
  c.pair = pair;
  c.release = release;
  c.contact_feasible = true;
  c.rotors[0].position = {{0.0, arm_y, 0.0}};
  c.rotors[1].position = {{0.0, arm_y, 0.0}};
  c.rotors[0].unit_force = {{0.0, 0.0, 1.0}};
  c.rotors[1].unit_force = {{0.0, 0.0, 1.0}};
  c.extension_min_thrust = extension_thrust;
  c.release_fraction = release ? 1.0 : 0.0;
  c.transition_seconds = release ? 0.3 : 0.0;
  return c;
}

int main() {
  // Jacobian transpose and r x F are identical in planar roll.
  assert(near(rollLever({{0.0, 0.07, 0.02}}, {{0.0, 0.0, 1.0}},
                       {{0.0, 0.02, 0.01}}), 0.05));
  const auto q90 = Quaternion{std::cos(M_PI/4), std::sin(M_PI/4), 0.0, 0.0};
  Pose start;start.position = {{0.2, 0.3, -0.1}};start.rotation = q90;
  const Vec3 body_center{{0.0, 0.05, -0.1}};
  const Vec3 world_center = objectCenterWorld(body_center, start);
  const Vec3 unchanged = objectCenterBody(world_center, start);
  for (int a = 0; a < 3; ++a) assert(near(unchanged[a], body_center[a]));
  Pose moved; moved.position = {{0.1, -0.02, 0.03}};
  const Vec3 moved_center = objectCenterBody(world_center, moved);
  assert(near(moved_center[0], world_center[0] - moved.position[0]));
  assert(!near(moved_center[1], body_center[1]));

  const Vec3 center = instantaneousCenter({{0., 1., 2.}},
      {{0., -0.2, 0.4}}, 2.0);
  assert(near(center[1], 0.8) && near(center[2], 1.9));
  assert(std::isnan(instantaneousCenter({{0., 0., 0.}},
      {{0., 1., 1.}}, 0.0)[1]));

  // Four candidates, no hardcoded diameter or chosen arm pair.
  std::array<Candidate, 4> small{{
      candidate(0, false, 0.12), candidate(0, true, 0.07, 1.0),
      candidate(1, false, -0.06), candidate(1, true, 0.03, 1.0)}};
  Config cfg; cfg.thrust_limit = 4.0; cfg.release_penalty = 0.2;
  cfg.transition_penalty = 0.2;
  const auto small_choice = select(small, 0.2, cfg);
  assert(small_choice.valid && small_choice.index == 0);
  assert(small_choice.candidates[small_choice.index].pair == 0);
  assert(!small_choice.candidates[small_choice.index].release);
  assert(near(small_choice.evaluation.roll_thrust, 0.2 / 0.24));
  assert(!small_choice.alternatives[2].feasible); // Wrong-sign moment.

  // Geometry change (a different radius/contact configuration) can make the
  // released 2+3 pair superior WITHOUT an object-specific if statement.
  std::array<Candidate, 4> large{{
      candidate(0, false, 0.008), candidate(0, true, 0.005, 1.2),
      candidate(1, false, -0.05), candidate(1, true, 0.2, 1.0)}};
  const auto large_choice = select(large, 0.2, cfg);
  assert(large_choice.valid && large_choice.index == 3);
  // A shared thrust that both extends and restores is bounded by max,
  // rather than the sum of the two requirements.
  assert(near(large_choice.evaluation.roll_thrust, 0.5));
  assert(near(large_choice.evaluation.command_thrust, 1.0));

  // With a coupled pneumatic arm, more thrust straightens the rotor-mounted
  // arm and simultaneously generates restoring roll. A 0.2-Nm moment needs
  // 1.0 N when the combined lever grows from 0.02 to 0.20 m.
  Candidate coupled = candidate(1, true, 0.01);
  auto geometry_at_thrust = [&](double thrust) {
    auto geometry = coupled.rotors;
    const double y = thrust < 1.0 ? 0.01 : 0.10;
    for (auto& rotor : geometry) rotor.position[1] = y;
    return geometry;
  };
  const auto coupled_eval = evaluateCoupled(coupled, 0.2, cfg,
                                             geometry_at_thrust, 0.1);
  assert(coupled_eval.feasible && near(coupled_eval.command_thrust, 1.0));

  // A released topology must not be accepted below the thrust at which the
  // calibrated pressure/thrust/bend lookup predicts the target extension.
  // The very same 1.4-N command then creates roll moment; it is not added to
  // the lower moment-only requirement.
  coupled.extension_min_thrust = 1.4;
  const auto extension_bounded = evaluateCoupled(
      coupled, 0.2, cfg, geometry_at_thrust, 0.1);
  assert(extension_bounded.feasible &&
         near(extension_bounded.command_thrust, 1.4));

  // A finite switching margin prevents toggling between close candidates.
  auto close = small;
  close[1] = candidate(0, true, 0.125, 0.0);
  close[1].release_fraction = 0.0;
  close[1].transition_seconds = 0.0;
  const auto without_memory = select(close, 0.2, cfg);
  const auto with_memory = select(close, 0.2, cfg, 0);
  assert(without_memory.index == 1);
  assert(with_memory.index == 0);

  // Urgent extension is based on outward-rate/time-to-risk, not angle alone.
  assert(imminentOutwardFall(0.3, 1.0, 0.7, 0.3, 0.15));
  assert(!imminentOutwardFall(0.6, -0.1, 0.7, 0.3, 0.15));
  assert(!imminentOutwardFall(0.1, 0.1, 0.7, 0.3, 0.15));
  std::puts("PASS: independent roll recovery geometry, candidate costs, hysteresis, risk");
}
'''

with tempfile.TemporaryDirectory(prefix='hugmy_recovery_planner_') as tmp:
    binary = str(Path(tmp) / 'test')
    subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'include'), '-x', 'c++', '-', '-o', binary],
                   input=code, text=True, check=True)
    subprocess.run([binary], check=True)
