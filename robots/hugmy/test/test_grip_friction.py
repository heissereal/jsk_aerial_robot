#!/usr/bin/env python3
"""Pure contact-law regression; not a substitute for MuJoCo validation."""
from pathlib import Path
import subprocess
import tempfile

code = r'''
#include <hugmy/mujoco/grip_friction.h>
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  hugmy::GripFriction c;
  const hugmy::GripFriction::Vec n{{0,0,1}}, zero{};
  auto f=c.force({{.01,0,0}},n,.01,180,4,2.88);
  assert(f[0]<0 && f[1]==0 && f[2]==0);
  f=c.force(zero,n,.01,180,4,2.88);
  assert(f[0]<0); // stored static shear, unlike velocity-only friction
  for(int j=0;j<10000;++j) {
    f=c.force({{1.,.5,0}},n,.001,180,4,2.88);
    assert(std::hypot(f[0],f[1])<=2.88000001);
    assert(f[0]+.5*f[1]<0); // sliding force resists slip
  }
  f=c.force(zero,n,.001,180,4,.1);
  assert(std::hypot(f[0],f[1])<=.10000001); // reduced pressure lowers bound
  c.force(zero,n,.001,180,4,0.);
  f=c.force(zero,n,.001,180,4,2.88);
  assert(f==zero); // released contact has no stored force
  c.force({{.01,0,0}},n,.01,180,4,2.88);
  f=c.force(zero,{{1,0,0}},.01,180,4,2.88);
  assert(f[0]==0); // old shear cannot become a normal force
  c.reset();
  f=c.force({{.01,0,0}},n,.01,0,4,2.88);
  assert(std::abs(f[0]+.04)<1e-12);
  f=c.force(zero,n,.01,0,4,2.88);
  assert(f==zero); // explicit legacy comparison
  std::cout<<"PASS: static shear, Coulomb limit, vent/release, tangent transport, legacy mode\n";
}
'''
with tempfile.TemporaryDirectory() as directory:
    source = Path(directory) / 'test.cpp'
    source.write_text(code)
    executable = Path(directory) / 'test'
    subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra',
                    '-I'+str(Path(__file__).resolve().parents[1]/'include'),
                    str(source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
