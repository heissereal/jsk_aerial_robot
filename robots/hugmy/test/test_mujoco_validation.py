#!/usr/bin/env python3
"""Noise-aware endpoint criteria must reject drift and missing data."""
import importlib.util
from pathlib import Path
import random
import xml.etree.ElementTree as ET

spec=importlib.util.spec_from_file_location('validator',Path(__file__).with_name('validate_hybrid_mujoco.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rng=random.Random(17)
poses=[(k*.01,[.02+rng.gauss(0,.001),0.,.25]) for k in range(301)]
assert module.settled_x_summary(poses,3.)[2]
drift=[(t,[p[0]+.005*t,0.,.25]) for t,p in poses]
assert not module.settled_x_summary(drift,3.)[2]
assert not module.settled_x_summary(poses[200:],3.)[2]
assert not module.settled_x_summary(poses[:200],3.)[2]
assert not module.settled_x_summary([],3.)[2]
cycle_logs=[(i,'Hybrid C/D result: completed=1 safe=1 retained %.2f mm' % x)
            for i,x in enumerate([2.,1.5,.2,-1.,3.,2.,1.])]
assert module.forward_cycle_summary(cycle_logs)==([2.,1.5,.2,-1.,3.,2.,1.],3)
cycle_logs.insert(6,(5.5,'Hybrid C/D result: completed=0 safe=0 retained 20.0 mm'))
assert module.forward_cycle_summary(cycle_logs)[1]==2
assert module.forward_cycle_summary([(0.,'powered x 50 mm')])==([],0)
root=Path(__file__).resolve().parents[1]
fixture=ET.parse(str(root/'launch/hybrid_high_friction_mujoco.launch'))
arguments={a.get('name'):a.get('value') for a in fixture.findall('./include/arg')}
assert arguments['simulation']=='true' and arguments['real_machine']=='false'
assert arguments['mujoco']=='true' and arguments['fixed_arms']=='false'
assert not fixture.findall('.//node') # no automated commands on the user's master
src=(root/'src/control/hybrid_locomotion.cpp').read_text()
assert 'std::abs(body_angular_velocity_[0])>=hybrid_config_.roll_rate_limit' in src
validator=Path(module.__file__).read_text()
assert "parser.add_argument('--lateral-limit', type=float, default=.03" in validator
assert "abs(result['lateral_displacement'])<args.lateral_limit" in validator
assert "abs(result['lateral_displacement'])<.02" in validator  # grasp-only negative control remains strict
print('PASS: noisy stationary endpoint accepted; drift, incomplete/stale endpoints rejected')
