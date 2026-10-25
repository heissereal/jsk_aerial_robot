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
assert module.contact_regrasped([15.,15.,30.,30.],[1.,1.,1.,1.])
assert not module.contact_regrasped([13.,15.,30.,30.],[1.,1.,1.,1.])
assert not module.contact_regrasped([15.,15.,30.,30.],[1.,0.,0.,1.])
root=Path(__file__).resolve().parents[1]
fixture=ET.parse(str(root/'launch/hybrid_high_friction_mujoco.launch'))
arguments={a.get('name'):a.get('value') for a in fixture.findall('./include/arg')}
assert arguments['simulation']=='true' and arguments['real_machine']=='false'
assert arguments['mujoco']=='true' and arguments['fixed_arms']=='false'
assert arguments['mujoco_model']=='$(arg mujoco_model)'
assert arguments['mujoco_forearm_proximal_width']=='$(arg forearm_width)'
assert arguments['mujoco_forearm_distal_width']=='$(arg forearm_width)'
assert arguments['mujoco_forearm_proximal_depth']=='$(arg forearm_depth)'
assert arguments['mujoco_forearm_distal_depth']=='$(arg forearm_depth)'
assert arguments['mujoco_grip_friction']=='$(arg grip_friction)'
assert arguments['mujoco_surface_friction']=='$(arg surface_friction)'
fixture_args={a.get('name'):a.get('default') for a in fixture.findall('./arg')}
assert fixture_args['grip_friction']=='4.5'
assert fixture_args['surface_friction']=='4.0'
assert fixture_args['tangential_stiffness']=='1200.0'
assert fixture_args['forearm_width']=='0.07625'
assert fixture_args['forearm_depth']=='0.059075'
assert not fixture.findall('.//node') # no automated commands on the user's master
src=(root/'src/control/hybrid_locomotion.cpp').read_text()
assert 'input.directional_pitch=hybrid_directional_pitch_sign_*' in src
assert 'input.pitch_rate=hybrid_directional_pitch_sign_*' in src
bringup=(root/'launch/bringup.launch').read_text()
assert '<arg name="hybrid_directional_pitch_sign" default="-1.0"/>' in bringup
controller_launch=(root/'launch/attitude_pressure_controller.launch').read_text()
assert '<arg name="hybrid_directional_pitch_sign" default="-1.0" />' in controller_launch
assert ('<arg name="hybrid_lift_unload_pair_mismatch_rad" '
        'default="0.17453292519943295" />') in controller_launch
assert 'hybrid_lift_unload_pair_memory_sec' in controller_launch
assert 'roll_rate_limit_confirmed' in src
assert 'clock-hybrid_roll_rate_limit_since_>=hybrid_roll_rate_confirm_sec_' in src
assert 'if(dt==0.) return;' in src
assert 'if(!s.fresh || dt<0. || dt>.1 || clock<hybrid_task_start_)' in src
assert 'reverse_reference_x = hybrid_start_mean_valid_' in src
assert 'locomotion_position[0]-reverse_reference_x' in src
validator=Path(module.__file__).read_text()
assert "parser.add_argument('--unload-pitch-deg', type=float, default=4." in validator
assert "parser.add_argument('--lateral-limit', type=float, default=.03" in validator
assert "abs(result['lateral_displacement'])<args.lateral_limit" in validator
assert "abs(result['lateral_displacement'])<.02" in validator  # grasp-only negative control remains strict
assert "inflated_terminal=abs(result['final_bottom_kpa']-args.bottom)<=3." in validator
assert "empty_terminal=abs(result['final_bottom_kpa'])<=3." in validator
assert "terminal_roll_limit=15. if inflated_terminal else 8." in validator
assert "result['validation_progress_mm']" in validator
assert "result['trajectory_settings']" in validator
assert "result['max_rear_reverse_thrust_n']" in validator
assert "result['rear_positive_pwm_confirmed']" in validator
assert "result['rear_positive_started']" in validator
assert "result['paired_extension_memory_seen']" in validator
assert "result['motion2_to_3_seen']" in validator
assert "pair[0]<.75-1e-6 and pair[1]<.75-1e-6" in validator
print('PASS: noisy stationary endpoint accepted; drift, incomplete/stale endpoints rejected')
