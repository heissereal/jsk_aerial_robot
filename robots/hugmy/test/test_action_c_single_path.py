#!/usr/bin/env python3
"""Single C routing/configuration, and frozen non-C action-body regressions."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import yaml

root=Path(__file__).resolve().parents[1]
controller=(root/'src/control/attitude_pressure_controller.cpp').read_text()
adapter=(root/'src/control/hybrid_locomotion.cpp').read_text()
header=(root/'include/hugmy/control/attitude_pressure_controller.h').read_text()
for removed in ('initializeActionCOptimizer','finishActionCOptimizerTrial',
                'beginActionCOptimizerTrial','selectNextActionCCandidate',
                'updateActionCPressurePull','action_c_pressure_pull_ceiling_n_',
                'hybrid_locomotion_enabled_', 'hybrid_locomotion_shadow_',
                'action_c_reference_pulse_mode_', 'action_c_optimizer_trial_active_'):
    assert removed not in controller+adapter+header,removed
assert 'case ACTION_C_PREPARE:' not in controller
assert 'case ACTION_C_HORIZONTAL_PULL:' not in controller
assert 'return inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;' in adapter
assert 'updateHybridLocomotion(now, dt);\n      return;' in controller
assert 'startHybridLocomotion(now);\n      return;' in controller
assert 'Hybrid pre-inflation endpoint measured' in adapter
assert 'pre-inflation/post-deflation endpoint means used' in adapter
assert 'publishBottomTarget(s.bottom_command);' in adapter
assert 'beginHybridTerminalSettle(now,' in adapter
config=yaml.safe_load((root/'config/attitude_pressure_controller.yaml').read_text())
removed_args={'hybrid_locomotion_enabled','hybrid_locomotion_shadow','action_c_reference_pulse_mode',
              'action_c_reference_single_pulse','action_c_pulse_search_enabled','action_c_reference_thrust_n'}
assert not (removed_args & config.keys())
assert config['hybrid_locomotion_outer_enabled'] is True
for filename in ('bringup.launch','attitude_pressure_controller.launch'):
    tree=ET.parse(root/'launch'/filename).getroot()
    args={item.attrib['name'] for item in tree.findall('arg')}
    assert not (removed_args & args)
    assert {'inchworm_action_c','hybrid_locomotion_outer_enabled'} <= args
    assert '$(arg hybrid_locomotion_outer_enabled)' in (root/'launch'/filename).read_text()
# SHA256 of action bodies at checkpoint 0a2db4956. No git executable/history
# needed at test time. Change these only with an intentional non-C gait change.
for first,last,expected in (
 ('ACTION_D_PREPARE','FRONT_RELEASE','25c80eb2a82b8da24dd55d8f62316470e93db714a76015ae369dc76bc43407d1'),
 ('FRONT_RELEASE','ROCK_FRONT_UNLOAD','f2c2275e4898efcca56967eafa7ae84cd0d5f770bb09768327d737ca99f23db0'),
 ('ACTION_B_FRONT_REACH','SETTLE','9dec9e18564c91c26ca2f311b9576aae94272e2bb121645933eb8af910ec9ec2'),
 ('ROCK_FRONT_UNLOAD','ACTION_B_FRONT_REACH','00bd1139714f86e5da726e2035a733bcef942f6872b839c17a720799c11f8955')):
    start=controller.index('    case '+first+':')
    text=controller[start:controller.index('    case '+last+':',start)]
    assert hashlib.sha256(text.encode()).hexdigest()==expected,first
print('PASS: single predictive C path; obsolete switches removed; A/B/D/rocking bodies unchanged from checkpoint')
