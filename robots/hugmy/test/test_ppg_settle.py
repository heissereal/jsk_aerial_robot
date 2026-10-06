#!/usr/bin/env python3
"""PPG quiet windows must hold safe outputs and remain human-mode opt-in."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
controller = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
hybrid = (root / 'src/control/hybrid_locomotion.cpp').read_text()
config = (root / 'config/attitude_pressure_controller.yaml').read_text()

assert 'pnh_.param("ppg_settle_sec", ppg_settle_sec_, 0.0)' in controller
assert 'resume_ready, now.toSec(), ppg_settle_sec_' in controller
assert 'publishCorrectionThrust(zero_thrust);' in controller
assert 'publishBottomTarget(action_c_resume_pending_ ? 0.0' in controller

phase2 = hybrid[hybrid.index('  if(hybrid_phase_==2) {'):
                hybrid.index('  publishPressureTarget(inchworm_pressure_',
                             hybrid.index('  if(hybrid_phase_==2) {'))]
assert 'measurement_ready,clock,ppg_settle_sec_' in phase2
assert 'if(ppg_settle_ready)' in phase2
assert 'hybrid_config_.regrip_timeout+ppg_settle_sec_' in phase2
assert 'inchworm_pressure_=hybrid_grasp_pressure_' in phase2
assert 's.bottom_command=((hybrid_phase_==0 && !hybrid_preinflate_measured_) ||\n      hybrid_phase_==2)?0.' in hybrid

generic = ET.parse(root / 'launch/attitude_pressure_controller.launch').getroot()
generic_args = {arg.get('name'): arg.get('default') for arg in generic.findall('arg')}
assert generic_args['ppg_settle_sec'] == '0.0'
human = ET.parse(root / 'launch/human_arm_locomotion.launch').getroot()
human_args = {arg.get('name'): arg.get('default') for arg in human.findall('arg')}
assert human_args['ppg_settle_sec'] == '10.0'
assert 'ppg_settle_sec: 0.0' in config
print('PASS: PPG quiet window is 10 s in human mode, opt-in elsewhere, and holds safe gait outputs')
