#!/usr/bin/env python3
"""PPG quiet windows must hold safe outputs and remain human-mode opt-in."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
controller = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
hybrid = (root / 'src/control/hybrid_locomotion.cpp').read_text()
config = (root / 'config/attitude_pressure_controller.yaml').read_text()

assert 'pnh_.param("ppg_settle_sec", ppg_settle_sec_, 0.0)' in controller
assert 'pnh_.param("hybrid_terminal_posture_monitor_sec",' in controller
assert 'resume_ready, now.toSec(), ppg_settle_sec_' in controller
assert 'publishCorrectionThrust(zero_thrust);' in controller
grasp = controller[controller.index('        case GRASP:'):
                   controller.index('        case PREPARE:',
                                    controller.index('        case GRASP:'))]
assert 'action_c_resume_pending_ ||' in grasp
assert 'recovery_candidate_reselect_pending_) ? 0.0' in grasp

phase2 = hybrid[hybrid.index('  if(hybrid_phase_==2) {'):
                hybrid.index('  publishPressureTarget(inchworm_pressure_',
                             hybrid.index('  if(hybrid_phase_==2) {'))]
assert 'hybrid_regrip_complete_latched_=true' in phase2
assert 'hybrid_bottom_deflated_latched_=true' in phase2
assert 'hybrid_endpoint_captured_latched_=true' in phase2
assert 'const bool target_reached_at_endpoint=' in phase2
assert 'std::max(ppg_settle_sec_,hybrid_terminal_posture_monitor_sec_)' in phase2
assert 'hybrid_position_window_.stable(clock)' in phase2
assert 'Hybrid terminal hold retained %.2f mm less progress' in phase2
assert 'clock-hybrid_ppg_wait_since_>=endpoint_hold_sec' in phase2
assert 'if(ppg_settle_ready && terminal_position_ready)' in phase2
assert '!hybrid_regrip_complete_latched_' in phase2
assert '(now-state_start_).toSec()>hybrid_config_.regrip_timeout' in phase2
assert 'inchworm_pressure_=hybrid_grasp_pressure_' in phase2
assert 'hybrid_regrip_bottom_=std::max(0.,hybrid_regrip_bottom_-' in phase2
assert 'hybrid_trial_x_=empty_x;' in hybrid
assert 'const double inflated_start_x=hybrid_settled_window_.mean();' in hybrid
assert 'hybrid_trial_x_=hybrid_settled_window_.mean();' not in hybrid

generic = ET.parse(root / 'launch/attitude_pressure_controller.launch').getroot()
generic_args = {arg.get('name'): arg.get('default') for arg in generic.findall('arg')}
assert generic_args['ppg_settle_sec'] == '0.0'
assert generic_args['hybrid_terminal_posture_monitor_sec'] == '8.0'
human = ET.parse(root / 'launch/human_arm_locomotion.launch').getroot()
human_args = {arg.get('name'): arg.get('default') for arg in human.findall('arg')}
assert human_args['ppg_settle_sec'] == '0.0'
assert human_args['hybrid_terminal_posture_monitor_sec'] == '8.0'
human_include = next(include for include in human.findall('include')
                     if include.get('file', '').endswith('attitude_pressure_controller.launch'))
human_forwarded = {arg.get('name'): arg.get('value')
                   for arg in human_include.findall('arg')}
assert human_forwarded['hybrid_terminal_posture_monitor_sec'] == \
       '$(arg hybrid_terminal_posture_monitor_sec)'
assert 'ppg_settle_sec: 0.0' in config
assert 'hybrid_terminal_posture_monitor_sec: 8.0' in config
print('PASS: endpoint stages are latched independently; PPG dwell is opt-in and terminal posture monitoring is separate')
