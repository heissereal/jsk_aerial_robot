#!/usr/bin/env python3
"""Verify the posture-control translation unit and locomotion handoff wiring."""
from pathlib import Path


root = Path(__file__).resolve().parents[1]
parent = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
recovery = (root / 'src/control/posture_recovery_controller.cpp').read_text()
header = (root / 'include/hugmy/control/attitude_pressure_controller.h').read_text()
cmake = (root / 'CMakeLists.txt').read_text()
locomotion = (root / 'src/control/hybrid_locomotion.cpp').read_text()

# Exactly one implementation of each shared-machine entry point.
assert 'void updatePostureRecovery(const ros::Time& now' in header
assert 'updatePostureRecovery(now, dt, maximum_step, body_imu_fresh,' in parent
assert 'void AttitudePressureController::updatePostureRecovery(' in recovery
assert 'void AttitudePressureController::updatePostureRecovery(' not in parent
assert 'case POSTURE_SAFE_TRANSFER:' in recovery
assert 'case POSTURE_SAFE_TRANSFER:' not in parent

assert 'void AttitudePressureController::requestActionCPostureRecovery(' in recovery
assert 'void AttitudePressureController::requestActionCPostureRecovery(' not in parent
assert 'requestActionCPostureRecovery(now,' in locomotion
assert 'resumeActionCAfterRecovery(now)' in recovery

# Retain the tested fast, immediate recovery authority and separate fault exit.
assert 'recovery_preinflate_thrust_rise_n_s_' in recovery
assert 'active_roll_recovery_config.rise = std::max(' in recovery
assert 'continuous_roll_recovery_.update(' in recovery
assert 'beginPostureSafeTransfer(now,' in recovery
assert 'recoverySafeThrustRundown(' in recovery
assert 'POSTURE_SAFE_TRANSFER = 29' in header
assert 'src/control/posture_recovery_controller.cpp' in cmake

print('PASS: posture recovery lives in a separately compiled source, shares the existing locomotion ownership handoff')
