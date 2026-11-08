#!/usr/bin/env python3
"""ROS-independent regression: fault transfer never becomes posture PI.

Checks both the production safety gates and the 2026-11-05 hardware
scenario (pressures still far above 0 kPa when a model/rate fault fired).
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "src/control/attitude_pressure_controller.cpp").read_text() + "\n" + \
    (root / "src/control/posture_recovery_controller.cpp").read_text()
header = (root / "include/hugmy/control/attitude_pressure_controller.h").read_text()

assert "POSTURE_SAFE_TRANSFER = 29" in header  # keep recorded ROS IDs stable
assert '"POSTURE_SAFE_TRANSFER"' in source
assert '"posture_recovery_phase"' in source
assert "recovery_response_evaluation_ready &&" in source
assert "recovery_preinflate_thrust_rise_n_s_" in source
assert "active_roll_recovery_config.rise = std::max(" in source
assert "recovery_contact_restore_thrust_limit_n_" in source
assert "if (!pressure_geometry_settled || !support_transfer_equalized)" in source
assert '"contact restoration lost near-target capture"' in source

rotor_exit = source[source.index('ROS_WARN("Action C continuous recovery ended:'):
                    source.index("resetRecoveryHandoffState();\n                  enterState(RECOVER",
                                 source.index('ROS_WARN("Action C continuous recovery ended:'))]
assert "recoveryExitDecision(" in rotor_exit
assert "beginPostureSafeTransfer(now," in rotor_exit
assert "continuous_roll_recovery_.finished = true" not in rotor_exit

safe_state = source[source.index("case POSTURE_SAFE_TRANSFER:"):
                    source.index("case RECOVER:", source.index("case POSTURE_SAFE_TRANSFER:"))]
assert "recoveryBoundedPressureStep(" in safe_state
assert "recoverySafeThrustRundown(" in safe_state
assert "measured_support_ready" in safe_state
assert "RecoverySupportObservation" in safe_state
assert "RecoverySafeTransferPhase::SUPPORT_VERIFY" in safe_state
assert "RecoverySafeTransferPhase::THRUST_RELEASE" in safe_state
assert "recovery_support_monitor_.update(" in safe_state
assert "continuous_recovery_blocked_" in safe_state
assert "continuous_roll_recovery_.update(" not in safe_state
assert "recovery_handoff_integral_n_ +" not in safe_state

code = r"""
#include <hugmy/control/posture_recovery_phase.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
int main() {
  using namespace hugmy;
  const double d=M_PI/180.;
  const double capture=5.0*d;
  assert(recoveryExitDecision(false,false,false,50*d,capture)==RecoveryExit::CONTINUE);
  assert(recoveryExitDecision(false,true,false,50*d,capture)==RecoveryExit::SAFE_TRANSFER);
  assert(recoveryExitDecision(true,false,false,2*d,capture)==RecoveryExit::CONTACT_RESTORE);
  assert(recoveryExitDecision(true,false,false,53*d,capture)==RecoveryExit::SAFE_TRANSFER);
  assert(recoveryExitDecision(false,true,true,-1*d,capture)==RecoveryExit::TARGET_CROSSED);
  assert(recoveryExitDecision(false,true,true,-30*d,capture)==RecoveryExit::SAFE_TRANSFER);
  assert(recoveryExitDecision(true,false,false,NAN,capture)==RecoveryExit::SAFE_TRANSFER);

  // The recorded t205.01 pressure was [13.85,14.41,31.08,30.82] kPa;
  // a released 0+1 pair had not reached its 0-kPa target. Even at >=3 N,
  // no geometry/no-response fault may be inferred from that transient.
  const bool pressure_settled=(std::abs(13.85-0.)<=1.) &&
                              (std::abs(14.41-0.)<=1.);
  assert(!recoveryResponseCanBeJudged(pressure_settled,true,3.17,1.,2.,1.2));
  assert(!recoveryResponseCanBeJudged(true,true,.5,1.,2.,1.2));
  assert(!recoveryResponseCanBeJudged(true,true,3.17,1.,.6,1.2));
  assert(!recoveryResponseCanBeJudged(true,false,3.17,1.,2.,1.2));
  assert(recoveryResponseCanBeJudged(true,true,3.17,1.,1.21,1.2));

  // Safe pneumatic support builds without a pressure step; no rotor command
  // can grow while the contact state is being restored.
  double p=13.85,t=3.17;
  for(int k=0;k<200;++k) {
    double new_p=recoveryBoundedPressureStep(p,25.,6.,.02);
    double new_t=recoverySafeThrustRundown(t,.75,.02);
    assert(new_p>=p && new_p-p<=.12000001 && new_p<=25.);
    assert(new_t<=t && t-new_t<=.01500001);
    p=new_p;t=new_t;
  }
  assert(std::abs(p-25.)<1.e-8 && t<.18);
  assert(recoverySafeThrustRundown(5.5,3.,.02)<5.5);
  assert(recoverySafeThrustRundown(NAN,3.,.02)==0.);
  // Pressure alone is not physical support. Bend and body attitude must stay
  // stable through the verification dwell, and thrust-off stability is a
  // separate final condition.
  RecoverySupportConfig support_cfg;
  RecoverySupportObservation support;
  support.sensors_valid=true;
  support.pressure={{30.,30.,30.,30.}};
  support.target_pressure={{30.,30.,30.,30.}};
  support.bend={{1.,1.,1.,1.}};
  support.roll=.2;support.pitch=.1;
  support.roll_rate=0.;support.pitch_rate=0.;support.maximum_thrust=1.;
  RecoverySupportMonitor monitor;
  assert(!monitor.update(support,0.,support_cfg,false));
  assert(!monitor.update(support,.2,support_cfg,false));
  assert(monitor.update(support,.31,support_cfg,false));
  monitor.reset();
  support.maximum_thrust=0.;
  assert(!monitor.update(support,1.,support_cfg,true));
  support.bend[0]+=.1;
  assert(!monitor.update(support,1.2,support_cfg,true));
  support.bend[0]-=.1;
  assert(!monitor.update(support,1.4,support_cfg,true));
  assert(monitor.update(support,1.71,support_cfg,true));
  monitor.reset();support.pressure[0]=0.;support.pressure[1]=0.;
  assert(!monitor.update(support,2.,support_cfg,true));
  assert(std::string(recoveryFaultReasonName(
      RecoveryFaultReason::ABSOLUTE_WATCHDOG))=="absolute_watchdog");
  std::puts("PASS: phase exits, pressure-settled probe, monotone safe thrust and bounded support transfer");
}
"""
with tempfile.TemporaryDirectory(prefix="hugmy_posture_phase_") as tmp:
    exe = Path(tmp) / "test"
    subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-I",
                    str(root / "include"), "-x", "c++", "-", "-o",
                    str(exe)], input=code, text=True, check=True)
    subprocess.run([str(exe)], check=True)
