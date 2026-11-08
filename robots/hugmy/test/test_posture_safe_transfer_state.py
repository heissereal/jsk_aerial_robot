#!/usr/bin/env python3
"""Compile and run the production SAFE_TRANSFER case with minimal ROS stubs.

Simulates the pressure snapshot from the human-arm 2026-11-05 failure,
and verifies no positive rotor-thrust step, measured pressure confirmation,
and timeout/IMU fault behavior without building the full ROS workspace.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
controller = (root / "src/control/posture_recovery_controller.cpp").read_text()
start = controller.index("        case POSTURE_SAFE_TRANSFER:")
end = controller.index("        case RECOVER:", start)
production_case = controller[start:end]

code = r"""
#include <hugmy/control/posture_recovery_phase.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cassert>
#include <cstdio>
#define ROS_ERROR(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
namespace ros {
struct Time {
  double time;
  explicit Time(double value=0.) : time(value) {}
  bool isZero()const{return time==0.;}
  double toSec()const{return time;}
};
Time operator-(Time a,Time b){return Time(a.time-b.time);}
}
class Controller {
public:
  static constexpr size_t ARM_COUNT=4;
  enum { GRASP=0,RECOVER=3,POSTURE_SAFE_TRANSFER=29 };
  using PressureArray=std::array<double,4>;
  int state_=POSTURE_SAFE_TRANSFER;
  PressureArray prepare_pressure_{{0.,0.,30.,30.}};
  PressureArray measured_pressure_{{13.85,14.41,31.08,30.82}};
  PressureArray recovery_safe_transfer_target_pressure_{{30.,30.,30.,30.}};
  PressureArray prepare_shape_thrust_{{3.17,3.17,0.,0.}};
  PressureArray grasp_pressure_{{30.,30.,30.,30.}};
  PressureArray hybrid_grasp_pressure_{};
  bool hybrid_hold_pressure_=false;
  bool action_c_resume_pending_=true;
  bool recovery_contact_grasp_active_=false;
  bool recovery_safe_transfer_retry_allowed_=false;
  bool recovery_safe_transfer_failed_=false;
  bool continuous_recovery_started_=true;
  bool continuous_recovery_waiting_for_settle_=false;
  bool continuous_recovery_blocked_=true;
  bool recovery_candidate_reselect_pending_=false;
  bool continuous_recovery_retry_opposite_pending_=false;
  bool body_imu_fresh=true,pressure_valid=true,neurons_fresh=true;
  bool recovery_pressure_fresh=true;
  std::array<double,3> body_angular_velocity_{{0.,0.,0.}};
  double roll_=53.*M_PI/180.,pitch_=0.;
  PressureArray bend_{{1.,1.,1.,1.}};
  double recovery_roll_rate=0.01;
  double roll_error=53.*M_PI/180.;
  double recovery_measurement_rate_limit_rad_s_=1.5;
  double recovery_safe_transfer_timeout_sec_=8.;
  double recovery_safe_transfer_rate_n_s_=3.;
  double recovery_safe_transfer_pressure_tolerance_kpa_=1.5;
  double recovery_handoff_pressure_rate_kpa_s_=6.;
  double recovery_handoff_thrust_fall_n_s_=.75;
  double recovery_handoff_settle_sec_=.3;
  double recovery_handoff_min_pressure_kpa_=15.;
  struct {double settle_rate=.05,stable_bend=.035,stop_margin=.0174533;}
      roll_recovery_config_;
  ros::Time recovery_safe_transfer_settled_since_,state_start_{1.};
  hugmy::RecoverySafeTransferPhase recovery_safe_transfer_phase_=
      hugmy::RecoverySafeTransferPhase::REGRASP;
  hugmy::RecoveryFaultReason recovery_fault_reason_=
      hugmy::RecoveryFaultReason::UNCLASSIFIED;
  hugmy::RecoverySupportMonitor recovery_support_monitor_;
  bool recovery_event_active_=true;
  hugmy::PostureRecoveryPhase posture_recovery_phase_=
      hugmy::PostureRecoveryPhase::SAFE_TRANSFER;
  PressureArray last_rotor_{{3.17,3.17,0.,0.}};
  PressureArray last_pressure_{};
  int thrust_publications_=0,pressure_publications_=0;
  void publishBottomTarget(double pressure){assert(pressure==0.);}
  void publishPressureTarget(const PressureArray& p,double){
    last_pressure_=p;++pressure_publications_;
  }
  void publishCorrectionThrust(const PressureArray& t){
    for(size_t arm=0;arm<ARM_COUNT;++arm){
      assert(t[arm]>=0. && t[arm]<=last_rotor_[arm]+1.e-9);
    }
    last_rotor_=t;++thrust_publications_;
  }
  bool hybridLocomotionActive()const{return false;}
  double measuredArmBendAngle(size_t arm)const{return bend_[arm];}
  void enterState(int state,ros::Time){state_=state;}
  void tick(double time,double dt){
    const ros::Time now(time);
    switch(state_){
CASE
      case RECOVER: break;
      case GRASP: break;
      default: assert(false);
    }
  }
};
int main(){
  Controller c;
  // No premature handoff: 0/1 pressure is far above commanded zero at t205.
  c.tick(1.02,.02);
  assert(c.state_==c.POSTURE_SAFE_TRANSFER);
  assert(!c.continuous_recovery_waiting_for_settle_);
  assert(c.prepare_pressure_[0]<.13 && c.last_rotor_[0]<3.17);
  // Simulated slower pneumatic response reaches the safe command at ~5s.
  for(int k=2;k<350 && c.state_!=c.GRASP;++k){
    for(int arm=0;arm<4;++arm)
      c.measured_pressure_[arm]=hugmy::recoveryBoundedPressureStep(
          c.measured_pressure_[arm],c.prepare_pressure_[arm],10.,.02);
    c.tick(1.+.02*k,.02);
  }
  assert(c.state_==c.GRASP);
  assert(c.continuous_recovery_blocked_ && !c.continuous_recovery_started_);
  assert(!c.continuous_recovery_waiting_for_settle_);
  assert(c.recovery_contact_grasp_active_);
  assert(c.prepare_shape_thrust_[0]<=.01);
  assert(c.grasp_pressure_[0]==30. && c.grasp_pressure_[2]==30.);
  // A large but finite restorative rate delays support verification; unlike
  // a stale sensor it must not skip pneumatic REGRASP and enter FAILED.
  Controller fast;fast.recovery_roll_rate=3.0;
  fast.body_angular_velocity_[1]=2.0;fast.tick(1.02,.02);
  assert(fast.recovery_safe_transfer_phase_==
         hugmy::RecoverySafeTransferPhase::REGRASP);
  assert(!fast.recovery_safe_transfer_failed_);
  // A broken pressure/IMU sample is never treated as completed support.
  Controller stale;stale.body_imu_fresh=false;stale.tick(1.02,.02);
  assert(stale.state_==stale.POSTURE_SAFE_TRANSFER);
  assert(stale.posture_recovery_phase_==hugmy::PostureRecoveryPhase::ABORT);
  assert(stale.prepare_shape_thrust_[0]<3.17 &&
         stale.prepare_shape_thrust_[0]>0.);
  assert(stale.recovery_safe_transfer_phase_==
         hugmy::RecoverySafeTransferPhase::FAILED);
  // Pressure cannot reach its target: the fault times out, with no retry.
  Controller timeout;timeout.recovery_safe_transfer_retry_allowed_=true;
  for(int k=1;k<460;++k) timeout.tick(1.+.02*k,.02);
  assert(timeout.state_==timeout.POSTURE_SAFE_TRANSFER);
  assert(timeout.recovery_safe_transfer_failed_);
  assert(!timeout.recovery_safe_transfer_retry_allowed_);
  assert(timeout.continuous_recovery_blocked_);
  assert(timeout.prepare_shape_thrust_[0]<=.01);
  std::puts("PASS: actual SAFE_TRANSFER case: slow pressure, no force growth, settled/timeout/sensor stop");
}
""".replace("CASE", production_case)

with tempfile.TemporaryDirectory(prefix="hugmy_safe_state_") as tmp:
    exe = Path(tmp) / "test"
    subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-I",
                    str(root / "include"), "-x", "c++", "-", "-o",
                    str(exe)], input=code, text=True, check=True)
    subprocess.run([str(exe)], check=True)
