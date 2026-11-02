#!/usr/bin/env python3
"""Exercise the production predictive-C roll boundary and immediate guards."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
source=(root/'src/control/hybrid_locomotion.cpp').read_text()
config=(root/'config/attitude_pressure_controller.yaml').read_text()
# Automatic locomotion recovery accepts a true +/-3-deg roll band; it must
# not silently regress to an exact-zero completion test.
assert 'action_c_posture_resume_rad: 0.05235987755982989' in config
start=source.index('  const double measured_roll_rate=')
# The reverse-travel guard immediately following the posture guard belongs to
# retained-position validation and needs the full controller fixture.  Keep
# this focused unit test limited to the hard roll/rate handoff.
hard=source[start:source.index('  // Below the hard boundary',start)]
assert 'Hybrid guarded support catch' in source
start=source.index('  const double boundary_bottom_tolerance=')
conditions=source[start:source.index('  const bool quiet=',start)]
controller=(root/'src/control/attitude_pressure_controller.cpp').read_text()
recovery=controller.split('void AttitudePressureController::requestActionCPostureRecovery(')[1].split('bool AttitudePressureController::resumeActionCAfterRecovery(')[0]
assert 'publishBottomTarget(0.0)' in recovery
# A completed/interrupted hybrid action must be scored only after bottom
# exhaust.  Inflation is resumed by phase 0 for the next pulse, rather than
# being held across a terminal endpoint.
assert 'hybrid_hold_bottom_=false' in controller
# Ordinary terminal scoring still deflates the bottom, while an emergency
# Motion-2 roll handoff retains bottom support until four-arm pressure is back.
assert 'hybrid_regrip_bottom_=fast_roll_brake ?' in source
assert 'publishBottomTarget(hybrid_regrip_bottom_)' in source
assert 'hybrid_preinflate_measured_' in source
assert 'hybrid_regrip_bottom_=slew' not in source
assert 'preserving zero thrust and requesting a fresh measured-contact pass' in controller
assert 'recovery_handoff_reopen_angle_rad_' in controller
assert '-handoff_fall_rate' in controller
assert 'std::max(3.0, recovery_handoff_thrust_fall_n_s_)' in controller
assert 'const double handoff_settle_error = recovery_stop_angle' in controller
assert 'const bool strict_target_observed_before_regrasp =' in controller
assert 'abs_error <= active_handoff_settle_error' in controller
assert 'continuous_recovery_waiting_for_settle_ = false;' in controller
assert 'action_c_requested_recovery_trigger_rad_ =' in controller
code=r"""
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <hugmy/control/hybrid_single_pass.h>
#include <hugmy/control/posture_recovery_gate.h>
#include <hugmy/control/roll_trend_gate.h>
#include <cassert>
#include <iostream>
#define ROS_WARN_THROTTLE(...)
#define ROS_WARN(...)
struct Controller {
 hugmy::hybrid::Config hybrid_config_;
 struct {struct {double bottom=30.;} current;} hybrid_search_;
 struct {double pull_loaded_catch_pressure=18.;
         double pull_support_transfer_pressure=18.;
         double pull_regrip_min_flex=.30;
         double pre_stroke_roll_limit=15.*M_PI/180.;} hybrid_single_config_;
 hugmy::hybrid::SinglePass hybrid_single_;
 std::array<double,4> inchworm_thrust_{},inchworm_pressure_{};
 std::array<double,4> measured_pressure_{{30.,30.,30.,30.}};
 double pressure_tolerance_kpa_=2.,action_c_pre_roll_rate_tolerance_rad_s_=.174532925;
 double action_c_bottom_deflation_tolerance_kpa_=5.;
 double action_c_posture_settle_sec_=.5,action_c_posture_recovery_trigger_rad_=10*M_PI/180.;
 double correction_trigger_confirm_sec_=.3;
 double hybrid_roll_rate_confirm_sec_=.10,hybrid_roll_rate_limit_since_=-1.;
 double hybrid_lift_worsening_roll_stop_rad_=8.*M_PI/180.;
 double hybrid_lift_worsening_roll_rate_rad_s_=1.*M_PI/180.;
 double hybrid_lift_worsening_roll_confirm_sec_=.10;
 double hybrid_lift_worsening_roll_prediction_sec_=.25;
 double hybrid_lift_worsening_roll_since_=-1.;
 double hybrid_preinflate_roll_min_angle_rad_=8.*M_PI/180.;
 double hybrid_preinflate_roll_growth_rad_=.1*M_PI/180.;
 double hybrid_preinflate_roll_rate_rad_s_=.4*M_PI/180.;
 double hybrid_preinflate_roll_prediction_sec_=.30;
 double hybrid_preinflate_roll_confirm_sec_=.08;
 double hybrid_preinflate_roll_reference_rad_=NAN;
 hugmy::PostureRecoveryGate hybrid_preinflate_roll_gate_;
 double hybrid_regrip_roll_recovery_rad_=15.*M_PI/180.;
 double hybrid_regrip_roll_emergency_rad_=30.*M_PI/180.;
 double recovery_handoff_min_pressure_kpa_=15.;
 bool hybrid_regrip_fast_roll_brake_=false;
 bool hybrid_recovery_pair_preview_valid_=false;
 size_t hybrid_recovery_pair_preview_index_=0,recovery_selected_pair_index_=0;
 int hybrid_recovery_pair_preview_direction_=0,recovery_rejected_direction_sign_=0;
 double hybrid_recovery_pair_preview_last_attempt_=-1.;
 unsigned recovery_rejected_candidate_mask_=0;
 bool recovery_circle_event_locked_=false,recovery_preferred_pair_locked_=false;
 bool continuous_recovery_use_opposite_pair_=false;
 bool recovery_pair_selection_available_=true;
 std::array<double,2> correction_axis_{{0.,0.}};
 hugmy::RollTrendGate action_c_roll_trend_;
 hugmy::PostureRecoveryGate action_c_boundary_settle_gate_,action_c_recovery_gate_;
 int hybrid_phase_=0;bool arms_ready=true,ready=false,recovery=false,terminal=false;
 double bottom=30.;
 std::array<double,3> body_angular_velocity_{};
 bool filtered_quiet=false,hybrid_identification_settle_=false;
 bool hybrid_preinflate_measured_=true;
 Controller(){hybrid_config_.roll_limit=15.*M_PI/180.;}
 bool pressureReached(const std::array<double,4>&){return arms_ready;}
 void selectContinuousRecoveryPair(double){
   recovery_pair_selection_available_=true;recovery_selected_pair_index_=0;
 }
 void requestActionCPostureRecovery(double,const char*,double=NAN,bool=false){recovery=true;}
 void beginHybridTerminalSettle(double,const char*,bool=false){terminal=true;}
 void tick(double clock,double roll,double rate=0.){
   using hugmy::hybrid::bottomPressureReady;
   using hugmy::hybrid::SinglePass;
   ready=false;recovery=false;terminal=false;double now=clock;
   hugmy::hybrid::State s;s.roll=roll*M_PI/180.;s.roll_rate=rate*M_PI/180.;s.bottom=bottom;s.bottom_command=hybrid_search_.current.bottom;
   bool all_pressure=arms_ready;
   body_angular_velocity_[0]=s.roll_rate;
   if(filtered_quiet) s.roll_rate=0.;
   HARD
   CONDITIONS
   ready=posture_ready;
 }
 bool drive(double start,double seconds,double roll,double rate=0.){
   bool seen=false;
   for(int i=0;i<=int(seconds/.02);++i){tick(start+i*.02,roll+i*.02*rate,rate);seen=seen||recovery;}
   return seen;
 }
};
int main(){
 for(double sign:{-1.,1.}) {
   Controller stable;assert(!stable.drive(0.,1.2,10.*sign));assert(stable.ready);
   Controller moderate;assert(!moderate.drive(0.,1.2,8.*sign,.5*sign));assert(moderate.ready);
   Controller worsening;assert(worsening.drive(0.,1.2,14.*sign,3.*sign));
   Controller improving;assert(!improving.drive(0.,1.2,12.*sign,-3.*sign));assert(improving.ready);
   Controller powered;powered.hybrid_phase_=1;
   powered.hybrid_single_.phase=hugmy::hybrid::SinglePass::Lift;
   powered.inchworm_thrust_={{3,0,0,3}};
   // Eight degrees only arms the look-ahead. Slow supported motion remains
   // allowed; a sustained outward motion predicted beyond the ordinary
   // recovery boundary hands directly to shared recovery in any phase.
   powered.tick(0.,8.1*sign,2.*sign);assert(!powered.terminal);
   powered.tick(.12,8.34*sign,2.*sign);assert(!powered.terminal);
   powered.tick(.24,11.0*sign,20.*sign);assert(!powered.terminal);
   powered.tick(.36,13.4*sign,20.*sign);
   assert(!powered.terminal && powered.recovery);
   Controller inflation;inflation.bottom=10.;assert(!inflation.drive(0.,1.2,9.*sign));assert(!inflation.ready);
   inflation.bottom=30.;assert(!inflation.drive(2.,1.2,10.*sign));assert(inflation.ready);
   // A quiet high-roll contact is not rejected by magnitude alone. Once
   // bottom inflation produces a small persistent outward drift, however,
   // the dedicated guard must hand off before the 15-deg hard boundary.
   Controller preinflate;
   preinflate.hybrid_preinflate_roll_reference_rad_=14.4*sign*M_PI/180.;
   assert(preinflate.drive(0.,.3,14.45*sign,.8*sign));
   Controller regrip;regrip.hybrid_phase_=2;regrip.arms_ready=false;
   regrip.measured_pressure_.fill(0.);
   // A hard posture boundary has priority even while pneumatic support is
   // rebuilding; shared recovery fills the anchor while correction starts.
   assert(regrip.drive(0.,1.2,16.*sign,3.*sign));assert(!regrip.ready);
   // A stationary post-regrasp contact bias is allowed even when it is above
   // the normal recovery trigger.  Only a continuing outward trend (or the
   // independent hard guard below) starts recovery.
   regrip.arms_ready=true;regrip.measured_pressure_.fill(30.);
   assert(!regrip.drive(2.,1.2,12.*sign));assert(regrip.ready);
   // Once the transfer is settled, a continuing outward trend is a residual
   // posture error and can use the normal shared recovery path.
   assert(regrip.drive(4.,1.2,14.*sign,3.*sign));
   Controller loaded_regrip;loaded_regrip.hybrid_phase_=2;
   // Once all four arms are load-bearing, hand off at 15 deg rather than
   // waiting until the old 30-deg emergency angle.
   loaded_regrip.tick(6.,14.*sign);assert(!loaded_regrip.recovery);
   loaded_regrip.tick(6.1,16.*sign);assert(loaded_regrip.recovery);
   // Absolute roll remains immediate. Angular rate keeps the same limit but
   // requires persistence so one impact/IMU sample cannot hand off control.
   powered.tick(4.,16.*sign);assert(!powered.terminal && powered.recovery);
   powered.tick(4.1,1.,50.*sign);assert(!powered.terminal && !powered.recovery);
   powered.filtered_quiet=true;powered.tick(4.21,1.,50.*sign);
   assert(!powered.terminal && powered.recovery);
 }
 Controller invalid;invalid.bottom=NAN;assert(!invalid.drive(0.,1.,11.));assert(!invalid.ready);
 Controller low;low.hybrid_search_.current.bottom=0.;low.bottom=3.;
 assert(!low.drive(0.,1.2,11.));assert(low.ready);
 low.bottom=5.01;low.tick(2.,11.);assert(!low.ready);
 Controller reset;assert(!reset.drive(2.,1.,9.));assert(reset.ready);
 reset.tick(1.,9.);assert(reset.ready && !reset.recovery);
 std::cout<<"PASS: predictive C +/- residual roll, transient/inflation isolation, regrip gating, confirmed rate guard, immediate roll guard and clock reset\n";
}
""".replace('HARD',hard).replace('CONDITIONS',conditions)
with tempfile.TemporaryDirectory(prefix='hugmy_phase_recovery_') as directory:
    binary=str(Path(directory)/'test')
    subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),
                    '-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
