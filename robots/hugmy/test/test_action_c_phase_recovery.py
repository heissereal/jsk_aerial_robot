#!/usr/bin/env python3
"""Exercise the production predictive-C roll boundary and immediate guards."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
source=(root/'src/control/hybrid_locomotion.cpp').read_text()
start=source.index('  if(std::abs(s.roll)>=hybrid_config_.roll_limit')
hard=source[start:source.index('  if(!hybrid_terminal_settle_',start)]
start=source.index('  const double boundary_bottom_tolerance=')
conditions=source[start:source.index('  const bool quiet=',start)]
controller=(root/'src/control/attitude_pressure_controller.cpp').read_text()
recovery=controller.split('void AttitudePressureController::requestActionCPostureRecovery(')[1].split('bool AttitudePressureController::resumeActionCAfterRecovery(')[0]
assert 'publishBottomTarget(0.0)' in recovery
# A completed/interrupted hybrid action must be scored only after bottom
# exhaust.  Inflation is resumed by phase 0 for the next pulse, rather than
# being held across a terminal endpoint.
assert 'hybrid_hold_bottom_=false' in controller
assert 'publishBottomTarget(0.)' in source
assert 'hybrid_preinflate_measured_' in source
assert 'hybrid_regrip_bottom_=slew' not in source
code=r"""
#include <hugmy/control/hybrid_locomotion_mpc.h>
#include <hugmy/control/posture_recovery_gate.h>
#include <hugmy/control/roll_trend_gate.h>
#include <cassert>
#include <iostream>
struct Controller {
 hugmy::hybrid::Config hybrid_config_;
 struct {struct {double bottom=30.;} current;} hybrid_search_;
 std::array<double,4> inchworm_thrust_{},inchworm_pressure_{};
 double pressure_tolerance_kpa_=2.,action_c_pre_roll_rate_tolerance_rad_s_=.174532925;
 double action_c_bottom_deflation_tolerance_kpa_=5.;
 double action_c_posture_settle_sec_=.5,action_c_posture_recovery_trigger_rad_=8*M_PI/180.;
 double correction_trigger_confirm_sec_=.3;
 hugmy::RollTrendGate action_c_roll_trend_;
 hugmy::PostureRecoveryGate action_c_boundary_settle_gate_,action_c_recovery_gate_;
 int hybrid_phase_=0;bool arms_ready=true,ready=false,recovery=false;
 double bottom=30.;
 std::array<double,3> body_angular_velocity_{};
 bool filtered_quiet=false;
 bool pressureReached(const std::array<double,4>&){return arms_ready;}
 void requestActionCPostureRecovery(double,const char*){recovery=true;}
 void tick(double clock,double roll,double rate=0.){
   using hugmy::hybrid::bottomPressureReady;
   ready=false;recovery=false;double now=clock;
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
   Controller worsening;assert(worsening.drive(0.,1.2,9.*sign,3.*sign));
   Controller improving;assert(!improving.drive(0.,1.2,12.*sign,-3.*sign));assert(improving.ready);
   Controller powered;powered.hybrid_phase_=1;powered.inchworm_thrust_={{3,0,0,3}};
   assert(!powered.drive(0.,1.2,9.*sign,3.*sign));
   Controller inflation;inflation.bottom=10.;assert(!inflation.drive(0.,1.2,9.*sign,3.*sign));assert(!inflation.ready);
   inflation.bottom=30.;assert(!inflation.drive(2.,1.2,10.*sign));assert(inflation.ready);
   Controller regrip;regrip.hybrid_phase_=2;regrip.arms_ready=false;
   assert(!regrip.drive(0.,1.2,9.*sign,3.*sign));assert(!regrip.ready);
   regrip.arms_ready=true;assert(!regrip.drive(2.,1.2,10.*sign));assert(regrip.ready);
   // Hard limits are immediate, regardless of phase/pressure readiness.
   powered.tick(4.,16.*sign);assert(powered.recovery);
   powered.tick(4.1,1.,50.*sign);assert(powered.recovery);
   powered.filtered_quiet=true;powered.tick(4.2,1.,50.*sign);assert(powered.recovery);
 }
 Controller invalid;invalid.bottom=NAN;assert(!invalid.drive(0.,1.,10.));assert(!invalid.ready);
 Controller low;low.hybrid_search_.current.bottom=0.;low.bottom=3.;
 assert(!low.drive(0.,1.2,10.));assert(low.ready);
 low.bottom=5.01;low.tick(2.,10.);assert(!low.ready);
 Controller reset;assert(!reset.drive(2.,1.,10.));assert(reset.ready);
 reset.tick(1.,10.);assert(!reset.ready && !reset.recovery);
 std::cout<<"PASS: predictive C +/- residual roll, transient/inflation isolation, regrip gating, immediate hard guards and clock reset\n";
}
""".replace('HARD',hard).replace('CONDITIONS',conditions)
with tempfile.TemporaryDirectory(prefix='hugmy_phase_recovery_') as directory:
    binary=str(Path(directory)/'test')
    subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),
                    '-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
