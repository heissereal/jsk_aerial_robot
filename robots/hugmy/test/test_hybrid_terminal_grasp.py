#!/usr/bin/env python3
"""Completed grasp retention must never survive a fault or manual stop."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
src=(root/'src/control/attitude_pressure_controller.cpp').read_text()
hybrid=(root/'src/control/hybrid_locomotion.cpp').read_text()
mpc=(root/'include/hugmy/control/hybrid_locomotion_mpc.h').read_text()
config=(root/'config/attitude_pressure_controller.yaml').read_text()
transfer=hybrid[hybrid.index('  if(hybrid_phase_==2) {'):hybrid.index('  publishPressureTarget(inchworm_pressure_',hybrid.index('  if(hybrid_phase_==2) {'))]
assert 'verifiedPairRollMoments(' in transfer
assert 'transfer_moment,hybrid_single_config_.reverse_geometry_roll_moment)' in transfer
assert 'transfer_roll_worsening?' in transfer
assert 'slew(transfer_thrust[j],balanced[j],' in transfer
assert 'hybrid_single_config_.identification_tracking_rise,2.,dt)' in transfer
assert 'hybrid_single_config_.regrip_rear_hold_pressure' in transfer
assert 'if(thrust_off) hybrid_regrip_thrust_zero_latched_=true;' in transfer
assert 'if(!hybrid_regrip_thrust_zero_latched_)' in transfer
assert transfer.index('const bool thrust_off=') < transfer.index(
    'if(thrust_off) hybrid_regrip_thrust_zero_latched_=true;')
assert transfer.index('if(!hybrid_regrip_thrust_zero_latched_)') < transfer.index(
    'mechanical_regrip_ready')
assert 'finishHybridLocomotion(now,!terminal,true)' in transfer
assert 'startInchwormCycle(now);return;' in transfer
assert 'hybrid_terminal_retry_' in transfer
assert 'beginHybridTerminalSettle(now,"hybrid measured bend/rate/tilt guard; score retained endpoint and retry")' in hybrid
# Regrip is an intentional pressure-driven return, not another powered gait
# excursion.  Its bend must not be compared with Motion-2's old start pose.
regrip_guard=hybrid[hybrid.index('  hugmy::hybrid::State safety_state=s;'):
    hybrid.index('  const bool stopping_after_slip=',hybrid.index('  hugmy::hybrid::State safety_state=s;'))]
assert 'if(hybrid_phase_==2)' in regrip_guard
assert 'safety_state.start_bend=s.bend;' in regrip_guard
assert 'hybrid_single_config_.identification_return_bend_rate_limit' in regrip_guard
assert 'clock-hybrid_task_start_>hybrid_config_.task_timeout' not in hybrid
# Hybrid C is an anytime controller. A completed recovery/retry must retain
# the original distance goal indefinitely; only phase safety guards, hard
# faults, target completion or an operator stop may terminate the task.
for text in (src,hybrid,mpc,config):
    assert 'hybrid_config_.task_timeout' not in text
    assert 'hybrid_locomotion_task_timeout_sec' not in text
assert 'task_timeout=' not in mpc
begin=src.index('void AttitudePressureController::stopInchwormToGrasp(')
stop=src[begin:src.index('void AttitudePressureController::requestActionCPostureRecovery(',begin)]
code=r'''
#include <array>
#include <limits>
#include <string>
#include <cassert>
#define ROS_WARN(...) ((void)0)
namespace ros {struct Time {};}
struct AttitudePressureController {
 using PressureArray=std::array<double,4>;
 bool hybrid=true,hybrid_trial_active_=false,hybrid_retain_grasp_pressure_=true,hybrid_hold_pressure_=true;
 bool hybrid_keep_bottom_between_strokes_=true;double ppg_settle_sec_=0.;
 bool continuous_recovery_waiting_for_settle_=true,inchworm_active_=true;
 bool action_c_resume_pending_=true,action_c_preserve_adaptation_on_start_=true;
 double action_c_requested_recovery_trigger_rad_=std::numeric_limits<double>::quiet_NaN();
 struct Gate {void reset(){}} action_c_recovery_gate_;
 PressureArray hybrid_grasp_pressure_{{50,30,30,50}},inchworm_thrust_{},prepare_shape_thrust_{},output{};
 double base_pressure_kpa_=30.,action_a_bottom_low_kpa_=0.;
 bool hybrid_hold_bottom_=false;double bottom_output=-1.;
 struct {double bottom_kpa=30.;} hybrid_single_config_;
 ros::Time last_correction_end_;
 enum {GRASP=1};int state=0;
 bool hybridLocomotionActive(){return hybrid;}
 void finishHybridLocomotion(ros::Time,bool,bool){hybrid_trial_active_=false;}
 void publishCorrectionThrust(const PressureArray& p){for(double v:p)assert(v==0.);}
 void publishBottomTarget(double p){bottom_output=p;}
 void publishPressureTarget(const PressureArray& p,double){output=p;}
 void enterState(int s,ros::Time){state=s;}
 void stopInchwormToGrasp(const ros::Time&,const char*);
};
STOP
int main(){
 AttitudePressureController success;success.stopInchwormToGrasp({},"hybrid target reached");
 assert(success.hybrid_hold_pressure_ && success.output==success.hybrid_grasp_pressure_);
 assert(success.hybrid_hold_bottom_ && success.bottom_output==30.);
 assert(!success.inchworm_active_ && !success.action_c_resume_pending_ && success.state==success.GRASP);
 for(const char* reason:{"manual stop","hybrid stale sensors","hybrid cycle limit"}){
   AttitudePressureController fault;fault.stopInchwormToGrasp({},reason);
   assert(!fault.hybrid_hold_pressure_);for(double v:fault.output)assert(v==30.);
   assert(!fault.hybrid_hold_bottom_ && fault.bottom_output==0.);
 }
 AttitudePressureController legacy;legacy.hybrid_retain_grasp_pressure_=false;
 legacy.stopInchwormToGrasp({},"hybrid target reached");
 assert(!legacy.hybrid_hold_pressure_);for(double v:legacy.output)assert(v==30.);
 AttitudePressureController ppg;ppg.ppg_settle_sec_=10.;
 ppg.stopInchwormToGrasp({},"hybrid target reached");
 assert(!ppg.hybrid_hold_bottom_ && ppg.bottom_output==0.);
}
'''.replace('STOP',stop)
with tempfile.TemporaryDirectory(prefix='hybrid_terminal_') as folder:
    binary=str(Path(folder)/'test')
    subprocess.run(['g++','-std=c++14','-x','c++','-','-o',binary],input=code,text=True,check=True)
    subprocess.run([binary],check=True)
print('PASS: target retains stable inflated support unless PPG dwell requests zero; faults still restore base grasp and zero bottom')
