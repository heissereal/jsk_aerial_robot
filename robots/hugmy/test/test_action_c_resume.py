#!/usr/bin/env python3
"""Compile production auto-resume and continuous recovery sign selection."""
import pathlib
import subprocess
import tempfile
root=pathlib.Path(__file__).resolve().parents[1]
source=(root/'src/control/attitude_pressure_controller.cpp').read_text()
start=source.index('bool AttitudePressureController::resumeActionCAfterRecovery(')
resume=source[start:source.index('bool AttitudePressureController::actionCRearReleased(',start)]
start=source.index('correction_axis_ = {{roll_error >= 0.0')
pair=source[start:source.index('continuous_roll_recovery_.reset(',start)]
helper_start=source.index('void AttitudePressureController::selectContinuousRecoveryPair(')
helper=source[helper_start:source.index('\n}\n',helper_start)+3]
start=source.index('const bool progress_timeout = continuous_roll_recovery_.timed_out')
timeout=source[start:source.index('// No repeated timed search pulses',start)]
assert 'recovered = pressure_valid && actionCPressureReady(grasp_pressure_, now)' in source
assert 'if (resumeActionCAfterRecovery(now)) break;' in source
code=r'''
#include <hugmy/control/continuous_roll_recovery.h>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
namespace ros {struct Time {double t;double toSec()const{return t;}};
Time operator-(Time a,Time b){return {a.t-b.t};}}
class AttitudePressureController {
public:
 static constexpr size_t ARM_COUNT=4;
 using Vector3=std::array<double,3>;
 bool action_c_resume_pending_=true, hybrid_trial_active_=true;
 bool mocap_received_=true, action_c_preserve_adaptation_on_start_=false;
 bool active=false;
 bool continuous_recovery_waiting_for_settle_=false, continuous_recovery_blocked_=false;
 bool continuous_recovery_started_=true;
 hugmy::ContinuousRollRecovery continuous_roll_recovery_;
 struct Gate {int resets=0;void reset(){++resets;}} action_c_resume_gate_, action_c_boundary_settle_gate_;
 ros::Time state_start_{0};double prepare_timeout_=60.;
 std::array<double,4> prepare_shape_thrust_{{2.,2.,2.,2.}}, grasp_pressure_{{30.,30.,30.,30.}};
 int publications=0, state=0;enum {RECOVER=1};
 void publishCorrectionThrust(const std::array<double,4>& t) {
   for(double v:t) assert(v==0.);++publications;
 }
 void publishBottomTarget(double p){assert(p==0.);++publications;}
 void publishPressureTarget(const std::array<double,4>& p,double rate) {
   assert(p==grasp_pressure_ && std::isinf(rate));++publications;
 }
 void enterState(int s,ros::Time){state=s;}
 void timeoutTransition(ros::Time now){do {TIMEOUT} while(false);}
 ros::Time mocap_stamp_{10};double inchworm_mocap_timeout_sec_=1.;
 double bottom_pressure_kpa_=0.,action_c_bottom_deflation_tolerance_kpa_=5.;
 struct EndpointWindow {bool ready=true;bool stable(double)const{return ready;}}
   hybrid_endpoint_mocap_window_;
 double action_c_direction_sign_x_=1., action_c_absolute_target_x_m_=.1;
 double inchworm_min_cycle_progress_m_=.001, commanded_distance=0.;
 std::array<double,3> mocap_position_{{.04,0,0}};
 std::array<double,2> action_c_resume_direction_{{1,0}}, correction_axis_{};
 std::array<bool,4> prepare_arm_active_{};
 bool correction_pair_reverse_spinal_mapping_=false;
 int finished=0, starts=0;
 void finishHybridLocomotion(const ros::Time&,bool,bool) {if(hybrid_trial_active_) ++finished; hybrid_trial_active_=false;}
 void startInchwormStep(std::array<double,2>,double d,ros::Time) {
   assert(action_c_preserve_adaptation_on_start_); commanded_distance=d;active=true;++starts;hybrid_trial_active_=true;
 }
 bool resumeActionCAfterRecovery(const ros::Time&);
 Vector3 locomotionPosition()const{return mocap_position_;}
 bool locomotionMocapFresh(const ros::Time& now)const {
   const double age=(now-mocap_stamp_).toSec();return mocap_received_&&age>=0.&&age<=inchworm_mocap_timeout_sec_;
 }
 void selectContinuousRecoveryPair(double);
 bool hybridResumeAllowed(const ros::Time&) {return true;} // deadline tested separately
 void finishHybridRecovery(const ros::Time& now) {finishHybridLocomotion(now,false,true);}
 void select(double roll_error) { PAIR }
};
RESUME
HELPER
int main() {
 for(double sign: {-1.,1.}) {
   AttitudePressureController c;
   c.action_c_direction_sign_x_=sign;c.action_c_resume_direction_[0]=sign;
   c.action_c_absolute_target_x_m_=.1*sign;c.mocap_position_[0]=.04*sign;
   assert(c.resumeActionCAfterRecovery({10.1}));
   assert(c.active && !c.action_c_resume_pending_);
   assert(std::abs(c.commanded_distance-.06)<1e-9);
   assert(c.action_c_absolute_target_x_m_==.1*sign && c.finished==1 && c.starts==1);
   assert(!c.resumeActionCAfterRecovery({10.2})); // One automatic resume, not repeated starts.
   c.select(-sign*.2); // Current +/-roll, target zero.
   assert(c.prepare_arm_active_[0]==(sign<0) && c.prepare_arm_active_[1]==(sign<0));
   assert(c.prepare_arm_active_[2]==(sign>0) && c.prepare_arm_active_[3]==(sign>0));
   hugmy::ContinuousRollRecovery law;law.reset(.2,0.);
   hugmy::RollRecoveryConfig cfg; hugmy::RollRecoveryModel model;
   hugmy::RollRecoveryResponse response;
   double v=law.update((-sign*.2)*c.correction_axis_[0],0.,.02,.02,model,cfg,response);
   assert(v>0. && !law.finished && !law.fault); // Both signs actually produce thrust.
 }
 AttitudePressureController stale;
 assert(!stale.resumeActionCAfterRecovery({12.}));
 assert(stale.action_c_resume_pending_ && !stale.active && stale.finished==0);
 stale.mocap_stamp_={12.}; assert(stale.resumeActionCAfterRecovery({12.1}));
 AttitudePressureController soft;
 soft.continuous_recovery_waiting_for_settle_=true;soft.continuous_recovery_blocked_=true;
 soft.hybrid_trial_active_=false; // evaluated at timeout, before manual motion
 assert(soft.resumeActionCAfterRecovery({10.1}));
 assert(!soft.continuous_recovery_blocked_ && !soft.continuous_recovery_waiting_for_settle_);
 assert(soft.active && soft.finished==0 && soft.starts==1); // no double trial evaluation
 for(bool overall:{false,true}) {
   AttitudePressureController waiting;
   waiting.continuous_roll_recovery_.timed_out=!overall;
   waiting.continuous_roll_recovery_.fault=!overall;
   waiting.timeoutTransition({overall?61.:11.});
   assert(waiting.action_c_resume_pending_ && !waiting.continuous_recovery_blocked_);
   assert(waiting.continuous_recovery_waiting_for_settle_ && waiting.state==waiting.RECOVER);
   assert(!waiting.continuous_recovery_started_);
   assert(waiting.finished==0 && waiting.publications==3 && !waiting.active);
   assert(waiting.action_c_resume_gate_.resets==1 && waiting.action_c_boundary_settle_gate_.resets==1);
   waiting.mocap_stamp_={62.};assert(waiting.resumeActionCAfterRecovery({62.1}));
   assert(waiting.active && waiting.finished==1 && waiting.starts==1);
 }
 AttitudePressureController hard;
 hard.continuous_roll_recovery_.fault=true;hard.timeoutTransition({61.});
 assert(hard.continuous_recovery_waiting_for_settle_ && !hard.continuous_recovery_blocked_ &&
        !hard.continuous_recovery_started_ && hard.publications==3);
 AttitudePressureController no_goal;no_goal.action_c_resume_pending_=false;
 no_goal.timeoutTransition({61.});assert(!no_goal.continuous_recovery_waiting_for_settle_);
 AttitudePressureController done;done.mocap_position_[0]=.11;
 assert(done.resumeActionCAfterRecovery({10.1}));assert(!done.active && !done.action_c_resume_pending_);
 AttitudePressureController endpoint;
 endpoint.bottom_pressure_kpa_=8.;
 assert(!endpoint.resumeActionCAfterRecovery({10.1}));
 endpoint.bottom_pressure_kpa_=0.;endpoint.hybrid_endpoint_mocap_window_.ready=false;
 assert(!endpoint.resumeActionCAfterRecovery({10.2}));
 endpoint.hybrid_endpoint_mocap_window_.ready=true;
 assert(endpoint.resumeActionCAfterRecovery({10.3}));
 puts("PASS: automatic continuation +/-x, preserved goal, stale-mocap wait, and +/-roll pair-to-thrust symmetry");
}
'''.replace('PAIR',pair).replace('RESUME',resume).replace('HELPER',helper).replace('TIMEOUT',timeout)
with tempfile.TemporaryDirectory(prefix='hugmy_resume_') as tmp:
 binary=str(pathlib.Path(tmp)/'test')
 subprocess.run(['g++','-std=c++14','-Wall','-Wextra','-I',str(root/'include'),
                 '-x','c++','-','-o',binary],input=code,text=True,check=True)
 subprocess.run([binary],check=True)
