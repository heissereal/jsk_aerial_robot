#!/usr/bin/env python3
"""Production roll/bend decisions and zero-output recovery ownership, no ROS."""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    src = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    start = src.index('void AttitudePressureController::requestActionCPostureRecovery(')
    request = src[start:src.index('bool AttitudePressureController::resumeActionCAfterRecovery(', start)]
    on = src[src.index('case ACTION_C_HORIZONTAL_PULL:'):src.index('case ACTION_C_EVALUATE:')]
    branch = on[on.index('if (action_c_pulse_roll_handoff_enabled_'):on.index('const double common_front_thrust')]
    assert 'requestActionCPostureRecovery(now,' in branch and 'return;' in branch
    assert 'action_c_optimizer_trial_guard_reasons_ |= guard_reasons;' in branch
    assert 'const bool pressure_pull_mode = action_c_pressure_pull_enabled_;' in on
    assert 'if (action_c_pressure_pull_enabled_ &&' in on
    assert on.count('action_c_reference_pulse_mode_ || action_c_pressure_pull_enabled_') >= 3
    code = r'''
#include <hugmy/control/action_c_feedback.h>
#include <array>
#include <vector>
#include <cassert>
#include <limits>
#define ROS_WARN(...) ((void)0)
namespace ros {struct Time {};}
struct AttitudePressureController {
 bool action_c_adaptive_burst_enabled_=true, target_received_=true;
 bool action_c_optimizer_trial_active_=true, action_c_optimizer_recovery_pending_=false;
 double action_c_optimizer_recovery_start_x_m_=0.;
 std::array<double,3> mocap_position_{{.2,0,0}};
 int action_c_posture_recovery_count_=0, action_c_max_posture_recoveries_=5;
 bool continuous_recovery_started_=true, continuous_recovery_waiting_for_settle_=true;
 struct Gate {void reset(){}} action_c_recovery_gate_;
 bool action_c_resume_pending_=false, action_c_pressure_pull_active_=true;
 struct {bool filling=true;double peak_at=9.;} action_c_bottom_timing_;
 std::array<double,2> action_c_resume_direction_{}, inchworm_direction_body_{{1,0}};
 bool action_c_posture_ready_latched_=true;
 int action_c_burst_pulse_count_=2;
 double action_c_command_thrust_n_=3., base_pressure_kpa_=30.;
 std::array<double,4> inchworm_thrust_{{3.,0.,0.,2.8}}, inchworm_pressure_{};
 bool inchworm_active_=true, stopped=false;
 enum {GRASP=1}; int state=0;
 std::vector<int> publications;
 void stopInchwormToGrasp(ros::Time,const char*) {stopped=true;}
 void finishActionCOptimizerTrial(bool,const char*) {}
 void publishCorrectionThrust(const std::array<double,4>& t) {
   for(double v:t) assert(v==0.);publications.push_back(0);
 }
 void publishPressureTarget(const std::array<double,4>& p,double rate) {
   for(double v:p) assert(v==30.);assert(std::isinf(rate));publications.push_back(1);
 }
 void publishBottomTarget(double p) {assert(p==0.);publications.push_back(2);}
 void enterState(int s,ros::Time){state=s;}
 void requestActionCPostureRecovery(const ros::Time&,const char*);
};
REQUEST
int main() {
 const double d=M_PI/180.;
 auto handoff=[=](double roll,double start,double rate,double margin) {
   return hugmy::actionCPulseRollHandoff(roll*d,start*d,rate*d,margin*d,
       2*d,8*d,15*d,.8,1*d,1*d);
 };
 for(double sign:{-1.,1.}) {
   assert(handoff(sign*10.,sign*8.,sign*3.,1.));
   assert(!handoff(sign*10.,sign*8.,-sign*3.,1.)); // improving
   assert(!handoff(sign*10.,sign*8.,0.,1.)); // stable offset
   assert(!handoff(sign*10.,sign*9.9,sign*3.,1.)); // no real growth
   assert(!handoff(sign*10.,sign*8.,sign*3.,8.)); // room: finish pulse
   assert(!handoff(sign*7.,sign*5.,sign*3.,-1.)); // angle guard, not roll recovery
   assert(handoff(sign*16.,sign*14.,0.,100.)); // hard angle, no OFF wait
   assert(handoff(sign*2.,0.,sign*50.,100.)); // hard gyro
 }
 assert(!handoff(NAN,0.,0.,0.));
 // Forward and reverse tasks preserve their direction, stop all pulse output
 // before pressure transfer, cancel inflation and any pressure-pull owner.
 for(double sign:{-1.,1.}) {
   AttitudePressureController c;c.inchworm_direction_body_[0]=sign;
   c.requestActionCPostureRecovery({},"test");
   assert((c.publications==std::vector<int>{0,1,2}));
   assert(c.state==c.GRASP && !c.inchworm_active_ && !c.stopped);
   assert(c.action_c_resume_pending_ && c.action_c_resume_direction_[0]==sign);
   assert(!c.action_c_pressure_pull_active_ && !c.action_c_bottom_timing_.filling);
   assert(c.action_c_bottom_timing_.peak_at<0. && c.action_c_command_thrust_n_==0.);
   assert(c.action_c_optimizer_recovery_pending_ && c.action_c_posture_recovery_count_==1);
 }
 AttitudePressureController disabled;disabled.target_received_=false;
 disabled.requestActionCPostureRecovery({},"test");assert(disabled.stopped);
}
'''.replace('REQUEST', request)
    with tempfile.TemporaryDirectory(prefix='hugmy_pulse_handoff_') as tmp:
        binary = str(pathlib.Path(tmp) / 'test')
        subprocess.run(['g++', '-std=c++14', '-I'+str(root/'include'), '-x', 'c++', '-', '-o', binary],
                       input=code, text=True, check=True)
        subprocess.run([binary], check=True)
    print('PASS: +/-roll near-bend handoff, stable/improving rejection, hard limits, zero-first ownership, normal-C pressure pull wiring')


if __name__ == '__main__':
    main()
