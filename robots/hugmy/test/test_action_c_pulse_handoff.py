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
    adapter = (root / 'src/control/hybrid_locomotion.cpp').read_text()
    assert 'requestActionCPostureRecovery(now,' in adapter
    assert 'pauseHybridLocomotion(now)' in request
    code = r'''
#include <cmath>
#include <array>
#include <vector>
#include <cassert>
#include <limits>
#define ROS_WARN(...) ((void)0)
namespace ros {struct Time {};}
struct AttitudePressureController {
 bool target_received_=true;
 std::array<double,3> mocap_position_{{.2,0,0}};
 int action_c_posture_recovery_count_=0, action_c_max_posture_recoveries_=5;
 bool continuous_recovery_started_=true, continuous_recovery_waiting_for_settle_=true;
 bool continuous_recovery_use_opposite_pair_=false;
 bool continuous_recovery_retry_opposite_pending_=false;
 bool recovery_candidate_reselect_pending_=false;
 bool recovery_circle_event_locked_=false;
 bool recovery_preferred_pair_locked_=false;
 unsigned continuous_recovery_pair_switches_=0;
 unsigned recovery_rejected_candidate_mask_=0;
 int recovery_rejected_direction_sign_=0;
 struct Gate {void reset(){}} action_c_recovery_gate_;
 bool action_c_resume_pending_=false;
 std::array<double,2> action_c_resume_direction_{}, inchworm_direction_body_{{1,0}};
 double base_pressure_kpa_=30.;
 double action_c_requested_recovery_trigger_rad_=std::numeric_limits<double>::quiet_NaN();
 double action_c_posture_recovery_trigger_rad_=0.2;
 double action_c_absolute_target_x_m_=0.1;
 std::array<double,4> inchworm_thrust_{{3.,0.,0.,2.8}}, inchworm_pressure_{};
 bool inchworm_active_=true, stopped=false;
 enum {GRASP=1}; int state=0;
 std::vector<int> publications;
 void stopInchwormToGrasp(ros::Time,const char*) {stopped=true;}
 void publishCorrectionThrust(const std::array<double,4>& t) {
   for(double v:t) assert(v==0.);publications.push_back(0);
 }
 void publishPressureTarget(const std::array<double,4>& p,double rate) {
   for(double v:p) assert(v==30.);assert(std::isinf(rate));publications.push_back(1);
 }
 void publishBottomTarget(double p) {assert(p==0.);publications.push_back(2);}
 void enterState(int s,ros::Time){state=s;}
 void requestActionCPostureRecovery(const ros::Time&,const char*,
     double=std::numeric_limits<double>::quiet_NaN());
 void pauseHybridLocomotion(const ros::Time&) {} // isolated accounting hook
 std::array<double,3> locomotionPosition() const {return mocap_position_;}
};
REQUEST
int main() {
 for(double sign:{-1.,1.}) {
   AttitudePressureController c;c.inchworm_direction_body_[0]=sign;
   c.requestActionCPostureRecovery({},"test");
   assert((c.publications==std::vector<int>{0,1,2}));
   assert(c.state==c.GRASP && !c.inchworm_active_ && !c.stopped);
   assert(c.action_c_resume_pending_ && c.action_c_resume_direction_[0]==sign);
   assert(c.action_c_posture_recovery_count_==1);
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
    print('PASS: predictive C handoff preserves direction, zeros thrust before pressure/recovery, requires a target')


if __name__ == '__main__':
    main()
