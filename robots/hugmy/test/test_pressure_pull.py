#!/usr/bin/env python3
"""Run the production pressure-pull methods with data-only ROS stubs.

No ROS master, actuator publication, simulation process, or hardware required.
"""
import pathlib
import subprocess
import tempfile


def main():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / 'src/control/attitude_pressure_controller.cpp').read_text()
    start = source.index('bool AttitudePressureController::actionCRearReleased(')
    end = source.index('double AttitudePressureController::actionCBottomTarget()', start)
    methods = source[start:end]
    # Verify the ON/EVALUATE integration, not just the isolated new methods.
    on = source[source.index('case ACTION_C_HORIZONTAL_PULL:'):source.index('case ACTION_C_EVALUATE:')]
    assert on.index('!actionCRearReleased(now)') < on.index('action_c_pulse_started_ = true;')
    assert 'action_c_abort_reverse_travel_m_' in on
    assert 'extension_guard || pulse_timeout)' in on
    assert 'transient ON retreat' in on and 'recoil_handoff' not in on
    assert '(action_c_pressure_pull_active_ ? 0.0 : base_pressure_kpa_)' in on
    assert 'std::min(action_c_command_thrust_n_, residual)' in on
    evaluate = source[source.index('case ACTION_C_EVALUATE:'):source.index('case ACTION_C_REGRIP:')]
    assert evaluate.index('updateActionCPressurePull(now);') < evaluate.index('approach(')
    code = r'''
#include <hugmy/control/action_c_feedback.h>
#include <hugmy/control/pulse_bend_observer.h>
#include <hugmy/control/posture_recovery_gate.h>
#include <hugmy/control/roll_trend_gate.h>
#include <hugmy/control/rear_release_gate.h>
#include <hugmy/control/action_c_propulsion.h>
#include <array>
#include <vector>
#include <limits>
#include <cassert>
#include <cstdio>
#define ROS_INFO(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_WARN(...) ((void)0)
namespace ros {
struct Time { double value=0.; Time(double v=0.):value(v){}
  double toSec() const {return value;}
  bool isZero() const {return value==0.;}
  Time operator-(Time rhs) const {return Time(value-rhs.value);}
};
}
namespace std_msgs {struct Float32MultiArray {std::vector<float> data;};}
struct Publisher {template<class T> void publish(const T&) {}};
class AttitudePressureController {
public:
static constexpr size_t ARM_COUNT=4;
using PressureArray=std::array<double,4>;
bool actionCRearReleased(const ros::Time& now);
hugmy::RearReleaseGate action_c_rear_release_gate_;
double action_c_rear_release_hold_kpa_=10, action_c_rear_release_hard_kpa_=12;
double action_c_rear_release_grace_sec_=.1;
double actionCBottomTarget() const {return action_c_bottom_hold_kpa_;}
void updateActionCPressurePull(const ros::Time& now);
void observeActionCPropulsion(const ros::Time&) {}
hugmy::ActionCPropulsion action_c_propulsion_;
bool action_c_pulse_had_extension_=true;
bool action_c_propulsion_enabled_=false;
double action_c_powered_regrip_timeout_sec_=3.;
std::array<bool,4> inchworm_front_{{true,false,false,true}};
PressureArray measured_pressure_{{5,4,4,5}}, inchworm_pressure_{};
PressureArray inchworm_thrust_{{2,0,0,2}}, action_c_front_pressure_{{30,30,30,30}};
PressureArray action_c_pressure_pull_ceiling_n_{{1.05,0,0,1.05}};
PressureArray action_c_pulse_start_angle_rad_{{1.3,0,0,1.3}};
PressureArray action_c_pulse_min_angle_rad_{{1.15,0,0,1.15}}, action_c_front_angle_rad_{};
PressureArray grip_joint_bend_rad_, arm_yaw_{};
std::array<std::array<double,3>,4> neuron_gyro_{};
std::array<double,3> body_angular_velocity_{};
std::array<ros::Time,4> neuron_stamp_;
std::array<hugmy::PulseBendObserver,4> action_c_pulse_observer_;
ros::Time state_start_{10}, grip_state_stamp_, action_c_recovery_ready_start_;
ros::Time measured_pressure_stamp_{10};
hugmy::PostureRecoveryGate action_c_recovery_gate_;
hugmy::RollTrendGate action_c_roll_trend_;
bool action_c_pulse_roll_handoff_enabled_=true;
double action_c_pulse_roll_handoff_lookahead_sec_=.1, action_c_pulse_roll_handoff_margin_rad_=2*M_PI/180;
double action_c_posture_hard_roll_rad_=15*M_PI/180;
Publisher action_c_pulse_bend_pub_;
bool action_c_pressure_pull_active_=true, action_c_pulse_overextended_=false;
bool action_c_optimizer_trial_had_extension_guard_=false;
unsigned int action_c_optimizer_trial_guard_reasons_=0;
double roll_=0, pitch_=0, target_roll_=0;
double action_c_posture_recovery_trigger_rad_=8*M_PI/180, correction_trigger_confirm_sec_=.3;
double action_c_posture_rate_trigger_rad_s_=.8, action_c_max_body_tilt_change_rad_=10*M_PI/180;
double action_c_rear_release_max_kpa_=8, action_c_command_thrust_n_=2;
double action_c_extension_rate_limit_rad_s_=M_PI, action_c_min_safe_angle_rad_=55*M_PI/180;
double action_c_hard_min_bend_rad_=30*M_PI/180, action_c_start_extension_margin_rad_=2*M_PI/180;
double action_c_max_extension_rad_=20*M_PI/180, action_c_rebend_target_rad_=6*M_PI/180;
double action_c_extension_target_rad_=8*M_PI/180, action_c_max_extension_mismatch_rad_=15*M_PI/180;
double action_c_optimizer_peak_extension_mismatch_rad_=0;
double action_c_abort_reverse_travel_m_=.03, action_c_pulse_start_travel_m_=0;
double action_c_pulse_start_roll_=0, action_c_pulse_start_pitch_=0;
double action_c_pressure_pull_timeout_sec_=1.5, base_pressure_kpa_=30;
double pressure_tolerance_kpa_=1, action_c_bottom_hold_kpa_=30;
bool stopped=false, recovered=false, pressure_live=true; double travel=0;
static constexpr int ACTION_C_EVALUATE=1;
void enterState(int, ros::Time now) {state_start_=now;}
void stopInchwormToGrasp(ros::Time,const char*) {stopped=true;inchworm_thrust_.fill(0);}
void requestActionCPostureRecovery(ros::Time,const char*) {recovered=true;}
void publishCorrectionThrust(const PressureArray&) {}
void publishPressureTarget(const PressureArray&,double) {}
void publishBottomTarget(double) {}
double inchwormTravel() {return travel;}
double measuredArmBendAngle(size_t arm) {return grip_joint_bend_rad_[arm];}
static double wrapAngle(double x) {return std::atan2(std::sin(x),std::cos(x));}
AttitudePressureController() {
 grip_joint_bend_rad_.fill(std::numeric_limits<double>::quiet_NaN());
 for(auto& o:action_c_pulse_observer_) o.seed(1.15,0,10.);
}
void tick(int i) {
 double t=10.+.02*i;
 if(pressure_live) measured_pressure_stamp_=ros::Time(t);
 neuron_stamp_.fill(ros::Time(t));
 updateActionCPressurePull(ros::Time(t));
}
};
METHODS
int main() {
 using C=AttitudePressureController;
 C c;
 for(int i=1;i<=20;++i) {
   c.tick(i);
   assert(!c.stopped && c.action_c_pressure_pull_active_);
   assert(c.inchworm_pressure_[1]==0 && c.inchworm_pressure_[2]==0);
   assert(c.inchworm_pressure_[0]==30 && c.inchworm_pressure_[3]==30);
   assert(c.inchworm_thrust_[0]==1.05 && c.inchworm_thrust_[3]==1.05);
 }
 // Pressure-driven flexion: integrate real positive relative gyro; residual
 // is retained, then tapers. Rear regrip occurs only after flexion + pressure.
 c.measured_pressure_[0]=c.measured_pressure_[3]=30;
 c.neuron_gyro_[0][1]=c.neuron_gyro_[3][1]=.5;
 double previous=1.05;
 for(int i=21;i<=40 && c.action_c_pressure_pull_active_;++i) {
   c.tick(i);
   assert(c.inchworm_thrust_[0]<=previous); previous=c.inchworm_thrust_[0];
   if(c.action_c_pressure_pull_active_) assert(c.inchworm_pressure_[1]==0);
 }
 assert(!c.action_c_pressure_pull_active_ && !c.stopped);
 assert(c.action_c_propulsion_.pulse_completed);
 assert(c.inchworm_thrust_[0]==0 && c.inchworm_pressure_[1]==30);
 assert(c.state_start_.toSec()>10.4); // OFF timer restarted after pull.
 // No re-boost if rebound reverses or pressure is delayed.
 assert(hugmy::actionCPressurePullThrust(.3,1.,0.,.1,true)==.3);
 assert(hugmy::actionCPressurePullThrust(1.,1.,0.,.1,false)==0);
 assert(hugmy::actionCPressurePullThrust(1.,1.,NAN,.1,true)==0);
 // Pressure never reaches target: stop rather than power indefinitely.
 C timeout;
 for(int i=1;i<=76 && !timeout.stopped;++i) timeout.tick(i);
 assert(timeout.stopped && !timeout.action_c_pressure_pull_active_);
 // Pressure ready but no rebound: bounded end (no false angle success).
 C no_bend; no_bend.measured_pressure_[0]=no_bend.measured_pressure_[3]=30;
 for(int i=1;i<=76 && no_bend.action_c_pressure_pull_active_;++i) no_bend.tick(i);
 assert(!no_bend.stopped && !no_bend.action_c_pressure_pull_active_);
 assert(!no_bend.action_c_propulsion_.pulse_completed);
 // New normal-C sequence: keep thrust through flexion AND rear regrip.
 C held;held.action_c_propulsion_enabled_=true;held.action_c_pressure_pull_timeout_sec_=5.;
 held.action_c_propulsion_.both_powered_sec=.5;
 held.measured_pressure_[0]=held.measured_pressure_[3]=30;
 held.neuron_gyro_[0][1]=held.neuron_gyro_[3][1]=.5;
 for(int i=1;i<=12;++i) held.tick(i);
 assert(held.action_c_propulsion_.regrip_start>0 && held.action_c_pressure_pull_active_);
 assert(held.inchworm_thrust_[0]==1.05 && held.inchworm_pressure_[1]==30);
 held.neuron_gyro_[0][1]=held.neuron_gyro_[3][1]=0;
 for(int i=13;i<=100;++i) held.tick(i);
 assert(!held.stopped && held.action_c_pressure_pull_active_ && held.inchworm_thrust_[3]==1.05);
 held.measured_pressure_[1]=held.measured_pressure_[2]=30;
 held.tick(101); assert(held.action_c_pressure_pull_active_); // confirmation, not one sample
 for(int i=102;i<=112 && held.action_c_pressure_pull_active_;++i) held.tick(i);
 assert(!held.stopped && !held.action_c_pressure_pull_active_ && held.action_c_propulsion_.pulse_completed);
 assert(held.inchworm_thrust_[0]==0 && held.inchworm_pressure_[2]==30);
 C blocked;blocked.action_c_propulsion_enabled_=true;blocked.action_c_pressure_pull_timeout_sec_=.4;
 blocked.measured_pressure_[0]=blocked.measured_pressure_[3]=30;
 for(int i=1;i<=30 && !blocked.stopped;++i) blocked.tick(i);
 assert(blocked.stopped && !blocked.action_c_propulsion_.pulse_completed); // no false success on timeout
 C rear_wait;rear_wait.action_c_propulsion_enabled_=true;
 rear_wait.action_c_propulsion_.regrip_start=10.; rear_wait.action_c_powered_regrip_timeout_sec_=.3;
 for(int i=1;i<=20 && !rear_wait.stopped;++i) rear_wait.tick(i);
 assert(rear_wait.stopped && rear_wait.inchworm_thrust_[0]==0);
 C stale_regrip;stale_regrip.action_c_propulsion_enabled_=true;
 stale_regrip.action_c_propulsion_.regrip_start=10.;stale_regrip.pressure_live=false;
 stale_regrip.tick(15);assert(stale_regrip.stopped);
 // Every rear sensor is checked, for forward AND reverse travel groupings.
 for(int reverse=0;reverse<2;++reverse) for(int arm=0;arm<4;++arm) {
   C r; if(reverse) for(auto& front:r.inchworm_front_) front=!front;
   r.measured_pressure_.fill(4);
   r.measured_pressure_[arm]=9;
   assert(r.actionCRearReleased(ros::Time(10))==r.inchworm_front_[arm]);
   if(!r.inchworm_front_[arm]) {r.tick(1);assert(r.stopped);}
 }
 C nan; nan.measured_pressure_[1]=NAN; nan.tick(1); assert(nan.stopped);
 C stale; stale.measured_pressure_stamp_=9;stale.pressure_live=false;stale.tick(1);assert(stale.stopped);
 C gyro; gyro.neuron_gyro_[0][1]=NAN;gyro.tick(1);assert(gyro.stopped);
 assert(gyro.action_c_optimizer_trial_guard_reasons_ & hugmy::ACTION_C_ANGLE_UNRELIABLE);
 C retreat;retreat.travel=-.031;retreat.tick(1);assert(retreat.stopped);
 assert(retreat.action_c_optimizer_trial_guard_reasons_ & hugmy::ACTION_C_REVERSE_TRAVEL);
 C small;small.travel=-.006;small.tick(1);assert(!small.stopped);
 C rate;rate.body_angular_velocity_[0]=.9;rate.tick(1);assert(rate.recovered);
 C roll;roll.roll_=9*M_PI/180;
 for(int i=1;i<=18 && !roll.recovered;++i) roll.tick(i);
 assert(!roll.recovered); // Ordinary 8-deg assessment waits for recoil/regrip.
 for(double sign:{-1.,1.}) {
   C hard;hard.roll_=sign*16*M_PI/180;hard.action_c_pulse_start_roll_=hard.roll_;
   hard.tick(1);assert(hard.recovered && !hard.action_c_pressure_pull_active_);
   C near;near.roll_=sign*9*M_PI/180;near.action_c_pulse_start_roll_=sign*7*M_PI/180;
   near.body_angular_velocity_[0]=sign*.05;
   near.action_c_pulse_start_angle_rad_.fill(45*M_PI/180);
   near.action_c_pulse_min_angle_rad_.fill(36*M_PI/180);
   for(auto& o:near.action_c_pulse_observer_) o.seed(36*M_PI/180,0,10.);
   near.tick(1);
   assert(near.recovered && !near.action_c_pressure_pull_active_ && !near.stopped);
   assert(near.action_c_optimizer_trial_guard_reasons_==0); // predicted, not actual violation
 }
 puts("PASS: production pressure-pull sequence, rear release, gyro flexion, bounded residual, timeout and safety stops");
}
'''.replace('METHODS', methods)
    with tempfile.TemporaryDirectory(prefix='hugmy_pressure_pull_') as directory:
        binary = str(pathlib.Path(directory) / 'test')
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-I', str(root / 'include'),
                        '-x', 'c++', '-', '-o', binary], input=code, text=True, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
