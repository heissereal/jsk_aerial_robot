#!/usr/bin/env python3
"""Compile production auto-resume and continuous recovery sign selection."""
import pathlib
import subprocess
import tempfile
root=pathlib.Path(__file__).resolve().parents[1]
source=(root/'src/control/attitude_pressure_controller.cpp').read_text()
start=source.index('bool AttitudePressureController::resumeActionCAfterRecovery(')
resume=source[start:source.index('bool AttitudePressureController::actionCRearReleased(',start)]
start=source.index('const bool retry_opposite_pair =')
pair=source[start:source.index('continuous_roll_recovery_.reset(',start)]
helper_start=source.index('void AttitudePressureController::selectContinuousRecoveryPair(')
helper=source[helper_start:source.index('\n}\n',helper_start)+3]
start=source.index('const bool progress_timeout = continuous_roll_recovery_.timed_out')
timeout=source[start:source.index('// No repeated timed search pulses',start)]
assert 'recovered = pressure_valid && actionCPressureReady(grasp_pressure_, now)' in source
assert 'if (resumeActionCAfterRecovery(now)) break;' in source
assert 'support_transfer_equalized &&\n                  !std::isfinite(recovery_handoff_best_error_rad_)' in source
assert 'recovery_handoff_best_error_rad_ = abs_error;' in source
assert 'continuous_recovery_retry_opposite_pending_ =\n                      !recovery_candidate_reselect_pending_ &&' in source
assert 'recovery_rejected_candidate_mask_ |= candidate_bit;' in source
assert 'recovery_candidate_reselect_pending_ = true;' in source
assert 'continuous_roll_recovery_.target_crossed' in source
assert 'opposite-sign selection only if the residual leaves the stop band' in source
assert 'requested immediate opposite-sign geometry selection' in source
assert 'recoveryMeasuredContactPressure()' in source
measured_freeze = source.split(
    'AttitudePressureController::recoveryMeasuredContactPressure() const', 1
)[1].split('double AttitudePressureController::recoverySupportPressure()', 1)[0]
assert 'prepare_arm_active_[arm]' not in measured_freeze
assert 'measured_pressure_[arm], recovery_handoff_min_pressure_kpa_' in measured_freeze
assert 'recoveryReselectionPressure()' in source
assert 'clearing directional rejection mask' in source
assert 'const bool handoff_candidate_stalled =' in source
assert 'const bool handoff_candidate_stalled =\n                      abs_error > capture_error &&' in source
assert 'full-grasp roll remained at %.2f deg' in source
assert 'recovery_handoff_progress_reference_rad_ -' in source
assert 'const double handoff_retry_thrust = std::max(\n                      roll_recovery_config_.detect_thrust' in source
assert 'rejected handoff candidate reached zero thrust' in source
assert 'released terminal-brake deadband' in source
assert 'const double handoff_brake_entry = recovery_stop_angle;' in source
assert '!recovery_handoff_braking_ && command_at_grasp &&\n                  abs_error <= (strict_target_observed_before_regrasp' in source
assert 'const bool terminal_contact_plateau =' in source
assert 'if (terminal_contact_plateau)' in source
assert 'const bool terminal_pressure_search =' in source
assert 'signed_rate <= pressure_feedforward_rate_limit' in source
assert 'signed_error <= 0.0 ||' in source
assert 'recovery_contact_grasp_active_' in source
assert 'pressure_search_floor_for(arm)' in source
assert 'pressure_distance_from_best' in source
assert 'const double terminal_reopen_margin =' in source
assert 'const bool strict_target_observed_before_regrasp =' in source
assert 'recovery_handoff_best_error_rad_ <= handoff_settle_error' in source
assert 'const double active_handoff_settle_error =' in source
assert 'active_handoff_settle_error + terminal_reopen_margin' in source
assert 'abs_error >= terminal_reopen_error' in source
assert 'retained_handoff_error +\n                      terminal_reopen_margin' in source
assert 'abs_error <= handoff_drive_stop))' in source
assert 'abs_error > active_handoff_settle_error' in source
assert 'abs_error <= active_handoff_settle_error;' in source
assert 'monitoring final nominal regrasp' in source
hybrid_source=(root/'src/control/hybrid_locomotion.cpp').read_text()
assert 'pre_stroke_budget_exhausted=settled && hybrid_phase_==0' in hybrid_source
assert '!hybrid_preinflate_measured_' in hybrid_source
assert 'worsening_residual || pre_stroke_budget_exhausted' in hybrid_source
assert 'hybrid_single_config_.pre_stroke_roll_limit :' in hybrid_source
assert 'action_c_requested_recovery_trigger_rad_' in source
code=r'''
#include <hugmy/control/continuous_roll_recovery.h>
#include <hugmy/control/circular_grasp_geometry.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_ERROR(...) ((void)0)
namespace ros {struct Time {double t=0.;double toSec()const{return t;}bool isZero()const{return t==0.;}static Time now(){return {10.};}};
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
 bool continuous_recovery_use_opposite_pair_=false;
 bool continuous_recovery_retry_opposite_pending_=false;
 bool recovery_handoff_thrust_wrong_way_=false;
 bool recovery_handoff_pair_was_wrong_=false;
 bool recovery_handoff_pressure_trimmed_=false;
 unsigned continuous_recovery_pair_switches_=0;
 unsigned char recovery_rejected_candidate_mask_=0;
 int recovery_rejected_direction_sign_=0;
 bool recovery_circle_event_locked_=false;
 bool recovery_preferred_pair_locked_=false;
 size_t recovery_preferred_pair_index_=0;
 bool recovery_preferred_pair_used_fallback_=false;
 bool recovery_candidate_reselect_pending_=false;
 bool recovery_pair_selection_available_=true;
 size_t recovery_selected_pair_index_=0;
 hugmy::ContinuousRollRecovery continuous_roll_recovery_;
 struct Gate {int resets=0;void reset(){++resets;}} action_c_resume_gate_, action_c_boundary_settle_gate_;
 ros::Time state_start_{0};double prepare_timeout_=60.;
 double action_c_requested_recovery_trigger_rad_=NAN;
 std::array<double,4> prepare_shape_thrust_{{2.,2.,2.,2.}}, grasp_pressure_{{30.,30.,30.,30.}};
 std::array<double,4> angles{{M_PI/2,M_PI/2,M_PI/2,M_PI/2}};
 std::array<double,4> recovery_handoff_target_pressure_{};
 double recovery_handoff_min_pressure_kpa_=15.,base_pressure_kpa_=30.,maximum_pressure_kpa_=40.;
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
 bool recovery_circle_selection_enabled_=false,recovery_selected_pair_release_=true;
 bool recovery_selected_contact_pivot_enabled_=false;
 double recovery_selected_pair_mean_depth_rad_=NAN;
 double recovery_circle_min_confidence_=.35,recovery_circle_max_age_sec_=30.;
 double recovery_circle_side_margin_rad_=5*M_PI/180.;
 double recovery_circle_flexed_max_error_rad_=15*M_PI/180.;
 double recovery_circle_release_penalty_n_=.75,recovery_circle_min_signed_lever_m_=.005;
 double roll_moment_kp_=.1,target_roll_=0.,roll_=-.2;
 Vector3 gravity_body_{{0.,0.,-9.81}};
 double recovery_release_pressure_kpa_=0.,correction_planning_thrust_n_=2.,bend_model_thrust_offset_n_=1.8;
 hugmy::CircularGraspEstimate recovery_circle_estimate_;ros::Time recovery_circle_stamp_{};
 hugmy::RollRecoveryConfig roll_recovery_config_;hugmy::PressureThrustBendModel bend_model_;
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
 double measuredArmBendAngle(size_t arm)const{return angles[arm];}
 Vector3 armContactPointFromAngle(size_t arm,double bend)const{
   return {{0.,arm<2?.04:-.04,-.07-.01*std::sin(bend)}};
 }
 Vector3 recoveryContactPivot(const std::array<bool,4>&,bool)const{
   return {{0.,0.,-.07}};
 }
 double recoveryRotorRawLeverAbout(size_t arm,double bend,const Vector3&)const{
   const double pair01=bend<1.8?.04:-.04;return arm<2?pair01:-pair01;
 }
 double recoveryRotorRawLever(size_t arm,double bend)const{
   return recoveryRotorRawLeverAbout(arm,bend,{{0.,0.,-.07}});
 }
 bool hybridResumeAllowed(const ros::Time&) {return true;} // deadline tested separately
 void finishHybridRecovery(const ros::Time& now) {finishHybridLocomotion(now,false,true);}
 void resetRecoveryHandoffState() {}
 void stopInchwormToGrasp(const ros::Time&,const char*) {active=false;}
 void select(double roll_error) {ros::Time now{10.};do { PAIR } while(false);}
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
 // With a valid circle, a restorative pair below the object side remains
 // flexed/grasping; above the side it is deliberately released instead.
 AttitudePressureController circle;
 circle.recovery_circle_selection_enabled_=true;
 circle.recovery_circle_estimate_.valid=true;
 circle.recovery_circle_estimate_.confidence=1.;
 circle.recovery_circle_estimate_.radius=.05;
 circle.recovery_circle_estimate_.rms=.001;
 circle.recovery_circle_estimate_.depth_angle={{2.,2.,2.,2.}};
 circle.recovery_circle_stamp_={9.9};
 circle.select(.2);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
 assert(!circle.recovery_selected_pair_release_);
 // Reject only the measured flexed 0+1 contact state. Re-selection keeps
 // the geometrically correct pair but changes it to released/extended.
 circle.recovery_rejected_candidate_mask_=1u;
 circle.select(.2);
 assert(circle.recovery_pair_selection_available_);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
 assert(circle.recovery_selected_pair_release_);
 // Once both usable 0+1 states are disproved, this synthetic geometry has no
 // signed restoring 2+3 candidate and must refuse the fixed-map fallback.
 circle.recovery_rejected_candidate_mask_=3u;
 circle.select(.2);
 assert(!circle.recovery_pair_selection_available_);
 circle.recovery_rejected_candidate_mask_=0;
 // A large roll cannot use the flexed-contact shortcut even when that pair
 // remains below the circle side; it must transfer support before recovery.
 circle.select(.35);
 assert(circle.recovery_selected_pair_release_);
 circle.recovery_circle_estimate_.depth_angle[0]=1.;
 circle.recovery_circle_estimate_.depth_angle[1]=1.;
 circle.select(.2);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
 assert(circle.recovery_selected_pair_release_);
 // A later bend change cannot move actuation to the upper pair in the same
 // recovery event. The chosen lower-side pair remains locked; only its
 // contact state and thrust may be recomputed.
 circle.angles.fill(2.1);
 circle.recovery_circle_estimate_.depth_angle={{2.,2.,2.,2.}};
 circle.select(.2);
 assert(circle.recovery_preferred_pair_locked_ &&
        circle.recovery_preferred_pair_index_==0);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
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
   // A timed-out rotor pass now remains marked started until the same
   // closed-loop pneumatic handoff completes; it must not jump directly from
   // released pressure to nominal grasp.
   assert(waiting.continuous_recovery_started_);
   assert(waiting.finished==0 && waiting.publications==0 && !waiting.active);
   assert(waiting.action_c_resume_gate_.resets==1 && waiting.action_c_boundary_settle_gate_.resets==1);
   waiting.mocap_stamp_={62.};assert(waiting.resumeActionCAfterRecovery({62.1}));
   assert(waiting.active && waiting.finished==1 && waiting.starts==1);
 }
 AttitudePressureController hard;
 hard.continuous_roll_recovery_.fault=true;hard.timeoutTransition({61.});
 assert(hard.continuous_recovery_waiting_for_settle_ && !hard.continuous_recovery_blocked_ &&
        hard.continuous_recovery_started_ && hard.publications==0);
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
