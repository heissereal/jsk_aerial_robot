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
assert 'Recovery pair %zu+%zu retained a non-restoring measured lever' in source
assert 'const bool wrong_geometry_candidate =' in source
assert 'const bool pressure_settled_insufficient_decay =' in source
assert 'recovery_error_decay_rate <' in source
assert 'recovery_wrong_geometry_min_error_decay_rate_rad_s_' in source
assert 'if (pressure_settled_insufficient_decay ||\n                      geometry_worsened)' in source
assert 'positive_roll_stalled_at_arrest_cap' in source
assert '? roll_recovery_config_.limit\n                  : recovery_rate_arrest_max_thrust_n_' in source
assert 'locomotion goal retained, rebuilding support and retrying' in source
assert 'no restoring recovery pair after balanced-support geometry re-estimation' not in source
assert 'const double active_pressure_target =\n                            prepare_pressure_[arm];' in source
assert 'probe_pressure_target_changed' in source
assert 'recovery_wrong_geometry_probe_target_valid_' in source
assert 'Recovery candidates pair %zu+%zu' in source
assert 'candidate_bend[arm] = 0.0' in source
assert 'grasped_has_authority ? best_grasped : best_extended' in source
assert 'crossed_from_latched_extension' in source
assert 'recovery_extension_direction_sign_ != correction_direction' in source
assert 'const bool stable_small_angle_reselection =' in source
assert 'std::isfinite(measured_roll_rate)' in source
assert 'std::abs(measured_roll_rate) < correction_motion_start_rate_' in source
assert 're-evaluating all current-bend candidates' in source
assert 'recovery_preferred_pair_locked_ = false;' in source
assert 'requiring the opposite physical pair' in source
assert 'recovery_preferred_pair_index_ = 1 - previous_pair;' in source
assert 'const bool large_error_emergency =' in source
assert 'if (large_error_emergency || predicted_emergency)' in source
selector_request = source[source.index(
    'const bool emergency_extension_requested ='
):source.index('if (emergency_extension_requested)', source.index(
    'const bool emergency_extension_requested ='))]
assert 'action_c_recovery_urgent_thrust_rise_' not in selector_request
recovery_request = source[source.index(
    'void AttitudePressureController::requestActionCPostureRecovery('
):source.index('bool AttitudePressureController::resumeActionCAfterRecovery(',
               source.index(
                   'void AttitudePressureController::requestActionCPostureRecovery('))]
assert 'recovery_extension_mode_latched_ = false;' in recovery_request
assert 'recovery_extension_mode_latched_ = urgent_thrust_rise;' not in recovery_request
assert 'circle_fresh && selected_contact_retained' in source
assert 'rotorDepthAngleAbout(' in source
selector = source[source.index(
    'void AttitudePressureController::selectContinuousRecoveryPair('
):source.index('\n}\n', source.index(
    'void AttitudePressureController::selectContinuousRecoveryPair('
))]
assert 'armContactPointFromAngle(' not in selector
assert '10/20-kPa states are now transitions only' in source
assert 'latched topology lower-side pair' in source
assert 'selected instantaneous restoring pair' not in source
assert 'recovery_selected_path_requires_zero_thrust_' in source
assert 'negative roll puts arms 0+1 lower' in source
assert 'active_roll_recovery_config.angle_gain = 0.0;' not in source
assert 're-latching outward-rate arrest' not in source
assert 'const bool grasped_candidate = pressure_index == grasp_pressure_index;' in source
assert 'if (!recovery_selected_pair_release_) return nominal;' in source
assert 'const double retained_angle_progress =' in source
assert 'retained_angle_progress >=\n                              recovery_rate_arrest_release_progress_rad_' in source
assert 'recovery_wrong_geometry_response_observed_ = true;' in source
assert 'preserving %.2f N and switching from rate arrest to angle return' in source
assert 'const bool geometry_reestimate_ready =' in source
assert 'grasp_pressure_ready && recovery_pressure_fresh' in source
assert 'const bool recovery_restart_support_ready =' in source
assert '!continuous_recovery_waiting_for_settle_ ||\n              grasp_pressure_ready || c_hard_roll' in source
assert 'geometry_reestimate_ready && recovery_restart_support_ready &&' in source
assert 'recovery_geometry_reestimate_pending_ = true;' in source
assert 'Balanced four-arm support reached; cleared old contact-state candidate rejections' in source
assert 'recovery_rejected_candidate_mask_ = 0;' in source.split(
    'Balanced four-arm support reached; cleared old contact-state candidate rejections', 1
)[0].rsplit('if (continuous_c_recovery &&', 1)[1]
assert 'completed_recovery_restart_angle' in source
assert 'completed_recovery_restart_allowed' in source
assert 'continuous_roll_recovery_.target_crossed' in source
assert 'opposite-sign selection only if the residual leaves the stop band' in source
assert 'signed_error <= -opposite_selection_boundary;' in source
assert 'predicted_signed_error' not in source
assert 'requested immediate opposite-sign geometry selection' in source
assert 'const bool fast_pressure_worsening = pressure_worsening &&' in source
assert 'recovery_handoff_rollback_before_reselect_' in source
assert 'restoring best measured pressure' in source
assert 'best-pressure rollback settled at roll' in source
assert 'recoveryMeasuredContactPressure()' in source
measured_freeze = source.split(
    'AttitudePressureController::recoveryMeasuredContactPressure() const', 1
)[1].split(
    'AttitudePressureController::recoveryCandidateSwitchPressure() const', 1
)[0]
assert 'prepare_arm_active_[arm]' not in measured_freeze
assert 'measured_pressure_[arm], recovery_handoff_min_pressure_kpa_' in measured_freeze
candidate_switch = source.split(
    'AttitudePressureController::recoveryCandidateSwitchPressure() const', 1
)[1].split('double AttitudePressureController::recoverySupportPressure()', 1)[0]
assert 'prepare_arm_active_[arm] && recovery_selected_pair_release_' in candidate_switch
assert '? 0.0 : recovery_handoff_min_pressure_kpa_' in candidate_switch
assert source.count('recoveryCandidateSwitchPressure();') >= 4
assert 'recoveryReselectionPressure()' in source
assert 'clearing directional rejection mask' in source
assert 'const bool handoff_candidate_stalled =' in source
assert 'const bool handoff_candidate_stalled =\n                      abs_error > capture_error &&' in source
assert 'const bool flexed_handoff_response_active =' in source
assert 'recovery_handoff_candidate_reference_rad_' in source
assert 'handoff_candidate_stalled ||\n                      handoff_candidate_worsening' in source
assert 'full-grasp roll %s at %.2f deg' in source
assert 'recovery_handoff_progress_reference_rad_ -' in source
assert 'const double handoff_retry_thrust = std::max(\n                      roll_recovery_config_.detect_thrust' in source
assert 'rejected handoff candidate reached zero thrust' in source
assert 'preserving zero thrust and requesting a fresh measured-contact pass' in source
assert 'const double handoff_drive_stop = recovery_stop_angle +\n                  handoff_measurement_margin;' in source
assert 'const double handoff_brake_entry = handoff_drive_stop;' in source
assert '!recovery_handoff_braking_ && command_at_grasp &&\n                  abs_error <= (strict_target_observed_before_regrasp' in source
assert 'const bool terminal_contact_plateau =' in source
assert 'if (terminal_contact_plateau)' in source
assert 'const bool terminal_pressure_search =' in source
assert 'signed_rate <= pressure_feedforward_rate_limit' in source
assert 'signed_error <= 0.0 ||' in source
assert 'recovery_contact_grasp_active_' in source
assert 'pressure_search_floor_for(arm)' in source
assert 'pressure_distance_from_best' in source
assert 'const bool strict_target_observed_before_regrasp =' in source
assert 'recovery_handoff_best_error_rad_ <= handoff_settle_error' in source
assert 'const double handoff_measurement_margin = std::min(\n                  0.25 * M_PI / 180.0,\n                  0.25 * roll_recovery_config_.stop_margin);' in source
assert 'const double active_handoff_settle_error =' in source
assert 'recovery_candidate_reselect_pending_ = true;' in source
assert 'continuous_recovery_retry_opposite_pending_ ||' in source
assert 'const double retained_equilibrium_limit =\n              action_c_posture_resume_rad_;' in source
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
handoff_complete = source[source.index('const bool handoff_settled ='):
    source.index('// Do not reopen a partly regrasped pair',
                 source.index('const bool handoff_settled ='))]
assert 'continuous_recovery_waiting_for_settle_ = false;' in handoff_complete
assert 'action_c_requested_recovery_trigger_rad_ =\n                          std::numeric_limits<double>::quiet_NaN();' in handoff_complete
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
namespace ros {struct Time {double t;Time(double value=0.):t(value){}double toSec()const{return t;}bool isZero()const{return t==0.;}static Time now(){return {10.};}};
Time operator-(Time a,Time b){return {a.t-b.t};}}
class AttitudePressureController {
public:
 static constexpr size_t ARM_COUNT=4;
 using Vector3=std::array<double,3>;
 using PressureArray=std::array<double,4>;
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
 hugmy::CircularGraspEstimate recovery_circle_event_estimate_;
 bool recovery_event_contact_pivot_locked_=false;
 Vector3 recovery_event_contact_pivot_{{0.,0.,-.07}};
 bool recovery_preferred_pair_locked_=false;
 size_t recovery_preferred_pair_index_=0;
 bool recovery_preferred_pair_used_fallback_=false;
 bool recovery_candidate_reselect_pending_=false;
 bool recovery_pair_selection_available_=true;
 bool recovery_geometry_reestimate_attempted_=false;
 bool recovery_geometry_reestimate_pending_=false;
 bool action_c_recovery_urgent_thrust_rise_=false;
 bool recovery_extension_mode_latched_=false;
 int recovery_extension_direction_sign_=0;
 bool recovery_rate_arrest_active_=false;
 ros::Time recovery_rate_arrest_nonoutward_since_;
 double recovery_rate_arrest_reference_error_rad_=NAN;
 double recovery_extension_prediction_sec_=.30;
 double recovery_rate_arrest_release_sec_=.10;
 double recovery_rate_arrest_release_progress_rad_=.5*M_PI/180.;
 double correction_motion_start_rate_=.05;
 size_t recovery_selected_pair_index_=0;
 hugmy::ContinuousRollRecovery continuous_roll_recovery_;
 struct Gate {int resets=0;void reset(){++resets;}} action_c_resume_gate_, action_c_boundary_settle_gate_;
 ros::Time state_start_{0};double prepare_timeout_=60.;
 double action_c_requested_recovery_trigger_rad_=NAN;
 std::array<double,4> prepare_shape_thrust_{{2.,2.,2.,2.}}, grasp_pressure_{{30.,30.,30.,30.}};
 std::array<double,4> prepare_pressure_{{30.,30.,30.,30.}};
 std::array<double,4> measured_pressure_{{30.,30.,30.,30.}};
 std::array<double,4> recovery_selected_release_start_pressure_{{30.,30.,30.,30.}};
 std::array<double,4> angles{{M_PI/2,M_PI/2,M_PI/2,M_PI/2}};
 std::array<double,4> recovery_handoff_target_pressure_{};
 double recovery_handoff_min_pressure_kpa_=15.,base_pressure_kpa_=30.,maximum_pressure_kpa_=40.;
 double minimum_prepare_pressure_kpa_=0.,mass_kg_=1.244;
 int publications=0, state=0;enum {RECOVER=1,GRASP=2};
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
 double recovery_selected_pair_pressure_kpa_=0.;
 size_t recovery_selected_pressure_candidate_index_=0;
 bool recovery_selected_contact_pivot_enabled_=false;
 Vector3 recovery_selected_contact_pivot_{{0.,0.,-.07}};
 double recovery_selected_pair_mean_depth_rad_=NAN;
 double recovery_circle_min_confidence_=.35,recovery_circle_max_age_sec_=30.;
 double recovery_circle_side_margin_rad_=5*M_PI/180.;
 double recovery_circle_pair_depth_ambiguity_rad_=5*M_PI/180.;
 double recovery_circle_flexed_max_error_rad_=15*M_PI/180.;
 double recovery_circle_contact_pressure_kpa_=10.;
 double recovery_circle_release_penalty_n_=.75,recovery_circle_min_signed_lever_m_=.005;
 std::array<double,4> recovery_circle_pressure_candidates_kpa_{{0.,10.,20.,30.}};
 double recovery_circle_release_support_scale_=.20;
 double recovery_selected_support_loss_moment_nm_=0.;
 bool recovery_selected_path_requires_zero_thrust_=false;
 double recovery_selected_path_min_lever_m_=NAN;
 ros::Time recovery_wrong_geometry_since_,recovery_wrong_geometry_transition_bad_since_;
 ros::Time recovery_wrong_geometry_pressure_settled_since_;
 double recovery_wrong_geometry_reference_error_rad_=NAN;
 double recovery_wrong_geometry_reference_rate_rad_s_=NAN;
 double recovery_wrong_geometry_best_rate_rad_s_=-INFINITY;
 bool recovery_wrong_geometry_response_observed_=false;
 PressureArray recovery_wrong_geometry_probe_pressure_target_{};
 bool recovery_wrong_geometry_probe_target_valid_=false;
 double roll_moment_kp_=.1,roll_moment_kd_=0.,target_roll_=0.,roll_=-.2,pitch_=0.;
 Vector3 body_angular_velocity_{{0.,0.,0.}};
 Vector3 gravity_body_{{0.,0.,-9.81}};
 Vector3 contact_position_{{0.,0.,-.07}};
 double recovery_release_pressure_kpa_=0.,correction_planning_thrust_n_=2.,bend_model_thrust_offset_n_=1.8;
 hugmy::CircularGraspEstimate recovery_circle_estimate_;ros::Time recovery_circle_stamp_{};
 hugmy::RollRecoveryConfig roll_recovery_config_;hugmy::PressureThrustBendModel bend_model_;
 int finished=0, starts=0;
 void finishHybridLocomotion(const ros::Time&,bool,bool) {if(hybrid_trial_active_) ++finished; hybrid_trial_active_=false;}
 void startInchwormStep(std::array<double,2>,double d,ros::Time) {
   assert(action_c_preserve_adaptation_on_start_); commanded_distance=d;active=true;++starts;hybrid_trial_active_=true;
 }
 bool resumeActionCAfterRecovery(const ros::Time&);
 bool hybridLocomotionActive()const{return true;}
 Vector3 locomotionPosition()const{return mocap_position_;}
 bool locomotionMocapFresh(const ros::Time& now)const {
   const double age=(now-mocap_stamp_).toSec();return mocap_received_&&age>=0.&&age<=inchworm_mocap_timeout_sec_;
 }
 void selectContinuousRecoveryPair(double);
 double measuredArmBendAngle(size_t arm)const{return angles[arm];}
 Vector3 armContactPointFromAngle(size_t arm,double bend)const{
   return {{0.,arm<2?.04:-.04,-.07-.01*std::sin(bend)}};
 }
 Vector3 rotorPositionFromAngle(size_t arm,double bend)const{
   return {{0.,arm<2?.04:-.04,-.07-.01*std::sin(bend)}};
 }
 double rotorDepthAngleAbout(size_t,double bend,const Vector3&)const{
   return std::isfinite(bend)?bend:NAN;
 }
 Vector3 recoveryContactPivot(const std::array<bool,4>&,bool,
                              const PressureArray* = nullptr,
                              const PressureArray* = nullptr)const{
   return recovery_event_contact_pivot_locked_
       ? recovery_event_contact_pivot_ : contact_position_;
 }
 bool straight_pair_swap_=false;
 double recoveryRotorRawLeverAbout(size_t arm,double bend,const Vector3&)const{
   if(straight_pair_swap_ && bend<.1) return arm<2?.02:.06;
   const double pair01=bend<1.8?.04:-.04;return arm<2?pair01:-pair01;
 }
 double recoveryRotorRawLever(size_t arm,double bend)const{
   return recoveryRotorRawLeverAbout(arm,bend,{{0.,0.,-.07}});
 }
 static Vector3 cross(const Vector3&a,const Vector3&b){return {{
   a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}};}
 bool hybridResumeAllowed(const ros::Time&) {return true;} // deadline tested separately
 void finishHybridRecovery(const ros::Time& now) {finishHybridLocomotion(now,false,true);}
 void resetRecoveryHandoffState() {}
 void stopInchwormToGrasp(const ros::Time&,const char*) {active=false;}
 void select(double roll_error) {
   roll_=target_roll_-roll_error;
   ros::Time now{10.};do { PAIR } while(false);
 }
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
 // With a valid circle, normal recovery compares the current-bend grasped
 // geometry against the alpha=0 straight-arm geometry.  In this synthetic
 // case the grasped 0+1 pair requires less thrust and must remain grasped.
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
 const size_t first_pressure=circle.recovery_selected_pressure_candidate_index_;
 assert(first_pressure<4);
 // Reject only the selected active state. Re-selection keeps the
 // geometrically correct pair and switches only between 30-kPa grasped and
 // 0-kPa extended modes; 10/20 kPa are pneumatic transitions, not choices.
 circle.recovery_rejected_candidate_mask_=static_cast<unsigned char>(
     1u<<first_pressure);
 circle.select(.2);
 assert(circle.recovery_pair_selection_available_);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
 assert(circle.recovery_selected_pressure_candidate_index_!=first_pressure);
 assert(circle.recovery_selected_pressure_candidate_index_==0 ||
        circle.recovery_selected_pressure_candidate_index_==3);
 // Once both active 0+1 states are disproved, its event latch prevents an
 // unsafe pair jump before regrasp and must refuse further thrust.
 circle.recovery_rejected_candidate_mask_=0x0fu;
 circle.select(.2);
 assert(!circle.recovery_pair_selection_available_);
 circle.recovery_rejected_candidate_mask_=0;
 // A large roll cannot use the flexed-contact shortcut even when that pair
 // remains below the circle side; it must transfer support before recovery.
 circle.body_angular_velocity_[0]=-.2;
 circle.select(.35);
 assert(circle.recovery_selected_pair_release_);
 assert(circle.recovery_extension_mode_latched_ &&
        circle.recovery_rate_arrest_active_);
 circle.body_angular_velocity_[0]=0.;
 circle.recovery_circle_estimate_.depth_angle[0]=1.;
 circle.recovery_circle_estimate_.depth_angle[1]=1.;
 circle.select(.2);
 assert(circle.prepare_arm_active_[0] && circle.prepare_arm_active_[1]);
 // Explicitly verify the requested four-candidate policy.  At the measured
 // bend only pair 0+1 has a restoring grasped lever, while at alpha=0 the
 // straight 2+3 pair has the largest restoring lever.  Normal recovery uses
 // the lower-thrust grasped solution; emergency recovery excludes grasped
 // candidates and selects the straight 2+3 solution.
 AttitudePressureController normal_choice;
 normal_choice.recovery_circle_selection_enabled_=true;
 normal_choice.recovery_circle_estimate_=circle.recovery_circle_estimate_;
 normal_choice.recovery_circle_stamp_={9.9};
 normal_choice.recovery_circle_release_support_scale_=0.;
 normal_choice.recovery_circle_release_penalty_n_=0.;
 normal_choice.straight_pair_swap_=true;
 normal_choice.select(.2);
 assert(normal_choice.prepare_arm_active_[0] &&
        normal_choice.prepare_arm_active_[1]);
 assert(!normal_choice.recovery_selected_pair_release_);
 // The 30-kPa endpoint in the candidate table denotes the grasped topology,
 // not a request to partially exhaust an arm whose actual grasp pressure is
 // 40 kPa.  Such an arm must retain its measured bend and nominal pressure.
 AttitudePressureController high_pressure_grasp;
 high_pressure_grasp.recovery_circle_selection_enabled_=true;
 high_pressure_grasp.recovery_circle_estimate_=circle.recovery_circle_estimate_;
 high_pressure_grasp.recovery_circle_stamp_={9.9};
 high_pressure_grasp.recovery_circle_release_support_scale_=0.;
 high_pressure_grasp.recovery_circle_release_penalty_n_=0.;
 high_pressure_grasp.straight_pair_swap_=true;
 high_pressure_grasp.grasp_pressure_.fill(40.);
 high_pressure_grasp.measured_pressure_.fill(40.);
 high_pressure_grasp.select(.2);
 assert(high_pressure_grasp.prepare_arm_active_[0] &&
        high_pressure_grasp.prepare_arm_active_[1]);
 assert(!high_pressure_grasp.recovery_selected_pair_release_ &&
        std::abs(high_pressure_grasp.recovery_selected_pair_pressure_kpa_-40.)<1e-9);
 // A locomotion guard may request a fast rise while the small roll error is
 // no longer moving outward.  Fast slew alone must not exclude the safer
 // current-bend candidate.
 AttitudePressureController urgent_small_choice;
 urgent_small_choice.recovery_circle_selection_enabled_=true;
 urgent_small_choice.recovery_circle_estimate_=circle.recovery_circle_estimate_;
 urgent_small_choice.recovery_circle_stamp_={9.9};
 urgent_small_choice.recovery_circle_release_support_scale_=0.;
 urgent_small_choice.recovery_circle_release_penalty_n_=0.;
 urgent_small_choice.straight_pair_swap_=true;
 urgent_small_choice.action_c_recovery_urgent_thrust_rise_=true;
 urgent_small_choice.body_angular_velocity_[0]=0.;
 urgent_small_choice.select(.2);
 assert(urgent_small_choice.prepare_arm_active_[0] &&
        urgent_small_choice.prepare_arm_active_[1]);
 assert(!urgent_small_choice.recovery_selected_pair_release_ &&
        !urgent_small_choice.recovery_extension_mode_latched_);
 // The same small present error still uses straight-arm recovery when its
 // measured outward rate predicts crossing the 15-deg emergency boundary.
 AttitudePressureController predicted_emergency_choice;
 predicted_emergency_choice.recovery_circle_selection_enabled_=true;
 predicted_emergency_choice.recovery_circle_estimate_=
     circle.recovery_circle_estimate_;
 predicted_emergency_choice.recovery_circle_stamp_={9.9};
 predicted_emergency_choice.recovery_circle_release_support_scale_=0.;
 predicted_emergency_choice.recovery_circle_release_penalty_n_=0.;
 predicted_emergency_choice.straight_pair_swap_=true;
 predicted_emergency_choice.body_angular_velocity_[0]=-.35;
 predicted_emergency_choice.select(.2);
 assert(predicted_emergency_choice.prepare_arm_active_[2] &&
        predicted_emergency_choice.prepare_arm_active_[3]);
 assert(predicted_emergency_choice.recovery_selected_pair_release_ &&
        predicted_emergency_choice.recovery_extension_mode_latched_);
 AttitudePressureController emergency_choice;
 emergency_choice.recovery_circle_selection_enabled_=true;
 emergency_choice.recovery_circle_estimate_=circle.recovery_circle_estimate_;
 emergency_choice.recovery_circle_stamp_={9.9};
 emergency_choice.recovery_circle_release_support_scale_=0.;
 emergency_choice.straight_pair_swap_=true;
 emergency_choice.select(.35);
 assert(emergency_choice.prepare_arm_active_[2] &&
        emergency_choice.prepare_arm_active_[3]);
 assert(emergency_choice.recovery_selected_pair_release_);
 assert(emergency_choice.recovery_extension_mode_latched_ &&
        emergency_choice.recovery_extension_direction_sign_==1 &&
        emergency_choice.recovery_rate_arrest_active_);
 // Crossing the target leaves only a small opposite residual. The previous
 // emergency direction must not keep forcing a straight-arm pair.
 emergency_choice.angles.fill(2.1);
 emergency_choice.action_c_recovery_urgent_thrust_rise_=true;
 emergency_choice.select(-.10);
 assert(emergency_choice.prepare_arm_active_[0] &&
        emergency_choice.prepare_arm_active_[1]);
 assert(!emergency_choice.recovery_selected_pair_release_);
 assert(!emergency_choice.recovery_extension_mode_latched_ &&
        emergency_choice.recovery_extension_direction_sign_==0 &&
        !emergency_choice.action_c_recovery_urgent_thrust_rise_);
 // A stable small residual after an extended correction is a fresh contact
 // decision.  Do not force the opposite physical pair: in this geometry the
 // same physical 0+1 pair has become restorative after it is re-grasped.
 AttitudePressureController stable_small_reselection;
 stable_small_reselection.recovery_circle_selection_enabled_=true;
 stable_small_reselection.recovery_circle_estimate_=
     circle.recovery_circle_estimate_;
 stable_small_reselection.recovery_circle_stamp_={9.9};
 stable_small_reselection.recovery_circle_release_support_scale_=0.;
 stable_small_reselection.recovery_circle_release_penalty_n_=0.;
 stable_small_reselection.straight_pair_swap_=true;
 stable_small_reselection.recovery_extension_mode_latched_=true;
 stable_small_reselection.recovery_extension_direction_sign_=-1;
 stable_small_reselection.recovery_preferred_pair_index_=0;
 stable_small_reselection.recovery_preferred_pair_locked_=true;
 stable_small_reselection.body_angular_velocity_[0]=0.;
 stable_small_reselection.select(.10);
 assert(stable_small_reselection.prepare_arm_active_[0] &&
        stable_small_reselection.prepare_arm_active_[1]);
 assert(!stable_small_reselection.recovery_selected_pair_release_ &&
        !stable_small_reselection.recovery_extension_mode_latched_);
 // The same small angle with a fast outward body rate is not stable.  Keep
 // the old emergency extension latch so it can arrest the existing motion.
 AttitudePressureController moving_small=stable_small_reselection;
 moving_small.recovery_extension_mode_latched_=true;
 moving_small.recovery_extension_direction_sign_=1;
 moving_small.recovery_preferred_pair_index_=1;
 moving_small.recovery_preferred_pair_locked_=true;
 moving_small.body_angular_velocity_[0]=-.20;
 moving_small.select(.10);
 assert(moving_small.recovery_selected_pair_release_ &&
        moving_small.recovery_extension_mode_latched_);
 // Without a fresh circle, grasped candidates must use the same verified
 // actuator mapping as the measured-angle allocator.
 AttitudePressureController no_circle;
 no_circle.recovery_circle_selection_enabled_=true;
 no_circle.correction_pair_reverse_spinal_mapping_=true;
 no_circle.select(.2);
 assert(no_circle.prepare_arm_active_[2] &&
        no_circle.prepare_arm_active_[3]);
 assert(!no_circle.recovery_selected_pair_release_);
 // The earlier large-error release latch was deliberately cleared when the
 // subsequent small residual became quiet, so this valid grasped state is
 // retained rather than returning to the straight-arm topology.
 assert(!circle.recovery_selected_pair_release_);
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
