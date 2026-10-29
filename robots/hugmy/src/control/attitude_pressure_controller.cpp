#include <hugmy/control/attitude_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <tf/transform_datatypes.h>

namespace
{
constexpr double GRAVITY = 9.80665;
constexpr uint8_t ACTION_D_PULSE_RISE = 0;
constexpr uint8_t ACTION_D_PULSE_ON = 1;
constexpr uint8_t ACTION_D_PULSE_FALL = 2;
constexpr uint8_t ACTION_D_PULSE_OFF = 3;

template <size_t N>
void loadFixedVector(ros::NodeHandle& nh, const std::string& name,
                     std::array<double, N>& output)
{
  std::vector<double> values;
  if (!nh.getParam(name, values)) return;
  if (values.size() != N)
    {
      ROS_WARN("%s must contain %zu values; using defaults", name.c_str(), N);
      return;
    }
  std::copy(values.begin(), values.end(), output.begin());
}
}

AttitudePressureController::AttitudePressureController(ros::NodeHandle& nh,
                                                       ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  pnh_.param("roll_moment_kp_nm_per_rad", roll_moment_kp_, 0.10);
  pnh_.param("pitch_moment_kp_nm_per_rad", pitch_moment_kp_, 0.10);
  pnh_.param("roll_moment_kd_nm_per_rad_s", roll_moment_kd_, 0.015);
  pnh_.param("pitch_moment_kd_nm_per_rad_s", pitch_moment_kd_, 0.015);
  pnh_.param("mass_kg", mass_kg_, 1.244);
  pnh_.param("gravity_compensation", gravity_compensation_, true);
  pnh_.param("base_pressure_kpa", base_pressure_kpa_, 10.0);
  pnh_.param("maximum_pressure_kpa", maximum_pressure_kpa_, 55.0);
  pnh_.param("bend_model_thrust_offset_n", bend_model_thrust_offset_n_, 1.8);
  pnh_.param("pressure_rate_limit_kpa_s", pressure_rate_limit_kpa_s_, 10.0);
  pnh_.param("imu_timeout_sec", imu_timeout_sec_, 0.10);
  pnh_.param("thrust_timeout_sec", thrust_timeout_sec_, 0.20);
  pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
  pnh_.param("ppg_settle_sec", ppg_settle_sec_, 0.0);
  if (!std::isfinite(ppg_settle_sec_) || ppg_settle_sec_ < 0.0 ||
      ppg_settle_sec_ > 60.0)
    throw std::runtime_error("ppg_settle_sec must be finite and in [0, 60] s");
  pnh_.param("hybrid_terminal_posture_monitor_sec",
             hybrid_terminal_posture_monitor_sec_, 8.0);
  if (!std::isfinite(hybrid_terminal_posture_monitor_sec_) ||
      hybrid_terminal_posture_monitor_sec_ < 0.0 ||
      hybrid_terminal_posture_monitor_sec_ > 60.0)
    throw std::runtime_error(
        "hybrid_terminal_posture_monitor_sec must be finite and in [0, 60] s");
  pnh_.param("hybrid_regrip_bottom_ramp_kpa_s",
             hybrid_regrip_bottom_ramp_kpa_s_, 2.5);
  pnh_.param("hybrid_lift_worsening_roll_stop_rad",
             hybrid_lift_worsening_roll_stop_rad_, 8.0 * M_PI / 180.0);
  pnh_.param("hybrid_lift_worsening_roll_rate_rad_s",
             hybrid_lift_worsening_roll_rate_rad_s_, 1.0 * M_PI / 180.0);
  pnh_.param("hybrid_lift_worsening_roll_confirm_sec",
             hybrid_lift_worsening_roll_confirm_sec_, 0.10);
  pnh_.param("hybrid_lift_worsening_roll_prediction_sec",
             hybrid_lift_worsening_roll_prediction_sec_, 0.25);
  pnh_.param("hybrid_lift_target_no_unload_timeout_sec",
             hybrid_lift_target_no_unload_timeout_sec_, 0.35);
  pnh_.param("hybrid_regrip_roll_recovery_rad",
             hybrid_regrip_roll_recovery_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("hybrid_regrip_roll_emergency_rad",
             hybrid_regrip_roll_emergency_rad_, 30.0 * M_PI / 180.0);
  pnh_.param("hybrid_regrip_roll_brake_fall_n_s",
             hybrid_regrip_roll_brake_fall_n_s_, 8.0);
  pnh_.param("hybrid_keep_bottom_between_strokes",
             hybrid_keep_bottom_between_strokes_, false);
  if (!std::isfinite(hybrid_regrip_bottom_ramp_kpa_s_) ||
      !std::isfinite(hybrid_lift_worsening_roll_stop_rad_) ||
      !std::isfinite(hybrid_lift_worsening_roll_rate_rad_s_) ||
      !std::isfinite(hybrid_lift_worsening_roll_confirm_sec_) ||
      !std::isfinite(hybrid_lift_worsening_roll_prediction_sec_) ||
      !std::isfinite(hybrid_lift_target_no_unload_timeout_sec_) ||
      !std::isfinite(hybrid_regrip_roll_recovery_rad_) ||
      !std::isfinite(hybrid_regrip_roll_emergency_rad_) ||
      !std::isfinite(hybrid_regrip_roll_brake_fall_n_s_) ||
      hybrid_regrip_bottom_ramp_kpa_s_ <= 0.0 ||
      hybrid_lift_worsening_roll_stop_rad_ <= 0.0 ||
      hybrid_lift_worsening_roll_rate_rad_s_ < 0.0 ||
      hybrid_lift_worsening_roll_confirm_sec_ < 0.04 ||
      hybrid_lift_worsening_roll_confirm_sec_ > 0.5 ||
      hybrid_lift_worsening_roll_prediction_sec_ < 0.05 ||
      hybrid_lift_worsening_roll_prediction_sec_ > 0.5 ||
      hybrid_lift_target_no_unload_timeout_sec_ < 0.10 ||
      hybrid_lift_target_no_unload_timeout_sec_ > 2.0 ||
      hybrid_regrip_roll_recovery_rad_ <
          hybrid_lift_worsening_roll_stop_rad_ ||
      hybrid_regrip_roll_emergency_rad_ <
          hybrid_regrip_roll_recovery_rad_ ||
      hybrid_regrip_roll_brake_fall_n_s_ < 2.0 ||
      hybrid_regrip_roll_brake_fall_n_s_ > 30.0)
    throw std::runtime_error("invalid hybrid regrip transfer parameters");
  pnh_.param("allocation_iterations", allocation_iterations_, 6);
  pnh_.param("allocation_damping", allocation_damping_, 1e-5);
  pnh_.param("allocation_step_limit_kpa", allocation_step_limit_kpa_, 5.0);
  pnh_.param("allocation_pressure_epsilon_kpa", allocation_pressure_epsilon_kpa_, 0.25);
  pnh_.param("allocation_bias_weight", allocation_bias_weight_, 0.01);
  pnh_.param("state_machine_enabled", state_machine_enabled_, true);
  pnh_.param("correction_roll_only", correction_roll_only_, true);
  pnh_.param("correction_pair_reverse_spinal_mapping",
             correction_pair_reverse_spinal_mapping_, false);
  pnh_.param("action_c_continuous_recovery_enabled",
             action_c_continuous_recovery_enabled_, true);
  // These settings affect continuous Action-C posture recovery ONLY.
  auto recovery_param = [&](const char* name, double& value, double low, double high) {
    double configured = value;
    pnh_.param(name, configured, value);
    if (std::isfinite(configured)) value = hugmy::recoveryClamp(configured, low, high);
  };
  recovery_param("recovery_angle_gain_s_inv", roll_recovery_config_.angle_gain, 0.1, 3.0);
  recovery_param("recovery_max_thrust_n", roll_recovery_config_.limit, 0.5, 6.0);
  recovery_param("recovery_measurement_rate_limit_rad_s",
                 recovery_measurement_rate_limit_rad_s_, 0.8, 3.0);
  pnh_.param("recovery_response_learning_enabled", roll_recovery_config_.learn_response, true);
  recovery_param("recovery_target_rate_rad_s", roll_recovery_config_.max_target_rate, 0.02, 0.3);
  recovery_param("recovery_prediction_acceleration_limit_rad_s2",
                 roll_recovery_config_.max_prediction_acceleration, 0.1, 10.0);
  recovery_param("recovery_prediction_acceleration_horizon_sec",
                 roll_recovery_config_.prediction_acceleration_horizon,
                 0.02, 0.50);
  recovery_param("recovery_rate_kp_n_s_rad", roll_recovery_config_.rate_kp, 0.1, 10.0);
  recovery_param("recovery_rate_ki_n_rad", roll_recovery_config_.rate_ki, 0.0, 10.0);
  recovery_param("recovery_integral_slew_n_s", roll_recovery_config_.integral_rate, 0.05, 1.0);
  recovery_param("recovery_thrust_rise_n_s", roll_recovery_config_.rise, 0.2, 4.0);
  recovery_param("recovery_thrust_fall_n_s", roll_recovery_config_.fall, 0.5, 8.0);
  recovery_param("recovery_terminal_brake_fall_n_s",
                 roll_recovery_config_.terminal_brake_fall, 1.0, 15.0);
  roll_recovery_config_.terminal_brake_fall = std::max(
      roll_recovery_config_.fall, roll_recovery_config_.terminal_brake_fall);
  recovery_param("recovery_reapproach_max_thrust_n",
                 roll_recovery_config_.reapproach_limit, 0.25, 3.0);
  recovery_param("recovery_reapproach_rate_rad_s",
                 roll_recovery_config_.reapproach_rate, 0.01, 0.10);
  recovery_param("recovery_reapproach_authority_rise_n_s",
                 roll_recovery_config_.reapproach_authority_rise, 0.05, 3.0);
  recovery_param("recovery_reapproach_escalation_delay_sec",
                 roll_recovery_config_.reapproach_escalation_delay, 0.0, 3.0);
  recovery_param("recovery_response_fault_min_thrust_n",
                 roll_recovery_config_.response_fault_min_thrust, 0.5, 4.0);
  recovery_param("recovery_settle_rate_rad_s", roll_recovery_config_.settle_rate, 0.02, 0.3);
  recovery_param("recovery_settle_sec", roll_recovery_config_.settle_sec, 0.04, 0.5);
  recovery_param("recovery_stop_margin_rad", roll_recovery_config_.stop_margin, 0.0, 0.05236);
  recovery_param("recovery_contact_minimum_limit_rad",
                 roll_recovery_config_.contact_minimum_limit, 0.0, 0.349066);
  recovery_param("recovery_model_min_lever_m", roll_recovery_config_.min_lever, 0.005, 0.1);
  recovery_param("recovery_model_fallback_thrust_n", roll_recovery_config_.fallback_thrust, 0.0, 1.0);
  recovery_param("recovery_response_delay_sec", roll_recovery_config_.response_delay, 0.0, 1.0);
  recovery_param("recovery_response_window_sec", roll_recovery_config_.response_window, 0.2, 2.0);
  recovery_param("recovery_progress_window_sec", roll_recovery_config_.progress_window, 0.5, 2.0);
  recovery_param("recovery_pair_retry_sec", roll_recovery_config_.pair_retry_sec, 1.0, 10.0);
  recovery_param("recovery_pair_retry_thrust_fraction",
                 roll_recovery_config_.pair_retry_thrust_fraction, 0.25, 1.0);
  pnh_.param("recovery_predictive_enabled", recovery_predictive_config_.enabled, true);
  pnh_.param("recovery_predictive_shadow", recovery_predictive_config_.shadow, true);
  pnh_.param("recovery_predictive_adapt", recovery_predictive_config_.adapt, true);
  recovery_param("recovery_predictive_period_sec", recovery_predictive_config_.period, 0.08, 0.2);
  recovery_param("recovery_predictive_horizon_sec", recovery_predictive_config_.horizon, 0.6, 2.0);
  recovery_param("recovery_predictive_budget_ms", recovery_predictive_config_.budget_ms, 1.0, 10.0);
  recovery_param("recovery_predictive_pressure_tau_sec", recovery_predictive_config_.pressure_tau, 0.15, 2.0);
  recovery_param("recovery_predictive_bend_tau_sec", recovery_predictive_config_.bend_tau, 0.15, 2.0);
  recovery_param("recovery_predictive_moment_gain", recovery_predictive_config_.moment_gain, 1.0, 20.0);
  recovery_param("recovery_predictive_thrust_step_n", recovery_predictive_config_.thrust_step, 0.1, 0.8);
  recovery_param("recovery_predictive_pressure_step_kpa", recovery_predictive_config_.pressure_step, 0.5, 3.0);
  recovery_param("recovery_stable_pressure_kpa", roll_recovery_config_.stable_pressure, 0.2, 3.0);
  recovery_param("recovery_stable_bend_rad", roll_recovery_config_.stable_bend, 0.01, 0.1);
  recovery_param("recovery_rotor_cant_rad", recovery_rotor_cant_rad_, -0.3, 0.3);
  recovery_param("recovery_release_pressure_kpa", recovery_release_pressure_kpa_, 0.0, 30.0);
  recovery_param("recovery_pressure_down_rate_kpa_s", recovery_pressure_down_rate_, 0.0, 2.0);
  recovery_param("recovery_handoff_pressure_rate_kpa_s",
                 recovery_handoff_pressure_rate_kpa_s_, 0.5, 30.0);
  recovery_param("recovery_handoff_stage_a_thrust_max_n",
                 recovery_handoff_stage_a_thrust_max_n_, 0.25, 3.0);
  recovery_param("recovery_handoff_thrust_fall_n_s",
                 recovery_handoff_thrust_fall_n_s_, 0.1, 3.0);
  recovery_param("recovery_handoff_reopen_angle_rad",
                 recovery_handoff_reopen_angle_rad_, 0.005, 0.0872665);
  recovery_param("recovery_handoff_settle_sec",
                 recovery_handoff_settle_sec_, 0.05, 2.0);
  recovery_param("recovery_handoff_timeout_sec",
                 recovery_handoff_timeout_sec_, 3.0, 60.0);
  recovery_param("recovery_handoff_min_pressure_kpa",
                 recovery_handoff_min_pressure_kpa_, 5.0, 30.0);
  recovery_param("recovery_handoff_trim_margin_rad",
                 recovery_handoff_trim_margin_rad_, 0.005, 0.0872665);
  recovery_param("recovery_handoff_trim_confirm_sec",
                 recovery_handoff_trim_confirm_sec_, 0.05, 2.0);
  recovery_param("recovery_handoff_contact_equilibrium_rad",
                 recovery_handoff_contact_equilibrium_rad_,
                 0.05236, 0.174533);
  pnh_.param("recovery_circle_selection_enabled",
             recovery_circle_selection_enabled_, true);
  recovery_param("recovery_circle_min_radius_m",
                 recovery_circle_min_radius_m_, 0.015, 0.10);
  recovery_param("recovery_circle_max_radius_m",
                 recovery_circle_max_radius_m_, 0.03, 0.25);
  recovery_param("recovery_circle_max_rms_m",
                 recovery_circle_max_rms_m_, 0.002, 0.04);
  recovery_param("recovery_circle_min_confidence",
                 recovery_circle_min_confidence_, 0.0, 1.0);
  recovery_param("recovery_circle_contact_pressure_kpa",
                 recovery_circle_contact_pressure_kpa_, 1.0, 30.0);
  recovery_param("recovery_circle_side_margin_rad",
                 recovery_circle_side_margin_rad_, 0.0, 0.35);
  recovery_param("recovery_circle_pair_depth_ambiguity_rad",
                 recovery_circle_pair_depth_ambiguity_rad_, 0.0, 0.35);
  recovery_param("recovery_circle_small_radius_m",
                 recovery_circle_small_radius_m_, 0.02, 0.08);
  recovery_param("recovery_circle_small_radius_capture_margin_factor",
                 recovery_circle_small_radius_capture_margin_factor_, 0.5, 6.0);
  recovery_param("recovery_circle_small_radius_support_pressure_kpa",
                 recovery_circle_small_radius_support_pressure_kpa_, 5.0, 40.0);
  recovery_param("recovery_circle_flexed_max_error_rad",
                 recovery_circle_flexed_max_error_rad_, 0.05, 1.0);
  recovery_param("recovery_circle_release_penalty_n",
                 recovery_circle_release_penalty_n_, 0.0, 3.0);
  recovery_param("recovery_circle_min_signed_lever_m",
                 recovery_circle_min_signed_lever_m_, 0.001, 0.05);
  recovery_param("recovery_circle_release_support_scale",
                 recovery_circle_release_support_scale_, 0.0, 1.0);
  recovery_param("recovery_wrong_sign_shaping_thrust_n",
                 recovery_wrong_sign_shaping_thrust_n_, 0.0, 2.0);
  recovery_param("recovery_wrong_geometry_timeout_sec",
                 recovery_wrong_geometry_timeout_sec_, 0.25, 5.0);
  recovery_param("recovery_wrong_geometry_worsening_rad",
                 recovery_wrong_geometry_worsening_rad_, 0.0, 0.174533);
  recovery_param("recovery_wrong_geometry_response_confirm_sec",
                 recovery_wrong_geometry_response_confirm_sec_, 0.10, 1.0);
  recovery_param("recovery_wrong_geometry_rate_improvement_rad_s",
                 recovery_wrong_geometry_rate_improvement_rad_s_, 0.0, 0.35);
  recovery_param("recovery_wrong_geometry_transition_confirm_sec",
                 recovery_wrong_geometry_transition_confirm_sec_, 0.10, 1.0);
  recovery_param("recovery_wrong_geometry_transition_worsening_rad",
                 recovery_wrong_geometry_transition_worsening_rad_,
                 0.0, 0.0872665);
  recovery_param("recovery_wrong_geometry_transition_outward_rate_rad_s",
                 recovery_wrong_geometry_transition_outward_rate_rad_s_,
                 0.0, 0.35);
  recovery_param("recovery_circle_max_age_sec",
                 recovery_circle_max_age_sec_, 0.5, 120.0);
  if (recovery_circle_max_radius_m_ <= recovery_circle_min_radius_m_)
    throw std::runtime_error(
        "recovery_circle_max_radius_m must exceed minimum radius");
  pnh_.param("action_c_recovery_no_progress_timeout_sec",
             action_c_recovery_no_progress_timeout_sec_, 30.0);
  action_c_recovery_no_progress_timeout_sec_ = std::max(1.0,
      action_c_recovery_no_progress_timeout_sec_);
  ROS_INFO("Posture recovery axes: %s",
           correction_roll_only_ ? "Spinal roll only (pitch ignored)" : "Spinal roll/pitch");
  pnh_.param("correction_trigger_angle_rad", correction_trigger_angle_, 8.0 * M_PI / 180.0);
  pnh_.param("correction_stop_angle_rad", correction_stop_angle_, 5.0 * M_PI / 180.0);
  pnh_.param("correction_trigger_confirm_sec", correction_trigger_confirm_sec_, 0.30);
  correction_trigger_confirm_sec_ = std::max(0.0, correction_trigger_confirm_sec_);
  pnh_.param("correction_max_angular_rate_rad_s", correction_max_angular_rate_, 0.8);
  pnh_.param("correction_thrust_duration_sec", correction_thrust_duration_, 0.20);
  pnh_.param("prepare_timeout_sec", prepare_timeout_, 60.0);
  pnh_.param("prepare_stable_duration_sec", prepare_stable_duration_, 0.30);
  pnh_.param("recover_timeout_sec", recover_timeout_, 5.0);
  pnh_.param("cooldown_duration_sec", cooldown_duration_, 1.0);
  pnh_.param("pressure_tolerance_kpa", pressure_tolerance_kpa_, 1.0);
  pnh_.param("arm_angle_tolerance_rad", arm_angle_tolerance_, 0.20);
  pnh_.param("arm_gyro_tolerance_rad_s", arm_gyro_tolerance_, 0.20);
  pnh_.param("correction_planning_thrust_n", correction_planning_thrust_n_, 2.0);
  pnh_.param("correction_min_thrust_n", correction_min_thrust_n_, 2.0);
  pnh_.param("correction_max_thrust_n", correction_max_thrust_n_, 4.0);
  pnh_.param("correction_hold_min_thrust_n", correction_hold_min_thrust_n_, 0.3);
  pnh_.param("correction_motion_detection_min_thrust_n",
             correction_motion_detection_min_thrust_n_, 0.5);
  pnh_.param("correction_motion_start_rate_rad_s", correction_motion_start_rate_, 0.05);
  pnh_.param("correction_motion_confirm_sec",
             correction_motion_confirm_sec_, 0.15);
  pnh_.param("correction_motion_min_error_reduction_rad",
             correction_motion_min_error_reduction_, M_PI / 180.0);
  pnh_.param("correction_motion_stop_rate_rad_s", correction_motion_stop_rate_, 0.02);
  pnh_.param("correction_slowdown_angle_rad",
             correction_slowdown_angle_, 10.0 * M_PI / 180.0);
  pnh_.param("correction_prediction_horizon_sec",
             correction_prediction_horizon_sec_, 0.20);
  pnh_.param("correction_stall_timeout_sec",
             correction_stall_timeout_sec_, 0.50);
  pnh_.param("correction_stall_abort_timeout_sec",
             correction_stall_abort_timeout_sec_, 5.0);
  pnh_.param("correction_stall_thrust_ramp_n_s",
             correction_stall_thrust_ramp_n_s_, 0.30);
  pnh_.param("correction_progress_epsilon_rad",
             correction_progress_epsilon_, 0.5 * M_PI / 180.0);
  pnh_.param("correction_thrust_down_rate_n_s", correction_thrust_down_rate_, 6.0);
  pnh_.param("correction_motion_stop_timeout_sec", correction_motion_stop_timeout_, 0.30);
  pnh_.param("prepare_angle_per_moment_rad_per_nm", prepare_angle_per_moment_, 1.0);
  pnh_.param("prepare_min_straighten_angle_rad", prepare_min_straighten_angle_, 0.0174533);
  pnh_.param("prepare_max_straighten_angle_rad", prepare_max_straighten_angle_, 0.2617994);
  pnh_.param("prepare_max_rotor_tilt_rad", prepare_max_rotor_tilt_, 1.3962634);
  pnh_.param("prepare_initial_thrust_n", prepare_shape_thrust_n_, 0.0);
  pnh_.param("prepare_thrust_ramp_rate_n_s", prepare_thrust_ramp_rate_, 1.5);
  pnh_.param("prepare_max_thrust_n", prepare_max_thrust_n_, 9.0);
  pnh_.param("pwm_test_max_thrust_n", pwm_test_max_thrust_n_, 11.2);
  pnh_.param("pwm_test_max_reverse_thrust_n",
             pwm_test_max_reverse_thrust_n_, 4.4);
  pnh_.param("pwm_test_min_pwm", pwm_test_min_pwm_, 0.5);
  pnh_.param("pwm_test_neutral_pwm", pwm_test_neutral_pwm_, 0.75);
  pnh_.param("pwm_test_max_pwm", pwm_test_max_pwm_, 0.975);
  pnh_.param("pwm_test_voltage", pwm_test_voltage_, 21.2);
  pnh_.param("pwm_test_reference_voltage", pwm_test_reference_voltage_, 21.2);
  pnh_.param("pwm_test_hard_forward_thrust_n",
             pwm_test_hard_forward_thrust_n_, 6.0);
  pnh_.param("pwm_test_hard_reverse_thrust_n",
             pwm_test_hard_reverse_thrust_n_, 4.4);
  pnh_.param("pwm_test_output_rise_n_s", pwm_test_output_rise_n_s_, 20.0);
  pnh_.param("pwm_test_output_fall_n_s", pwm_test_output_fall_n_s_, 30.0);
  if (!std::isfinite(pwm_test_hard_forward_thrust_n_) ||
      !std::isfinite(pwm_test_hard_reverse_thrust_n_) ||
      !std::isfinite(pwm_test_output_rise_n_s_) ||
      !std::isfinite(pwm_test_output_fall_n_s_) ||
      pwm_test_hard_forward_thrust_n_ <= 0.0 ||
      pwm_test_hard_reverse_thrust_n_ <= 0.0 ||
      pwm_test_output_rise_n_s_ <= 0.0 || pwm_test_output_fall_n_s_ <= 0.0)
    throw std::runtime_error("invalid final PWM thrust safety parameters");
  pwm_test_hard_forward_thrust_n_ = std::min(
      pwm_test_hard_forward_thrust_n_, pwm_test_max_thrust_n_);
  pwm_test_hard_reverse_thrust_n_ = std::min(
      pwm_test_hard_reverse_thrust_n_, pwm_test_max_reverse_thrust_n_);
  pnh_.param("pwm_test_positive_thrust_below_neutral",
             pwm_test_positive_thrust_below_neutral_, true);
  loadFixedVector(pnh_, "pwm_test_forward_polynomial",
                  pwm_test_forward_polynomial_);
  loadFixedVector(pnh_, "pwm_test_reverse_polynomial",
                  pwm_test_reverse_polynomial_);
  bool pwm_calibration_finite = std::isfinite(pwm_test_min_pwm_) &&
      std::isfinite(pwm_test_neutral_pwm_) &&
      std::isfinite(pwm_test_max_pwm_) && std::isfinite(pwm_test_voltage_) &&
      std::isfinite(pwm_test_reference_voltage_);
  for (const double coefficient : pwm_test_forward_polynomial_)
    pwm_calibration_finite = pwm_calibration_finite &&
        std::isfinite(coefficient);
  for (const double coefficient : pwm_test_reverse_polynomial_)
    pwm_calibration_finite = pwm_calibration_finite &&
        std::isfinite(coefficient);
  if (!pwm_calibration_finite || pwm_test_min_pwm_ >= pwm_test_neutral_pwm_ ||
      pwm_test_neutral_pwm_ >= pwm_test_max_pwm_ ||
      pwm_test_voltage_ <= 0.0 || pwm_test_reference_voltage_ <= 0.0)
    throw std::runtime_error("invalid PWM/MotorInfo calibration");
  pnh_.param("prepare_pressure_ramp_rate_kpa_s", prepare_pressure_ramp_rate_, 10.0);
  pnh_.param("minimum_prepare_pressure_kpa", minimum_prepare_pressure_kpa_, 0.0);
  loadFixedVector(pnh_, "recovery_circle_pressure_candidates_kpa",
                  recovery_circle_pressure_candidates_kpa_);
  for (double& pressure : recovery_circle_pressure_candidates_kpa_)
    {
      if (!std::isfinite(pressure))
        throw std::runtime_error(
            "recovery_circle_pressure_candidates_kpa must be finite");
      pressure = hugmy::recoveryClamp(
          pressure, minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
    }
  std::sort(recovery_circle_pressure_candidates_kpa_.begin(),
            recovery_circle_pressure_candidates_kpa_.end());
  pnh_.param("prepare_angle_tolerance_rad", prepare_angle_tolerance_, 0.00872665);
  pnh_.param("prepare_progress_epsilon_rad", prepare_progress_epsilon_, 0.0174533);
  pnh_.param("prepare_progress_timeout_sec", prepare_progress_timeout_, 0.5);
  pnh_.param("neuron_timeout_sec", neuron_timeout_sec_, 0.10);
  pnh_.param("neuron_acc_lpf_tau_sec", neuron_acc_lpf_tau_sec_, 0.20);
  pnh_.param("pressure_safety_limit_kpa", pressure_safety_limit_kpa_, 60.0);
  pnh_.param("pressure_safety_confirm_sec", pressure_safety_confirm_sec_, 0.08);
  pnh_.param("fixed_arm_test_enabled", fixed_arm_test_enabled_, false);
  pnh_.param("require_arm_geometry_for_prepare",
             require_arm_geometry_for_prepare_, true);
  pnh_.param("fixed_test_arm_index", fixed_test_arm_index_, 0);
  pnh_.param("fixed_test_pressure_kpa", fixed_test_pressure_kpa_, 40.0);
  pnh_.param("fixed_test_thrust_n", fixed_test_thrust_n_, 2.0);
  pnh_.param("use_pwm_test_for_correction", use_pwm_test_for_correction_, true);
  pnh_.param("inchworm_front_release_kpa", inchworm_front_release_kpa_, 5.0);
  pnh_.param("inchworm_swing_pressure_kpa", inchworm_swing_pressure_kpa_, 0.0);
  pnh_.param("inchworm_transfer_pressure_kpa", inchworm_transfer_pressure_kpa_, 20.0);
  pnh_.param("inchworm_rear_unload_kpa", inchworm_rear_unload_kpa_, 8.0);
  pnh_.param("inchworm_pull_pressure_kpa", inchworm_pull_pressure_kpa_, 5.0);
  pnh_.param("inchworm_front_pull_pressure_kpa",
             inchworm_front_pull_pressure_kpa_, 30.0);
  pnh_.param("inchworm_reach_thrust_n", inchworm_reach_thrust_n_, 2.0);
  pnh_.param("inchworm_reach_max_thrust_n", inchworm_reach_max_thrust_n_, 6.0);
  pnh_.param("inchworm_reach_angle_kp_n_rad", inchworm_reach_angle_kp_n_rad_, 45.0);
  pnh_.param("inchworm_reach_min_pressure_kpa", inchworm_reach_min_pressure_kpa_, 10.0);
  pnh_.param("inchworm_reach_pressure_ramp_kpa_s", inchworm_reach_pressure_ramp_kpa_s_, 5.0);
  pnh_.param("inchworm_min_usable_reach_angle_rad", inchworm_min_usable_reach_angle_rad_, 0.0349066);
  pnh_.param("inchworm_rear_lift_thrust_n", inchworm_rear_lift_thrust_n_, 3.0);
  pnh_.param("inchworm_rear_lift_max_thrust_n", inchworm_rear_lift_max_thrust_n_, 7.0);
  pnh_.param("inchworm_rear_reverse_thrust_n",
             inchworm_rear_reverse_thrust_n_, 0.0);
  pnh_.param("inchworm_thrust_ramp_n_s", inchworm_thrust_ramp_n_s_, 1.0);
  pnh_.param("inchworm_default_distance_m", inchworm_default_distance_m_, 0.002);
  pnh_.param("inchworm_stroke_distance_m", inchworm_stroke_distance_m_, 0.002);
  pnh_.param("inchworm_min_cycle_progress_m", inchworm_min_cycle_progress_m_, 0.0002);
  pnh_.param("inchworm_max_cycles", inchworm_max_cycles_, 10);
  pnh_.param("inchworm_reach_angle_rad", inchworm_reach_angle_rad_, 0.0872665);
  pnh_.param("inchworm_target_bend_angle_rad", inchworm_target_bend_angle_rad_, 0.0872665);
  pnh_.param("inchworm_target_straightening_angle_rad",
             inchworm_target_straightening_angle_rad_, -1.0);
  pnh_.param("inchworm_angular_rate_fault_delay_sec",
             inchworm_angular_rate_fault_delay_sec_, 0.20);
  pnh_.param("inchworm_mocap_timeout_sec", inchworm_mocap_timeout_sec_, 0.20);
  pnh_.param("hand_relative_mocap_enabled",
             hand_relative_mocap_enabled_, false);
  pnh_.param("hand_target_selects_direction",
             hand_target_selects_direction_, false);
  pnh_.param("use_mocap_attitude_for_inchworm",
             use_mocap_attitude_for_inchworm_, false);
  if (hand_relative_mocap_enabled_)
    {
      // Human-arm locomotion uses the hand marker only as a translating
      // reference.  Its quaternion must never change the vehicle attitude
      // target or the Spinal attitude estimate.
      use_mocap_attitude_for_inchworm_ = false;
      target_roll_ = 0.0;
      target_pitch_ = 0.0;
      target_received_ = true;
    }
  pnh_.param("inchworm_release_timeout_sec", inchworm_release_timeout_sec_, 30.0);
  pnh_.param("inchworm_reach_duration_sec", inchworm_reach_duration_sec_, 10.0);
  pnh_.param("inchworm_anchor_timeout_sec", inchworm_anchor_timeout_sec_, 8.0);
  pnh_.param("inchworm_pull_timeout_sec", inchworm_pull_timeout_sec_, 5.0);
  pnh_.param("inchworm_settle_duration_sec", inchworm_settle_duration_sec_, 0.5);
  pnh_.param("action_a_place_bend_target_rad",
             action_a_place_bend_target_rad_, 90.0 * M_PI / 180.0);
  pnh_.param("action_a_place_hold_sec", action_a_place_hold_sec_, 0.0);
  pnh_.param("action_a_place_thrust_down_rate_n_s",
             action_a_place_thrust_down_rate_n_s_, 1.5);
  pnh_.param("action_a_rear_thrust_down_rate_n_s",
             action_a_rear_thrust_down_rate_n_s_, 2.0);
  pnh_.param("action_a_place_timeout_sec", action_a_place_timeout_sec_, 15.0);
  pnh_.param("action_a_place_contact_gap_m",
             action_a_place_contact_gap_m_, 0.030);
  pnh_.param("action_a_place_min_bend_change_rad",
             action_a_place_min_bend_change_rad_, 5.0 * M_PI / 180.0);
  pnh_.param("action_a_tilt_soft_limit_rad",
             action_a_tilt_soft_limit_rad_, 5.0 * M_PI / 180.0);
  pnh_.param("action_a_tilt_hard_limit_rad",
             action_a_tilt_hard_limit_rad_, 12.0 * M_PI / 180.0);
  pnh_.param("action_a_tilt_prediction_horizon_sec",
             action_a_tilt_prediction_horizon_sec_, 0.15);
  pnh_.param("action_a_tilt_thrust_down_rate_n_s",
             action_a_tilt_thrust_down_rate_n_s_, 3.0);
  pnh_.param("action_a_tilt_limited_bend_target_rad",
             action_a_tilt_limited_bend_target_rad_,
             45.0 * M_PI / 180.0);
  pnh_.param("action_a_yaw_assist_enabled",
             action_a_yaw_assist_enabled_, false);
  pnh_.param("action_a_yaw_target_rad",
             action_a_yaw_target_rad_, 10.0 * M_PI / 180.0);
  pnh_.param("action_a_yaw_tolerance_rad",
             action_a_yaw_tolerance_rad_, 1.0 * M_PI / 180.0);
  pnh_.param("action_a_yaw_max_reverse_thrust_n",
             action_a_yaw_max_reverse_thrust_n_, 0.6);
  pnh_.param("action_a_yaw_thrust_ramp_n_s",
             action_a_yaw_thrust_ramp_n_s_, 0.3);
  pnh_.param("action_a_yaw_thrust_down_rate_n_s",
             action_a_yaw_thrust_down_rate_n_s_, 1.5);
  pnh_.param("action_a_yaw_kp_n_rad",
             action_a_yaw_kp_n_rad_, 3.5);
  pnh_.param("action_a_yaw_max_rate_rad_s",
             action_a_yaw_max_rate_rad_s_, 20.0 * M_PI / 180.0);
  pnh_.param("action_a_yaw_wrong_way_limit_rad",
             action_a_yaw_wrong_way_limit_rad_, 3.0 * M_PI / 180.0);
  pnh_.param("action_a_diagonal_assist_enabled",
             action_a_diagonal_assist_enabled_, true);
  pnh_.param("action_a_diagonal_assist_thrust_n",
             action_a_diagonal_assist_thrust_n_, 2.0);
  pnh_.param("action_a_diagonal_assist_thrust_ramp_n_s",
             action_a_diagonal_assist_thrust_ramp_n_s_, 0.3);
  pnh_.param("action_a_diagonal_assist_thrust_down_rate_n_s",
             action_a_diagonal_assist_thrust_down_rate_n_s_, 1.5);
  pnh_.param("action_a_arm0_rebend_pressure_kpa",
             action_a_arm0_rebend_pressure_kpa_, 40.0);
  pnh_.param("action_a_bottom_enabled", action_a_bottom_enabled_, true);
  pnh_.param("action_a_bottom_low_kpa", action_a_bottom_low_kpa_, 0.0);
  pnh_.param("action_a_bottom_high_kpa", action_a_bottom_high_kpa_, 30.0);
  pnh_.param("inchworm_motion_start_speed_m_s", inchworm_motion_start_speed_m_s_, 0.001);
  pnh_.param("inchworm_motion_stop_speed_m_s", inchworm_motion_stop_speed_m_s_, 0.0002);
  pnh_.param("inchworm_motion_stop_timeout_sec", inchworm_motion_stop_timeout_sec_, 0.3);
  pnh_.param("inchworm_progress_epsilon_m", inchworm_progress_epsilon_m_, 0.00025);
  pnh_.param("inchworm_progress_regression_m", inchworm_progress_regression_m_, 0.00020);
  pnh_.param("inchworm_action_b_enabled", inchworm_action_b_enabled_, false);
  pnh_.param("inchworm_action_c_enabled", inchworm_action_c_enabled_, false);
  pnh_.param("inchworm_action_d_enabled", inchworm_action_d_enabled_, false);
  pnh_.param("rocking_gait_enabled", rocking_gait_enabled_, false);
  pnh_.param("rocking_bottom_low_kpa", rocking_bottom_low_kpa_, 0.0);
  pnh_.param("rocking_bottom_high_kpa", rocking_bottom_high_kpa_, 50.0);
  pnh_.param("rocking_front_release_kpa", rocking_front_release_kpa_, 0.0);
  pnh_.param("rocking_rear_release_kpa", rocking_rear_release_kpa_, 0.0);
  pnh_.param("rocking_front_windup_pressure_kpa",
             rocking_front_windup_pressure_kpa_, 30.0);
  pnh_.param("rocking_front_extend_thrust_n",
             rocking_front_extend_thrust_n_, 6.0);
  pnh_.param("rocking_rear_reverse_thrust_n",
             rocking_rear_reverse_thrust_n_, 0.8);
  pnh_.param("rocking_front_windup_target_rad",
             rocking_front_windup_target_rad_, 120.0 * M_PI / 180.0);
  pnh_.param("rocking_front_extend_target_rad",
             rocking_front_extend_target_rad_, 20.0 * M_PI / 180.0);
  pnh_.param("rocking_front_place_bend_target_rad",
             rocking_front_place_bend_target_rad_, 90.0 * M_PI / 180.0);
  pnh_.param("rocking_front_place_pressure_kpa",
             rocking_front_place_pressure_kpa_, 40.0);
  pnh_.param("rocking_front_min_advance_m",
             rocking_front_min_advance_m_, 0.003);
  pnh_.param("rocking_front_angle_tolerance_rad",
             rocking_front_angle_tolerance_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("rocking_front_tilt_target_rad",
             rocking_front_tilt_target_rad_, 13.5 * M_PI / 180.0);
  pnh_.param("rocking_recovery_tilt_target_rad",
             rocking_recovery_tilt_target_rad_, 0.5 * M_PI / 180.0);
  pnh_.param("rocking_phase_timeout_sec", rocking_phase_timeout_sec_, 10.0);
  pnh_.param("rocking_thrust_ramp_n_s", rocking_thrust_ramp_n_s_, 6.0);
  pnh_.param("rocking_front_windup_hold_sec",
             rocking_front_windup_hold_sec_, 0.20);
  pnh_.param("rocking_front_extend_hold_sec",
             rocking_front_extend_hold_sec_, 0.05);
  pnh_.param("rocking_front_lower_rate_n_s",
             rocking_front_lower_rate_n_s_, 0.25);
  pnh_.param("rocking_land_settle_sec", rocking_land_settle_sec_, 0.4);
  pnh_.param("rocking_max_tilt_rad", rocking_max_tilt_rad_, 30.0 * M_PI / 180.0);
  pnh_.param("action_d_target_angle_rad", action_d_target_angle_rad_, M_PI / 2.0);
  pnh_.param("action_d_soft_min_angle_rad", action_d_soft_min_angle_rad_,
             75.0 * M_PI / 180.0);
  pnh_.param("action_d_soft_max_angle_rad", action_d_soft_max_angle_rad_,
             105.0 * M_PI / 180.0);
  pnh_.param("action_d_hard_min_angle_rad", action_d_hard_min_angle_rad_,
             60.0 * M_PI / 180.0);
  pnh_.param("action_d_hard_max_angle_rad", action_d_hard_max_angle_rad_,
             120.0 * M_PI / 180.0);
  pnh_.param("action_d_max_angle_rate_rad_s", action_d_max_angle_rate_rad_s_,
             45.0 * M_PI / 180.0);
  pnh_.param("action_d_immediate_angle_rate_rad_s",
             action_d_immediate_angle_rate_rad_s_, 120.0 * M_PI / 180.0);
  pnh_.param("action_d_angle_rate_confirm_sec",
             action_d_angle_rate_confirm_sec_, 0.10);
  pnh_.param("action_d_rear_extension_margin_rad",
             action_d_rear_extension_margin_rad_, 10.0 * M_PI / 180.0);
  pnh_.param("action_d_rear_rate_filter_alpha",
             action_d_rear_rate_filter_alpha_, 0.25);
  pnh_.param("action_d_prediction_horizon_sec",
             action_d_prediction_horizon_sec_, 0.20);
  pnh_.param("action_d_pressure_kp_kpa_rad",
             action_d_pressure_kp_kpa_rad_, 20.0);
  pnh_.param("action_d_pressure_rate_kpa_s",
             action_d_pressure_rate_kpa_s_, 5.0);
  pnh_.param("action_d_max_pressure_kpa", action_d_max_pressure_kpa_, 30.0);
  pnh_.param("action_d_bottom_pressure_kpa",
             action_d_bottom_pressure_kpa_, 30.0);
  pnh_.param("action_d_front_pressure_per_thrust_kpa_n",
             action_d_front_pressure_per_thrust_kpa_n_, 2.5);
  pnh_.param("action_d_front_pressure_lead_n",
             action_d_front_pressure_lead_n_, 2.0);
  pnh_.param("action_d_rear_pressure_drop_per_thrust_kpa_n",
             action_d_rear_pressure_drop_per_thrust_kpa_n_, 0.0);
  pnh_.param("action_d_rear_min_pressure_kpa",
             action_d_rear_min_pressure_kpa_, 0.0);
  pnh_.param("action_d_front_thrust_ramp_n_s",
             action_d_front_thrust_ramp_n_s_, 2.0);
  pnh_.param("action_d_rear_thrust_ramp_n_s",
             action_d_rear_thrust_ramp_n_s_, 1.0);
  pnh_.param("action_d_thrust_step_n", action_d_thrust_step_n_, 1.0);
  pnh_.param("action_d_pulse_on_sec", action_d_pulse_on_sec_, 0.6);
  pnh_.param("action_d_pulse_off_sec", action_d_pulse_off_sec_, 1.0);
  pnh_.param("action_d_thrust_down_rate_n_s",
             action_d_thrust_down_rate_n_s_, 20.0);
  pnh_.param("action_d_max_front_thrust_n", action_d_max_front_thrust_n_, 6.0);
  pnh_.param("action_d_max_rear_thrust_n", action_d_max_rear_thrust_n_, 1.5);
  pnh_.param("action_d_prepare_tolerance_rad",
             action_d_prepare_tolerance_rad_, 3.0 * M_PI / 180.0);
  pnh_.param("action_d_prepare_stable_sec",
             action_d_prepare_stable_sec_, 0.50);
  pnh_.param("action_d_prepare_timeout_sec",
             action_d_prepare_timeout_sec_, 30.0);
  pnh_.param("action_d_ramp_timeout_sec", action_d_ramp_timeout_sec_, 30.0);
  pnh_.param("action_d_max_tilt_rad", action_d_max_tilt_rad_,
             20.0 * M_PI / 180.0);
  pnh_.param("action_d_tilt_soft_limit_rad",
             action_d_tilt_soft_limit_rad_, 12.0 * M_PI / 180.0);
  pnh_.param("action_d_tilt_resume_rad",
             action_d_tilt_resume_rad_, 8.0 * M_PI / 180.0);
  pnh_.param("action_d_max_body_rate_rad_s",
             action_d_max_body_rate_rad_s_, 0.8);
  pnh_.param("action_d_posture_thrust_down_rate_n_s",
             action_d_posture_thrust_down_rate_n_s_, 1.0);
  pnh_.param("action_d_motion_min_front_thrust_n",
             action_d_motion_min_front_thrust_n_, 0.5);
  pnh_.param("action_d_motion_min_travel_m",
             action_d_motion_min_travel_m_, 0.002);
  pnh_.param("action_d_motion_confirm_sec",
             action_d_motion_confirm_sec_, 0.20);
  pnh_.param("action_b_front_duration_sec", action_b_front_duration_sec_, 20.0);
  pnh_.param("action_b_thrust_ramp_n_s", action_b_thrust_ramp_n_s_, 3.0);
  pnh_.param("action_b_front_thrust_ramp_up_n_s",
             action_b_front_thrust_ramp_up_n_s_, 0.8);
  pnh_.param("action_b_shape_target_thrust_n",
             action_b_shape_target_thrust_n_, 8.0);
  pnh_.param("action_b_prediction_horizon_sec",
             action_b_prediction_horizon_sec_, 0.25);
  pnh_.param("action_b_front_transition_tilt_rad",
             action_b_front_transition_tilt_rad_, 0.0872665);
  pnh_.param("action_b_min_lift_tilt_rad",
             action_b_min_lift_tilt_rad_, 0.0872665);
  pnh_.param("action_b_hold_tilt_tolerance_rad",
             action_b_hold_tilt_tolerance_rad_, 0.00872665);
  pnh_.param("action_b_front_regrip_timeout_sec",
             action_b_front_regrip_timeout_sec_, 100.0);
  pnh_.param("action_b_hold_thrust_up_rate_n_s",
             action_b_hold_thrust_up_rate_n_s_, 3.0);
  pnh_.param("action_b_hold_thrust_down_rate_n_s",
             action_b_hold_thrust_down_rate_n_s_, 0.8);
  pnh_.param("action_b_hold_stable_duration_sec",
             action_b_hold_stable_duration_sec_, 0.3);
  pnh_.param("action_b_pulse_amplitude_n", action_b_pulse_amplitude_n_, 1.0);
  pnh_.param("action_b_max_thrust_n", action_b_max_thrust_n_, 9.0);
  pnh_.param("action_b_pre_pulse_max_thrust_n",
             action_b_pre_pulse_max_thrust_n_, 7.0);
  pnh_.param("action_b_pulse_on_duration_sec",
             action_b_pulse_on_duration_sec_, 1.25);
  pnh_.param("action_b_pulse_off_duration_sec",
             action_b_pulse_off_duration_sec_, 0.50);
  pnh_.param("action_b_pulse_repetitions", action_b_pulse_repetitions_, 3);
  pnh_.param("action_b_pulse_angle_pressure_gain_kpa_rad",
             action_b_pulse_angle_pressure_gain_kpa_rad_, 30.0);
  pnh_.param("action_b_shape_max_pressure_kpa",
             action_b_shape_max_pressure_kpa_, 30.0);
  pnh_.param("action_b_shape_pressure_rate_kpa_s",
             action_b_shape_pressure_rate_kpa_s_, 5.0);
  pnh_.param("action_b_shape_search_timeout_sec",
             action_b_shape_search_timeout_sec_, 60.0);
  pnh_.param("action_b_shape_settle_timeout_sec",
             action_b_shape_settle_timeout_sec_, 15.0);
  pnh_.param("action_b_hold_tilt_kp_n_rad",
             action_b_hold_tilt_kp_n_rad_, 8.0);
  pnh_.param("action_b_hold_tilt_ki_n_rad_s",
             action_b_hold_tilt_ki_n_rad_s_, 12.0);

  if (fixed_test_arm_index_ < 0 || fixed_test_arm_index_ >= static_cast<int>(ARM_COUNT))
    {
      ROS_WARN("fixed_test_arm_index must be 0..3; using arm 1");
      fixed_test_arm_index_ = 0;
    }
  fixed_test_pressure_kpa_ = std::max(0.0,
      std::min(pressure_safety_limit_kpa_, fixed_test_pressure_kpa_));
  fixed_test_thrust_n_ = std::max(0.0,
      std::min(correction_max_thrust_n_, fixed_test_thrust_n_));
  action_a_yaw_target_rad_ = std::max(0.0, std::min(
      0.5 * M_PI, action_a_yaw_target_rad_));
  action_a_yaw_tolerance_rad_ = std::max(0.0, std::min(
      action_a_yaw_target_rad_, action_a_yaw_tolerance_rad_));
  action_a_yaw_max_reverse_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_reverse_thrust_n_, action_a_yaw_max_reverse_thrust_n_));
  action_a_yaw_thrust_ramp_n_s_ = std::max(
      0.0, action_a_yaw_thrust_ramp_n_s_);
  action_a_yaw_thrust_down_rate_n_s_ = std::max(
      action_a_yaw_thrust_ramp_n_s_, action_a_yaw_thrust_down_rate_n_s_);
  action_a_yaw_kp_n_rad_ = std::max(0.0, action_a_yaw_kp_n_rad_);
  action_a_yaw_max_rate_rad_s_ = std::max(0.0,
      action_a_yaw_max_rate_rad_s_);
  action_a_yaw_wrong_way_limit_rad_ = std::max(0.0,
      action_a_yaw_wrong_way_limit_rad_);
  action_a_tilt_limited_bend_target_rad_ = std::max(
      inchworm_target_bend_angle_rad_, std::min(
          M_PI, action_a_tilt_limited_bend_target_rad_));
  action_a_diagonal_assist_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_thrust_n_, action_a_diagonal_assist_thrust_n_));
  action_a_diagonal_assist_thrust_ramp_n_s_ = std::max(
      0.0, action_a_diagonal_assist_thrust_ramp_n_s_);
  action_a_diagonal_assist_thrust_down_rate_n_s_ = std::max(
      action_a_diagonal_assist_thrust_ramp_n_s_,
      action_a_diagonal_assist_thrust_down_rate_n_s_);
  action_a_arm0_rebend_pressure_kpa_ = std::max(0.0, std::min(
      maximum_pressure_kpa_, action_a_arm0_rebend_pressure_kpa_));
  prepare_max_straighten_angle_ = std::max(
      prepare_min_straighten_angle_, prepare_max_straighten_angle_);
  correction_motion_detection_min_thrust_n_ = std::max(0.0, std::min(
      correction_max_thrust_n_, correction_motion_detection_min_thrust_n_));
  correction_motion_confirm_sec_ = std::max(0.0,
      correction_motion_confirm_sec_);
  correction_motion_min_error_reduction_ = std::max(0.0,
      correction_motion_min_error_reduction_);
  correction_prediction_horizon_sec_ = std::max(0.0,
      correction_prediction_horizon_sec_);
  correction_stall_timeout_sec_ = std::max(0.0,
      correction_stall_timeout_sec_);
  correction_stall_abort_timeout_sec_ = std::max(
      correction_stall_timeout_sec_, correction_stall_abort_timeout_sec_);
  correction_stall_thrust_ramp_n_s_ = std::max(0.0,
      correction_stall_thrust_ramp_n_s_);
  correction_progress_epsilon_ = std::max(0.0,
      correction_progress_epsilon_);
  rocking_front_windup_pressure_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, rocking_front_windup_pressure_kpa_));
  rocking_front_extend_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_thrust_n_, rocking_front_extend_thrust_n_));
  rocking_front_place_pressure_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, rocking_front_place_pressure_kpa_));
  rocking_front_windup_target_rad_ = std::max(0.0, std::min(
      M_PI, rocking_front_windup_target_rad_));
  rocking_front_extend_target_rad_ = std::max(0.0, std::min(
      rocking_front_windup_target_rad_, rocking_front_extend_target_rad_));
  rocking_front_place_bend_target_rad_ = std::max(
      rocking_front_extend_target_rad_, std::min(
          M_PI, rocking_front_place_bend_target_rad_));
  rocking_front_min_advance_m_ = std::max(0.0, rocking_front_min_advance_m_);
  rocking_front_angle_tolerance_rad_ = std::max(
      0.0, rocking_front_angle_tolerance_rad_);
  // Predictive Action C constraints and shared roll-recovery settings.
  pnh_.param("action_c_rear_release_max_kpa", action_c_rear_release_max_kpa_, 8.0);
  pnh_.param("action_c_rear_release_hold_kpa", action_c_rear_release_hold_kpa_, 10.0);
  pnh_.param("action_c_rear_release_hard_kpa", action_c_rear_release_hard_kpa_, 12.0);
  pnh_.param("action_c_rear_release_grace_sec", action_c_rear_release_grace_sec_, 0.10);
  pnh_.param("action_c_abort_reverse_travel_m", action_c_abort_reverse_travel_m_, 0.03);
  pnh_.param("action_c_max_thrust_n", action_c_max_thrust_n_, 6.0);
  pnh_.param("action_c_extension_target_rad",
             action_c_extension_target_rad_, 8.0 * M_PI / 180.0);
  pnh_.param("action_c_max_extension_rad",
             action_c_max_extension_rad_, 20.0 * M_PI / 180.0);
  pnh_.param("action_c_max_extension_mismatch_rad",
             action_c_max_extension_mismatch_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("action_c_min_safe_angle_rad",
             action_c_min_safe_angle_rad_, 55.0 * M_PI / 180.0);
  pnh_.param("action_c_hard_min_bend_rad",
             action_c_hard_min_bend_rad_, 30.0 * M_PI / 180.0);
  pnh_.param("action_c_start_extension_margin_rad",
             action_c_start_extension_margin_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_extension_rate_limit_rad_s",
             action_c_extension_rate_limit_rad_s_, 180.0 * M_PI / 180.0);
  pnh_.param("action_c_pre_roll_rate_tolerance_rad_s",
             action_c_pre_roll_rate_tolerance_rad_s_, 10.0 * M_PI / 180.0);
  pnh_.param("action_c_prepare_timeout_sec",
             action_c_prepare_timeout_sec_, 20.0);
  pnh_.param("action_c_posture_recovery_trigger_rad",
             action_c_posture_recovery_trigger_rad_,
             10.0 * M_PI / 180.0);
  pnh_.param("action_c_preload_recovery_trigger_rad",
             action_c_preload_recovery_trigger_rad_,
             15.0 * M_PI / 180.0);
  pnh_.param("action_c_posture_resume_rad",
             action_c_posture_resume_rad_, 3.0 * M_PI / 180.0);
  pnh_.param("action_c_posture_settle_sec", action_c_posture_settle_sec_, 0.5);
  pnh_.param("action_c_posture_rate_trigger_rad_s",
             action_c_posture_rate_trigger_rad_s_, 0.8);
  pnh_.param("action_c_emergency_stop_roll_rad",
             action_c_emergency_stop_roll_rad_, M_PI / 2.0);
  pnh_.param("action_c_max_posture_recoveries",
             action_c_max_posture_recoveries_, 5);
  pnh_.param("action_c_bottom_deflation_tolerance_kpa", action_c_bottom_deflation_tolerance_kpa_, 5.0);
  pnh_.param("action_c_roll_trend_window_sec", action_c_roll_trend_.window_sec, 0.5);
  pnh_.param("action_c_roll_trend_growth_rad", action_c_roll_trend_.growth_rad, M_PI / 180.0);
  pnh_.param("action_c_roll_trend_rate_rad_s", action_c_roll_trend_.outward_rate_rad_s, M_PI / 180.0);
  pnh_.param("action_c_posture_hard_roll_rad", action_c_posture_hard_roll_rad_, 20.0 * M_PI / 180.0);
  pnh_.param("action_c_pressure_hold_tolerance_kpa", action_c_pressure_hold_tolerance_kpa_, 5.0);
  pnh_.param("action_c_pressure_dropout_sec", action_c_pressure_dropout_sec_, 0.10);
  pnh_.param("action_c_pressure_max_deviation_kpa", action_c_pressure_max_deviation_kpa_, 8.0);
  action_c_rear_release_max_kpa_ = std::max(0.0, std::min(10.0, action_c_rear_release_max_kpa_));
  action_c_abort_reverse_travel_m_ = std::max(0.005, std::min(0.03, action_c_abort_reverse_travel_m_));
  action_c_posture_settle_sec_ = std::max(0.0, action_c_posture_settle_sec_);
  action_c_extension_target_rad_ = std::max(0.0,
      action_c_extension_target_rad_);
  action_c_max_extension_rad_ = std::max(action_c_extension_target_rad_,
      action_c_max_extension_rad_);
  action_c_max_extension_mismatch_rad_ = std::max(0.0,
      action_c_max_extension_mismatch_rad_);
  action_c_extension_rate_limit_rad_s_ = std::max(0.0,
      action_c_extension_rate_limit_rad_s_);
  action_c_roll_trend_.window_sec = std::max(0.2, std::min(1.0, action_c_roll_trend_.window_sec));
  action_c_roll_trend_.growth_rad = std::max(0.1 * M_PI / 180.0, action_c_roll_trend_.growth_rad);
  action_c_roll_trend_.outward_rate_rad_s = std::max(0.0, action_c_roll_trend_.outward_rate_rad_s);
  action_c_posture_hard_roll_rad_ = std::max(action_c_posture_recovery_trigger_rad_,
      std::min(30.0 * M_PI / 180.0, action_c_posture_hard_roll_rad_));
  action_c_preload_recovery_trigger_rad_ = std::max(
      action_c_posture_recovery_trigger_rad_,
      std::min(action_c_posture_hard_roll_rad_,
               action_c_preload_recovery_trigger_rad_));
  action_c_pressure_hold_tolerance_kpa_ = std::max(0.0, std::min(5.0, action_c_pressure_hold_tolerance_kpa_));
  action_c_pressure_dropout_sec_ = std::max(0.0, std::min(0.10, action_c_pressure_dropout_sec_));
  action_c_pressure_max_deviation_kpa_ = std::max(action_c_pressure_hold_tolerance_kpa_,
      std::min(8.0, action_c_pressure_max_deviation_kpa_));
  action_c_bottom_deflation_tolerance_kpa_ = std::max(0.0, std::min(5.0, action_c_bottom_deflation_tolerance_kpa_));
  action_c_pre_roll_rate_tolerance_rad_s_ = std::max(0.0,
      action_c_pre_roll_rate_tolerance_rad_s_);
  action_c_prepare_timeout_sec_ = std::max(1.0,
      action_c_prepare_timeout_sec_);
  action_c_posture_resume_rad_ = std::max(0.0, std::min(
      action_c_posture_recovery_trigger_rad_,
      action_c_posture_resume_rad_));
  action_c_posture_rate_trigger_rad_s_ = std::max(0.0,
      action_c_posture_rate_trigger_rad_s_);
  action_c_emergency_stop_roll_rad_ = std::max(
      action_c_posture_hard_roll_rad_, std::min(M_PI,
          action_c_emergency_stop_roll_rad_));
  action_c_max_posture_recoveries_ = std::max(1,
      action_c_max_posture_recoveries_);

  if (inchworm_action_c_enabled_)
    {
      for (double value : {action_c_max_thrust_n_, action_c_extension_target_rad_,
          action_c_max_extension_rad_, action_c_min_safe_angle_rad_, action_c_hard_min_bend_rad_,
          action_c_start_extension_margin_rad_, action_c_posture_recovery_trigger_rad_,
          action_c_posture_resume_rad_, action_c_rear_release_max_kpa_,
          action_c_rear_release_hold_kpa_, action_c_rear_release_hard_kpa_,
          action_c_rear_release_grace_sec_, action_c_abort_reverse_travel_m_})
        if (!std::isfinite(value) || value < 0.0)
          throw std::invalid_argument("Invalid Action C constraint");
      if (action_c_hard_min_bend_rad_ > action_c_min_safe_angle_rad_ ||
          action_c_start_extension_margin_rad_ <= 0.0 ||
          action_c_rear_release_max_kpa_ > action_c_rear_release_hold_kpa_ ||
          action_c_rear_release_hold_kpa_ > action_c_rear_release_hard_kpa_ ||
          action_c_rear_release_hard_kpa_ > 12.0 || action_c_rear_release_grace_sec_ > 0.3 ||
          action_c_posture_resume_rad_ > action_c_posture_recovery_trigger_rad_)
        throw std::invalid_argument("Inconsistent Action C limits");
    }
  action_d_target_angle_rad_ = std::max(0.0, std::min(
      M_PI, action_d_target_angle_rad_));
  action_d_soft_min_angle_rad_ = std::max(0.0, std::min(
      action_d_target_angle_rad_, action_d_soft_min_angle_rad_));
  action_d_soft_max_angle_rad_ = std::max(action_d_target_angle_rad_, std::min(
      M_PI, action_d_soft_max_angle_rad_));
  action_d_hard_min_angle_rad_ = std::max(0.0, std::min(
      action_d_target_angle_rad_, action_d_hard_min_angle_rad_));
  action_d_hard_max_angle_rad_ = std::max(action_d_target_angle_rad_, std::min(
      M_PI, action_d_hard_max_angle_rad_));
  action_d_max_angle_rate_rad_s_ = std::max(0.0,
      action_d_max_angle_rate_rad_s_);
  action_d_immediate_angle_rate_rad_s_ = std::max(
      action_d_max_angle_rate_rad_s_, action_d_immediate_angle_rate_rad_s_);
  action_d_angle_rate_confirm_sec_ = std::max(0.0,
      action_d_angle_rate_confirm_sec_);
  action_d_rear_extension_margin_rad_ = std::max(0.0, std::min(
      M_PI, action_d_rear_extension_margin_rad_));
  action_d_rear_rate_filter_alpha_ = std::max(0.0, std::min(
      1.0, action_d_rear_rate_filter_alpha_));
  pressure_safety_confirm_sec_ = std::max(0.0,
      pressure_safety_confirm_sec_);
  action_d_max_pressure_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, action_d_max_pressure_kpa_));
  action_d_bottom_pressure_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, action_d_bottom_pressure_kpa_));
  action_d_front_pressure_per_thrust_kpa_n_ = std::max(
      0.0, action_d_front_pressure_per_thrust_kpa_n_);
  action_d_front_pressure_lead_n_ = std::max(
      0.0, action_d_front_pressure_lead_n_);
  action_d_rear_pressure_drop_per_thrust_kpa_n_ = std::max(
      0.0, action_d_rear_pressure_drop_per_thrust_kpa_n_);
  action_d_rear_min_pressure_kpa_ = std::max(0.0, std::min(
      action_d_max_pressure_kpa_, action_d_rear_min_pressure_kpa_));
  action_d_max_front_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_thrust_n_, action_d_max_front_thrust_n_));
  action_d_max_rear_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_reverse_thrust_n_, action_d_max_rear_thrust_n_));
  action_d_front_thrust_ramp_n_s_ = std::max(
      0.1, action_d_front_thrust_ramp_n_s_);
  action_d_rear_thrust_ramp_n_s_ = std::max(
      0.0, action_d_rear_thrust_ramp_n_s_);
  action_d_thrust_step_n_ = std::max(0.1, std::min(
      action_d_max_front_thrust_n_, action_d_thrust_step_n_));
  action_d_pulse_on_sec_ = std::max(0.0, action_d_pulse_on_sec_);
  action_d_pulse_off_sec_ = std::max(0.0, action_d_pulse_off_sec_);
  action_d_max_tilt_rad_ = std::max(0.0, action_d_max_tilt_rad_);
  action_d_tilt_soft_limit_rad_ = std::max(0.0, std::min(
      action_d_max_tilt_rad_, action_d_tilt_soft_limit_rad_));
  action_d_tilt_resume_rad_ = std::max(0.0, std::min(
      action_d_tilt_soft_limit_rad_, action_d_tilt_resume_rad_));
  action_d_max_body_rate_rad_s_ = std::max(
      0.0, action_d_max_body_rate_rad_s_);
  action_d_posture_thrust_down_rate_n_s_ = std::max(
      0.0, action_d_posture_thrust_down_rate_n_s_);

  loadFixedVector(pnh_, "contact_position_main_body_m", contact_position_);
  hybrid_contact_position_=contact_position_;
  loadFixedVector(pnh_, "hybrid_contact_position_main_body_m", hybrid_contact_position_);
  loadFixedVector(pnh_, "center_of_mass_main_body_m", center_of_mass_);
  std::vector<double> bases;
  if (pnh_.getParam("arm_base_positions_main_body_m", bases))
    {
      if (bases.size() == ARM_COUNT * 3)
        for (size_t arm = 0; arm < ARM_COUNT; ++arm)
          std::copy_n(bases.begin() + arm * 3, 3, arm_base_position_[arm].begin());
      else
        ROS_WARN("arm_base_positions_main_body_m must contain 12 values; using URDF defaults");
    }
  loadFixedVector(pnh_, "arm_yaw_rad", arm_yaw_);
  std::vector<int> neuron_ids;
  if (pnh_.getParam("neuron_slave_ids", neuron_ids) && neuron_ids.size() == ARM_COUNT)
    std::copy(neuron_ids.begin(), neuron_ids.end(), neuron_slave_ids_.begin());

  last_pressure_.fill(base_pressure_kpa_);
  imu_sub_ = nh_.subscribe("imu", 1, &AttitudePressureController::imuCallback, this);
  if (nh_.resolveName("imu") != "/imu")
    root_imu_sub_ = nh_.subscribe("/imu", 1,
                                  &AttitudePressureController::rootImuCallback, this);
  thrust_sub_ = nh_.subscribe("target_thrust", 1,
                              &AttitudePressureController::thrustCallback, this);
  if (nh_.resolveName("target_thrust") != "/target_thrust")
    root_thrust_sub_ = nh_.subscribe("/target_thrust", 1,
      &AttitudePressureController::rootThrustCallback, this);
  neuron_imu_sub_ = nh_.subscribe("neuron/imu_states", 1,
                                  &AttitudePressureController::neuronImuCallback, this);
  if (nh_.resolveName("neuron/imu_states") != "/neuron/imu_states")
    root_neuron_imu_sub_ = nh_.subscribe("/neuron/imu_states", 1,
      &AttitudePressureController::rootNeuronImuCallback, this);
  pressure_sub_ = nh_.subscribe("independent_arm_pressure_controller/pressure", 1,
                                &AttitudePressureController::pressureCallback, this);
  grip_state_sub_ = nh_.subscribe(
      "pneumatic/grip_state", 1,
      &AttitudePressureController::gripStateCallback, this);
  bottom_pressure_sub_ = nh_.subscribe(
      "independent_arm_pressure_controller/bottom_pressure", 1,
      &AttitudePressureController::bottomPressureCallback, this);
  target_sub_ = pnh_.subscribe("target_attitude", 1,
                               &AttitudePressureController::targetCallback, this);
  inchworm_command_sub_ = pnh_.subscribe("inchworm_command", 1,
      &AttitudePressureController::inchwormCommandCallback, this);
  mocap_sub_ = nh_.subscribe("mocap/pose", 1,
      &AttitudePressureController::mocapCallback, this);
  if (nh_.resolveName("mocap/pose") != "/mocap/pose")
    root_mocap_sub_ = nh_.subscribe("/mocap/pose", 1,
        &AttitudePressureController::rootMocapCallback, this);
  hand_mocap_sub_ = nh_.subscribe("mocap/hand_pose", 1,
      &AttitudePressureController::handMocapCallback, this);
  if (nh_.resolveName("mocap/hand_pose") != "/mocap/hand_pose")
    root_hand_mocap_sub_ = nh_.subscribe("/mocap/hand_pose", 1,
        &AttitudePressureController::rootHandMocapCallback, this);
  pressure_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "independent_arm_pressure_controller/target_pressure", 1);
  bottom_target_pub_ = nh_.advertise<std_msgs::Float32>(
      "independent_arm_pressure_controller/bottom_target_pressure", 1);
  bottom_direction_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      "pneumatic/bottom_force_direction", 1, true);
  rpy_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("current_rpy", 1);
  attitude_error_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("attitude_error", 1);
  arm_relative_angle_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "arm_relative_angle", 1);
  arm_target_relative_angle_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "arm_target_relative_angle", 1);
  arm_shape_error_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "arm_shape_error", 1);
  arm_shape_active_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "arm_shape_active", 1);
  arm_bend_angle_deg_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "arm_bend_angle_deg", 1);
  recovery_circle_geometry_pub_ =
      pnh_.advertise<std_msgs::Float32MultiArray>(
          "recovery_circle_geometry", 1);
  hand_relative_position_pub_ =
      pnh_.advertise<geometry_msgs::Vector3Stamped>(
          "hand_relative_position", 1);
  desired_moment_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("desired_moment", 1);
  achieved_moment_pub_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("predicted_moment", 1);
  correction_thrust_pub_ = nh_.advertise<spinal::PerchingThrustCommand>(
      "perching_correction/thrust_command", 1);
  if (nh_.resolveName("perching_correction/thrust_command") !=
      "/perching_correction/thrust_command")
    root_correction_thrust_pub_ = nh_.advertise<spinal::PerchingThrustCommand>(
        "/perching_correction/thrust_command", 1);
  pwm_test_pub_ = nh_.advertise<spinal::PwmTest>("pwm_test", 1);
  if (nh_.resolveName("pwm_test") != "/pwm_test")
    root_pwm_test_pub_ = nh_.advertise<spinal::PwmTest>("/pwm_test", 1);
  flight_config_pub_ = nh_.advertise<spinal::FlightConfigCmd>("flight_config_cmd", 1);
  if (nh_.resolveName("flight_config_cmd") != "/flight_config_cmd")
    root_flight_config_pub_ = nh_.advertise<spinal::FlightConfigCmd>(
        "/flight_config_cmd", 1);
  state_pub_ = pnh_.advertise<std_msgs::UInt8>("state", 1, true);
  pnh_.param("hybrid_locomotion_outer_enabled", hybrid_search_.enabled, true);
  pnh_.param("hybrid_locomotion_initial_bottom_kpa", hybrid_search_.initial.bottom, 30.0);
  hybrid_single_config_.bottom_kpa=hybrid_search_.initial.bottom;
  pnh_.param("hybrid_slip_probe_sec",hybrid_single_config_.slip_seconds,20.0);
  pnh_.param("hybrid_slip_no_progress_sec",
      hybrid_single_config_.slip_no_progress_seconds,3.0);
  pnh_.param("hybrid_slip_confirm_sec",
      hybrid_single_config_.slip_confirm_seconds,.3);
  pnh_.param("hybrid_slip_min_progress_m",
      hybrid_single_config_.slip_min_progress,.002);
  pnh_.param("hybrid_slip_min_speed_m_s",
      hybrid_single_config_.slip_min_speed,.0015);
  pnh_.param("hybrid_slip_settle_seconds",
      hybrid_single_config_.slip_settle_seconds,.5);
  pnh_.param("hybrid_slip_settle_timeout_seconds",
      hybrid_single_config_.slip_settle_timeout,2.0);
  pnh_.param("hybrid_slip_retained_ratio",
      hybrid_single_config_.slip_retained_ratio,.75);
  pnh_.param("hybrid_slip_peak_retained_ratio",
      hybrid_single_config_.slip_peak_retained_ratio,.50);
  pnh_.param("hybrid_slip_min_thrust_n",
      hybrid_single_config_.slip_min_thrust,.75);
  pnh_.param("hybrid_soft_roll_confirm_sec",
      hybrid_single_config_.soft_roll_confirm,.10);
  pnh_.param("hybrid_slip_roll_abort_angle_rad",
      hybrid_single_config_.slip_roll_abort_angle,15.*M_PI/180.);
  pnh_.param("hybrid_slip_roll_abort_rate_rad_s",
      hybrid_single_config_.slip_roll_abort_rate,.02);
  pnh_.param("hybrid_stop_roll_recovery_confirm_sec",
      hybrid_single_config_.stop_roll_recovery_confirm,.75);
  pnh_.param("hybrid_pre_stroke_roll_limit_rad",
      hybrid_single_config_.pre_stroke_roll_limit,6.*M_PI/180.);
  pnh_.param("hybrid_inner_roll_threshold_rad",
      hybrid_single_config_.soft_roll,6.*M_PI/180.);
  pnh_.param("hybrid_roll_rate_confirm_sec",
      hybrid_roll_rate_confirm_sec_,.10);
  pnh_.param("hybrid_lift_probe_sec",hybrid_single_config_.lift_seconds,20.0);
  pnh_.param("hybrid_front_vent_timeout_sec",hybrid_single_config_.vent_seconds,30.0);
  pnh_.param("hybrid_front_vent_tolerance_kpa",hybrid_single_config_.vent_tolerance,5.0);
  pnh_.param("hybrid_front_vent_pressure_fall_kpa_s",
      hybrid_single_config_.vent_pressure_fall,15.0);
  pnh_.param("hybrid_reused_lift_roll_grace_sec",
      hybrid_single_config_.reused_lift_roll_grace,.5);
  pnh_.param("hybrid_lift_tilt_target_rad",hybrid_single_config_.tilt_target,3.*M_PI/180.);
  pnh_.param("hybrid_pull_sec",hybrid_single_config_.pull_seconds,4.0);
  pnh_.param("hybrid_pull_min_sec",hybrid_single_config_.pull_min_seconds,.5);
  pnh_.param("hybrid_front_catch_tolerance_rad",
      hybrid_single_config_.front_catch_tolerance,4.*M_PI/180.);
  pnh_.param("hybrid_pull_progress_threshold_m",
      hybrid_single_config_.pull_progress_threshold,.002);
  pnh_.param("hybrid_pull_adaptation_step_n",
      hybrid_single_config_.pull_adaptation_step,.5);
  pnh_.param("hybrid_pull_adaptation_max_n",
      hybrid_single_config_.pull_adaptation_max,6.0);
  pnh_.param("hybrid_slip_seed_pull_max_n",
      hybrid_single_config_.slip_seed_pull_max,2.55);
  pnh_.param("hybrid_slip_seed_pull_min_n",
      hybrid_single_config_.slip_seed_pull_min,2.5);
  pnh_.param("hybrid_goal_scale_min",
      hybrid_single_config_.goal_scale_min,.30);
  pnh_.param("hybrid_goal_conditioning_start_ratio",
      hybrid_single_config_.goal_conditioning_start_ratio,.55);
  pnh_.param("hybrid_goal_progress_max_scale",
      hybrid_single_config_.goal_progress_max_scale,1.5);
  pnh_.param("hybrid_goal_preferred_stride_m",
      hybrid_single_config_.goal_preferred_stride,.010);
  pnh_.param("hybrid_goal_extension_min_scale",
      hybrid_single_config_.goal_extension_min_scale,.55);
  pnh_.param("hybrid_goal_pull_taper_fraction",
      hybrid_single_config_.goal_pull_taper_fraction,.15);
  pnh_.param("hybrid_front_catch_stall_confirm_sec",
      hybrid_single_config_.catch_stall_confirm,.5);
  pnh_.param("hybrid_front_catch_stall_rate_rad_s",
      hybrid_single_config_.catch_stall_rate,.06);
  pnh_.param("hybrid_pull_regrip_min_flex_ratio",
      hybrid_single_config_.pull_regrip_min_flex,.08);
  pnh_.param("hybrid_pull_regrip_confirm_sec",
      hybrid_single_config_.pull_regrip_confirm,.20);
  pnh_.param("hybrid_pull_residual_thrust_ratio",
      hybrid_single_config_.pull_residual_thrust_ratio,.35);
  pnh_.param("hybrid_pull_full_thrust_flex_ratio",
      hybrid_single_config_.pull_full_thrust_flex_ratio,.35);
  pnh_.param("hybrid_pull_up_hold_flex_ratio",
      hybrid_single_config_.pull_up_hold_flex_ratio,.60);
  pnh_.param("hybrid_pull_forward_start_flex_ratio",
      hybrid_single_config_.pull_forward_start_flex_ratio,.05);
  pnh_.param("hybrid_pull_rear_positive_thrust_n",
      hybrid_single_config_.pull_rear_positive_thrust,1.2);
  pnh_.param("hybrid_pull_rear_positive_duration_sec",
      hybrid_single_config_.pull_rear_positive_duration,1.5);
  pnh_.param("hybrid_pull_rear_positive_front_start_n",
      hybrid_single_config_.pull_rear_positive_front_start,.8);
  pnh_.param("hybrid_pull_rear_positive_front_full_n",
      hybrid_single_config_.pull_rear_positive_front_full,3.0);
  pnh_.param("hybrid_pull_rear_positive_rise_n_s",
      hybrid_single_config_.pull_rear_positive_rise,4.0);
  pnh_.param("hybrid_pull_rear_positive_fall_n_s",
      hybrid_single_config_.pull_rear_positive_fall,6.0);
  pnh_.param("hybrid_pull_rear_positive_max_pressure_kpa",
      hybrid_single_config_.pull_rear_positive_max_pressure,12.0);
  pnh_.param("hybrid_pull_rear_positive_pressure_fade_kpa",
      hybrid_single_config_.pull_rear_positive_pressure_fade,8.0);
  pnh_.param("hybrid_pull_rear_positive_flex_end_ratio",
      hybrid_single_config_.pull_rear_positive_flex_end,.60);
  pnh_.param("hybrid_pull_terminal_rear_lift_enabled",
      hybrid_single_config_.pull_terminal_rear_lift_enabled,true);
  pnh_.param("hybrid_pull_terminal_rear_lift_thrust_n",
      hybrid_single_config_.pull_terminal_rear_lift_thrust,1.2);
  pnh_.param("hybrid_pull_terminal_rear_lift_duration_sec",
      hybrid_single_config_.pull_terminal_rear_lift_duration,.40);
  pnh_.param("hybrid_pull_terminal_rear_lift_rise_n_s",
      hybrid_single_config_.pull_terminal_rear_lift_rise,8.0);
  pnh_.param("hybrid_pull_terminal_rear_lift_fall_n_s",
      hybrid_single_config_.pull_terminal_rear_lift_fall,12.0);
  pnh_.param("hybrid_pull_terminal_front_pressure_tolerance_kpa",
      hybrid_single_config_.pull_terminal_front_pressure_tolerance,.75);
  pnh_.param("hybrid_pull_terminal_rear_pressure_tolerance_kpa",
      hybrid_single_config_.pull_terminal_rear_pressure_tolerance,2.0);
  pnh_.param("hybrid_pull_terminal_anchor_wait_timeout_sec",
      hybrid_single_config_.pull_terminal_anchor_wait_timeout,2.0);
  pnh_.param("hybrid_pull_terminal_front_pressure_stall_sec",
      hybrid_single_config_.pull_terminal_front_pressure_stall_seconds,.25);
  pnh_.param("hybrid_pull_terminal_front_pressure_progress_kpa",
      hybrid_single_config_.pull_terminal_front_pressure_progress_kpa,.20);
  pnh_.param("hybrid_pull_golden_roll_limit_rad",
      hybrid_single_config_.pull_golden_roll_limit,12.*M_PI/180.);
  pnh_.param("hybrid_lift_rear_release_start_ratio",
      hybrid_single_config_.lift_rear_release_start_ratio,.10);
  pnh_.param("hybrid_lift_rear_pressure_floor_kpa",
      hybrid_single_config_.lift_rear_pressure_floor,0.0);
  pnh_.param("hybrid_rear_pressure_adaptation_enabled",
      hybrid_single_config_.rear_pressure_adaptation_enabled,true);
  pnh_.param("hybrid_rear_pressure_adaptation_min_kpa",
      hybrid_single_config_.rear_pressure_adaptation_min,0.0);
  pnh_.param("hybrid_rear_pressure_adaptation_max_kpa",
      hybrid_single_config_.rear_pressure_adaptation_max,30.0);
  pnh_.param("hybrid_rear_pressure_adaptation_up_step_kpa",
      hybrid_single_config_.rear_pressure_adaptation_up_step,5.0);
  pnh_.param("hybrid_rear_pressure_adaptation_down_step_kpa",
      hybrid_single_config_.rear_pressure_adaptation_down_step,2.5);
  pnh_.param("hybrid_rear_pressure_adaptation_roll_limit_rad",
      hybrid_single_config_.rear_pressure_adaptation_roll_limit,
      12.*M_PI/180.);
  pnh_.param("hybrid_rear_pressure_adaptation_progress_m",
      hybrid_single_config_.rear_pressure_adaptation_progress,.002);
  pnh_.param("hybrid_slip_preserve_rear_grasp",
      hybrid_single_config_.slip_preserve_rear_grasp,true);
  pnh_.param("hybrid_rear_support_probe_sliding",
      hybrid_single_config_.rear_support_probe_sliding,-1.0);
  pnh_.param("hybrid_rear_support_probe_stable",
      hybrid_single_config_.rear_support_probe_stable,0.0);
  pnh_.param("hybrid_rear_support_probe_unstable",
      hybrid_single_config_.rear_support_probe_unstable,.25);
  pnh_.param("hybrid_rear_support_roll_start_rad",
      hybrid_single_config_.rear_support_roll_start,4.*M_PI/180.);
  pnh_.param("hybrid_rear_support_roll_full_rad",
      hybrid_single_config_.rear_support_roll_full,10.*M_PI/180.);
  pnh_.param("hybrid_rear_motion_pressure_cap_kpa",
      hybrid_single_config_.rear_motion_pressure_cap,30.0);
  pnh_.param("hybrid_rear_release_angle_min_rad",
      hybrid_single_config_.rear_release_angle_min,3.*M_PI/180.);
  pnh_.param("hybrid_rear_release_angle_max_rad",
      hybrid_single_config_.rear_release_angle_max,5.*M_PI/180.);
  pnh_.param("hybrid_rear_release_angle_filter_tau_sec",
      hybrid_single_config_.rear_release_angle_filter_tau,.06);
  pnh_.param("hybrid_rear_release_assist_deadband_rad",
      hybrid_single_config_.rear_release_assist_deadband,.5*M_PI/180.);
  pnh_.param("hybrid_rear_release_assist_confirm_sec",
      hybrid_single_config_.rear_release_assist_confirm,.10);
  pnh_.param("hybrid_rear_release_pressure_kp_kpa_rad",
      hybrid_single_config_.rear_release_pressure_kp,40.0);
  pnh_.param("hybrid_rear_release_pressure_kd_kpa_s_rad",
      hybrid_single_config_.rear_release_pressure_kd,2.0);
  pnh_.param("hybrid_rear_release_pressure_trim_max_kpa",
      hybrid_single_config_.rear_release_pressure_trim_max,5.0);
  pnh_.param("hybrid_rear_release_thrust_kp_n_rad",
      hybrid_single_config_.rear_release_thrust_kp,3.0);
  pnh_.param("hybrid_rear_release_thrust_kd_n_s_rad",
      hybrid_single_config_.rear_release_thrust_kd,.25);
  pnh_.param("hybrid_rear_release_thrust_trim_max_n",
      hybrid_single_config_.rear_release_thrust_trim_max,.5);
  pnh_.param("hybrid_rear_release_mismatch_soft_rad",
      hybrid_single_config_.rear_release_mismatch_soft,6.*M_PI/180.);
  pnh_.param("hybrid_rear_release_mismatch_hard_rad",
      hybrid_single_config_.rear_release_mismatch_hard,12.*M_PI/180.);
  pnh_.param("hybrid_rear_release_realization_ratio",
      hybrid_single_config_.rear_release_realization_ratio,.5);
  pnh_.param("hybrid_regrip_rear_hold_pressure_kpa",
      hybrid_single_config_.regrip_rear_hold_pressure,10.0);
  pnh_.param("hybrid_pull_rear_release_start_ratio",
      hybrid_single_config_.pull_rear_release_start_ratio,.05);
  pnh_.param("hybrid_pull_rear_release_end_ratio",
      hybrid_single_config_.pull_rear_release_end_ratio,.65);
  pnh_.param("hybrid_pull_capture_pressure_kpa",
      hybrid_single_config_.pull_capture_pressure,15.0);
  pnh_.param("hybrid_pull_angle_kp_n_rad",
      hybrid_single_config_.pull_angle_kp,4.0);
  pnh_.param("hybrid_pull_angle_kd_n_s_rad",
      hybrid_single_config_.pull_angle_kd,.8);
  pnh_.param("hybrid_pull_angle_hold_max_n",
      hybrid_single_config_.pull_angle_hold_max,.5);
  pnh_.param("hybrid_pull_extension_brake_start_rad_s",
      hybrid_single_config_.pull_extension_brake_start,5.*M_PI/180.);
  pnh_.param("hybrid_pull_extension_brake_full_rad_s",
      hybrid_single_config_.pull_extension_brake_full,15.*M_PI/180.);
  pnh_.param("hybrid_pull_extension_brake_fall_n_s",
      hybrid_single_config_.pull_extension_brake_fall,8.0);
  pnh_.param("hybrid_pull_pressure_margin_kpa",
      hybrid_single_config_.pull_pressure_margin,0.0);
  pnh_.param("hybrid_pull_pressure_sync_gain_kpa",
      hybrid_single_config_.pull_pressure_sync_gain,60.0);
  pnh_.param("hybrid_pull_pressure_sync_max_drop_kpa",
      hybrid_single_config_.pull_pressure_sync_max_drop,20.0);
  pnh_.param("hybrid_pull_support_transfer_pressure_kpa",
      hybrid_single_config_.pull_support_transfer_pressure,5.0);
  pnh_.param("hybrid_pull_support_transfer_min_flex_ratio",
      hybrid_single_config_.pull_support_transfer_min_flex_ratio,.55);
  pnh_.param("hybrid_pull_loaded_catch_pressure_kpa",
      hybrid_single_config_.pull_loaded_catch_pressure,18.0);
  pnh_.param("hybrid_pull_loaded_catch_min_sec",
      hybrid_single_config_.pull_loaded_catch_min_seconds,3.0);
  pnh_.param("hybrid_front_catch_roll_angle_rad",
      hybrid_single_config_.catch_roll_angle,10.*M_PI/180.);
  pnh_.param("hybrid_front_catch_safe_margin_rad",
      hybrid_single_config_.catch_safe_margin,M_PI/180.);
  pnh_.param("hybrid_front_catch_pressure_tolerance_kpa",
      hybrid_single_config_.catch_pressure_tolerance,3.0);
  pnh_.param("hybrid_front_catch_roll_rate_rad_s",
      hybrid_single_config_.catch_roll_rate,.08);
  pnh_.param("hybrid_front_catch_roll_prediction_tolerance_rad",
      hybrid_single_config_.catch_roll_prediction_tolerance,M_PI/180.);
  pnh_.param("hybrid_front_regrip_timeout_sec",hybrid_single_config_.front_wait_seconds,8.0);
  pnh_.param("hybrid_rear_release_sec",hybrid_single_config_.rear_low_seconds,2.0);
  pnh_.param("hybrid_rear_grasp_sec",hybrid_single_config_.rear_grasp_seconds,1.0);
  pnh_.param("hybrid_rear_low_kpa",hybrid_single_config_.rear_low_kpa,0.0);
  pnh_.param("hybrid_rear_preload_kpa",
      hybrid_single_config_.rear_preload_kpa,15.0);
  pnh_.param("hybrid_rear_vent_lead_margin_sec",
      hybrid_single_config_.rear_vent_lead_margin,.10);
  pnh_.param("hybrid_rear_vent_start_margin_rad",
      hybrid_single_config_.rear_vent_start_margin,2.*M_PI/180.);
  pnh_.param("hybrid_probe_extension_rad",hybrid_single_config_.extension_rad,8.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_bend_rad",
      hybrid_single_config_.identification_bend,45.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_tolerance_rad",
      hybrid_single_config_.identification_tolerance,2.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_confirm_sec",
      hybrid_single_config_.identification_confirm,.06);
  pnh_.param("hybrid_lift_identification_extension_limit_rad",
      hybrid_single_config_.identification_extension_limit,99.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_sync_deadband_rad",
      hybrid_single_config_.identification_sync_deadband,3.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_sync_gain_n_rad",
      hybrid_single_config_.identification_sync_gain,10.);
  pnh_.param("hybrid_lift_identification_sync_assist_max_n",
      hybrid_single_config_.identification_sync_assist_max,1.0);
  pnh_.param("hybrid_lift_identification_tracking_start_rad",
      hybrid_single_config_.identification_tracking_start,8.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_angle_kp_n_rad",
      hybrid_single_config_.identification_angle_kp,4.);
  pnh_.param("hybrid_lift_identification_rate_kd_n_s_rad",
      hybrid_single_config_.identification_rate_kd,.8);
  pnh_.param("hybrid_lift_identification_tracking_rise_n_s",
      hybrid_single_config_.identification_tracking_rise,4.);
  pnh_.param("hybrid_lift_identification_rate_feedback_min_bend_rad",
      hybrid_single_config_.identification_rate_feedback_min_bend,45.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_rate_filter_tau_sec",
      hybrid_single_config_.identification_rate_filter_tau,.25);
  pnh_.param("hybrid_lift_identification_rate_confirm_sec",
      hybrid_single_config_.identification_rate_confirm,.15);
  pnh_.param("hybrid_lift_identification_rate_deadband_rad_s",
      hybrid_single_config_.identification_rate_deadband,2.*M_PI/180.);
  pnh_.param("hybrid_lift_identification_rate_control_gain_n_rad",
      hybrid_single_config_.identification_rate_control_gain,1.5);
  pnh_.param("hybrid_lift_identification_rate_control_fall_n_s",
      hybrid_single_config_.identification_rate_control_fall,3.0);
  pnh_.param("hybrid_lift_identification_bend_rate_limit_rad_s",
      hybrid_single_config_.identification_bend_rate_limit,6.);
  pnh_.param("hybrid_lift_identification_return_bend_rate_limit_rad_s",
      hybrid_single_config_.identification_return_bend_rate_limit,8.);
  pnh_.param("hybrid_lift_identification_return_pressure_slew_kpa_s",
      hybrid_single_config_.identification_return_pressure_slew,30.);
  pnh_.param("hybrid_lift_unload_min_extension_rad",
      hybrid_single_config_.unload_min_extension,3.*M_PI/180.);
  pnh_.param("hybrid_lift_unload_pair_mismatch_rad",
      hybrid_single_config_.unload_pair_mismatch,10.*M_PI/180.);
  pnh_.param("hybrid_lift_unload_pair_memory_sec",
      hybrid_single_config_.unload_pair_memory,.15);
  pnh_.param("hybrid_lift_unload_pitch_rad",
      hybrid_single_config_.unload_pitch,4.*M_PI/180.);
  pnh_.param("hybrid_lift_unload_height_m",
      hybrid_single_config_.unload_height,.003);
  pnh_.param("hybrid_directional_pitch_sign",
      hybrid_directional_pitch_sign_,-1.0);
  pnh_.param("hybrid_lift_unload_min_pitch_rad",
      hybrid_single_config_.unload_min_pitch,2.*M_PI/180.);
  pnh_.param("hybrid_lift_unload_height_floor_m",
      hybrid_single_config_.unload_height_floor,-.0005);
  pnh_.param("hybrid_lift_unload_min_up_force_n",
      hybrid_single_config_.unload_min_up_force,.20);
  pnh_.param("hybrid_lift_unload_confirm_sec",
      hybrid_single_config_.unload_confirm,.06);
  pnh_.param("hybrid_lift_unload_event_fusion_sec",
      hybrid_single_config_.unload_event_fusion,.75);
  pnh_.param("hybrid_lift_unload_prefill_start_ratio",
      hybrid_single_config_.unload_prefill_start_ratio,.70);
  pnh_.param("hybrid_lift_unload_prefill_kpa",
      hybrid_single_config_.unload_prefill_kpa,3.);
  pnh_.param("hybrid_lift_unload_prefill_max_prediction_sec",
      hybrid_single_config_.unload_prefill_max_prediction_sec,.60);
  pnh_.param("hybrid_lift_unload_prefill_max_prediction_lead_ratio",
      hybrid_single_config_.unload_prefill_max_prediction_lead_ratio,.15);
  pnh_.param("hybrid_slip_probe_max_thrust_n",hybrid_single_config_.slip_thrust_max,5.0);
  pnh_.param("hybrid_slip_probe_max_pressure_kpa",
      hybrid_single_config_.slip_pressure_max,40.0);
  pnh_.param("hybrid_slip_probe_rise_n_s",
      hybrid_single_config_.probe_rise,.25);
  pnh_.param("hybrid_slip_pressure_tracking_tolerance_kpa",
      hybrid_single_config_.slip_pressure_tracking_tolerance,2.0);
  pnh_.param("hybrid_slip_extension_hold_margin_rad",
      hybrid_single_config_.slip_extension_hold_margin,2.*M_PI/180.);
  pnh_.param("hybrid_slip_hard_extension_margin_rad",
      hybrid_single_config_.slip_hard_extension_margin,3.*M_PI/180.);
  pnh_.param("hybrid_slip_extension_brake_fall_n_s",
      hybrid_single_config_.slip_extension_brake_fall,.5);
  pnh_.param("hybrid_slip_max_extension_rad",
      hybrid_single_config_.slip_max_extension,10.*M_PI/180.);
  pnh_.param("hybrid_slip_pressure_drop_max_kpa",
      hybrid_single_config_.slip_pressure_drop_max,10.0);
  pnh_.param("hybrid_lift_search_rate_n_s",hybrid_single_config_.lift_rise,.7);
  pnh_.param("hybrid_reused_lift_rise_n_s",
      hybrid_single_config_.reused_lift_rise,8.0);
  pnh_.param("hybrid_initial_extension_pulse_thrust_n",
      hybrid_single_config_.initial_pulse_thrust,2.0);
  pnh_.param("hybrid_initial_extension_pulse_max_n",
      hybrid_single_config_.initial_pulse_max,4.0);
  pnh_.param("hybrid_initial_extension_pulse_step_n",
      hybrid_single_config_.initial_pulse_step,.5);
  pnh_.param("hybrid_initial_extension_pulse_step_sec",
      hybrid_single_config_.initial_pulse_step_seconds,1.0);
  pnh_.param("hybrid_initial_extension_pulse_rise_n_s",
      hybrid_single_config_.initial_pulse_rise,8.0);
  pnh_.param("hybrid_initial_extension_pulse_rate_rad_s",
      hybrid_single_config_.initial_pulse_extension_rate,.70);
  pnh_.param("hybrid_initial_extension_pulse_min_sec",
      hybrid_single_config_.initial_pulse_min_duration,.12);
  pnh_.param("hybrid_trajectory_min_extension_rad",
      hybrid_single_config_.trajectory_min_extension,15.*M_PI/180.);
  pnh_.param("hybrid_trajectory_extension_rate_rad_s",
      hybrid_single_config_.trajectory_extension_rate,.70);
  pnh_.param("hybrid_trajectory_extension_min_sec",
      hybrid_single_config_.trajectory_extension_min_duration,.30);
  pnh_.param("hybrid_trajectory_extension_max_sec",
      hybrid_single_config_.trajectory_extension_max_duration,3.0);
  pnh_.param("hybrid_trajectory_tracking_slow_error_rad",
      hybrid_single_config_.trajectory_tracking_slow_error,5.*M_PI/180.);
  pnh_.param("hybrid_trajectory_tracking_pause_error_rad",
      hybrid_single_config_.trajectory_tracking_pause_error,15.*M_PI/180.);
  pnh_.param("hybrid_trajectory_rate_filter_tau_sec",
      hybrid_single_config_.trajectory_rate_filter_tau,.04);
  pnh_.param("hybrid_trajectory_acceleration_filter_tau_sec",
      hybrid_single_config_.trajectory_acceleration_filter_tau,.08);
  pnh_.param("hybrid_trajectory_acceleration_limit_rad_s2",
      hybrid_single_config_.trajectory_acceleration_limit,20.0);
  pnh_.param("hybrid_trajectory_prediction_horizon_sec",
      hybrid_single_config_.trajectory_prediction_horizon,.08);
  pnh_.param("hybrid_trajectory_prediction_max_delta_rad",
      hybrid_single_config_.trajectory_prediction_max_delta,15.*M_PI/180.);
  pnh_.param("hybrid_trajectory_rate_lag_horizon_sec",
      hybrid_single_config_.trajectory_rate_lag_horizon,.08);
  pnh_.param("hybrid_trajectory_phase_correction_gain",
      hybrid_single_config_.trajectory_phase_correction_gain,1.5);
  pnh_.param("hybrid_trajectory_clock_max_scale",
      hybrid_single_config_.trajectory_clock_max_scale,1.5);
  pnh_.param("hybrid_trajectory_extension_kp_n_rad",
      hybrid_single_config_.trajectory_extension_kp,5.0);
  pnh_.param("hybrid_trajectory_extension_kd_n_s_rad",
      hybrid_single_config_.trajectory_extension_kd,.8);
  pnh_.param("hybrid_trajectory_extension_ki_n_rad_s",
      hybrid_single_config_.trajectory_extension_ki,.35);
  pnh_.param("hybrid_trajectory_extension_trim_max_n",
      hybrid_single_config_.trajectory_extension_trim_max,1.25);
  pnh_.param("hybrid_trajectory_windup_pitch_rad",
      hybrid_single_config_.trajectory_windup_pitch,17.*M_PI/180.);
  pnh_.param("hybrid_trajectory_golden_handoff_enabled",
      hybrid_single_config_.trajectory_golden_handoff,true);
  pnh_.param("hybrid_trajectory_windup_pitch_margin_rad",
      hybrid_single_config_.trajectory_windup_pitch_margin,3.*M_PI/180.);
  pnh_.param("hybrid_trajectory_windup_confirm_sec",
      hybrid_single_config_.trajectory_windup_confirm,.04);
  pnh_.param("hybrid_trajectory_windup_thrust_max_n",
      hybrid_single_config_.trajectory_windup_thrust_max,6.0);
  pnh_.param("hybrid_trajectory_windup_thrust_rise_n_s",
      hybrid_single_config_.trajectory_windup_thrust_rise,2.0);
  pnh_.param("hybrid_trajectory_windup_pitch_kp_n_rad",
      hybrid_single_config_.trajectory_windup_pitch_kp,3.0);
  pnh_.param("hybrid_trajectory_windup_pitch_kd_n_s_rad",
      hybrid_single_config_.trajectory_windup_pitch_kd,1.0);
  pnh_.param("hybrid_trajectory_windup_pitch_rate_max_rad_s",
      hybrid_single_config_.trajectory_windup_pitch_rate_max,0.25);
  pnh_.param("hybrid_trajectory_windup_bottom_kpa",
      hybrid_single_config_.trajectory_windup_bottom_kpa,30.0);
  pnh_.param("hybrid_trajectory_windup_extra_extension_rad",
      hybrid_single_config_.trajectory_windup_extra_extension,60.*M_PI/180.);
  pnh_.param("hybrid_trajectory_windup_hold_pressure_kpa",
      hybrid_single_config_.trajectory_windup_hold_pressure_max,30.0);
  pnh_.param("hybrid_trajectory_windup_brake_pressure_min_kpa",
      hybrid_single_config_.trajectory_windup_brake_pressure_min,5.0);
  pnh_.param("hybrid_trajectory_windup_brake_pressure_max_kpa",
      hybrid_single_config_.trajectory_windup_brake_pressure_max,12.0);
  pnh_.param("hybrid_trajectory_windup_brake_angle_kp_kpa_rad",
      hybrid_single_config_.trajectory_windup_brake_angle_kp,15.0);
  pnh_.param("hybrid_trajectory_windup_brake_rate_kd_kpa_s_rad",
      hybrid_single_config_.trajectory_windup_brake_rate_kd,2.0);
  pnh_.param("hybrid_trajectory_windup_capture_extension_rad",
      hybrid_single_config_.trajectory_windup_capture_extension,3.*M_PI/180.);
  pnh_.param("hybrid_trajectory_windup_capture_rate_rad_s",
      hybrid_single_config_.trajectory_windup_capture_rate,10.*M_PI/180.);
  pnh_.param("hybrid_trajectory_windup_capture_confirm_sec",
      hybrid_single_config_.trajectory_windup_capture_confirm,0.04);
  pnh_.param("hybrid_trajectory_windup_pressure_slew_kpa_s",
      hybrid_single_config_.trajectory_windup_pressure_slew,12.0);
  pnh_.param("hybrid_trajectory_windup_min_reachable_pitch_rad",
      hybrid_single_config_.trajectory_windup_min_reachable_pitch,4.*M_PI/180.);
  pnh_.param("hybrid_trajectory_windup_stall_sec",
      hybrid_single_config_.trajectory_windup_stall_seconds,1.25);
  pnh_.param("hybrid_trajectory_windup_rear_reverse_max_n",
      hybrid_single_config_.trajectory_windup_rear_reverse_max,1.5);
  pnh_.param("hybrid_trajectory_windup_rear_reverse_rise_n_s",
      hybrid_single_config_.trajectory_windup_rear_reverse_rise,0.75);
  pnh_.param("hybrid_trajectory_flex_rate_rad_s",
      hybrid_single_config_.trajectory_flex_rate,1.2);
  pnh_.param("hybrid_trajectory_flex_min_sec",
      hybrid_single_config_.trajectory_flex_min_duration,.65);
  pnh_.param("hybrid_trajectory_flex_max_sec",
      hybrid_single_config_.trajectory_flex_max_duration,2.0);
  pnh_.param("hybrid_trajectory_pressure_kp_kpa_rad",
      hybrid_single_config_.trajectory_pressure_kp,28.0);
  pnh_.param("hybrid_trajectory_pressure_kd_kpa_s_rad",
      hybrid_single_config_.trajectory_pressure_kd,3.0);
  pnh_.param("hybrid_trajectory_pressure_ki_kpa_rad_s",
      hybrid_single_config_.trajectory_pressure_ki,4.0);
  pnh_.param("hybrid_trajectory_pressure_trim_max_kpa",
      hybrid_single_config_.trajectory_pressure_trim_max,8.0);
  pnh_.param("hybrid_trajectory_pressure_lead_max_sec",
      hybrid_single_config_.trajectory_pressure_lead_max,.60);
  pnh_.param("hybrid_trajectory_pull_thrust_kp_n_rad",
      hybrid_single_config_.trajectory_pull_thrust_kp,2.0);
  pnh_.param("hybrid_trajectory_pull_thrust_kd_n_s_rad",
      hybrid_single_config_.trajectory_pull_thrust_kd,.35);
  pnh_.param("hybrid_trajectory_pull_thrust_trim_max_n",
      hybrid_single_config_.trajectory_pull_thrust_trim_max,.75);
  pnh_.param("hybrid_reused_event_checkpoint_margin_rad",
      hybrid_single_config_.reused_event_checkpoint_margin,4.*M_PI/180.);
  pnh_.param("hybrid_reused_event_search_rate_n_s",
      hybrid_single_config_.reused_event_search_rise,.25);
  pnh_.param("hybrid_reused_lift_brake_margin_rad",
      hybrid_single_config_.reused_lift_brake_margin,4.*M_PI/180.);
  pnh_.param("hybrid_reused_lift_hold_pressure_max_kpa",
      hybrid_single_config_.reused_lift_hold_pressure_max,10.);
  pnh_.param("hybrid_reused_lift_pressure_start_ratio",
      hybrid_single_config_.reused_lift_pressure_start_ratio,.90);
  pnh_.param("hybrid_lift_pressure_gain_kpa_s_rad",hybrid_single_config_.lift_pressure_gain,20.);
  pnh_.param("hybrid_lift_overlap_pitch_start_rad",
      hybrid_single_config_.lift_overlap_pitch_start,5.*M_PI/180.);
  pnh_.param("hybrid_lift_overlap_pressure_start_ratio",
      hybrid_single_config_.lift_overlap_pressure_start_ratio,.55);
  pnh_.param("hybrid_lift_overlap_handoff_ratio",
      hybrid_single_config_.lift_overlap_handoff_ratio,.60);
  pnh_.param("hybrid_lift_overlap_min_reach_ratio",
      hybrid_single_config_.lift_overlap_min_reach_ratio,.50);
  pnh_.param("hybrid_lift_overlap_prefill_kpa",
      hybrid_single_config_.lift_overlap_prefill_kpa,3.);
  pnh_.param("hybrid_lift_overlap_min_measured_kpa",
      hybrid_single_config_.lift_overlap_min_measured_kpa,2.);
  pnh_.param("hybrid_lift_overlap_max_prediction_sec",
      hybrid_single_config_.lift_overlap_max_prediction_sec,.60);
  pnh_.param("hybrid_lift_overlap_max_prediction_lead_ratio",
      hybrid_single_config_.lift_overlap_max_prediction_lead_ratio,.12);
  pnh_.param("hybrid_lift_overlap_min_up_scale",
      hybrid_single_config_.lift_overlap_min_up_scale,.65);
  pnh_.param("hybrid_lift_approach_rate_n_s",hybrid_single_config_.approach_rise,2.);
  pnh_.param("hybrid_lift_onset_margin_n",hybrid_single_config_.onset_margin_n,.6);
  pnh_.param("hybrid_lift_onset_angle_rad",hybrid_single_config_.onset_angle,M_PI/180.);
  pnh_.param("hybrid_lift_onset_rate_rad_s",hybrid_single_config_.onset_rate,.02);
  pnh_.param("hybrid_lift_onset_confirm_sec",hybrid_single_config_.onset_confirm,.10);
  pnh_.param("hybrid_lift_extension_rate_rad_s",hybrid_single_config_.extension_rate,.20);
  pnh_.param("hybrid_locomotion_initial_m", hybrid_search_.initial.minimum_ratio, 1.0);
  hybrid_search_.current=hybrid_search_.initial;
  pnh_.param("hybrid_locomotion_horizon_sec", hybrid_config_.horizon, 2.0);
  pnh_.param("hybrid_locomotion_budget_ms", hybrid_config_.budget_ms, 12.0);
  pnh_.param("hybrid_locomotion_target_speed_m_s", hybrid_config_.target_speed, .005);
  pnh_.param("hybrid_locomotion_sliding_rear_pressure_kpa", hybrid_config_.sliding_rear_pressure, 0.);
  pnh_.param("hybrid_locomotion_retain_grasp_pressure", hybrid_retain_grasp_pressure_, false);
  hybrid_grasp_pressure_.fill(base_pressure_kpa_);
  pnh_.param("hybrid_locomotion_search_initial_thrust_n", hybrid_config_.search_initial_thrust, 1.0);
  pnh_.param("hybrid_locomotion_search_thrust_step_n", hybrid_config_.search_thrust_step, .5);
  pnh_.param("hybrid_locomotion_search_interval_sec", hybrid_config_.search_interval, .5);
  pnh_.param("hybrid_locomotion_stroke_m", hybrid_config_.stroke, .02);
  pnh_.param("hybrid_locomotion_run_max_sec", hybrid_config_.run_max, 3.0);
  pnh_.param("hybrid_locomotion_regrip_timeout_sec", hybrid_config_.regrip_timeout, 20.0);
  pnh_.param("hybrid_locomotion_roll_limit_rad", hybrid_config_.roll_limit,
      action_c_posture_hard_roll_rad_);
  pnh_.param("hybrid_locomotion_pitch_change_limit_rad",
      hybrid_config_.pitch_change_limit,20.*M_PI/180.);
  pnh_.param("hybrid_locomotion_extension_limit_rad", hybrid_config_.extension_limit, 20.*M_PI/180.);
  pnh_.param("hybrid_locomotion_grasp_bend_margin_rad",
      hybrid_config_.grasp_bend_margin,5.*M_PI/180.);
  pnh_.param("hybrid_locomotion_up_force_max_n", hybrid_config_.up_force_max, 6.0);
  hybrid_config_.up_force_max=std::min(hybrid_config_.up_force_max, .6*mass_kg_*9.81);
  hybrid_config_.thrust_max=std::min(6., action_c_max_thrust_n_);
  hybrid_config_.thrust_max=std::min(hybrid_config_.thrust_max,
      use_pwm_test_for_correction_ ? pwm_test_max_thrust_n_ : correction_max_thrust_n_);
  hybrid_config_.pressure_max=std::min(50., maximum_pressure_kpa_);
  hybrid_config_.pressure_measurement_margin=action_c_pressure_max_deviation_kpa_;
  // Inner gait feedback starts before the ordinary 10-deg recovery request;
  // both still require an outward roll trend. A stable offset is allowed to
  // continue until the independent hard boundary.
  hybrid_single_config_.soft_roll=std::min(
      hybrid_single_config_.soft_roll,
      action_c_posture_recovery_trigger_rad_);
  hybrid_single_config_.pre_stroke_roll_limit=std::min(
      hybrid_single_config_.pre_stroke_roll_limit,
      action_c_posture_recovery_trigger_rad_);
  // Shared posture recovery follows the Spinal/body-roll convention.  The
  // front-pair contact moment used inside the gait is a separate mapping:
  // MuJoCo's applied front-rotor/contact response is opposite to hardware.
  // Keep the two signs independently configurable so simulation validation
  // cannot silently change the real-machine recovery convention.
  pnh_.param("hybrid_reverse_geometry_roll_moment",
      hybrid_single_config_.reverse_geometry_roll_moment,
      correction_pair_reverse_spinal_mapping_);
  hybrid_config_.roll_limit=std::min(hybrid_config_.roll_limit, action_c_posture_hard_roll_rad_);
  hybrid_config_.roll_rate_limit=std::min(hybrid_config_.roll_rate_limit, action_c_posture_rate_trigger_rad_s_);
  hybrid_config_.extension_limit=std::min(hybrid_config_.extension_limit, action_c_max_extension_rad_);
  hybrid_config_.bend_rate_limit=std::min(hybrid_config_.bend_rate_limit, action_c_extension_rate_limit_rad_s_);
  hybrid_config_.mismatch_limit=std::min(hybrid_config_.mismatch_limit, action_c_max_extension_mismatch_rad_);
  if (inchworm_action_c_enabled_ && (!hybrid_single_config_.valid() || !hybrid_config_.valid() ||
      !std::isfinite(hybrid_directional_pitch_sign_) ||
      std::abs(std::abs(hybrid_directional_pitch_sign_)-1.0)>1.0e-6 ||
      !std::isfinite(hybrid_roll_rate_confirm_sec_) ||
      hybrid_roll_rate_confirm_sec_<.04 || hybrid_roll_rate_confirm_sec_>.5 ||
      !std::isfinite(hybrid_search_.initial.bottom) || hybrid_search_.initial.bottom<0. || hybrid_search_.initial.bottom>30. ||
      !std::isfinite(hybrid_search_.initial.minimum_ratio) || hybrid_search_.initial.minimum_ratio<0. || hybrid_search_.initial.minimum_ratio>1.))
    throw std::runtime_error("Invalid hybrid locomotion configuration (fail closed)");
  if (inchworm_action_c_enabled_ && (inchworm_action_d_enabled_ || rocking_gait_enabled_ || inchworm_action_b_enabled_))
    throw std::runtime_error("Action C cannot be combined with D/B/rocking modes");
  hybrid_planner_.reset(hybrid_config_);
  hybrid_diagnostic_pub_=pnh_.advertise<std_msgs::Float32MultiArray>("hybrid_locomotion", 1);
  enable_server_ = pnh_.advertiseService("enable",
      &AttitudePressureController::enableCallback, this);
  pressure_enable_client_ = nh_.serviceClient<std_srvs::SetBool>(
      "independent_arm_pressure_controller/enable");
  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_hz_)),
                           &AttitudePressureController::update, this);
}

void AttitudePressureController::imuCallback(const spinal::Imu::ConstPtr& msg)
{
  updateImu(msg, false);
}

void AttitudePressureController::rootImuCallback(const spinal::Imu::ConstPtr& msg)
{
  updateImu(msg, true);
}

void AttitudePressureController::updateImu(const spinal::Imu::ConstPtr& msg,
                                           bool root_topics)
{
  const double x = msg->quaternion[0], y = msg->quaternion[1];
  const double z = msg->quaternion[2], w = msg->quaternion[3];
  const double norm = std::sqrt(x*x + y*y + z*z + w*w);
  if (!std::isfinite(norm) || norm < 1e-6) return;

  tf::Quaternion q(x / norm, y / norm, z / norm, w / norm);
  tf::Matrix3x3 rotation(q);
  if (!use_mocap_attitude_for_inchworm_ || !mocap_received_)
    rotation.getRPY(roll_, pitch_, yaw_);
  const tf::Vector3 gravity = rotation.inverse() * tf::Vector3(0.0, 0.0, -GRAVITY);
  gravity_body_ = {{gravity.x(), gravity.y(), gravity.z()}};
  for (size_t axis = 0; axis < 3; ++axis)
    body_angular_velocity_[axis] = msg->gyro[axis];
  imu_stamp_ = ros::Time::now();
  imu_received_ = true;
  if (!spinal_topic_route_initialized_ || use_root_spinal_topics_ != root_topics)
    ROS_INFO("Attitude pressure controller selected %s Spinal topic route",
             root_topics ? "root (/imu, bridge.launch)" : "robot-namespaced");
  use_root_spinal_topics_ = root_topics;
  spinal_topic_route_initialized_ = true;
}

void AttitudePressureController::thrustCallback(const spinal::Thrust::ConstPtr& msg)
{
  if (msg->thrust.size() != ARM_COUNT)
    {
      // Spinal publishes an empty target_thrust while normal flight control is
      // halted.  That is the expected condition for perching correction: its
      // independent thrust command is planned locally below.
      if (!state_machine_enabled_ || !msg->thrust.empty())
        ROS_WARN_THROTTLE(1.0, "target_thrust must contain exactly four values");
      return;
    }
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      if (!std::isfinite(msg->thrust[arm]) || msg->thrust[arm] < 0.0) return;
      thrust_n_[arm] = msg->thrust[arm];
    }
  thrust_stamp_ = ros::Time::now();
  thrust_received_ = true;
}

void AttitudePressureController::rootThrustCallback(const spinal::Thrust::ConstPtr& msg)
{
  thrustCallback(msg);
}

void AttitudePressureController::neuronImuCallback(const spinal::NeuronImuStates::ConstPtr& msg)
{
  updateNeuronImu(msg);
}

void AttitudePressureController::rootNeuronImuCallback(
    const spinal::NeuronImuStates::ConstPtr& msg)
{
  updateNeuronImu(msg);
}

void AttitudePressureController::updateNeuronImu(
    const spinal::NeuronImuStates::ConstPtr& msg)
{
  const ros::Time now = ros::Time::now();
  for (const auto& imu : msg->imus)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      if (imu.slave_id == neuron_slave_ids_[arm])
        {
          const double dt = neuron_stamp_[arm].isZero() ? 0.0
              : std::max(0.0, std::min(0.1, (now - neuron_stamp_[arm]).toSec()));
          const double alpha = neuron_acc_initialized_[arm]
              ? dt / std::max(1e-6, neuron_acc_lpf_tau_sec_ + dt) : 1.0;
          for (size_t axis = 0; axis < 3; ++axis)
            {
              neuron_acc_[arm][axis] += alpha *
                  (imu.acc[axis] - neuron_acc_[arm][axis]);
              neuron_gyro_[arm][axis] = imu.gyro[axis];
            }
          neuron_acc_initialized_[arm] = true;
          neuron_acc_norm_[arm] = std::sqrt(imu.acc[0]*imu.acc[0] +
              imu.acc[1]*imu.acc[1] + imu.acc[2]*imu.acc[2]);
          neuron_stamp_[arm] = now;
          break;
        }
}//imu callback stampいるのか？

void AttitudePressureController::pressureCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != ARM_COUNT) return;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm) if (!std::isfinite(msg->data[arm])) return;
  std::copy(msg->data.begin(), msg->data.end(), measured_pressure_.begin());
  measured_pressure_stamp_ = ros::Time::now();
}

void AttitudePressureController::gripStateCallback(
    const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // HugmyPneumaticHWSim preserves active/slip/force in the first 12 values
  // and publishes cylinder-axis coordinate and surface gap in [12:16] and
  // [20:24]. MuJoCo joint-sum bend telemetry is optional in [24:28].
  // Ignore legacy publishers without the trajectory contact telemetry.
  if (msg->data.size() < 6 * ARM_COUNT) return;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double axis_coordinate = msg->data[3 * ARM_COUNT + arm];
      const double surface_gap = msg->data[5 * ARM_COUNT + arm];
      if (!std::isfinite(axis_coordinate) || !std::isfinite(surface_gap))
        return;
      grip_active_[arm] = msg->data[arm] > 0.5;
      grip_axis_coordinate_m_[arm] = axis_coordinate;
      grip_surface_gap_m_[arm] = surface_gap;
      grip_joint_bend_rad_[arm] = msg->data.size() >= 7 * ARM_COUNT &&
              std::isfinite(msg->data[6 * ARM_COUNT + arm])
          ? msg->data[6 * ARM_COUNT + arm]
          : std::numeric_limits<double>::quiet_NaN();
    }
  grip_state_stamp_ = ros::Time::now();
  grip_state_valid_ = true;
}

void AttitudePressureController::bottomPressureCallback(
    const std_msgs::Float32::ConstPtr& msg)
{
  if (std::isfinite(msg->data))
    {
      bottom_pressure_kpa_ = msg->data;
      measured_bottom_pressure_stamp_ = ros::Time::now();
    }
}

void AttitudePressureController::targetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  if (!std::isfinite(msg->vector.x) || !std::isfinite(msg->vector.y)) return;
  if (hand_relative_mocap_enabled_)
    {
      if (std::abs(msg->vector.x) > 1.0e-6 ||
          std::abs(msg->vector.y) > 1.0e-6 ||
          std::abs(msg->vector.z) > 1.0e-6)
        ROS_WARN_THROTTLE(1.0,
            "Human-arm mode ignores nonzero target attitude; using [0, 0, 0] rad");
      target_roll_ = 0.0;
      target_pitch_ = 0.0;
      target_received_ = true;
      recovery_rejected_candidate_mask_ = 0;
      recovery_rejected_direction_sign_ = 0;
      recovery_candidate_reselect_pending_ = false;
      recovery_geometry_reestimate_attempted_ = false;
      recovery_geometry_reestimate_pending_ = false;
      recovery_circle_event_locked_ = false;
      recovery_preferred_pair_locked_ = false;
      return;
    }
  target_roll_ = msg->vector.x;
  target_pitch_ = msg->vector.y;
  target_received_ = true;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_preferred_pair_locked_ = false;
}//目標姿勢

void AttitudePressureController::mocapCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (mocap_topic_route_initialized_ && use_root_mocap_topic_) return;
  if (!mocap_topic_route_initialized_)
    {
      mocap_topic_route_initialized_ = true;
      use_root_mocap_topic_ = false;
      ROS_INFO("Using namespaced mocap/pose for inchworm displacement");
    }
  updateMocap(msg);
}

void AttitudePressureController::rootMocapCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (mocap_topic_route_initialized_ && !use_root_mocap_topic_) return;
  if (!mocap_topic_route_initialized_)
    {
      mocap_topic_route_initialized_ = true;
      use_root_mocap_topic_ = true;
      ROS_INFO("Using root /mocap/pose for inchworm displacement");
    }
  updateMocap(msg);
}

void AttitudePressureController::updateMocap(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mocap_position_ = {{msg->pose.position.x, msg->pose.position.y,
                      msg->pose.position.z}};
  if (use_mocap_attitude_for_inchworm_)
    {
      tf::Quaternion quaternion;
      tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
      const double norm = quaternion.length();
      if (std::isfinite(norm) && norm > 1.0e-6)
        {
          quaternion /= norm;
          tf::Matrix3x3(quaternion).getRPY(roll_, pitch_, yaw_);
        }
    }
  mocap_stamp_ = ros::Time::now();
  mocap_received_ = true;
  if (hand_mocap_received_)
    {
      geometry_msgs::Vector3Stamped relative;
      relative.header.stamp = mocap_stamp_;
      relative.header.frame_id = "world";
      relative.vector.x = mocap_position_[0] - hand_mocap_position_[0];
      relative.vector.y = mocap_position_[1] - hand_mocap_position_[1];
      // ID 2 is an XY reference only. Its height is intentionally excluded.
      relative.vector.z = 0.0;
      hand_relative_position_pub_.publish(relative);
    }
  // Keep a position window alive while Action C temporarily hands actuator
  // ownership to posture recovery. This provides a quiet, deflated endpoint
  // for an interrupted trial instead of sampling while the bottom is full.
  if (hybrid_trial_active_ || action_c_resume_pending_)
    hybrid_endpoint_mocap_window_.add(
        mocap_stamp_.toSec(), locomotionPosition()[0]);
}

void AttitudePressureController::handMocapCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (hand_mocap_topic_route_initialized_ && use_root_hand_mocap_topic_) return;
  if (!hand_mocap_topic_route_initialized_)
    {
      hand_mocap_topic_route_initialized_ = true;
      use_root_hand_mocap_topic_ = false;
      ROS_INFO("Using namespaced mocap/hand_pose as position-only locomotion reference");
    }
  updateHandMocap(msg);
}

void AttitudePressureController::rootHandMocapCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (hand_mocap_topic_route_initialized_ && !use_root_hand_mocap_topic_) return;
  if (!hand_mocap_topic_route_initialized_)
    {
      hand_mocap_topic_route_initialized_ = true;
      use_root_hand_mocap_topic_ = true;
      ROS_INFO("Using root /mocap/hand_pose as position-only locomotion reference");
    }
  updateHandMocap(msg);
}

void AttitudePressureController::updateHandMocap(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Rigid body ID 2 is an XY reference fixed to the hand. Deliberately do
  // not read either its height or its orientation.
  hand_mocap_position_ = {{msg->pose.position.x, msg->pose.position.y, 0.0}};
  hand_mocap_stamp_ = ros::Time::now();
  hand_mocap_received_ = true;
}

AttitudePressureController::Vector3
AttitudePressureController::locomotionPosition() const
{
  if (!hand_relative_mocap_enabled_)
    return mocap_position_;
  const double dx = hand_mocap_position_[0] - mocap_position_[0];
  const double dy = hand_mocap_position_[1] - mocap_position_[1];
  // Encode closeness as a signed 1-D task coordinate.  Pure translation or
  // rotation of the human arm preserves the robot--hand distance, so it does
  // not look like locomotion.  The sign keeps the existing +/-x Action-C
  // progress convention valid when the hand is on either side of the robot.
  const double distance = std::hypot(dx, dy);
  return {{-hand_progress_direction_sign_x_ * distance, -dy, 0.0}};
}

bool AttitudePressureController::locomotionMocapReady() const
{
  return mocap_received_ &&
      (!hand_relative_mocap_enabled_ || hand_mocap_received_);
}

bool AttitudePressureController::locomotionMocapFresh(
    const ros::Time& now) const
{
  if (!mocap_received_ || (now - mocap_stamp_).toSec() < 0.0 ||
      (now - mocap_stamp_).toSec() > inchworm_mocap_timeout_sec_)
    return false;
  if (!hand_relative_mocap_enabled_)
    return true;
  return hand_mocap_received_ &&
      (now - hand_mocap_stamp_).toSec() >= 0.0 &&
      (now - hand_mocap_stamp_).toSec() <= inchworm_mocap_timeout_sec_;
}

// --------------------------------------------------------------------------
// Inchworm locomotion command
// --------------------------------------------------------------------------
void AttitudePressureController::inchwormCommandCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  if (!enabled_)
    {
      ROS_ERROR("Enable the attitude pressure controller before commanding an inchworm step");
      return;
    }
  const double norm = std::hypot(msg->vector.x, msg->vector.y);
  if (!std::isfinite(norm) || !std::isfinite(msg->vector.z)) return;
  if (norm < 1e-6 || msg->vector.z <= 0.0)
    {
      stopInchwormToGrasp(ros::Time::now(), "zero/invalid motion command");
      return;
    }
  Vector2 direction{{msg->vector.x / norm, msg->vector.y / norm}};
  if (hand_relative_mocap_enabled_ && hand_target_selects_direction_ &&
      std::abs(direction[0]) >= 0.9 && hand_mocap_received_)
    {
      // In the longitudinal gait, "forward" means toward the hand marker.
      // Only ID 2 position is used; its orientation is intentionally ignored.
      direction = Vector2{{
          hand_mocap_position_[0] >= mocap_position_[0] ? 1.0 : -1.0,
          0.0}};
    }
  if (hand_relative_mocap_enabled_)
    hand_progress_direction_sign_x_ = direction[0] >= 0.0 ? 1.0 : -1.0;
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
      std::abs(direction[0]) < 0.9)
    {
      ROS_ERROR("Action C currently supports longitudinal commands only; use body direction [+1, 0] or [-1, 0]");
      return;
    }
  if (!locomotionMocapReady() || !locomotionMocapFresh(ros::Time::now()))
    {
      if (hand_relative_mocap_enabled_)
        ROS_ERROR("Cannot start inchworm motion before receiving fresh robot and hand mocap positions (ID 1 and ID 2)");
      else
        ROS_ERROR("Cannot start inchworm motion before receiving a fresh robot mocap position (ID 1)");
      return;
    }
  // Do not discover this missing prerequisite only after a powered pulse
  // has made recovery necessary. Zero/stop commands above still always work.
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ && !target_received_)
    {
      ROS_ERROR("Cannot start Action C before receiving target_attitude; set an explicit roll/pitch recovery target first");
      return;
    }
  // Action A deliberately releases two diagonal arms. Never enter that
  // sequence while the nominal support grasp is still inflating: the two
  // remaining arms would not be able to support the body, and FRONT_RELEASE
  // would only time out without ever publishing a rotor command.
  bool grasp_pressure_ready = true;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    grasp_pressure_ready = grasp_pressure_ready &&
        std::isfinite(measured_pressure_[arm]) &&
        std::abs(measured_pressure_[arm] - base_pressure_kpa_) <=
            pressure_tolerance_kpa_;
  if (!grasp_pressure_ready)
    {
      ROS_ERROR("Cannot start inchworm motion: initial grasp is not ready; measured arm pressure [%.1f, %.1f, %.1f, %.1f] kPa, required %.1f +/- %.1f kPa",
                measured_pressure_[0], measured_pressure_[1],
                measured_pressure_[2], measured_pressure_[3],
                base_pressure_kpa_, pressure_tolerance_kpa_);
      return;
    }
  // A new operator command supersedes any saved Action-C continuation.
  continuous_recovery_blocked_ = false;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_use_opposite_pair_ = false;
  continuous_recovery_retry_opposite_pending_ = false;
  continuous_recovery_pair_switches_ = 0;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_preferred_pair_locked_ = false;
  action_c_resume_pending_ = false;
  action_c_requested_recovery_trigger_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  action_c_preserve_adaptation_on_start_ = false;
  startInchwormStep(direction, msg->vector.z, ros::Time::now());
}

bool AttitudePressureController::enableCallback(std_srvs::SetBool::Request& req,
                                                std_srvs::SetBool::Response& res)
{
  if (req.data && !imu_received_)
    {
      res.success = false;
      res.message = "receive valid body IMU first";
      return true;
    }

  // Treat repeated enable=true requests as idempotent.  Some bringup/joy
  // paths reassert enable while a task is running.  Resetting the state here
  // used to cancel ACTION_B immediately after it entered the pulse phase and
  // jump directly to GRASP without passing through ACTION_B_REGRASP.
  if (req.data && enabled_)
    {
      res.success = true;
      res.message = "perching correction already enabled; current state preserved";
      ROS_INFO("Ignoring repeated enable=true in state %u; preserving the active task",
               state_);
      return true;
    }

  last_pressure_.fill(base_pressure_kpa_);
  hybrid_hold_pressure_=false;
  hybrid_hold_bottom_=false;
  hybrid_grasp_pressure_.fill(base_pressure_kpa_);
  continuous_recovery_blocked_ = false;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_use_opposite_pair_ = false;
  continuous_recovery_retry_opposite_pending_ = false;
  continuous_recovery_pair_switches_ = 0;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_preferred_pair_locked_ = false;
  grasp_baseline_valid_ = false;
  inchworm_active_ = false;
  action_c_recovery_gate_.reset();
  action_c_resume_pending_ = false;
  action_c_requested_recovery_trigger_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  action_c_preserve_adaptation_on_start_ = false;
  const ros::Time now = ros::Time::now();
  enterState(GRASP, now);

  // The state machine owns the pressure target while it is active, so its
  // enable operation must also arm the lower pressure loop.  Publish a safe
  // grasp target first; the lower controller initializes to a measured-pressure
  // hold if this first topic has not crossed the ROS connection yet.
  if (req.data)
    {
      publishPressureTarget(PressureArray{{base_pressure_kpa_, base_pressure_kpa_,
                                           base_pressure_kpa_, base_pressure_kpa_}},
                            0.0);
      publishBottomTarget(rocking_bottom_low_kpa_);
    }
  std_srvs::SetBool pressure_enable;
  pressure_enable.request.data = req.data;
  if (!pressure_enable_client_.call(pressure_enable) ||
      !pressure_enable.response.success)
    {
      enabled_ = false;
      res.success = false;
      res.message = "failed to switch independent arm pressure controller";
      return true;
    }

  enabled_ = req.data;
  spinal::FlightConfigCmd config;
  config.cmd = enabled_ ? spinal::FlightConfigCmd::PERCHING_CORRECTION_ON_CMD
                        : spinal::FlightConfigCmd::PERCHING_CORRECTION_OFF_CMD;
  if (use_root_spinal_topics_ && root_flight_config_pub_)
    root_flight_config_pub_.publish(config);
  else
    flight_config_pub_.publish(config);
  PressureArray zero{{0.0, 0.0, 0.0, 0.0}};
  publishCorrectionThrust(zero);
  if (!enabled_) publishBottomTarget(rocking_bottom_low_kpa_);
  res.success = true;
  res.message = enabled_ ? "perching correction state machine enabled"
                         : "perching correction disabled";
  return true;
}

void AttitudePressureController::update(const ros::TimerEvent& event)
{
  const ros::Time now = ros::Time::now();
  const double roll_error = wrapAngle(target_roll_ - roll_);
  const double pitch_error = wrapAngle(target_pitch_ - pitch_);
  geometry_msgs::Vector3Stamped rpy, error_msg;
  rpy.header.stamp = error_msg.header.stamp = now;
  rpy.vector.x = roll_; rpy.vector.y = pitch_; rpy.vector.z = yaw_;
  error_msg.vector.x = roll_error; error_msg.vector.y = pitch_error;
  rpy_pub_.publish(rpy);
  attitude_error_pub_.publish(error_msg);

  // This is the directly measured Spinal--Neuron bend estimate: 0 deg is a
  // straight arm and positive values are the one permitted bending direction.
  // It deliberately does not include the pressure/thrust table, so it can be
  // inspected independently from the model used for feed-forward control.
  std_msgs::Float32MultiArray arm_bend_deg;
  arm_bend_deg.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    arm_bend_deg.data[arm] = static_cast<float>(
        measuredArmBendAngle(arm) * 180.0 / M_PI);
  arm_bend_angle_deg_pub_.publish(arm_bend_deg);
  updateCircularGraspGeometry(now);

  const double dt = std::max(0.0, std::min(0.1, (event.current_real - event.last_real).toSec()));
  const double maximum_step = pressure_rate_limit_kpa_s_ * dt;

  if (state_machine_enabled_ && enabled_)
    {
      PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
      const bool body_imu_fresh = imu_received_ && (now - imu_stamp_).toSec() <= imu_timeout_sec_;
      bool pressure_finite = true;
      bool pressure_over_limit = false;
      double maximum_measured_pressure_kpa = -std::numeric_limits<double>::infinity();
      for (const double pressure : measured_pressure_)
        {
          pressure_finite = pressure_finite && std::isfinite(pressure);
          if (std::isfinite(pressure))
            {
              maximum_measured_pressure_kpa = std::max(
                  maximum_measured_pressure_kpa, pressure);
              pressure_over_limit = pressure_over_limit ||
                  pressure >= pressure_safety_limit_kpa_;
            }
        }
      const bool action_a_selected = !rocking_gait_enabled_ &&
          !inchworm_action_b_enabled_ && !inchworm_action_c_enabled_ &&
          !inchworm_action_d_enabled_;
      const bool action_a_uses_bottom = action_a_bottom_enabled_ &&
          action_a_selected && inchworm_active_;
      if (rocking_gait_enabled_ || action_a_uses_bottom ||
          (inchworm_action_c_enabled_ && inchworm_active_) ||
          (inchworm_action_d_enabled_ && inchworm_active_))
        {
          pressure_finite = pressure_finite &&
              std::isfinite(bottom_pressure_kpa_);
          if (std::isfinite(bottom_pressure_kpa_))
            {
              maximum_measured_pressure_kpa = std::max(
                  maximum_measured_pressure_kpa, bottom_pressure_kpa_);
              pressure_over_limit = pressure_over_limit ||
                  bottom_pressure_kpa_ >= pressure_safety_limit_kpa_;
            }
        }
      if (pressure_over_limit)
        {
          if (pressure_over_limit_start_.isZero())
            pressure_over_limit_start_ = now;
        }
      else
        pressure_over_limit_start_ = ros::Time(0);
      const double pressure_over_limit_duration =
          pressure_over_limit_start_.isZero() ? 0.0 :
          (now - pressure_over_limit_start_).toSec();
      const bool pressure_over_limit_confirmed = pressure_over_limit &&
          pressure_over_limit_duration >= pressure_safety_confirm_sec_;
      const bool pressure_valid = pressure_finite &&
          !pressure_over_limit_confirmed;
      bool neurons_fresh = true;
      for (const auto& stamp : neuron_stamp_) neurons_fresh = neurons_fresh && !stamp.isZero() && (now - stamp).toSec() <= neuron_timeout_sec_;

      std_msgs::UInt8 state_msg;
      state_msg.data = state_;
      state_pub_.publish(state_msg);

      if (!body_imu_fresh || !pressure_valid || !neurons_fresh)
        {
          correction_entry_gate_.reset();
          action_c_recovery_gate_.reset();
          action_c_resume_gate_.reset();
          action_c_boundary_settle_gate_.reset();
          action_c_ppg_settle_gate_.reset();
          action_c_pressure_ready_gate_.reset();
          action_c_rear_release_gate_.reset();
          action_c_roll_trend_.reset();

          publishCorrectionThrust(zero_thrust);
          if (!inchworm_active_ && state_ != GRASP && state_ != RECOVER)
            {
              ROS_ERROR("Perching correction cancelled by stale body IMU, Neuron IMU, or pressure");
              enterState(RECOVER, now);
            }
          // Never let a recovery branch publish nonzero thrust after this
          // fault stop in the same timer tick.
          if (!inchworm_active_) return;
        }

      if (inchworm_active_)
        {
          const bool mocap_fresh = locomotionMocapFresh(now);
          const double angular_rate = std::hypot(body_angular_velocity_[0],
                                                 body_angular_velocity_[1]);
          // FRONT_REACH resolves target completion before applying the
          // angular-rate abort. Otherwise the sample that first reaches the
          // requested body tilt can be sent to RECOVER instead of advancing
          // to FRONT_ANCHOR.
          const bool angular_rate_abort =
              state_ != FRONT_REACH &&
              state_ != ACTION_B_FRONT_REACH &&
              state_ != ACTION_B_FRONT_REGRIP &&
              state_ != ACTION_B_REGRASP &&
              state_ != ACTION_C_PREPARE &&
              state_ != ACTION_C_HORIZONTAL_PULL &&
              state_ != ACTION_C_REGRIP &&
              state_ != ACTION_D_PREPARE &&
              state_ != ACTION_D_RAMP &&
              state_ != ACTION_D_REGRIP &&
              state_ != ROCK_FRONT_UNLOAD &&
              state_ != ROCK_REAR_INFLATE &&
              state_ != ROCK_FRONT_LAND &&
              state_ != ROCK_REAR_UNLOAD &&
              state_ != ROCK_REAR_RECOVER &&
              state_ != ROCK_REAR_LAND &&
              angular_rate >= correction_max_angular_rate_;
          if (!body_imu_fresh || !pressure_valid || !neurons_fresh ||
              !mocap_fresh || angular_rate_abort)
            {
              const double body_imu_age = imu_received_
                  ? (now - imu_stamp_).toSec() : -1.0;
              const double mocap_age = mocap_received_
                  ? (now - mocap_stamp_).toSec() : -1.0;
              const double hand_mocap_age = hand_mocap_received_
                  ? (now - hand_mocap_stamp_).toSec() : -1.0;
              double oldest_neuron_age = 0.0;
              for (const auto& stamp : neuron_stamp_)
                {
                  if (stamp.isZero())
                    {
                      oldest_neuron_age = -1.0;
                      break;
                    }
                  oldest_neuron_age = std::max(
                      oldest_neuron_age, (now - stamp).toSec());
                }
              ROS_ERROR(
                  "Inchworm cancelled in state %u: body_imu=%d age=%.3f/%.3f s, "
                  "pressure=%d, neurons=%d oldest_age=%.3f/%.3f s, "
                  "mocap=%d robot_age=%.3f hand_age=%.3f/%.3f s, angular_rate=%.3f/%.3f rad/s, "
                  "max_pressure=%.2f/%.2f kPa over_limit_for=%.3f/%.3f s",
                  state_, body_imu_fresh, body_imu_age, imu_timeout_sec_,
                  pressure_valid, neurons_fresh, oldest_neuron_age,
                  neuron_timeout_sec_, mocap_fresh, mocap_age,
                  hand_relative_mocap_enabled_ ? hand_mocap_age : 0.0,
                  inchworm_mocap_timeout_sec_, angular_rate,
                  correction_max_angular_rate_, maximum_measured_pressure_kpa,
                  pressure_safety_limit_kpa_, pressure_over_limit_duration,
                  pressure_safety_confirm_sec_);
              stopInchwormToGrasp(now,
                  "stale data, invalid pressure, mocap timeout, or angular-rate limit");
            }
          else
            {
              updateInchworm(now, dt, pressure_valid, neurons_fresh);
              return;
            }
        }

      // The bottom bladder deliberately changes pitch. Apply the same axis
      // mask to ordinary recovery and Action-C handoff: pitch must neither
      // start recovery nor keep it active after roll reaches the target.
      const bool roll_only_recovery = correctionRollOnly();
      const double attitude_error = roll_only_recovery
          ? std::abs(roll_error) : std::hypot(roll_error, pitch_error);
      const bool model_c_recovery = action_c_continuous_recovery_enabled_ &&
          inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !fixed_arm_test_enabled_ && roll_only_recovery;
      const double recovery_roll_rate = model_c_recovery
          ? hugmy::recoveryRollRate(roll_, pitch_, body_angular_velocity_)
          : body_angular_velocity_[0];
      const double angular_rate = roll_only_recovery
          ? std::abs(recovery_roll_rate)
          : std::hypot(body_angular_velocity_[0], body_angular_velocity_[1]);

      const double recovery_stop_angle = (action_c_resume_pending_ || model_c_recovery)
          ? action_c_posture_resume_rad_ : correction_stop_angle_;
      const bool continuous_c_recovery = action_c_continuous_recovery_enabled_ &&
          inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !fixed_arm_test_enabled_ && roll_only_recovery;
      if (action_c_resume_pending_ &&
          std::abs(wrapAngle(roll_ - target_roll_)) >=
              action_c_emergency_stop_roll_rad_)
        {
          stopInchwormToGrasp(now,
              "Action C emergency roll reached configured stop limit");
          return;
        }
      const bool recovery_pressure_fresh =
          !measured_pressure_stamp_.isZero() &&
          (now - measured_pressure_stamp_).toSec() >= 0.0 &&
          (now - measured_pressure_stamp_).toSec() <= 0.25 &&
          !measured_bottom_pressure_stamp_.isZero() &&
          (now - measured_bottom_pressure_stamp_).toSec() >= 0.0 &&
          (now - measured_bottom_pressure_stamp_).toSec() <= 0.25;
      switch (state_)
        {
        case GRASP:
          publishCorrectionThrust(zero_thrust);
          if (rocking_gait_enabled_ || inchworm_action_c_enabled_)
            publishBottomTarget(
                (action_c_resume_pending_ ||
                 recovery_candidate_reselect_pending_) ? 0.0 :
                (hybridLocomotionActive() && hybrid_hold_bottom_ ? hybrid_single_config_.bottom_kpa : rocking_bottom_low_kpa_));
          {
          // Keep the contact-conditioned regrasp pressure through the
          // recovery boundary. Returning to a hard-coded 30 kPa here would
          // recreate the contact moment which the handoff just removed.
          const PressureArray steady_grasp =
              (action_c_resume_pending_ || recovery_candidate_reselect_pending_ ||
               recovery_contact_grasp_active_)
              ? grasp_pressure_
              : ((hybridLocomotionActive() && hybrid_hold_pressure_)
                    ? hybrid_grasp_pressure_
                    : PressureArray{{base_pressure_kpa_,base_pressure_kpa_,
                                     base_pressure_kpa_,base_pressure_kpa_}});
          publishPressureTarget(steady_grasp, maximum_step);
          bool grasp_pressure_ready = pressure_valid;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            grasp_pressure_ready = grasp_pressure_ready &&
                std::abs(measured_pressure_[arm] - steady_grasp[arm]) <=
                pressure_tolerance_kpa_;
          if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
            grasp_pressure_ready = pressure_valid && actionCPressureReady(
                steady_grasp, now);
          if (!grasp_pressure_ready)
            ROS_INFO_THROTTLE(1.0,
                "GRASP waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                base_pressure_kpa_, pressure_tolerance_kpa_);
          // Completion and restart use different boundaries.  Recovery is
          // complete only inside +/-3 deg, but a 3.1-deg contact rebound must
          // not immediately start another pair.  After a completed/handoff
          // pass, require more than 5 deg *and* a persistent worsening trend;
          // the independent hard-roll trigger remains immediate.
          const double completed_recovery_restart_angle = std::max(
              5.0 * M_PI / 180.0,
              action_c_posture_resume_rad_ +
                  2.0 * roll_recovery_config_.stop_margin);
          const double active_correction_trigger =
              recovery_candidate_reselect_pending_
              ? action_c_posture_resume_rad_
              : (action_c_resume_pending_
              ? (continuous_recovery_waiting_for_settle_
                    ? completed_recovery_restart_angle
                    : (std::isfinite(action_c_requested_recovery_trigger_rad_)
                        ? std::min(action_c_posture_recovery_trigger_rad_,
                            action_c_requested_recovery_trigger_rad_)
                        : action_c_posture_recovery_trigger_rad_))
              : correction_trigger_angle_);
          // Recovery thrust starts immediately and does not wait for bottom
          // exhaust. Automatic locomotion resume is different: finish the
          // retained-position sample at the same deflated boundary used at
          // the start of a stride before a new inflation can begin.
          const bool recovery_bottom_ready =
              (!action_c_resume_pending_ &&
               !recovery_candidate_reselect_pending_) ||
              (recovery_pressure_fresh && std::isfinite(bottom_pressure_kpa_) &&
               std::abs(bottom_pressure_kpa_) <= action_c_bottom_deflation_tolerance_kpa_);
          const bool c_trend_mode = inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;
          if (c_trend_mode)
            action_c_roll_trend_.update(body_imu_fresh && pressure_valid && neurons_fresh &&
                (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready)),
                wrapAngle(roll_ - target_roll_), recovery_roll_rate, now.toSec());
          const bool c_hard_roll = c_trend_mode &&
              std::abs(wrapAngle(roll_ - target_roll_)) >= action_c_posture_hard_roll_rad_;
          // Pressure inflation changes the contact moment and may carry a
          // tilted body farther from level.  Apply the same absolute-angle
          // criterion for both roll signs instead of waiting for a signed
          // worsening trend on one side.  Outside this transient, a stable
          // 10--15-deg contact offset remains permitted as before.
          const bool c_preload_roll = continuous_c_recovery &&
              !grasp_pressure_ready &&
              attitude_error >= action_c_preload_recovery_trigger_rad_;
          const bool stable_c_roll = c_trend_mode && !c_hard_roll &&
              action_c_roll_trend_.ready && !action_c_roll_trend_.worsening;
          const bool c_recovery_requested = !c_trend_mode || c_hard_roll ||
              c_preload_roll || action_c_roll_trend_.worsening;
          // A closed-loop handoff may have proved that raising a released
          // pair toward nominal pressure worsens roll and retained the best
          // load-bearing pressure instead.  That quiet measured equilibrium
          // is the OUTPUT of recovery, not a fresh recovery request.  The old
          // code accepted it in RECOVER and then, one GRASP tick later,
          // started the identical correction again solely because its static
          // bias remained just above 10 deg.  Permit one automatic gait
          // restart while the retained equilibrium is stationary and inside
          // the independent locomotion hard boundary.  A worsening trend or
          // the hard boundary still requests correction immediately.
          const double retained_equilibrium_limit = std::min(
              action_c_posture_hard_roll_rad_,
              std::max(action_c_posture_recovery_trigger_rad_,
                  hybrid_config_.roll_limit - M_PI / 180.0));
          const bool retained_contact_equilibrium =
              action_c_resume_pending_ &&
              recovery_handoff_pressure_trimmed_ && stable_c_roll &&
              attitude_error <= retained_equilibrium_limit;
          const bool completed_recovery_restart_allowed =
              !continuous_recovery_waiting_for_settle_ ||
              recovery_candidate_reselect_pending_ || c_hard_roll ||
              (attitude_error > completed_recovery_restart_angle &&
               action_c_roll_trend_.worsening);
          // A geometry re-estimation changes the contact state by restoring
          // equal four-arm support.  Candidate evidence from the old state is
          // invalid only after the measured chamber pressures have reached
          // that new state.  Until then, hold zero thrust in GRASP instead of
          // immediately selecting again from unchanged geometry.
          const bool geometry_reestimate_ready =
              !recovery_geometry_reestimate_pending_ ||
              (grasp_pressure_ready && recovery_pressure_fresh);
          // A handoff already confirmed the recovery request. Do not demand
          // another pressure/settling/trend cycle before correcting it.
          const bool continuous_recovery_requested = continuous_c_recovery &&
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              recovery_pressure_fresh && !continuous_recovery_blocked_ &&
              geometry_reestimate_ready &&
              angular_rate < correction_max_angular_rate_ &&
              attitude_error > active_correction_trigger &&
              completed_recovery_restart_allowed &&
              (recovery_candidate_reselect_pending_ ||
               (action_c_resume_pending_ && !continuous_recovery_started_ &&
                !retained_contact_equilibrium) || c_hard_roll);
          const bool recovery_settled = action_c_boundary_settle_gate_.update(
              body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && recovery_bottom_ready &&
              angular_rate <= action_c_pre_roll_rate_tolerance_rad_s_,
              now.toSec(), action_c_posture_settle_sec_);
          if (action_c_resume_pending_ && !recovery_settled)
            ROS_INFO_THROTTLE(1.0,
                "Action C auto-resume waiting: bottom %.1f kPa (need <= %.1f kPa for retained endpoint), regrasp=%d, roll rate %.3f rad/s, blocked=%d; saved goal retained",
                bottom_pressure_kpa_,action_c_bottom_deflation_tolerance_kpa_,
                grasp_pressure_ready, angular_rate, continuous_recovery_blocked_);
          if (action_c_resume_pending_ && !continuous_c_recovery && !recovery_settled && !continuous_recovery_requested &&
              (now - state_start_).toSec() >= action_c_prepare_timeout_sec_)
            {
              stopInchwormToGrasp(now, "Action C recovery deflation/regrasp settling timeout");
              break;
            }
          // C correction succeeds inside 3 deg. At regrasp, also accept a stable
          // offset below the hard limit, so a completed recovery does not
          // immediately trigger again solely because roll remains above 10 deg.
          const bool resume_ready = action_c_resume_gate_.update(
              action_c_resume_pending_ && target_received_ &&
              (!continuous_recovery_blocked_ || continuous_recovery_waiting_for_settle_) &&
              body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && recovery_settled &&
              (continuous_recovery_waiting_for_settle_
                  ? attitude_error <= action_c_posture_resume_rad_
                  : (attitude_error <= active_correction_trigger || stable_c_roll)) &&
              recovery_pressure_fresh &&
              angular_rate <= (continuous_recovery_waiting_for_settle_
                  ? std::min(0.05, action_c_pre_roll_rate_tolerance_rad_s_)
                  : action_c_pre_roll_rate_tolerance_rad_s_),
              now.toSec(), continuous_recovery_waiting_for_settle_ ? 1.0 : correction_trigger_confirm_sec_);
          // A human-arm trial needs a motion-free PPG interval after posture
          // recovery.  Count only while the ordinary recovery completion
          // conditions remain true. GRASP already commands zero rotor thrust,
          // zero bottom pressure and the steady arm grasp above.
          const bool ppg_resume_ready = action_c_ppg_settle_gate_.update(
              resume_ready, now.toSec(), ppg_settle_sec_);
          if (action_c_resume_pending_ && resume_ready && !ppg_resume_ready)
            ROS_INFO_THROTTLE(1.0,
                "Action C PPG settle after posture recovery: holding bottom 0 kPa and thrust 0 for %.1f s",
                ppg_settle_sec_);
          if (ppg_resume_ready)
            {
              if (resumeActionCAfterRecovery(now)) break;
            }
          const bool correction_confirmed = correction_entry_gate_.update(
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              (!continuous_c_recovery || (!continuous_recovery_blocked_ &&
               recovery_pressure_fresh && geometry_reestimate_ready &&
               angular_rate < correction_max_angular_rate_)) &&
              (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready &&
               (!action_c_resume_pending_ || recovery_settled))) &&
              attitude_error > active_correction_trigger && c_recovery_requested,
              now.toSec(), correction_trigger_confirm_sec_);
          if ((correction_confirmed || continuous_recovery_requested) &&
              (continuous_recovery_requested || last_correction_end_.isZero() ||
               (now - last_correction_end_).toSec() >= cooldown_duration_))
            {
              if (continuous_c_recovery &&
                  recovery_geometry_reestimate_pending_)
                {
                  // The new load-bearing state is now physically present.
                  // A pressure/contact candidate rejected before this
                  // equalisation is not evidence against the same nominal
                  // command under the new geometry, so reopen the candidate
                  // set exactly once at this measured boundary.
                  recovery_geometry_reestimate_pending_ = false;
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  recovery_circle_event_locked_ = false;
                  recovery_preferred_pair_locked_ = false;
                  ROS_INFO("Balanced four-arm support reached; cleared old contact-state candidate rejections and re-estimating recovery geometry");
                }
              ROS_INFO("Posture recovery accepted: error %.2f deg > %.2f deg; confirmation %.2f s, confirmed handoff=%d preload=%d; stop at %.2f deg",
                       attitude_error * 180.0 / M_PI,
                       active_correction_trigger * 180.0 / M_PI,
                       correction_trigger_confirm_sec_,
                       continuous_recovery_requested,
                       c_preload_roll,
                       recovery_stop_angle * 180.0 / M_PI);
              Vector2 desired_moment = desiredThrustMoment();
              if (continuous_c_recovery)
                desired_moment = {{std::copysign(std::max(1e-6, std::abs(desired_moment[0])), roll_error), 0.0}};
              const double desired_moment_norm = std::hypot(
                  desired_moment[0], desired_moment[1]);
              if (desired_moment_norm > 1e-8)
                correction_axis_ = {{desired_moment[0] / desired_moment_norm,
                                     desired_moment[1] / desired_moment_norm}};
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  // Preserve the pressure of the balanced recovery handoff.
                  // Replacing its 15-kPa contact-conditioned support with a
                  // hard-coded 30 kPa here recreated the large roll moment
                  // before every retry. A new independent posture request
                  // still starts from the nominal base pressure.
                  grasp_pressure_[arm] =
                      (action_c_resume_pending_ ||
                       recovery_candidate_reselect_pending_)
                      ? steady_grasp[arm] : base_pressure_kpa_;
                  grasp_imu_angle_[arm] = measuredArmAngle(arm);
                  grasp_model_angle_[arm] = bendAngle(arm, grasp_pressure_[arm]);
                  // Moment-dependent target, bounded by the straight-arm
                  // geometry. Also straighten enough to keep rotor tilt below
                  // 90 deg (80 deg default margin), where nonnegative thrust
                  // retains useful upward and correction authority.
                  const double moment_based = std::max(
                      prepare_min_straighten_angle_,
                      prepare_angle_per_moment_ * desired_moment_norm);
                  const double feasibility_required = std::max(0.0,
                      grasp_model_angle_[arm] - prepare_max_rotor_tilt_);
                  prepare_straighten_angle_by_arm_[arm] =
                      std::min(prepare_max_straighten_angle_,
                          std::min(grasp_model_angle_[arm],
                              std::max(moment_based, feasibility_required)));

                  prepare_target_angle_[arm] = grasp_imu_angle_[arm] -
                      prepare_straighten_angle_by_arm_[arm];
                }
              grasp_baseline_valid_ = true;

              // First choose the correcting pair from the current grasp
              // geometry. During PREPARE, unload only that pair and give it a
              // small shaping thrust so it can become straighter.
              correction_thrust_ = allocateCorrectionThrust(
                  desired_moment, grasp_pressure_);
              prepare_pressure_ = grasp_pressure_;
              prepare_shape_thrust_.fill(0.0);
              prepare_arm_active_.fill(false);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (correction_thrust_[arm] > 0.01)
                  {
                    prepare_arm_active_[arm] = true;
                  prepare_pressure_[arm] = grasp_pressure_[arm];
                  prepare_shape_thrust_[arm] = prepare_shape_thrust_n_;
                  ROS_INFO("Arm %zu shape search: grasp IMU angle %.3f rad, target %.3f rad, pressure starts at %.1f kPa",
                           arm + 1, grasp_imu_angle_[arm], prepare_target_angle_[arm],
                           grasp_pressure_[arm]);
                  }
              best_prepare_angle_ = std::numeric_limits<double>::infinity();
              body_motion_detected_ = false;
              prepare_pair_pressure_ready_latched_ = false;
              correction_motion_candidate_start_ = ros::Time(0);
              correction_motion_candidate_error_ = attitude_error;
              correction_last_progress_ = now;
              correction_best_error_ = attitude_error;
              motion_hold_thrust_.fill(0.0);
              body_motion_last_seen_ = ros::Time(0);
              last_angle_progress_ = now;
              if (fixed_arm_test_enabled_)
                {
                  prepare_pressure_.fill(base_pressure_kpa_);
                  correction_thrust_.fill(0.0);
                  prepare_shape_thrust_.fill(0.0);
                  prepare_arm_active_.fill(false);
                  prepare_pressure_[fixed_test_arm_index_] = fixed_test_pressure_kpa_;
                  correction_thrust_[fixed_test_arm_index_] = fixed_test_thrust_n_;
                  prepare_arm_active_[fixed_test_arm_index_] = true;
                  ROS_WARN("Fixed arm test: arm %d target %.1f kPa, motor %d thrust %.2f N via %s",
                           fixed_test_arm_index_ + 1, fixed_test_pressure_kpa_,
                           fixed_test_arm_index_, fixed_test_thrust_n_,
                           use_pwm_test_for_correction_ ? "pwm_test" : "perching command");
                }
              enterState(PREPARE, now);
              if (continuous_c_recovery)
                {
                  const bool retry_opposite_pair =
                      continuous_recovery_retry_opposite_pending_;
                  const bool contact_candidate_retry =
                      recovery_candidate_reselect_pending_;
                  continuous_recovery_started_ = true;
                  // A previous timed pass may have been waiting only for a
                  // safe regrasp.  Once a new measured correction actually
                  // starts, return to the normal post-success resume gate;
                  // otherwise a stable 3--10 deg result could wait forever.
                  continuous_recovery_waiting_for_settle_ = false;
                  // Use the verified Spinal pair sign independently of model
                  // magnitude; measured geometry sets magnitude in PREPARE.
                  correction_axis_ = {{roll_error >= 0.0 ? 1.0 : -1.0, 0.0}};
                  // Start with the verified Spinal mapping.  If the previous
                  // pair produced a sustained measured wrong-sign response,
                  // the balanced handoff carries exactly one opposite-pair
                  // retry through GRASP.  Consume it here so it cannot leak
                  // into a later, independent posture event.
                  continuous_recovery_use_opposite_pair_ =
                      retry_opposite_pair && !contact_candidate_retry;
                  continuous_recovery_pair_switches_ =
                      retry_opposite_pair ? 1 : 0;
                  continuous_recovery_retry_opposite_pending_ = false;
                  selectContinuousRecoveryPair(roll_error);
                  recovery_candidate_reselect_pending_ = false;
                  if (!recovery_pair_selection_available_)
                    {
                      if (!recovery_geometry_reestimate_attempted_)
                        {
                          // No pair has a trustworthy instantaneous restoring
                          // lever.  Equalise all four supports once and let the
                          // circle/contact estimator observe that common state;
                          // never apply a speculative rotor command here.
                          recovery_geometry_reestimate_attempted_ = true;
                          recovery_geometry_reestimate_pending_ = true;
                          recovery_candidate_reselect_pending_ = true;
                          continuous_recovery_started_ = false;
                          continuous_recovery_waiting_for_settle_ = true;
                          prepare_arm_active_.fill(false);
                          prepare_shape_thrust_.fill(0.0);
                          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                            grasp_pressure_[arm] = std::min(
                                maximum_pressure_kpa_,
                                std::max(base_pressure_kpa_,
                                         grasp_pressure_[arm]));
                          prepare_pressure_ = grasp_pressure_;
                          publishCorrectionThrust(prepare_shape_thrust_);
                          publishPressureTarget(prepare_pressure_,
                              std::numeric_limits<double>::infinity());
                          enterState(GRASP, now);
                          ROS_WARN("No instantaneous restoring recovery pair; equalising four-arm support once before geometry re-estimation (zero thrust)");
                        }
                      else
                        {
                          continuous_recovery_blocked_ = true;
                          stopInchwormToGrasp(now,
                              "no restoring recovery pair after balanced-support geometry re-estimation");
                        }
                      break;
                    }
                  continuous_roll_recovery_.reset(attitude_error, now.toSec());
                  recovery_wrong_geometry_since_ = ros::Time(0);
                  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
                  recovery_candidate_retained_worsening_since_ = ros::Time(0);
                  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
                  recovery_wrong_geometry_reference_error_rad_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_reference_rate_rad_s_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_best_rate_rad_s_ =
                      -std::numeric_limits<double>::infinity();
                  recovery_wrong_geometry_response_observed_ = false;
                  recovery_pressure_brake_started_ = false;
                  // Every contact-conditioned retry initially sees motion
                  // inherited from the old pair (target crossing or measured
                  // sign reversal). Give the new pair enough authority to
                  // brake that motion before attributing its sign to the new
                  // actuator/contact state.
                  const bool braking_inherited_crossing_motion =
                      contact_candidate_retry;
                  resetRecoveryHandoffState();
                  recovery_predictive_.reset();
                  roll_recovery_config_.stop = recovery_stop_angle;
                  // A compliant high-friction contact can have a repeatable
                  // low-rate equilibrium outside the ideal +/-3-deg target.
                  // Keep the configured contact-minimum band here: forcing
                  // the same rotor pair beyond that measured minimum makes
                  // its effective moment reverse as the arm extends.  The
                  // hard roll guard remains independent, and the handoff
                  // below verifies the final grasped equilibrium.
                  roll_recovery_config_.contact_minimum_limit = std::max(
                      recovery_stop_angle,
                      roll_recovery_config_.contact_minimum_limit);
                  // A released/extended pair has a large, unambiguous
                  // free-space lever and should carry the body nearly to the
                  // real +/-3-deg target before pneumatic regrasp. Retain a
                  // half-margin for sensor/contact discretisation: requiring
                  // an exact 3.00-deg sample trapped the 70-mm fixture at
                  // 3.0--3.2 deg indefinitely. A flexed pair hands off up to
                  // two full margins early because its loaded contact lever
                  // can reverse as bend changes.
                  const bool small_radius_release =
                      recovery_selected_contact_pivot_enabled_ &&
                      recovery_selected_pair_release_ &&
                      recovery_circle_estimate_.valid &&
                      recovery_circle_estimate_.radius <=
                          recovery_circle_small_radius_m_;
                  roll_recovery_config_.capture_margin_factor =
                      small_radius_release
                      ? recovery_circle_small_radius_capture_margin_factor_
                      : (recovery_selected_contact_pivot_enabled_ &&
                         recovery_selected_pair_release_ ? 0.5 : 2.0);
                  roll_recovery_config_.horizon = correction_prediction_horizon_sec_;
                  // This ceiling belongs only to continuous Action-C roll
                  // recovery. Do not raise the locomotion/fixed-test ceiling
                  // through the shared correction_max_thrust_n parameter.
                  roll_recovery_config_.limit = std::min(
                      roll_recovery_config_.limit, prepare_max_thrust_n_);
                  if (braking_inherited_crossing_motion)
                    {
                      continuous_roll_recovery_.wrong_way_detection_thrust =
                          std::max(roll_recovery_config_.detect_thrust,
                              0.5 * roll_recovery_config_.limit);
                      continuous_roll_recovery_.wrong_way_detection_until =
                          now.toSec() + roll_recovery_config_.progress_window;
                    }
                  roll_recovery_config_.start_rate = correction_motion_start_rate_;
                  roll_recovery_config_.detect_thrust = correction_motion_detection_min_thrust_n_;
                  roll_recovery_config_.confirm_sec = correction_motion_confirm_sec_;
                  roll_recovery_config_.confirm_angle = correction_motion_min_error_reduction_;
                  roll_recovery_config_.progress_angle = correction_progress_epsilon_;
                  roll_recovery_config_.stall_sec = correction_stall_timeout_sec_;
                  roll_recovery_config_.abort_sec = action_c_recovery_no_progress_timeout_sec_;
                  roll_recovery_config_.max_rate = std::max(
                      correction_max_angular_rate_,
                      recovery_measurement_rate_limit_rad_s_);
                  roll_recovery_config_.max_target_rate = std::min(
                      roll_recovery_config_.max_target_rate, 0.5 * correction_max_angular_rate_);
                  prepare_shape_thrust_.fill(0.0);
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      prepare_pressure_[arm] = recovery_selected_pair_release_
                          ? recoverySelectedPairPressure(arm)
                          : std::min(maximum_pressure_kpa_,
                              std::max(base_pressure_kpa_,
                                       grasp_pressure_[arm]));
                    else
                      // A contact-conditioned pressure from the previous
                      // pair is not the next pair's anchor pressure. Restore
                      // the non-selected support to at least the nominal gait
                      // grasp while the selected pair is exhausted.
                      prepare_pressure_[arm] = std::min(maximum_pressure_kpa_,
                          std::max(base_pressure_kpa_, grasp_pressure_[arm]));
                  publishBottomTarget(0.0);
                  publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
                  ROS_INFO("Action C continuous roll recovery: exhaust bottom; selected pair target pressure %.1f kPa (%s); no zero-pressure wait; limit %.2f N per rotor",
                           recovery_selected_pair_pressure_kpa_,
                           recovery_selected_pair_release_
                               ? "partial/full release" : "nominal grasp",
                           roll_recovery_config_.limit);
                }
            }
          }
          break;

        case PREPARE:
          {
          if (continuous_c_recovery)
            {
              publishBottomTarget(0.0);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  if (!prepare_arm_active_[arm])
                    prepare_pressure_[arm] = std::min(maximum_pressure_kpa_,
                        std::max(base_pressure_kpa_, grasp_pressure_[arm]));
                  else if (recovery_selected_pair_release_ &&
                           (!recovery_predictive_config_.enabled || recovery_predictive_config_.shadow) &&
                           continuous_roll_recovery_.stalled &&
                           continuous_roll_recovery_.thrust >= 0.8 * roll_recovery_config_.limit)
                    prepare_pressure_[arm] = std::max(
                        recoverySelectedPairPressure(arm),
                        prepare_pressure_[arm] - recovery_pressure_down_rate_ * std::max(0.0, dt));
                }
              const double signed_error = roll_error * correction_axis_[0];
              const double omega = recovery_roll_rate * correction_axis_[0];
              const auto model = measuredRecoveryModel();
              auto& response = roll_recovery_response_[correction_axis_[0] > 0.0 ? 0 : 1];
              const bool recovery_inputs_valid = recovery_pressure_fresh &&
                  std::isfinite(bottom_pressure_kpa_) && bottom_pressure_kpa_ < pressure_safety_limit_kpa_ &&
                  std::isfinite(body_angular_velocity_[0]) &&
                  std::abs(body_angular_velocity_[0]) <
                      recovery_measurement_rate_limit_rad_s_;
              double predictive_target = std::numeric_limits<double>::quiet_NaN();
              if (recovery_predictive_config_.enabled && recovery_inputs_valid && model.valid &&
                  std::isfinite(omega) &&
                  std::abs(omega) < recovery_measurement_rate_limit_rad_s_ &&
                  signed_error > roll_recovery_config_.stop && !continuous_roll_recovery_.fault)
                {
                  std::array<size_t, 2> arms{};
                  size_t count = 0;
                  for (size_t arm=0; arm<ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm] && count<arms.size()) arms[count++]=arm;
                  hugmy::RecoveryPredictiveState observation;
                  observation.now=now.toSec(); observation.error=signed_error; observation.rate=omega;
                  observation.thrust=continuous_roll_recovery_.thrust; observation.bottom=bottom_pressure_kpa_;
                  for(size_t j=0;j<2;++j) {
                    observation.pressure[j]=measured_pressure_[arms[j]];
                    observation.pressure_command[j]=prepare_pressure_[arms[j]];
                    observation.bend[j]=measuredArmBendAngle(arms[j]);
                  }
                  auto config=recovery_predictive_config_;
                  config.pressure_min=recovery_selected_pair_release_
                      ? recoverySelectedPairPressure(arms[0])
                      : minimum_prepare_pressure_kpa_;
                  config.pressure_max=std::min(maximum_pressure_kpa_,
                      std::min(grasp_pressure_[arms[0]],grasp_pressure_[arms[1]]));
                  config.pressure_slew=pressure_rate_limit_kpa_s_;
                  const auto candidate=recovery_predictive_.update(observation,config,roll_recovery_config_,
                      [&](size_t, double pressure, double force) {
                        const auto range=bend_model_.ranges();
                        const double coordinate=force+bend_model_thrust_offset_n_;
                        if(pressure<range.first.first-5. || pressure>range.first.second ||
                           coordinate<range.second.first || coordinate>range.second.second)
                          return std::numeric_limits<double>::quiet_NaN();
                        // Permit small sub-atmospheric sensor offsets; no
                        // negative-pressure actuation or table extrapolation.
                        return bend_model_.angleRad(std::max(range.first.first,pressure),coordinate);
                      }, [&](size_t j, double angle) { return recoveryRotorLever(arms[j],angle); });
                  if(!config.shadow) {
                    if(!candidate.feasible || count!=2) {
                      // A single infeasible horizon is common while chamber
                      // pressure and bend pass between two contact modes.  It
                      // does not prove that the selected physical contact has
                      // the wrong moment sign.  Brake for this control cycle
                      // and re-plan from the next measurement; the measured
                      // wrong-way/no-progress logic below remains responsible
                      // for rejecting and re-selecting a contact candidate.
                      predictive_target = 0.0;
                      ROS_WARN_THROTTLE(1., "Action C predictive recovery has no feasible horizon; braking this cycle and replanning from measured contact state");
                    } else {
                      predictive_target=candidate.thrust;
                      for(size_t j=0;j<2;++j)
                        prepare_pressure_[arms[j]]+=hugmy::recoveryClamp(
                            candidate.pressure[j]-prepare_pressure_[arms[j]],
                            -config.pressure_slew*dt,config.pressure_slew*dt);
                    }
                  }
                  ROS_INFO_THROTTLE(.5,
                      "Action C predictive recovery: shadow=%d feasible=%d target %.2f -> %.2f N, pressure [%.1f %.1f] kPa, worst terminal roll %.2f deg, score %.3f, candidates %u rejected %u budget_exhausted=%d, confidence %.2f (heuristic), prediction errors rate %.3f rad/s pressure %.2f kPa bend %.2f deg, observations %u",
                      config.shadow,candidate.feasible,candidate.thrust,candidate.later_thrust,
                      candidate.pressure[0],candidate.pressure[1],candidate.worst_terminal_error*180./M_PI,
                      candidate.score,candidate.evaluated,candidate.rejected,candidate.budget_exhausted,recovery_predictive_.confidence,
                      recovery_predictive_.prediction_error,recovery_predictive_.pressure_prediction_error,
                      recovery_predictive_.bend_prediction_error*180./M_PI,recovery_predictive_.accepted_observations);
                }
              else if(recovery_predictive_config_.enabled && !recovery_predictive_config_.shadow &&
                      (!recovery_inputs_valid || !model.valid))
                continuous_roll_recovery_.fault=true;
              // Preserve the extended-arm recovery geometry throughout the
              // active correction.  Rotor speed/prediction braking can begin
              // far from level (31 deg in walk_c_3_after_build); it is not a
              // pneumatic regrasp event.  A non-shadow predictor may still
              // request pressure, but it cannot bend the selected arms above
              // the configured release pressure before a real terminal
              // handoff.
              if (recovery_selected_pair_release_ &&
                  !continuous_roll_recovery_.pressureHandoffReady() &&
                  !recovery_pressure_brake_started_)
                {
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        const double active_pressure_ceiling =
                            recoverySelectedPairPressure(arm);
                        prepare_pressure_[arm] = std::min(
                            prepare_pressure_[arm], active_pressure_ceiling);
                      }
                }
              publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
              bool recovery_response_faults_enabled = true;
              if (recovery_selected_pair_release_)
                {
                  // Venting the lower pair transfers support before the rotor
                  // moment has built.  Do not label that pneumatic transient
                  // as a wrong-sign rotor response.  The inner law also waits
                  // for its configured meaningful-thrust threshold after this
                  // measured-pressure gate opens.
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        const double release_target =
                            recoverySelectedPairPressure(arm);
                        recovery_response_faults_enabled =
                            recovery_response_faults_enabled &&
                            std::isfinite(measured_pressure_[arm]) &&
                            measured_pressure_[arm] <= release_target +
                                roll_recovery_config_.stable_pressure;
                      }
                }
              // The circle planner evaluates the geometry expected after a
              // released pair has extended.  On the positive-roll hardware
              // trial, its current measured lever was still negative while
              // that extension was taking place; applying the ordinary PI
              // force during this interval increased roll from 19 to 39 deg.
              // Permit only a small shaping force until the measured lever
              // itself has the restoring sign and useful magnitude.
              const bool measured_restoring_geometry = model.valid &&
                  std::isfinite(model.lever) &&
                  model.lever >= roll_recovery_config_.min_lever;
              const bool wrong_geometry_candidate =
                  recovery_selected_pair_release_ &&
                  !measured_restoring_geometry;
              if (wrong_geometry_candidate)
                {
                  // The kinematic lever is evaluated while the chamber and
                  // compliant contact are still moving toward the candidate
                  // state.  An outward angular velocity may therefore remain
                  // for a short time even when the new moment is already
                  // decelerating it.  Start the conclusive observation only
                  // after measured pressure has settled, and use improvement
                  // of the signed angular rate as empirical restoring
                  // evidence.  Angle growth alone before that point is not a
                  // wrong-sign result.
                  if (recovery_wrong_geometry_since_.isZero())
                    {
                      recovery_wrong_geometry_since_ = now;
                      recovery_wrong_geometry_reference_error_rad_ =
                          std::abs(signed_error);
                    }
                  // The endpoint selected by the circle model is not enough:
                  // a slow pneumatic exhaust traverses intermediate contact
                  // states.  If the *current* measured lever is wrong and the
                  // body keeps accelerating outward, reject the path before
                  // waiting several seconds for its target pressure.  Error
                  // growth and outward rate are both required so sensor noise
                  // or harmless contact settling cannot switch pairs.
                  const bool transition_error_worsened =
                      std::isfinite(
                          recovery_wrong_geometry_reference_error_rad_) &&
                      std::abs(signed_error) >=
                          recovery_wrong_geometry_reference_error_rad_ +
                          recovery_wrong_geometry_transition_worsening_rad_;
                  const bool transition_outward =
                      omega <=
                          -recovery_wrong_geometry_transition_outward_rate_rad_s_;
                  if (!recovery_response_faults_enabled &&
                      transition_error_worsened && transition_outward)
                    {
                      if (recovery_wrong_geometry_transition_bad_since_.isZero())
                        recovery_wrong_geometry_transition_bad_since_ = now;
                    }
                  else
                    recovery_wrong_geometry_transition_bad_since_ =
                        ros::Time(0);
                  const double transition_bad_age =
                      recovery_wrong_geometry_transition_bad_since_.isZero()
                      ? 0.0
                      : (now -
                          recovery_wrong_geometry_transition_bad_since_).toSec();
                  const bool transition_geometry_worsened =
                      transition_bad_age >=
                          recovery_wrong_geometry_transition_confirm_sec_;
                  if (transition_geometry_worsened)
                    {
                      continuous_roll_recovery_.fault = true;
                      continuous_roll_recovery_.wrong_way = true;
                      continuous_roll_recovery_.unresponsive = false;
                      ROS_ERROR("Recovery pair %zu+%zu has a non-restoring measured lever %.4f m during pressure transfer; roll worsened %.2f -> %.2f deg at %.2f deg/s for %.2f s. Cutting thrust and re-selecting without waiting for target pressure",
                                recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                                recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                                model.lever,
                                recovery_wrong_geometry_reference_error_rad_ *
                                    180.0 / M_PI,
                                std::abs(signed_error) * 180.0 / M_PI,
                                omega * 180.0 / M_PI,
                                transition_bad_age);
                    }
                  if (recovery_response_faults_enabled &&
                      recovery_wrong_geometry_pressure_settled_since_.isZero())
                    {
                      recovery_wrong_geometry_pressure_settled_since_ = now;
                      recovery_wrong_geometry_reference_rate_rad_s_ = omega;
                      recovery_wrong_geometry_best_rate_rad_s_ = omega;
                    }
                  if (!recovery_wrong_geometry_pressure_settled_since_.isZero())
                    recovery_wrong_geometry_best_rate_rad_s_ = std::max(
                        recovery_wrong_geometry_best_rate_rad_s_, omega);
                  const double pressure_settled_age =
                      recovery_wrong_geometry_pressure_settled_since_.isZero()
                      ? 0.0
                      : (now - recovery_wrong_geometry_pressure_settled_since_).toSec();
                  const bool rate_response_confirmed =
                      pressure_settled_age >=
                          recovery_wrong_geometry_response_confirm_sec_ &&
                      std::isfinite(recovery_wrong_geometry_reference_rate_rad_s_) &&
                      recovery_wrong_geometry_best_rate_rad_s_ >=
                          recovery_wrong_geometry_reference_rate_rad_s_ +
                          recovery_wrong_geometry_rate_improvement_rad_s_;
                  const bool response_was_observed =
                      recovery_wrong_geometry_response_observed_;
                  recovery_wrong_geometry_response_observed_ =
                      response_was_observed || rate_response_confirmed;
                  if (!response_was_observed &&
                      recovery_wrong_geometry_response_observed_)
                    {
                      // The old progress window contains only the pneumatic
                      // release transient.  Begin rotor wrong-way detection
                      // from the empirically confirmed deceleration instead
                      // of immediately rejecting the candidate for motion
                      // inherited before pressure settled.
                      continuous_roll_recovery_.resetResponseWindow(
                          signed_error, now.toSec());
                      ROS_INFO("Recovery pair %zu+%zu retained a model-wrong lever %.4f m, but pressure-settled signed rate improved %.2f -> %.2f deg/s; enabling measured-response authority",
                               recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                               recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                               model.lever,
                               recovery_wrong_geometry_reference_rate_rad_s_ *
                                   180.0 / M_PI,
                               recovery_wrong_geometry_best_rate_rad_s_ *
                                   180.0 / M_PI);
                    }
                  const bool geometry_worsened =
                      pressure_settled_age >=
                          recovery_wrong_geometry_response_confirm_sec_ &&
                      !recovery_wrong_geometry_response_observed_ &&
                      std::isfinite(recovery_wrong_geometry_reference_error_rad_) &&
                      std::abs(signed_error) >=
                          recovery_wrong_geometry_reference_error_rad_ +
                          recovery_wrong_geometry_worsening_rad_;
                  const bool pressure_settled_timeout =
                      !recovery_wrong_geometry_response_observed_ &&
                      pressure_settled_age >=
                          recovery_wrong_geometry_timeout_sec_;
                  if (pressure_settled_timeout ||
                      geometry_worsened)
                    {
                      // This check is independent of the ordinary 1.5-N
                      // response gate: the 0.6-N shaping cap is deliberately
                      // too small for rotor-response identification.  Reject
                      // the contact geometry from its measured signed lever,
                      // run thrust to zero, then use the existing balanced
                      // regrasp/re-selection path.
                      continuous_roll_recovery_.fault = true;
                      continuous_roll_recovery_.wrong_way = geometry_worsened;
                      continuous_roll_recovery_.unresponsive =
                          !geometry_worsened;
                      ROS_ERROR("Recovery pair %zu+%zu retained a non-restoring measured lever %.4f m for %.2f s after pressure settled (roll error %.2f -> %.2f deg, signed rate %.2f -> best %.2f deg/s); rejecting this contact state before the global timeout",
                                recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                                recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                                model.lever, pressure_settled_age,
                                recovery_wrong_geometry_reference_error_rad_ *
                                    180.0 / M_PI,
                                std::abs(signed_error) * 180.0 / M_PI,
                                recovery_wrong_geometry_reference_rate_rad_s_ *
                                    180.0 / M_PI,
                                recovery_wrong_geometry_best_rate_rad_s_ *
                                    180.0 / M_PI);
                    }
                }
              else
                {
                  recovery_wrong_geometry_since_ = ros::Time(0);
                  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
                  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
                  recovery_wrong_geometry_reference_error_rad_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_reference_rate_rad_s_ =
                      std::numeric_limits<double>::quiet_NaN();
                  recovery_wrong_geometry_best_rate_rad_s_ =
                      -std::numeric_limits<double>::infinity();
                  recovery_wrong_geometry_response_observed_ = false;
                }
              const double geometry_authority_limit =
                  recovery_selected_pair_release_ &&
                  !measured_restoring_geometry &&
                  !recovery_wrong_geometry_response_observed_
                  ? (recovery_selected_path_requires_zero_thrust_
                        ? 0.0 : recovery_wrong_sign_shaping_thrust_n_)
                  : std::numeric_limits<double>::infinity();
              double thrust = continuous_roll_recovery_.update(
                  recovery_inputs_valid ? signed_error : std::numeric_limits<double>::quiet_NaN(),
                  omega, now.toSec(), dt, model, roll_recovery_config_, response,
                  predictive_target,
                  recovery_response_faults_enabled &&
                      (!wrong_geometry_candidate ||
                       recovery_wrong_geometry_response_observed_),
                  geometry_authority_limit);
              // The inner response window is reset while bend/contact
              // geometry changes. That is necessary for model learning, but
              // it can hide a slower failure mode: a candidate first makes
              // real progress, then a low-authority retry steadily gives all
              // of it back. Detect retained error loss directly from body
              // roll after pressure has settled. Do not reject a quiet
              // static-friction plateau, and do not blame a candidate while
              // predictive braking is intentionally reducing force.
              const bool retained_candidate_worsening =
                  recovery_response_faults_enabled &&
                  continuous_roll_recovery_.progress_observed &&
                  !continuous_roll_recovery_.terminal_brake_latched &&
                  continuous_roll_recovery_.recent_thrust_change >= -0.1 &&
                  thrust >= roll_recovery_config_.detect_thrust &&
                  std::abs(signed_error) >=
                      continuous_roll_recovery_.best_error +
                          recovery_wrong_geometry_transition_worsening_rad_;
              if (retained_candidate_worsening)
                {
                  if (recovery_candidate_retained_worsening_since_.isZero())
                    recovery_candidate_retained_worsening_since_ = now;
                  if ((now -
                       recovery_candidate_retained_worsening_since_).toSec() >=
                      recovery_wrong_geometry_transition_confirm_sec_)
                    {
                      continuous_roll_recovery_.wrong_way = true;
                      continuous_roll_recovery_.timed_out = true;
                      ROS_ERROR_THROTTLE(0.5,
                          "Recovery candidate lost retained progress: best %.2f deg, current %.2f deg under %.2f N for %.2f s; braking and re-selecting contact geometry",
                          continuous_roll_recovery_.best_error *
                              180.0 / M_PI,
                          std::abs(signed_error) * 180.0 / M_PI,
                          thrust,
                          (now -
                           recovery_candidate_retained_worsening_since_).toSec());
                    }
                }
              else
                recovery_candidate_retained_worsening_since_ = ros::Time(0);
              // At the captured attitude the selected pair is no longer used
              // as a long moment arm.  Build only the minimum load-bearing
              // pressure while rotor thrust is already near zero, so contact
              // Pneumatic damping must start before the body enters the final
              // angle band.  In both MuJoCo and hardware the pressure path is
              // much slower than rotor rundown: waiting for +/-3 deg let a
              // correctly moving body coast through zero and start another
              // recovery on the opposite pair.  Once a full-window decrease
              // in roll error has been measured and the rotor controller is
              // braking, request the minimum load-bearing pressure.  This is
              // only the first support stage; full grasp remains owned by
              // RECOVER.  The independent pressure controller/pneumatic
              // dynamics still limit the actual pressure slew.
              const double pressure_brake_horizon = std::max(
                  correction_prediction_horizon_sec_,
                  recovery_predictive_config_.pressure_tau);
              const double pressure_brake_predicted_error =
                  std::abs(signed_error) -
                  std::max(0.0, omega) * pressure_brake_horizon;
              const bool pneumatic_capture_imminent =
                  omega > roll_recovery_config_.settle_rate &&
                  pressure_brake_predicted_error <=
                      roll_recovery_config_.stop +
                      2.0 * roll_recovery_config_.stop_margin;
              const bool predictive_pressure_brake =
                  (continuous_roll_recovery_.speed_braking ||
                   continuous_roll_recovery_.predicted_braking) &&
                  (continuous_roll_recovery_.progress_observed ||
                   pneumatic_capture_imminent);
              const double pressure_brake_thrust_gate = std::max(
                  0.35, 0.25 * roll_recovery_config_.limit);
              if ((continuous_roll_recovery_.capture_active ||
                   predictive_pressure_brake) &&
                  thrust <= pressure_brake_thrust_gate)
                recovery_pressure_brake_started_ = true;
              if (recovery_pressure_brake_started_)
                {
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      prepare_pressure_[arm] = std::max(
                          prepare_pressure_[arm],
                          std::min(grasp_pressure_[arm],
                                   recoverySupportPressure()));
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                }
              // Keep the extended-arm geometry through the completed angular
              // correction. RECOVER will close the pneumatic loop gradually;
              // a 0 -> 30 kPa step here re-bent the pair and recreated the
              // original roll error after an otherwise successful recovery.
              // The pair sign is fixed by the verified Spinal/MuJoCo mapping
              // for the whole recovery.  A temporary lack of progress is a
              // contact/load observation; it must not swap to the opposite
              // pair.  The rate PI may adapt magnitude, and the bounded
              // no-progress timeout below performs a smooth regrasp handoff.
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                prepare_shape_thrust_[arm] = prepare_arm_active_[arm] ? thrust : 0.0;
              publishCorrectionThrust(prepare_shape_thrust_);
              ROS_INFO_THROTTLE(0.25,
                  "Action C model-rate recovery: roll error %.2f deg, rate %.3f/%.3f rad/s, pair thrust %.2f/%.2f N (command/target), FF %.2f N, I %.2f N, breakaway %.2f N, moment %.3f Nm, lever %.3f m, model=%d learned=%d gain %.3f samples=%u, moving=%d stalled=%d wrong_way=%d response_gate=%d terminal_brake=%d reapproach=%d cap=%.2f N geometry_cap=%.2f N speed_brake=%d predicted_brake=%d horizon=%.3f s, gravity_ff=0, bottom %.1f kPa (exhaust, no wait), recent_progress %.2f deg / %.2f s, progress_age %.2f s",
                  signed_error * 180.0 / M_PI, omega, continuous_roll_recovery_.target_rate, thrust,
                  continuous_roll_recovery_.target_thrust,
                  continuous_roll_recovery_.feedforward, continuous_roll_recovery_.boost,
                  continuous_roll_recovery_.hold, model.moment, model.lever,
                  continuous_roll_recovery_.model_valid, continuous_roll_recovery_.learned_response,
                  response.gain, response.samples,
                  continuous_roll_recovery_.moving, continuous_roll_recovery_.stalled,
                  continuous_roll_recovery_.wrong_way,
                  recovery_response_faults_enabled,
                  continuous_roll_recovery_.terminal_brake_latched,
                  continuous_roll_recovery_.low_authority_reapproach,
                  continuous_roll_recovery_.reapproach_authority,
                  std::isfinite(geometry_authority_limit)
                      ? geometry_authority_limit : -1.0,
                  continuous_roll_recovery_.speed_braking, continuous_roll_recovery_.predicted_braking,
                  continuous_roll_recovery_.braking_horizon,
                  bottom_pressure_kpa_,
                  continuous_roll_recovery_.recent_progress * 180.0 / M_PI,
                  roll_recovery_config_.progress_window,
                  now.toSec() - continuous_roll_recovery_.last_progress);
              if (!continuous_roll_recovery_.model_valid &&
                  (!recovery_predictive_config_.enabled || recovery_predictive_config_.shadow))
                ROS_WARN_THROTTLE(1.0, "Recovery geometry unavailable/wrong-sign lever: bounded rate-PI fallback, no absolute-value inversion");
              if (continuous_roll_recovery_.fault || continuous_roll_recovery_.finished ||
                  (now - state_start_).toSec() >= prepare_timeout_)
                {
                  // A wrong-way flag is latched until the old pair's thrust
                  // has slewed to zero, so this is the first safe instant to
                  // change actuators. Transfer pneumatic support and retry
                  // with the other pair directly; a several-second full
                  // regrasp before the switch let a 17-deg body fall past 30
                  // deg in MuJoCo. Permit one measured opposite-pair retry per
                  // posture event. Switching back to the original pair after
                  // the retry had already improved roll recreated the first
                  // wrong-way response and caused a fall; the measured minimum
                  // is instead preserved by the ordinary handoff/regrasp.
                  // The effective contact pivot can move as the selected
                  // arms extend.  A pair may initially reduce the error and
                  // later reverse its roll moment.  A measured wrong-way
                  // response must therefore override an earlier progress
                  // observation.  Lack of response alone remains guarded by
                  // the no-progress condition, since static friction is not
                  // evidence that the pair sign is wrong.
                  const bool measured_pair_sign_reversal =
                      continuous_roll_recovery_.wrong_way;
                  const bool never_effective_pair =
                      continuous_roll_recovery_.unresponsive &&
                      !continuous_roll_recovery_.progress_observed;
                  // A pair that first produced real improvement and only
                  // later reversed has reached a contact-lever minimum.  Do
                  // not throw that measured progress away by opening the
                  // opposite support pair.  Regrasp the same pair under the
                  // closed-loop handoff, then make a fresh decision from the
                  // new contact state.  The alternate pair is reserved for a
                  // pair which was never effective at all.
                  const bool pair_never_corrected = never_effective_pair ||
                      (measured_pair_sign_reversal &&
                       !continuous_roll_recovery_.progress_observed);
                  // Crossing the target is evidence that this candidate did
                  // produce the requested moment; it is not a bad candidate.
                  // The rotor pair is one-directional, however, so first
                  // restore a full four-arm grasp and then select again for
                  // the newly measured error sign. This path is required in
                  // recovery-only validation as well as during a paused gait.
                  if (continuous_roll_recovery_.fault &&
                      continuous_roll_recovery_.target_crossed)
                    {
                      recovery_rejected_candidate_mask_ = 0;
                      recovery_rejected_direction_sign_ = 0;
                      recovery_candidate_reselect_pending_ = false;
                      continuous_recovery_blocked_ = false;
                      continuous_recovery_waiting_for_settle_ = true;
                      continuous_recovery_started_ = true;
                      continuous_roll_recovery_.finished = true;
                      resetRecoveryHandoffState();
                      // Crossing the requested attitude is not itself a
                      // failed contact state.  Freeze the actual chamber
                      // pressures and first try to catch the body there.  If
                      // it subsequently leaves the +/-stop band, RECOVER
                      // performs an immediate opposite-sign re-selection;
                      // if it remains inside, the same path completes the
                      // monitored pneumatic regrasp without starting an
                      // unnecessary second rotor pair.
                      recovery_handoff_target_pressure_ =
                          recoveryMeasuredContactPressure();
                      prepare_pressure_ = recovery_handoff_target_pressure_;
                      grasp_pressure_ = recovery_handoff_target_pressure_;
                      if (hybridLocomotionActive() && action_c_resume_pending_)
                        {
                          hybrid_grasp_pressure_ = grasp_pressure_;
                          hybrid_hold_pressure_ = true;
                        }
                      prepare_shape_thrust_.fill(0.0);
                      publishCorrectionThrust(prepare_shape_thrust_);
                      publishPressureTarget(prepare_pressure_,
                          std::numeric_limits<double>::infinity());
                      action_c_resume_gate_.reset();
                      action_c_boundary_settle_gate_.reset();
                      ROS_WARN("Continuous recovery crossed the target with residual %.2f deg; froze measured contact pressure [%.1f %.1f %.1f %.1f] kPa for monitored regrasp (opposite-sign selection only if the residual leaves the stop band)",
                               roll_error * 180.0 / M_PI,
                               grasp_pressure_[0], grasp_pressure_[1],
                               grasp_pressure_[2], grasp_pressure_[3]);
                      enterState(RECOVER, now);
                      break;
                    }
                  // The circular-contact model proposes a *contact state*,
                  // not merely a rotor pair: pair 0+1 / 2+3 can each be used
                  // either while flexed against the object or after release.
                  // A measured reversal (including one after useful initial
                  // progress) disproves only that exact state.  Likewise, a
                  // candidate which remains motionless after reaching the
                  // high-thrust retry gate is locally ineffective.  Remove
                  // that candidate, equalise pneumatic support, then infer
                  // the circle again and choose from the unused candidates.
                  const bool reject_contact_candidate =
                      recovery_selected_contact_pivot_enabled_ &&
                      continuous_roll_recovery_.fault &&
                      (measured_pair_sign_reversal ||
                       continuous_roll_recovery_.unresponsive);
                  if (reject_contact_candidate)
                    {
                      const uint8_t candidate_bit = static_cast<uint8_t>(
                          1u << (4u * recovery_selected_pair_index_ +
                                 recovery_selected_pressure_candidate_index_));
                      recovery_rejected_candidate_mask_ |= candidate_bit;
                      recovery_rejected_direction_sign_ =
                          roll_error >= 0.0 ? 1 : -1;
                      recovery_candidate_reselect_pending_ = true;
                      continuous_recovery_blocked_ = false;
                      continuous_recovery_waiting_for_settle_ = true;
                      continuous_recovery_started_ = false;
                      continuous_roll_recovery_.finished = true;
                      resetRecoveryHandoffState();
                      // The controller reports `fault` only after the old
                      // rotor command has reached zero.  At that safe instant
                      // freeze the measured chamber state and select again;
                      // waiting another 1--2 s for a prescribed support
                      // pressure allowed the disproved contact geometry to
                      // keep rolling the body.  The non-selected nominal pair
                      // remains the mechanical anchor throughout the switch.
                      recovery_handoff_target_pressure_ =
                          recoveryMeasuredContactPressure();
                      prepare_pressure_ = recovery_handoff_target_pressure_;
                      grasp_pressure_ = recovery_handoff_target_pressure_;
                      if (hybridLocomotionActive() && action_c_resume_pending_)
                        {
                          hybrid_grasp_pressure_ = grasp_pressure_;
                          hybrid_hold_pressure_ = true;
                        }
                      prepare_shape_thrust_.fill(0.0);
                      publishCorrectionThrust(prepare_shape_thrust_);
                      publishPressureTarget(prepare_pressure_,
                          std::numeric_limits<double>::infinity());
                      action_c_resume_gate_.reset();
                      action_c_boundary_settle_gate_.reset();
                      ROS_WARN("Circular recovery rejected candidate pair %zu+%zu at %.1f kPa after measured %s response; mask=0x%02x. Froze measured contact pressure [%.1f %.1f %.1f %.1f] kPa for immediate geometry re-selection",
                          recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                          recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                          recovery_selected_pair_pressure_kpa_,
                          measured_pair_sign_reversal
                              ? "reversed" : "no-progress",
                          recovery_rejected_candidate_mask_,
                          grasp_pressure_[0], grasp_pressure_[1],
                          grasp_pressure_[2], grasp_pressure_[3]);
                      enterState(GRASP, now);
                      break;
                    }
                  if (continuous_roll_recovery_.fault &&
                      pair_never_corrected &&
                      action_c_resume_pending_ &&
                      continuous_recovery_pair_switches_ < 1)
                    {
                      const bool retry_after_wrong_way =
                          continuous_roll_recovery_.wrong_way;
                      // Only a measured wrong-sign/no-response result selects
                      // the alternate pair.  Crossing the target after real
                      // progress is a braking/regrasp event, not permission
                      // to release the opposite support and fire a second
                      // correction pulse.
                      continuous_recovery_use_opposite_pair_ =
                          !continuous_recovery_use_opposite_pair_;
                      ++continuous_recovery_pair_switches_;
                      correction_axis_ = {{roll_error >= 0.0 ? 1.0 : -1.0,
                                           0.0}};
                      selectContinuousRecoveryPair(roll_error);
                      continuous_roll_recovery_.reset(
                          attitude_error, now.toSec());
                      resetRecoveryHandoffState();
                      recovery_predictive_.reset();
                      prepare_shape_thrust_.fill(0.0);
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        prepare_pressure_[arm] = prepare_arm_active_[arm]
                            ? (recovery_selected_pair_release_
                                ? recoverySelectedPairPressure(arm)
                                : std::min(maximum_pressure_kpa_,
                                    std::max(base_pressure_kpa_,
                                        grasp_pressure_[arm])))
                            : std::min(maximum_pressure_kpa_,
                                std::max(base_pressure_kpa_,
                                    grasp_pressure_[arm]));
                      publishCorrectionThrust(prepare_shape_thrust_);
                      publishPressureTarget(prepare_pressure_,
                          std::numeric_limits<double>::infinity());
                      enterState(PREPARE, now);
                      ROS_WARN("Measured %s roll response: thrust is zero; transferring support and retrying rotor pair (%u/1)",
                          (retry_after_wrong_way ?
                              "wrong-way" : "no-progress"),
                          continuous_recovery_pair_switches_);
                      break;
                    }
                  ROS_WARN("Action C continuous recovery ended: finished=%d fault=%d elapsed=%.2f s; restoring grasp",
                           continuous_roll_recovery_.finished, continuous_roll_recovery_.fault,
                           (now - state_start_).toSec());
                  if (!continuous_roll_recovery_.finished)
                    {
                      const bool progress_timeout = continuous_roll_recovery_.timed_out ||
                          (!continuous_roll_recovery_.fault && (now - state_start_).toSec() >= prepare_timeout_);
                      if ((progress_timeout || continuous_roll_recovery_.fault) &&
                          action_c_resume_pending_)
                        {
                          // The selected pair can reverse its effective
                          // moment after extending. Do not bypass the
                          // closed-loop pneumatic handoff with a 0 -> 30 kPa
                          // step on this controlled wrong-way/timeout exit.
                          // Mark the rotor pass complete only for handoff,
                          // restore minimum contact under feedback, then let
                          // GRASP retry from the measured residual roll.
                          continuous_recovery_blocked_ = false;
                          continuous_recovery_waiting_for_settle_ = true;
                          continuous_recovery_started_ = true;
                          const bool measured_wrong_way =
                              continuous_roll_recovery_.wrong_way;
                          const bool pair_proven_wrong_without_progress =
                              measured_wrong_way &&
                              !continuous_roll_recovery_.progress_observed;
                          continuous_roll_recovery_.finished = true;
                          resetRecoveryHandoffState();
                          // Preserve the measured sign result across RECOVER;
                          // otherwise the handoff starts another pulse with
                          // the pair which has just been proven wrong.
                          // A pair which first produced substantial measured
                          // correction and reversed only after the arm became
                          // over-extended is not globally the wrong pair.  It
                          // must retain bounded roll authority while pressure
                          // rebuilds its moment geometry.  Disable it during
                          // handoff only when it was wrong before producing
                          // any useful progress.
                          recovery_handoff_thrust_wrong_way_ =
                              pair_proven_wrong_without_progress;
                          recovery_handoff_pair_was_wrong_ =
                              pair_proven_wrong_without_progress;
                          if (measured_wrong_way)
                            {
                              recovery_handoff_pressure_trimmed_ = true;
                              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                                // Balance the contact before asking either
                                // rotor pair for another measured response.
                                // The old [30,30,15,15]-kPa handoff was a
                                // persistent roll command and could never be
                                // corrected by a bounded 1.5-N retained
                                // thrust.  The inactive pair decreases while
                                // the active pair fills, so total pneumatic
                                // support is retained through this transfer.
                                recovery_handoff_target_pressure_[arm] =
                                    std::min(recovery_handoff_min_pressure_kpa_,
                                        std::min(maximum_pressure_kpa_,
                                            std::max(base_pressure_kpa_,
                                                grasp_pressure_[arm])));
                            }
                          action_c_resume_gate_.reset();
                          action_c_boundary_settle_gate_.reset();
                          enterState(RECOVER, now);
                          ROS_WARN("Action C recovery pass ended without convergence: entering the same closed-loop minimum-grasp handoff before an automatic measured-roll retry; goal and trial retained.");
                          break;
                        }
                      // No repeated timed search pulses after a fault/stall.
                      // Re-arm only on a new operator command or enable cycle.
                      continuous_recovery_blocked_ = true;
                      stopInchwormToGrasp(now, "continuous recovery fault/timeout; new operator command required");
                      break;
                    }
                  resetRecoveryHandoffState();
                  enterState(RECOVER, now);
                }
              break;
            }
          publishCorrectionThrust(fixed_arm_test_enabled_ ? zero_thrust
                                                          : prepare_shape_thrust_);
          publishPressureTarget(prepare_pressure_, maximum_step);
          const double correction_omega =
              body_angular_velocity_[0] * correction_axis_[0] +
              body_angular_velocity_[1] * correction_axis_[1];
          // Brake against the posture predicted from the Spinal gyro instead
          // of waiting for the measured angle to arrive at the target. This
          // removes rotor/rig inertia before it can carry roll through zero.
          const double predicted_attitude_error = std::max(0.0,
              attitude_error - std::max(0.0, correction_omega) *
                                   correction_prediction_horizon_sec_);
          const double angle_scale = std::max(0.0, std::min(1.0,
              (predicted_attitude_error - recovery_stop_angle) /
              std::max(1e-6, correction_slowdown_angle_ -
                                recovery_stop_angle)));
          // Once body motion starts, keep the captured breakaway thrust
          // constant until the attitude enters the slowdown band.  The old
          // rate_scale reduced thrust as soon as angular velocity rose, so
          // static friction stopped the body before the attitude target and
          // the controller repeatedly searched for breakaway again.
          const double feedback_scale = angle_scale;
          // A correction pair is one actuator for posture purposes. Do not
          // start one rotor merely because its pneumatic chamber exhausted
          // faster than its partner (the exact failure observed in c7).
          bool active_pair_pressure_ready_now = true;
          size_t selected_arm_count = 0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (prepare_arm_active_[arm])
              {
                ++selected_arm_count;
                active_pair_pressure_ready_now = active_pair_pressure_ready_now &&
                    prepare_pressure_[arm] <=
                        minimum_prepare_pressure_kpa_ + 1.0e-6 &&
                    measured_pressure_[arm] <=
                        minimum_prepare_pressure_kpa_ + pressure_tolerance_kpa_;
              }
          if (selected_arm_count > 0 && active_pair_pressure_ready_now)
            prepare_pair_pressure_ready_latched_ = true;
          // Once both chambers have reached the floor, keep the thrust phase
          // enabled while the controller continues commanding that floor.
          // c10 repeatedly crossed the 2 kPa measurement boundary and the old
          // instantaneous gate switched both rotors off and on each time.
          const bool active_pair_pressure_ready =
              prepare_pair_pressure_ready_latched_;
          bool geometry_ready = fixed_arm_test_enabled_;
          bool target_angle_reached = !fixed_arm_test_enabled_;
          double active_angle_sum = 0.0;
          size_t active_count = 0;
          PressureArray relative_angle{{0.0, 0.0, 0.0, 0.0}};
          PressureArray achieved_bend{{0.0, 0.0, 0.0, 0.0}};
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              // PREPARE may alter only the selected correcting pair. Reassert
              // the captured GRASP command on the other two arms every cycle
              // so pressure search and shaping thrust cannot leak into them.
              if (!prepare_arm_active_[arm])
                {
                  prepare_pressure_[arm] = grasp_pressure_[arm];
                  prepare_shape_thrust_[arm] = 0.0;
                }
              const double imu_angle = measuredArmAngle(arm);
              const double relative_change = wrapAngle(imu_angle - grasp_imu_angle_[arm]);
              relative_angle[arm] = relative_change;
              achieved_bend[arm] = std::max(0.0,
                  std::min(2.966, grasp_model_angle_[arm] + relative_change));
              if (prepare_arm_active_[arm])
                {
                  target_angle_reached = target_angle_reached &&
                      imu_angle <= prepare_target_angle_[arm] + prepare_angle_tolerance_;
                  const double target_error =
                      imu_angle - prepare_target_angle_[arm];
                  if (body_motion_detected_)
                    {
                      // Neuron arm angle is useful while preparing the low-
                      // pressure contact, but after the Spinal IMU detects
                      // whole-body motion it must not interrupt posture
                      // correction.  Hold the breakaway thrust outside the
                      // slowdown band and taper only as the body attitude
                      // approaches the target.
                      const double running_floor = std::min(
                          correction_hold_min_thrust_n_,
                          motion_hold_thrust_[arm]);
                      const double feedback_target = running_floor +
                          (motion_hold_thrust_[arm] - running_floor) *
                          feedback_scale;
                      prepare_shape_thrust_[arm] = std::max(feedback_target,
                          prepare_shape_thrust_[arm] -
                              correction_thrust_down_rate_ * dt);
                    }
                  else if (target_error > prepare_angle_tolerance_)
                    {
                      if (prepare_pressure_[arm] > minimum_prepare_pressure_kpa_ + 1e-6)
                        {
                          prepare_pressure_[arm] = std::max(minimum_prepare_pressure_kpa_,
                              prepare_pressure_[arm] - prepare_pressure_ramp_rate_ * dt);
                          prepare_shape_thrust_[arm] = std::max(0.0,
                              prepare_shape_thrust_[arm] - prepare_thrust_ramp_rate_ * dt);
                        }
                      else if (active_pair_pressure_ready &&
                               correction_motion_candidate_start_.isZero())
                        prepare_shape_thrust_[arm] = std::min(
                            std::min(prepare_max_thrust_n_,
                                     correction_max_thrust_n_),
                            prepare_shape_thrust_[arm] + prepare_thrust_ramp_rate_ * dt);
                    }
                  else if (target_error < -prepare_angle_tolerance_)
                    {
                      if (prepare_shape_thrust_[arm] > 1e-6)
                        prepare_shape_thrust_[arm] = std::max(0.0,
                            prepare_shape_thrust_[arm] - prepare_thrust_ramp_rate_ * dt);
                      else
                        prepare_pressure_[arm] = std::min(grasp_pressure_[arm],
                            prepare_pressure_[arm] + prepare_pressure_ramp_rate_ * dt);
                    }
                  active_angle_sum += imu_angle;
                  ++active_count;
                }
            }
          if (!fixed_arm_test_enabled_ && selected_arm_count > 1)
            {
              // Preserve the verified two-rotor roll/pitch moment. If either
              // member is not ready or its angle feedback asks to stop, stop
              // both instead of creating an uncommanded yaw/roll impulse.
              double common_prepare_thrust =
                  std::numeric_limits<double>::infinity();
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  common_prepare_thrust = std::min(
                      common_prepare_thrust, prepare_shape_thrust_[arm]);
              if (!active_pair_pressure_ready ||
                  !std::isfinite(common_prepare_thrust))
                common_prepare_thrust = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  prepare_shape_thrust_[arm] = common_prepare_thrust;
              if (!active_pair_pressure_ready)
                ROS_INFO_THROTTLE(1.0,
                    "PREPARE holds both correction rotors at zero until both selected arm pressures reach %.1f +/- %.1f kPa",
                    minimum_prepare_pressure_kpa_, pressure_tolerance_kpa_);
            }
          publishArmShapeDebug(relative_angle);

          double active_prepare_thrust = 0.0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (prepare_arm_active_[arm])
              active_prepare_thrust = std::max(
                  active_prepare_thrust, prepare_shape_thrust_[arm]);
          if (!body_motion_detected_)
            {
              const bool candidate_motion = active_pair_pressure_ready &&
                  active_prepare_thrust >=
                      correction_motion_detection_min_thrust_n_ &&
                  correction_omega >= correction_motion_start_rate_;
              if (!candidate_motion)
                correction_motion_candidate_start_ = ros::Time(0);
              else
                {
                  if (correction_motion_candidate_start_.isZero())
                    {
                      correction_motion_candidate_start_ = now;
                      correction_motion_candidate_error_ = attitude_error;
                    }
                  const double candidate_duration =
                      (now - correction_motion_candidate_start_).toSec();
                  const double error_reduction =
                      correction_motion_candidate_error_ - attitude_error;
                  ROS_INFO_THROTTLE(0.5,
                      "Spinal correction-motion candidate: omega %.3f rad/s for %.2f/%.2f s, posture improvement %.2f/%.2f deg",
                      correction_omega, candidate_duration,
                      correction_motion_confirm_sec_,
                      error_reduction * 180.0 / M_PI,
                      correction_motion_min_error_reduction_ * 180.0 / M_PI);
                  if (candidate_duration >= correction_motion_confirm_sec_ &&
                      error_reduction >=
                          correction_motion_min_error_reduction_)
                    {
                      body_motion_detected_ = true;
                      motion_hold_thrust_ = prepare_shape_thrust_;
                      body_motion_last_seen_ = now;
                      correction_last_progress_ = now;
                      correction_best_error_ = attitude_error;
                      ROS_INFO("Spinal confirmed whole-body correction at %.3f rad/s after %.2f s and %.2f-deg posture improvement; holding %.2f N",
                               correction_omega, candidate_duration,
                               error_reduction * 180.0 / M_PI,
                               active_prepare_thrust);
                    }
                }
            }
          else if (body_motion_detected_)
            {
              if (correction_omega >= correction_motion_stop_rate_)
                body_motion_last_seen_ = now;
              else if (!body_motion_last_seen_.isZero() &&
                       (now - body_motion_last_seen_).toSec() >=
                           correction_motion_stop_timeout_ &&
                       attitude_error > correction_slowdown_angle_)
                {
                  // A short zero-rate interval is expected around stiction.
                  // Keep the motion latch and captured thrust instead of
                  // recovering to GRASP and starting another search pulse.
                  // The thrust is lowered only by attitude proximity above.
                  ROS_INFO_THROTTLE(1.0,
                      "Correction-direction body motion paused; holding captured thrust until the attitude reaches the slowdown band");
                  body_motion_last_seen_ = now;
                }
            }
          if (body_motion_detected_)
            {
              if (attitude_error <= correction_best_error_ -
                                        correction_progress_epsilon_)
                {
                  correction_best_error_ = attitude_error;
                  correction_last_progress_ = now;
                }

              const double stalled_for = correction_last_progress_.isZero()
                  ? 0.0 : (now - correction_last_progress_).toSec();
              const double hold_limit = std::min(
                  prepare_max_thrust_n_, correction_max_thrust_n_);

              // If the captured breakaway thrust settles into a new static
              // equilibrium, resume only a slow bounded search. Near the
              // target, scale the search down with the same braking schedule
              // to retain the overshoot protection.
              if (stalled_for >= correction_stall_timeout_sec_ &&
                  std::abs(correction_omega) <=
                      correction_motion_stop_rate_ &&
                  attitude_error > recovery_stop_angle)
                {
                  const double stall_ramp =
                      correction_stall_thrust_ramp_n_s_ *
                      std::max(0.20, angle_scale);
                  double adapted_hold = 0.0;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      {
                        motion_hold_thrust_[arm] = std::min(
                            hold_limit, motion_hold_thrust_[arm] +
                                            stall_ramp * dt);
                        adapted_hold = std::max(
                            adapted_hold, motion_hold_thrust_[arm]);
                      }
                  ROS_WARN_THROTTLE(0.5,
                      "Posture correction stalled for %.2f s; adapting hold thrust %.2f -> %.2f N (limit %.2f N)",
                      stalled_for, active_prepare_thrust,
                      adapted_hold, hold_limit);
                }

              if (stalled_for >= correction_stall_abort_timeout_sec_)
                {
                  ROS_WARN("Posture did not improve for %.1f s despite adaptive thrust (limit %.2f N); restoring grasp",
                           stalled_for, hold_limit);
                  enterState(RECOVER, now);
                  break;
                }
            }
          if (body_motion_detected_)
            ROS_INFO_THROTTLE(0.25,
                "Continuous posture correction: error %.2f deg, predicted %.2f deg, omega %.3f rad/s, thrust %.2f N",
                attitude_error * 180.0 / M_PI,
                predicted_attitude_error * 180.0 / M_PI,
                correction_omega, active_prepare_thrust);

          // PREPARE thrust may already have corrected the body attitude.
          // Stop increasing it and begin regripping as soon as the attitude
          // enters the stop band; RECOVER tapers the thrust instead of
          // dropping it in one cycle.
          if (!fixed_arm_test_enabled_ &&
              attitude_error <= recovery_stop_angle)
            {
              ROS_INFO("Attitude corrected during PREPARE; tapering thrust while restoring grasp pressure");
              enterState(RECOVER, now);
              break;
            }
          if (angular_rate >= correction_max_angular_rate_)
            {
              prepare_shape_thrust_.fill(0.0);
              ROS_WARN("Angular-rate limit reached during PREPARE; stopping thrust immediately");
              enterState(RECOVER, now);
              break;
            }

          if (!fixed_arm_test_enabled_ && body_motion_detected_)
            {
              // Do not fall through to the arm-geometry completion path: it
              // would enter THRUST_CORRECTION and turn this continuous hold
              // back into another pulse.  A long timeout remains as a final
              // pneumatic/mechanical fault escape.
              if ((now - state_start_).toSec() >= prepare_timeout_)
                {
                  ROS_WARN("Continuous posture correction timed out after %.1f s; restoring grasp",
                           prepare_timeout_);
                  enterState(RECOVER, now);
                }
              break;
            }

          if (!fixed_arm_test_enabled_ && active_count > 0)
            {
              const double average_angle = active_angle_sum / active_count;
              double mean_prepare_thrust = 0.0;
              double mean_prepare_pressure = 0.0;
              double mean_target_straighten = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  {
                    mean_prepare_thrust += prepare_shape_thrust_[arm];
                    mean_prepare_pressure += prepare_pressure_[arm];
                    mean_target_straighten += prepare_straighten_angle_by_arm_[arm];
                  }
              mean_prepare_thrust /= active_count;
              mean_prepare_pressure /= active_count;
              mean_target_straighten /= active_count;
              ROS_INFO_THROTTLE(0.5,
                  "PREPARE pair: relative angle %.2f deg (target %.2f +/- %.2f), target pressure %.2f kPa, search thrust %.2f N",
                  (average_angle - ([&]() {
                    double baseline = 0.0;
                    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                      if (prepare_arm_active_[arm]) baseline += grasp_imu_angle_[arm];
                    return baseline / active_count;
                  })()) * 180.0 / M_PI,
                  -mean_target_straighten * 180.0 / M_PI,
                  prepare_angle_tolerance_ * 180.0 / M_PI,
                  mean_prepare_pressure,
                  mean_prepare_thrust);
              ROS_INFO_THROTTLE(1.0,
                  "PREPARE pressure targets [%.1f, %.1f, %.1f, %.1f] kPa; active arms [%d, %d, %d, %d]",
                  prepare_pressure_[0], prepare_pressure_[1],
                  prepare_pressure_[2], prepare_pressure_[3],
                  prepare_arm_active_[0], prepare_arm_active_[1],
                  prepare_arm_active_[2], prepare_arm_active_[3]);
              if (!std::isfinite(best_prepare_angle_))
                best_prepare_angle_ = average_angle;
              else if (average_angle < best_prepare_angle_ -
                                       prepare_progress_epsilon_)
                {
                  best_prepare_angle_ = average_angle;
                  last_angle_progress_ = now;
                }
              bool output_limited = true;
              const double paired_prepare_thrust_limit = std::min(
                  prepare_max_thrust_n_, correction_max_thrust_n_);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  output_limited = output_limited &&
                      prepare_shape_thrust_[arm] >=
                          paired_prepare_thrust_limit - 0.01;
              const bool shape_limited = output_limited &&
                  (now - last_angle_progress_).toSec() >= prepare_progress_timeout_;
              const bool timed_out = (now - state_start_).toSec() >= prepare_timeout_;
              if (timed_out && !active_pair_pressure_ready)
                {
                  prepare_shape_thrust_.fill(0.0);
                  ROS_WARN("PREPARE timed out before both selected arm pressures were ready; aborting paired thrust and restoring grasp");
                  enterState(RECOVER, now);
                  break;
                }
              geometry_ready = active_pair_pressure_ready &&
                  (target_angle_reached || shape_limited || timed_out);
              if (geometry_ready)
                {
                  correction_thrust_ = allocateCorrectionThrustFromAngles(
                      desiredThrustMoment(), achieved_bend);
                  if (target_angle_reached)
                    ROS_INFO_THROTTLE(1.0,
                        "PREPARE relative-angle target reached; reallocating correction thrust from measured geometry");
                  else
                    ROS_WARN_THROTTLE(1.0,
                        "PREPARE accepted object-dependent reachable limit; reallocating correction thrust from measured geometry");
                }
            }
          else if (fixed_arm_test_enabled_ && pressure_valid)
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              geometry_ready = geometry_ready &&
                  std::abs(measured_pressure_[arm] - prepare_pressure_[arm]) <=
                  pressure_tolerance_kpa_;

          if (geometry_ready)
            {
              if (ready_start_.isZero()) ready_start_ = now;
              if ((now - ready_start_).toSec() >= prepare_stable_duration_)
                enterState(THRUST_CORRECTION, now);
            }
          else
            ready_start_ = ros::Time(0);
          break;
          }

        case THRUST_CORRECTION:
          publishPressureTarget(prepare_pressure_, maximum_step);
          {
          const double correction_omega =
              body_angular_velocity_[0] * correction_axis_[0] +
              body_angular_velocity_[1] * correction_axis_[1];
          if (correction_omega >= correction_motion_start_rate_)
            {
              const double angle_scale = std::max(0.0, std::min(1.0,
                  (attitude_error - recovery_stop_angle) /
                  std::max(1e-6, correction_slowdown_angle_ -
                                    recovery_stop_angle)));
              for (double& thrust : correction_thrust_)
                if (thrust > 0.0)
                  thrust = std::max(correction_hold_min_thrust_n_,
                      thrust - correction_thrust_down_rate_ *
                                   (1.0 - angle_scale) * dt);
            }
          publishCorrectionThrust(correction_thrust_);
          if (attitude_error <= recovery_stop_angle ||
              angular_rate >= correction_max_angular_rate_ ||
              (now - state_start_).toSec() >= correction_thrust_duration_)
            {
              if (angular_rate >= correction_max_angular_rate_)
                prepare_shape_thrust_.fill(0.0);
              else
                prepare_shape_thrust_ = correction_thrust_;
              enterState(RECOVER, now);
            }
          }
          break;

        case RECOVER:
        default:
          if (continuous_c_recovery && continuous_recovery_started_ &&
              continuous_roll_recovery_.finished)
            {
              // Repressurizing an extended arm changes both its moment arm and
              // its contact reaction.  Treat that as a measured disturbance:
              // advance pressure only inside the roll capture region, retain
              // the minimum rotor force needed to oppose rebound, and finish
              // only after pressure, roll and rotor force have all settled.
              publishBottomTarget(0.0);
              const bool handoff_sensors_fresh = body_imu_fresh &&
                  pressure_valid && neurons_fresh &&
                  recovery_pressure_fresh &&
                  std::isfinite(recovery_roll_rate);
              const bool handoff_rate_valid = handoff_sensors_fresh &&
                  std::abs(recovery_roll_rate) <
                      recovery_measurement_rate_limit_rad_s_;
              const bool handoff_inputs_valid = handoff_sensors_fresh &&
                  handoff_rate_valid;
              if (!handoff_inputs_valid)
                {
                  prepare_shape_thrust_.fill(0.0);
                  prepare_pressure_ = grasp_pressure_;
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  ready_start_ = ros::Time(0);
                  if (!handoff_sensors_fresh)
                    ROS_ERROR_THROTTLE(1.0,
                        "Action C recovery handoff lost fresh pressure/IMU data; removing rotor force and commanding safe grasp");
                  else
                    ROS_ERROR_THROTTLE(1.0,
                        "Action C recovery handoff roll rate %.1f deg/s exceeds the %.1f-deg/s measurement envelope; removing rotor force and commanding safe grasp",
                        recovery_roll_rate * 180.0 / M_PI,
                        recovery_measurement_rate_limit_rad_s_ *
                            180.0 / M_PI);
                  break;
                }

              const double signed_error = roll_error * correction_axis_[0];
              const double signed_rate = recovery_roll_rate * correction_axis_[0];
              const double abs_error = std::abs(roll_error);
              // grasp_pressure_ may contain the contact-conditioned pressure
              // left by an earlier handoff.  It is useful as the present
              // command, but it must not become a ratchet which lowers the
              // mechanical anchor on every automatic recovery retry.  The
              // nominal locomotion grasp pressure is therefore a lower bound
              // for the final regrasp target.  A larger operator-selected
              // grasp pressure is still preserved.
              const auto nominal_regrasp_pressure = [&](size_t arm) {
                return hugmy::recoveryClamp(
                    std::max(base_pressure_kpa_, grasp_pressure_[arm]),
                    recovery_handoff_min_pressure_kpa_,
                    maximum_pressure_kpa_);
              };
              const auto support_floor_for = [&](size_t arm) {
                // Regrasp without surrendering the established anchor.  For
                // a released correction pair, stage A raises only that pair
                // to the load-bearing floor while the opposite pair remains
                // at nominal grasp. Stage B then bends the correction pair
                // back to nominal under measured-roll feedback. Lowering the
                // anchor pair to the same 15-kPa floor made 70/100-mm bodies
                // coast by 20--30 deg after rotor thrust had already stopped.
                if (recovery_selected_pair_release_ &&
                    prepare_arm_active_[arm])
                  return std::min(recoverySupportPressure(),
                                  nominal_regrasp_pressure(arm));
                return nominal_regrasp_pressure(arm);
              };
              const auto pressure_search_floor_for = [&](size_t arm) {
                // The opposite pair remains the nominal-pressure anchor. A
                // selected flexed pair may be relaxed gradually only during
                // the monitored terminal search; this creates a partial
                // contact state without jumping directly to the released
                // candidate.
                if (!recovery_selected_pair_release_ &&
                    prepare_arm_active_[arm])
                  return std::min(recovery_handoff_min_pressure_kpa_,
                                  nominal_regrasp_pressure(arm));
                return support_floor_for(arm);
              };
              // Use a small hysteresis outside the final +/-3-deg target.
              // This is wide enough to start a pneumatic handoff without
              // asking the rotor loop to chatter on the target boundary,
              // but remains much smaller than the locomotion recovery
              // trigger.
              const double capture_error = recovery_stop_angle +
                  1.5 * roll_recovery_config_.stop_margin;
              // A 20-deg hard limit is not a valid regrasp target. Search for
              // a contact equilibrium only inside the ordinary recovery
              // envelope, while retaining the independent hard limit solely
              // as a safety boundary.
              // Regrasp is part of recovery, not permission to settle again
              // at the 10-deg locomotion trigger.  Pressure may advance only
              // while the body remains in the terminal capture band.  If
              // flexion pushes roll back out, pause pressure and let the
              // continuous thrust loop restore the attitude first.
              const double handoff_acceptance_error = std::min(
                  action_c_posture_hard_roll_rad_, capture_error);
              // Completion uses one additional measurement margin.  The
              // pressure-conditioned contact in MuJoCo repeatedly settled at
              // 12--13.5 deg: it entered the 12.5-deg band, but pneumatic
              // creep carried it a few tenths of a degree out again before
              // the 0.3-s dwell elapsed.  Reopening PI there produced an
              // unnecessary limit cycle.  This is hysteresis around a
              // measured low-rate equilibrium, not a wider recovery trigger;
              // the hard-roll guard is unchanged.
              // The requested result remains +/-3 deg.  Permit the configured
              // one-degree measurement/contact hysteresis only for a
              // zero-thrust, low-rate, pressure-settled completion. Without
              // it, a 3.2--3.7-deg passive equilibrium repeatedly reopened
              // the PI loop after thrust had already reached zero.
              const double handoff_measurement_margin = std::min(
                  0.25 * M_PI / 180.0,
                  0.25 * roll_recovery_config_.stop_margin);
              const double handoff_settle_error = recovery_stop_angle +
                  handoff_measurement_margin;
              // First demonstrate the strict terminal band. Re-bending the
              // selected pair can then shift the passive contact equilibrium
              // by a few degrees even at zero thrust. If the pressure search
              // has already observed the strict target and retained that
              // contact pressure, accept a quiet nearby equilibrium instead
              // of indefinitely exciting the stiff contact. A first recovery
              // which never reached the target remains strict.
              const bool strict_target_observed_before_regrasp =
                  recovery_handoff_pressure_trimmed_ &&
                  std::isfinite(recovery_handoff_best_error_rad_) &&
                  recovery_handoff_best_error_rad_ <= handoff_settle_error;
              const double active_handoff_settle_error =
                  strict_target_observed_before_regrasp
                  ? std::max(handoff_settle_error,
                        recovery_handoff_contact_equilibrium_rad_)
                  : handoff_settle_error;
              // The requested recovery result is the configured +/- stop
              // band itself.  Driving all the way to one degree (the former
              // stop-minus-two-margins value) kept 0.6--2 N applied while
              // the measured body was already oscillating across +/-3 deg,
              // so the handoff could never satisfy its own settle test. Stop
              // adding torque on entry to the requested band and let the
              // bounded fall rate/prediction remove the residual command.
              // Start rundown just before the strict +/-3-deg boundary.
              // Waiting for a sampled value at or below exactly 3.000 deg
              // left about 2 N applied at the 3.01-deg static-friction
              // plateau.  The quarter-degree lead is smaller than the
              // completion/IMU hysteresis and lets the finite slew-down
              // carry the body into, rather than back out of, the target.
              const double handoff_drive_stop = recovery_stop_angle +
                  handoff_measurement_margin;
              const double handoff_brake_entry = handoff_drive_stop;
              const double pressure_advance_rate_limit = std::min(
                  0.08, roll_recovery_config_.settle_rate);
              const bool pressure_not_worsening =
                  roll_error * recovery_roll_rate >= -0.002;
              // Regrasp is deliberately split into two stages.  The pair not
              // used for correction is the mechanical anchor and must remain
              // at its grasp pressure.  Stage A confirms that anchor while
              // the released correction pair reaches only the minimum
              // load-bearing pressure.  Stage B raises the correction pair
              // toward grasp under the measured roll loop.  Lowering the
              // anchor pair to 15 kPa here removed the only established
              // support and reproduced the roll error after an otherwise
              // successful rotor correction.
              bool support_commands_equalized = true;
              bool support_measurements_equalized = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double support_floor = support_floor_for(arm);
                  support_commands_equalized = support_commands_equalized &&
                    std::abs(prepare_pressure_[arm] -
                        support_floor) <= 0.05;
                  support_measurements_equalized =
                      support_measurements_equalized &&
                      std::isfinite(measured_pressure_[arm]) &&
                      std::abs(measured_pressure_[arm] -
                          support_floor) <= 1.0;
                }
              if (!recovery_handoff_support_equalized_ &&
                  support_commands_equalized &&
                  support_measurements_equalized)
                recovery_handoff_support_equalized_ = true;
              const bool support_transfer_equalized =
                  recovery_handoff_support_equalized_;
              // The pre-transfer [full grasp, released pair] pressure is not
              // a candidate equilibrium: restoring it after a wrong-way
              // response recreates exactly the pneumatic roll disturbance
              // that stage A removed.  Start the best-pressure search only
              // after all four chambers actually share the support load.
              if (support_transfer_equalized &&
                  !std::isfinite(recovery_handoff_best_error_rad_))
                {
                  recovery_handoff_best_error_rad_ = abs_error;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor =
                          pressure_search_floor_for(arm);
                      recovery_handoff_best_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                }
              if (!recovery_handoff_pressure_trimmed_ &&
                  !recovery_handoff_support_equalized_ &&
                  !support_commands_equalized)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  {
                    const double support_floor = support_floor_for(arm);
                    prepare_pressure_[arm] += hugmy::recoveryClamp(
                      support_floor - prepare_pressure_[arm],
                      -recovery_handoff_pressure_rate_kpa_s_ *
                          std::max(0.0, dt),
                      recovery_handoff_pressure_rate_kpa_s_ *
                          std::max(0.0, dt));
                  }

              const bool pressure_advance_allowed =
                  recovery_selected_pair_release_ &&
                  !recovery_handoff_pressure_trimmed_ &&
                  support_transfer_equalized &&
                  abs_error <= handoff_acceptance_error &&
                  std::abs(recovery_roll_rate) <= pressure_advance_rate_limit &&
                  pressure_not_worsening;
              if (pressure_advance_allowed)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  {
                    prepare_pressure_[arm] = std::min(nominal_regrasp_pressure(arm),
                        prepare_pressure_[arm] +
                            recovery_handoff_pressure_rate_kpa_s_ *
                                std::max(0.0, dt));
                  }

              // A flexed pair may correct most of the roll but retain a
              // 3--4-deg zero-thrust bias at full pressure. Rather than
              // rejecting that measured-good contact and jumping to the
              // fully released state, relax only the selected pair slowly
              // while the opposite pair remains the 30-kPa anchor. Search
              // starts only after a quiet retained-progress plateau near the
              // target. The best measured pressure is recorded below and is
              // restored if the contact moment changes sign.
              const bool terminal_pressure_search =
                  !recovery_selected_pair_release_ &&
                  !recovery_handoff_pressure_trimmed_ &&
                  support_transfer_equalized &&
                  signed_error > handoff_drive_stop &&
                  abs_error <= capture_error &&
                  std::abs(recovery_roll_rate) <=
                      pressure_advance_rate_limit &&
                  ((recovery_handoff_braking_ &&
                    abs_error > recovery_stop_angle) ||
                   (recovery_handoff_last_progress_time_ >= 0.0 &&
                    now.toSec() - recovery_handoff_last_progress_time_ >=
                        roll_recovery_config_.stall_sec));
              if (terminal_pressure_search)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  if (prepare_arm_active_[arm])
                    {
                      const double floor = pressure_search_floor_for(arm);
                      // Keep the command outside the approximately 1-kPa
                      // pressure-controller deadband. The chamber still
                      // vents at its physical/rate-limited speed; this is a
                      // target lead, not an instantaneous pressure change.
                      const double venting_target =
                          std::isfinite(measured_pressure_[arm])
                          ? measured_pressure_[arm] - 2.0
                          : prepare_pressure_[arm];
                      prepare_pressure_[arm] = std::max(floor,
                          std::min(venting_target,
                              prepare_pressure_[arm] -
                                  0.25 *
                                      recovery_handoff_pressure_rate_kpa_s_ *
                                      std::max(0.0, dt)));
                      recovery_handoff_target_pressure_[arm] =
                          prepare_pressure_[arm];
                    }

              // Record the actual chamber pressure at the best measured roll.
              // The command leads the pneumatic state, so retaining the
              // command value would overshoot the equilibrium by several kPa.
              double minimum_handoff_pressure = std::numeric_limits<double>::infinity();
              bool handoff_pressure_valid = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double support_floor =
                      pressure_search_floor_for(arm);
                  handoff_pressure_valid = handoff_pressure_valid &&
                      std::isfinite(measured_pressure_[arm]);
                  minimum_handoff_pressure = std::min(
                      minimum_handoff_pressure,
                      measured_pressure_[arm] - support_floor);
                }
              const bool pressure_candidate = support_transfer_equalized &&
                  handoff_pressure_valid &&
                  minimum_handoff_pressure >= 0.0 &&
                  std::abs(recovery_roll_rate) <= pressure_advance_rate_limit;
              if (!recovery_handoff_pressure_trimmed_ && pressure_candidate &&
                  abs_error < recovery_handoff_best_error_rad_)
                {
                  recovery_handoff_best_error_rad_ = abs_error;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor =
                          pressure_search_floor_for(arm);
                      recovery_handoff_best_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                }
              double pressure_distance_from_best = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                pressure_distance_from_best = std::max(
                    pressure_distance_from_best,
                    std::abs(measured_pressure_[arm] -
                             recovery_handoff_best_pressure_[arm]));
              const bool pressure_worsening = !recovery_handoff_pressure_trimmed_ &&
                  support_transfer_equalized &&
                  std::isfinite(recovery_handoff_best_error_rad_) &&
                  abs_error >= recovery_handoff_best_error_rad_ +
                      recovery_handoff_trim_margin_rad_ &&
                  pressure_distance_from_best >= 0.5;
              if (pressure_worsening)
                {
                  if (recovery_handoff_worsening_since_ < 0.0)
                    recovery_handoff_worsening_since_ = now.toSec();
                  if (now.toSec() - recovery_handoff_worsening_since_ >=
                      recovery_handoff_trim_confirm_sec_)
                    {
                      recovery_handoff_pressure_trimmed_ = true;
                      // Regrasp means restoring a load-bearing contact, not
                      // forcing every chamber to the same nominal pressure.
                      // On a curved object, the selected pair's moment can
                      // reverse while it is re-bent: in the 70-mm fixture the
                      // measured minimum occurred near [15,15,30,30] kPa and
                      // forcing the released side to 30 kPa recreated a
                      // persistent 3--6-deg oscillation. Retain the measured
                      // per-arm pressure at the roll minimum. The opposite
                      // anchor remains at nominal grasp because its support
                      // floor and best pressure are both nominal.
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        recovery_handoff_target_pressure_[arm] =
                            hugmy::recoveryClamp(
                                recovery_handoff_best_pressure_[arm],
                                pressure_search_floor_for(arm),
                                nominal_regrasp_pressure(arm));
                      ROS_WARN("Action C recovery handoff selected contact-conditioned grasp [%.1f %.1f %.1f %.1f] kPa after roll minimum %.2f deg; nominal repressurization worsened posture",
                               recovery_handoff_target_pressure_[0],
                               recovery_handoff_target_pressure_[1],
                               recovery_handoff_target_pressure_[2],
                               recovery_handoff_target_pressure_[3],
                               recovery_handoff_best_error_rad_ * 180.0 / M_PI);
                    }
                }
              else
                recovery_handoff_worsening_since_ = -1.0;
              if (recovery_handoff_pressure_trimmed_)
                for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                  prepare_pressure_[arm] += hugmy::recoveryClamp(
                      recovery_handoff_target_pressure_[arm] - prepare_pressure_[arm],
                      -recovery_handoff_pressure_rate_kpa_s_ * std::max(0.0, dt),
                      recovery_handoff_pressure_rate_kpa_s_ * std::max(0.0, dt));

              // Arm flexion during pneumatic handoff changes the contact
              // pivot.  Consequently a pair which corrected roll while the
              // arm was extended can reverse its effective moment after the
              // chamber becomes load bearing.  Detect that reversal from a
              // sustained measured loss relative to the best handoff pose;
              // never continue integrating the same pair to the thrust cap.
              double selected_handoff_thrust = 0.0;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  selected_handoff_thrust = std::max(
                      selected_handoff_thrust, prepare_shape_thrust_[arm]);
              const bool thrust_wrong_way_candidate =
                  !recovery_handoff_thrust_wrong_way_ &&
                  selected_handoff_thrust >=
                      roll_recovery_config_.detect_thrust &&
                  ((support_transfer_equalized &&
                    std::isfinite(recovery_handoff_best_error_rad_) &&
                    abs_error >= recovery_handoff_best_error_rad_ +
                        recovery_handoff_trim_margin_rad_ &&
                    roll_error * recovery_roll_rate < -0.002) ||
                   (!support_transfer_equalized &&
                    signed_error > recovery_stop_angle &&
                    signed_rate < -roll_recovery_config_.start_rate));
              if (thrust_wrong_way_candidate)
                {
                  if (recovery_handoff_thrust_worsening_since_ < 0.0)
                    recovery_handoff_thrust_worsening_since_ = now.toSec();
                  if (now.toSec() - recovery_handoff_thrust_worsening_since_ >=
                      recovery_handoff_trim_confirm_sec_)
                    {
                      recovery_handoff_thrust_wrong_way_ = true;
                      recovery_handoff_pressure_trimmed_ = true;
                      if (recovery_selected_contact_pivot_enabled_)
                        {
                          const uint8_t candidate_bit = static_cast<uint8_t>(
                              1u << (4u * recovery_selected_pair_index_ +
                                     recovery_selected_pressure_candidate_index_));
                          recovery_rejected_candidate_mask_ |= candidate_bit;
                          recovery_rejected_direction_sign_ =
                              roll_error >= 0.0 ? 1 : -1;
                          recovery_candidate_reselect_pending_ = true;
                          // The rejection mask identifies the exact
                          // pair/contact state.  Do not additionally force
                          // the opposite pair through the legacy mapping.
                          recovery_handoff_pair_was_wrong_ = false;
                        }
                      if (recovery_selected_contact_pivot_enabled_)
                        recovery_handoff_target_pressure_ =
                            recoveryReselectionPressure();
                      else
                        for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                          recovery_handoff_target_pressure_[arm] =
                              support_floor_for(arm);
                      recovery_handoff_braking_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      ready_start_ = ros::Time(0);
                      ROS_WARN("Action C recovery handoff measured a reversed rotor response after re-bending; braking pair at %.2f N and retaining pressure [%.1f %.1f %.1f %.1f] kPa before a fresh recovery decision",
                               selected_handoff_thrust,
                               recovery_handoff_target_pressure_[0],
                               recovery_handoff_target_pressure_[1],
                               recovery_handoff_target_pressure_[2],
                               recovery_handoff_target_pressure_[3]);
                    }
                }
              else if (!recovery_handoff_thrust_wrong_way_)
                recovery_handoff_thrust_worsening_since_ = -1.0;

              // The selected pair can create moment only in its original
              // correction direction. Model FF is used outside the terminal
              // band; rate PI supplies only the residual needed while arm
              // pressure and contact reaction are changing.
              const auto model = measuredRecoveryModel();
              // The requested +/-3-deg band is the final zero-thrust
              // tolerance, not the best point at which to turn the rotor
              // off. On small curved fixtures the contact spring returned
              // roll by about two degrees after rundown, producing a 3--6
              // deg limit cycle. During the pneumatic handoff drive to an
              // inner target, then use the outer band only for completion.
              const double requested_rate = signed_error > handoff_drive_stop
                  ? std::min(roll_recovery_config_.max_target_rate,
                        roll_recovery_config_.angle_gain *
                            (signed_error - handoff_drive_stop))
                  : 0.0;
              const double rate_error = requested_rate - signed_rate;
              double feedforward = 0.0;
              if (signed_error > handoff_drive_stop && model.valid &&
                  std::isfinite(model.lever) && model.lever >=
                      roll_recovery_config_.min_lever)
                feedforward = hugmy::recoveryClamp(
                    model.moment / model.lever, 0.0,
                    roll_recovery_config_.limit);
              double integral_delta = hugmy::recoveryClamp(
                  roll_recovery_config_.rate_ki * rate_error,
                  -roll_recovery_config_.integral_rate,
                  roll_recovery_config_.integral_rate) * std::max(0.0, dt);
              // At a quiet full-grasp plateau, gyro jitter can repeatedly
              // cancel the small positive rate error and leave the command
              // below contact breakaway indefinitely.  Preserve a slow,
              // bounded rise until the half-margin brake entry is reached.
              // Measured wrong-way motion and all amplitude/slew limits
              // remain independent and take priority.
              const bool terminal_contact_plateau =
                  support_transfer_equalized &&
                  !recovery_handoff_braking_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  signed_error > handoff_brake_entry &&
                  recovery_handoff_last_progress_time_ >= 0.0 &&
                  now.toSec() - recovery_handoff_last_progress_time_ >=
                      roll_recovery_config_.stall_sec &&
                  std::abs(recovery_roll_rate) <
                      roll_recovery_config_.start_rate;
              if (terminal_contact_plateau)
                integral_delta = std::max(integral_delta,
                    roll_recovery_config_.integral_rate *
                        std::max(0.0, dt));
              const double raw_target = feedforward +
                  roll_recovery_config_.rate_kp * rate_error +
                  recovery_handoff_integral_n_;
              if ((integral_delta > 0.0 && raw_target >= roll_recovery_config_.limit) ||
                  (integral_delta < 0.0 && raw_target <= 0.0))
                integral_delta = 0.0;
              recovery_handoff_integral_n_ = hugmy::recoveryClamp(
                  recovery_handoff_integral_n_ + integral_delta,
                  0.0, roll_recovery_config_.limit);
              double handoff_target = hugmy::recoveryClamp(feedforward +
                  roll_recovery_config_.rate_kp * rate_error +
                  recovery_handoff_integral_n_, 0.0,
                  roll_recovery_config_.limit);
              // Keep measured roll feedback active below the nominal contact
              // floor as well.  The same released geometry has just produced
              // the successful correction; disabling it until 15 kPa leaves
              // pneumatic rebound completely open loop. Wrong-way detection
              // below still removes thrust if the effective sign really
              // changes during flexion.
              // During stage A retain only a conservative amount of the
              // already verified correction force.  The pneumatic transfer
              // can otherwise move roll by more than ten degrees before the
              // four chambers reach equal support.  Do not integrate during
              // this changing geometry, and never use the retained force on
              // the far side of the target.  Once support is equalized, the
              // full measured model/rate loop resumes.
              if (!support_transfer_equalized)
                {
                  recovery_handoff_integral_n_ = 0.0;
                  if (signed_error > recovery_stop_angle &&
                      !recovery_handoff_thrust_wrong_way_)
                    handoff_target = std::min(handoff_target,
                        recovery_handoff_stage_a_thrust_max_n_);
                  else
                    handoff_target = 0.0;
                  // Repressurising an exhausted pneumatic arm creates a
                  // repeatable roll moment before the attitude error has had
                  // time to grow.  Pure error feedback therefore reacted one
                  // to two seconds late: pressure reached 15 kPa while the
                  // body had already rebounded from 1 to 6--8 deg.  Schedule
                  // a bounded model feed-forward with the measured support
                  // build-up, then leave all residual correction to the same
                  // rate loop.  If the target has actually been crossed, the
                  // one-direction pair is kept off as before.
                  // Do not add pressure feed-forward while inertia is
                  // already carrying the body toward the target faster than
                  // requested.  The former condition kept 1.5 N on until
                  // the body had crossed the target by the full stop angle;
                  // on the 130-mm fixture this converted a +0.95-deg,
                  // otherwise successful handoff into a -10-deg overshoot.
                  const double pressure_feedforward_rate_limit = std::max(
                      requested_rate,
                      0.5 * roll_recovery_config_.settle_rate);
                  if (signed_error > 0.0 &&
                      signed_rate <= pressure_feedforward_rate_limit &&
                      !recovery_handoff_thrust_wrong_way_)
                    {
                      double support_fraction = 0.0;
                      size_t support_count = 0;
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        if (prepare_arm_active_[arm])
                          {
                            support_fraction += hugmy::recoveryClamp(
                                prepare_pressure_[arm] /
                                    std::max(1.0,
                                      recovery_handoff_min_pressure_kpa_),
                                0.0, 1.0);
                            ++support_count;
                          }
                      if (support_count > 0)
                        support_fraction /= support_count;
                      const double pressure_feedforward =
                          roll_recovery_config_.fallback_thrust +
                          support_fraction * std::max(0.0,
                              recovery_handoff_stage_a_thrust_max_n_ -
                                  roll_recovery_config_.fallback_thrust);
                      handoff_target = std::max(handoff_target,
                          std::min(recovery_handoff_stage_a_thrust_max_n_,
                                   pressure_feedforward));
                    }
                }
              if (recovery_handoff_thrust_wrong_way_)
                {
                  handoff_target = 0.0;
                  recovery_handoff_integral_n_ = 0.0;
                }
              bool command_at_grasp = true;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                command_at_grasp = command_at_grasp &&
                    std::abs(prepare_pressure_[arm] -
                        recovery_handoff_target_pressure_[arm]) <= 0.05;
              if (!recovery_handoff_braking_ && command_at_grasp &&
                  abs_error <= (strict_target_observed_before_regrasp
                      ? active_handoff_settle_error : handoff_brake_entry) &&
                  std::abs(recovery_roll_rate) <=
                      0.5 * roll_recovery_config_.settle_rate)
                {
                  recovery_handoff_braking_ = true;
                  recovery_handoff_brake_error_rad_ = abs_error;
                }
              // Latch the final rundown at the requested +/-3-deg boundary.
              // Completion is checked separately after thrust reaches zero;
              // if elastic rebound leaves the band, the same measured-good
              // candidate is reopened below.  This avoids both accepting a
              // biased zero-thrust grasp and driving a curved contact through
              // its moment-sign reversal merely to obtain extra margin.
              // Re-enabling 0/5-N commands
              // at every 10-deg threshold crossing produced a limit cycle.
              // Regrasp pressure can move the contact equilibrium slowly,
              // with too little instantaneous angular rate to pass the old
              // rate-product test.  Therefore use retained attitude loss,
              // measured from both the braking point and the best observed
              // handoff pose, to reopen PI.  A small quarter-degree boundary
              // hysteresis rejects IMU noise, while the retained-pose test
              // still requires a meaningful rebound before thrust resumes.
              const double retained_handoff_error =
                  std::isfinite(recovery_handoff_best_error_rad_)
                  ? std::min(recovery_handoff_brake_error_rad_,
                        recovery_handoff_best_error_rad_)
                  : recovery_handoff_brake_error_rad_;
              const double terminal_reopen_margin =
                  handoff_measurement_margin;
              // Completion intentionally permits the small sensor/contact
              // hysteresis in `handoff_settle_error`.  Reopening at
              // stop_angle+0.25 deg while completion allowed +0.50 deg made
              // the two conditions overlap: the controller repeatedly
              // restarted 0.5--0.7 N around 3.3--3.6 deg and could never
              // accumulate its settle dwell.  Reopen only after leaving the
              // complete settle corridor.  A genuine rebound is still caught
              // well below the locomotion recovery trigger.
              const double terminal_reopen_error =
                  active_handoff_settle_error + terminal_reopen_margin;
              if (!recovery_handoff_thrust_wrong_way_ &&
                  recovery_handoff_braking_ &&
                  abs_error >= terminal_reopen_error &&
                  abs_error >= retained_handoff_error +
                      terminal_reopen_margin)
                {
                  recovery_handoff_braking_ = false;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  ready_start_ = ros::Time(0);
                }
              // While the released pair is still taking load, being briefly
              // inside the stop band must not erase the pressure-scheduled
              // feed-forward above.  Otherwise the changing contact force
              // rebounds the body before attitude feedback can react.  The
              // ordinary terminal brake is enabled once support has been
              // equalised; target crossing and verified wrong-way response
              // still stop thrust immediately.
              if (recovery_handoff_braking_ ||
                  // A selected pair is one-directional.  Once the measured
                  // attitude crosses the target, any remaining command can
                  // only increase overshoot.  Brake at the crossing; if the
                  // zero-thrust body later leaves the stop band, the code
                  // below freezes contact pressure and selects the opposite
                  // signed candidate.
                  signed_error <= 0.0 ||
                  // Entering the requested +/-3-deg band is already the
                  // signal to remove torque. Requiring a small angular rate
                  // here kept applying 0.6--1.2 N while the body crossed the
                  // target, which excited a persistent handoff limit cycle.
                  // Rate is still required by handoff_settled below, after
                  // the command has actually fallen to zero.
                  (support_transfer_equalized &&
                   abs_error <= handoff_drive_stop))
                {
                  handoff_target = 0.0;
                  recovery_handoff_integral_n_ = std::max(0.0,
                      recovery_handoff_integral_n_ -
                          roll_recovery_config_.integral_rate * std::max(0.0, dt));
                }
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  const double target = prepare_arm_active_[arm]
                      ? handoff_target : 0.0;
                  const double handoff_fall_rate =
                      recovery_handoff_thrust_wrong_way_
                      ? std::max(3.0, recovery_handoff_thrust_fall_n_s_)
                      : (recovery_handoff_braking_
                          ? std::max(1.5,
                              recovery_handoff_thrust_fall_n_s_)
                          : recovery_handoff_thrust_fall_n_s_);
                  prepare_shape_thrust_[arm] += hugmy::recoveryClamp(
                      target - prepare_shape_thrust_[arm],
                      -handoff_fall_rate *
                          std::max(0.0, dt),
                      roll_recovery_config_.rise * std::max(0.0, dt));
                }
              publishPressureTarget(prepare_pressure_,
                  std::numeric_limits<double>::infinity());
              publishCorrectionThrust(prepare_shape_thrust_);

              if (!command_at_grasp) action_c_pressure_ready_gate_.reset();
              const bool pressure_ready = command_at_grasp &&
                  actionCPressureReady(recovery_handoff_target_pressure_, now);
              const bool thrust_stopped = std::all_of(
                  prepare_shape_thrust_.begin(), prepare_shape_thrust_.end(),
                  [](double thrust) { return thrust <= 0.01; });
              // A useful rotor correction may disappear after the selected
              // pair bends back around the object.  In that case the moment
              // does not necessarily reverse: it can simply vanish at a
              // contact equilibrium outside the requested +/-3-deg band.
              // The existing wrong-way detector cannot observe that case.
              // Once full pneumatic grasp is measured, therefore retain a
              // progress reference and require at least the normal 0.5-deg
              // progress quantum within the bounded pair-retry interval.
              // Low thrust alone never rejects a candidate, and ongoing
              // angular motion is allowed to finish before this decision.
              const bool full_grasp_tracking =
                  recovery_selected_contact_pivot_enabled_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  command_at_grasp && pressure_ready &&
                  abs_error > active_handoff_settle_error;
              // The rate-sign detector above catches a clear reverse
              // rotation, but not the slow contact creep observed after a
              // flexed pair has reached full pressure.  Establish a fresh
              // reference only after pressure is settled and meaningful
              // thrust is present.  A quiet plateau remains eligible for
              // the normal high-authority retry below; only retained error
              // growth rejects the candidate at low authority.
              const bool flexed_handoff_response_active =
                  recovery_selected_contact_pivot_enabled_ &&
                  !recovery_selected_pair_release_ &&
                  !recovery_handoff_thrust_wrong_way_ &&
                  !recovery_handoff_braking_ &&
                  command_at_grasp && pressure_ready &&
                  signed_error > recovery_stop_angle &&
                  selected_handoff_thrust >=
                      roll_recovery_config_.detect_thrust;
              bool handoff_candidate_worsening = false;
              if (!flexed_handoff_response_active)
                {
                  recovery_handoff_candidate_reference_rad_ =
                      std::numeric_limits<double>::infinity();
                  recovery_handoff_candidate_worsening_since_ = -1.0;
                }
              else if (!std::isfinite(
                           recovery_handoff_candidate_reference_rad_))
                {
                  recovery_handoff_candidate_reference_rad_ = abs_error;
                  recovery_handoff_candidate_worsening_since_ = -1.0;
                }
              else if (recovery_handoff_candidate_reference_rad_ -
                           abs_error >= correction_progress_epsilon_)
                {
                  recovery_handoff_candidate_reference_rad_ = abs_error;
                  recovery_handoff_candidate_worsening_since_ = -1.0;
                }
              else if (abs_error >=
                           recovery_handoff_candidate_reference_rad_ +
                           recovery_wrong_geometry_transition_worsening_rad_)
                {
                  if (recovery_handoff_candidate_worsening_since_ < 0.0)
                    recovery_handoff_candidate_worsening_since_ = now.toSec();
                  handoff_candidate_worsening =
                      now.toSec() -
                          recovery_handoff_candidate_worsening_since_ >=
                      recovery_wrong_geometry_transition_confirm_sec_;
                }
              else
                recovery_handoff_candidate_worsening_since_ = -1.0;
              if (!full_grasp_tracking)
                {
                  recovery_handoff_progress_reference_rad_ =
                      std::numeric_limits<double>::infinity();
                  recovery_handoff_last_progress_time_ = -1.0;
                }
              else
                {
                  if (!std::isfinite(
                          recovery_handoff_progress_reference_rad_) ||
                      recovery_handoff_last_progress_time_ < 0.0)
                    {
                      recovery_handoff_progress_reference_rad_ = abs_error;
                      recovery_handoff_last_progress_time_ = now.toSec();
                    }
                  else if (recovery_handoff_progress_reference_rad_ -
                               abs_error >= correction_progress_epsilon_)
                    {
                      recovery_handoff_progress_reference_rad_ = abs_error;
                      recovery_handoff_last_progress_time_ = now.toSec();
                    }

                  const double quiet_rate_limit = std::max(
                      2.0 * roll_recovery_config_.settle_rate,
                      roll_recovery_config_.start_rate);
                  // A loaded flexed contact also has static breakaway
                  // friction.  The 70-mm fixture repeatedly improved from
                  // 12 to about 6 deg, then was rejected after only 0.9 N
                  // because the ordinary detection threshold was mistaken
                  // for proof that its moment had vanished.  Releasing that
                  // useful pair then produced a much larger roll excursion.
                  // For either contact state, declare no response only after
                  // the same bounded high-authority gate used by the rotor
                  // phase.  A measured wrong-sign response remains an
                  // immediate independent rejection path above.
                  const double handoff_retry_thrust = std::max(
                      roll_recovery_config_.detect_thrust,
                      roll_recovery_config_.limit *
                          roll_recovery_config_.pair_retry_thrust_fraction);
                  const double handoff_retry_dwell =
                      recovery_selected_pair_release_
                      ? 2.0 * roll_recovery_config_.pair_retry_sec
                      : roll_recovery_config_.pair_retry_sec;
                  const bool handoff_candidate_stalled =
                      abs_error > capture_error &&
                      selected_handoff_thrust >= handoff_retry_thrust &&
                      std::abs(recovery_roll_rate) <= quiet_rate_limit &&
                      now.toSec() - recovery_handoff_last_progress_time_ >=
                          handoff_retry_dwell;
                  if (handoff_candidate_stalled ||
                      handoff_candidate_worsening)
                    {
                      const uint8_t candidate_bit = static_cast<uint8_t>(
                          1u << (4u * recovery_selected_pair_index_ +
                                 recovery_selected_pressure_candidate_index_));
                      recovery_rejected_candidate_mask_ |= candidate_bit;
                      recovery_rejected_direction_sign_ =
                          roll_error >= 0.0 ? 1 : -1;
                      recovery_candidate_reselect_pending_ = true;
                      recovery_handoff_thrust_wrong_way_ = true;
                      recovery_handoff_pair_was_wrong_ = false;
                      recovery_handoff_pressure_trimmed_ = true;
                      recovery_handoff_braking_ = true;
                      recovery_handoff_integral_n_ = 0.0;
                      recovery_handoff_progress_reference_rad_ =
                          std::numeric_limits<double>::infinity();
                      recovery_handoff_last_progress_time_ = -1.0;
                      recovery_handoff_candidate_reference_rad_ =
                          std::numeric_limits<double>::infinity();
                      recovery_handoff_candidate_worsening_since_ = -1.0;
                      ready_start_ = ros::Time(0);
                      recovery_handoff_target_pressure_ =
                          recoveryReselectionPressure();
                      ROS_WARN("Circular recovery rejected pair %zu+%zu at %.1f kPa: full-grasp roll %s at %.2f deg under %.2f N; mask=0x%02x. Regrasping before candidate re-selection",
                          recovery_selected_pair_index_ == 0 ? 0ul : 2ul,
                          recovery_selected_pair_index_ == 0 ? 1ul : 3ul,
                          recovery_selected_pair_pressure_kpa_,
                          handoff_candidate_worsening
                              ? "worsened from its measured reference"
                              : "made no retained progress",
                          roll_error * 180.0 / M_PI,
                          selected_handoff_thrust,
                          recovery_rejected_candidate_mask_);
                    }
                }
              // The terminal brake has hysteresis so a small pneumatic
              // rebound does not chatter the rotors.  That hysteresis must
              // not become a deadband: a run can enter +/-3 deg, stop the
              // rotors, and then settle quietly at 4--5 deg without ever
              // reaching the one-degree reopen threshold.  After a full
              // second with zero thrust, full contact pressure, low roll
              // rate, and no retained improvement, reopen the same
              // closed-loop handoff.  This is not a candidate rejection;
              // the pair still has the correct moment sign and gets another
              // measured low-authority approach before geometry is changed.
              const double handoff_brake_deadlock_sec = std::max(
                  1.0, recovery_handoff_settle_sec_);
              const bool handoff_brake_deadlock = full_grasp_tracking &&
                  recovery_handoff_braking_ && thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate &&
                  recovery_handoff_last_progress_time_ >= 0.0 &&
                  now.toSec() - recovery_handoff_last_progress_time_ >=
                      handoff_brake_deadlock_sec;
              if (handoff_brake_deadlock)
                {
                  recovery_handoff_braking_ = false;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  // Reopening from zero with the integral cleared reproduced
                  // the same 0.2--0.35-N sub-breakaway pulse indefinitely on
                  // the 70-mm fixture. Seed only the already configured
                  // low-authority detection force; rate feedback and the
                  // predictive brake still reduce it immediately if the body
                  // starts moving too fast.
                  recovery_handoff_integral_n_ = std::min(
                      roll_recovery_config_.limit,
                      std::max(recovery_handoff_integral_n_,
                               roll_recovery_config_.detect_thrust));
                  recovery_handoff_progress_reference_rad_ = abs_error;
                  recovery_handoff_last_progress_time_ = now.toSec();
                  ready_start_ = ros::Time(0);
                  ROS_INFO("Action C recovery released terminal-brake deadband at %.2f deg after %.1f s without retained progress; resuming measured low-authority correction",
                           roll_error * 180.0 / M_PI,
                           handoff_brake_deadlock_sec);
                }
              // A contact candidate rejected during pneumatic handoff has
              // already demonstrated either a reversed moment or a loaded
              // no-response.  Once its rotor thrust reaches zero, prescribed
              // pressure equalisation no longer adds safety; it only leaves
              // the body in the disproved geometry. Freeze the measured
              // pressures and immediately re-run signed candidate selection.
              const bool rejected_handoff_candidate_stopped =
                  recovery_candidate_reselect_pending_ &&
                  recovery_handoff_thrust_wrong_way_ && thrust_stopped &&
                  !recovery_handoff_finalizing_regrasp_;
              if (rejected_handoff_candidate_stopped)
                {
                  recovery_handoff_target_pressure_ =
                      recoveryMeasuredContactPressure();
                  prepare_pressure_ = recovery_handoff_target_pressure_;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  prepare_shape_thrust_.fill(0.0);
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  action_c_resume_gate_.reset();
                  action_c_boundary_settle_gate_.reset();
                  ROS_WARN("Action C rejected handoff candidate reached zero thrust; froze measured pressure [%.1f %.1f %.1f %.1f] kPa for immediate contact-geometry re-selection (mask 0x%02x)",
                           grasp_pressure_[0], grasp_pressure_[1],
                           grasp_pressure_[2], grasp_pressure_[3],
                           recovery_rejected_candidate_mask_);
                  enterState(GRASP, now);
                  break;
                }
              // If the first correction coasted through zero, the selected
              // pair cannot correct the residual error on the other side.
              // Once pneumatic support is symmetric, retain that safe
              // intermediate grasp and immediately make a fresh pair
              // decision.  Previously this condition waited for the 60-s
              // handoff timeout because signed_error made the old pair's
              // target thrust identically zero.
              const double opposite_selection_boundary =
                  recovery_stop_angle +
                  2.0 * roll_recovery_config_.stop_margin;
              // Re-select only after the measured residual has left the
              // stop hysteresis. A rate-only look-ahead was tested here,
              // but contact-model sign errors could then select an
              // opposite pair while the measured posture was still safe.
              const bool crossed_target_after_rundown =
                  !recovery_candidate_reselect_pending_ &&
                  thrust_stopped &&
                  signed_error <= -opposite_selection_boundary;
              if (crossed_target_after_rundown)
                {
                  recovery_handoff_target_pressure_ =
                      recoveryMeasuredContactPressure();
                  // The old pair is already at zero thrust and can no longer
                  // brake motion on the opposite side of the target.  Do not
                  // wait for a prescribed support pressure: on a small
                  // cylinder that delay alone can double the roll error.
                  // Freeze the pressure which is carrying load now, preserve
                  // the opposite anchor, and select a new signed lever from
                  // the current circle/contact geometry immediately.
                  recovery_handoff_pressure_trimmed_ = true;
                  recovery_handoff_thrust_wrong_way_ = true;
                  recovery_handoff_braking_ = true;
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  // The correction direction changed after crossing the
                  // target. Candidate evidence is directional, so begin the
                  // opposite-sign recovery with a fresh candidate set.
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  recovery_candidate_reselect_pending_ = true;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  ready_start_ = ros::Time(0);
                  prepare_pressure_ = recovery_handoff_target_pressure_;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  prepare_shape_thrust_.fill(0.0);
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  action_c_resume_gate_.reset();
                  action_c_boundary_settle_gate_.reset();
                  ROS_WARN("Action C correction crossed the roll target and left the %.2f-deg stop hysteresis (residual %.2f deg); froze measured pressure [%.1f %.1f %.1f %.1f] kPa and requested immediate opposite-sign geometry selection",
                           opposite_selection_boundary * 180.0 / M_PI,
                           roll_error * 180.0 / M_PI,
                           grasp_pressure_[0], grasp_pressure_[1],
                           grasp_pressure_[2], grasp_pressure_[3]);
                  enterState(GRASP, now);
                  break;
                }
              // Re-selection first establishes only a load-bearing contact
              // on the old active pair. If that state is already inside the
              // terminal roll band, do not start another rotor candidate.
              // Instead, restore nominal grasp as a separate monitored
              // pressure phase. A posture loss during this last pressure
              // ramp freezes the measured per-arm pressure and returns to
              // geometry selection; it is never allowed to become an
              // unobserved 15 -> 30 kPa roll impulse.
              const bool retry_contact_settled =
                  recovery_candidate_reselect_pending_ && pressure_ready &&
                  thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate;
              if (retry_contact_settled &&
                  !recovery_handoff_finalizing_regrasp_ &&
                  abs_error <= active_handoff_settle_error)
                {
                  recovery_handoff_finalizing_regrasp_ = true;
                  recovery_rejected_candidate_mask_ = 0;
                  recovery_rejected_direction_sign_ = 0;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    recovery_handoff_target_pressure_[arm] =
                        nominal_regrasp_pressure(arm);
                  ready_start_ = ros::Time(0);
                  ROS_INFO("Action C load-bearing recovery is inside the roll band at %.2f deg; monitoring final nominal regrasp",
                           roll_error * 180.0 / M_PI);
                  break;
                }
              if (recovery_handoff_finalizing_regrasp_ &&
                  abs_error > active_handoff_settle_error +
                      recovery_handoff_reopen_angle_rad_)
                {
                  recovery_handoff_finalizing_regrasp_ = false;
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    {
                      const double support_floor = support_floor_for(arm);
                      recovery_handoff_target_pressure_[arm] =
                          hugmy::recoveryClamp(measured_pressure_[arm],
                              support_floor, nominal_regrasp_pressure(arm));
                    }
                  ready_start_ = ros::Time(0);
                  ROS_WARN("Action C final regrasp changed roll to %.2f deg; freezing contact-conditioned pressure [%.1f %.1f %.1f %.1f] kPa before re-selection",
                           roll_error * 180.0 / M_PI,
                           recovery_handoff_target_pressure_[0],
                           recovery_handoff_target_pressure_[1],
                           recovery_handoff_target_pressure_[2],
                           recovery_handoff_target_pressure_[3]);
                  break;
                }
              // A measured sign reversal means this contact geometry needs a
              // fresh pair decision.  Finish only the balanced support
              // transfer here; do not label a large roll error as recovered.
              // GRASP will immediately start a new measured-roll pass while
              // retaining the locomotion goal and optimizer trial.
              const bool handoff_retry_ready =
                  (recovery_candidate_reselect_pending_ ||
                   recovery_handoff_thrust_wrong_way_) &&
                  pressure_ready && thrust_stopped &&
                  std::abs(recovery_roll_rate) <=
                      roll_recovery_config_.settle_rate;
              if (handoff_retry_ready)
                {
                  last_correction_end_ = now;
                  continuous_recovery_started_ = false;
                  continuous_recovery_waiting_for_settle_ = true;
                  continuous_recovery_retry_opposite_pending_ =
                      !recovery_candidate_reselect_pending_ &&
                      recovery_handoff_pair_was_wrong_;
                  recovery_handoff_integral_n_ = 0.0;
                  recovery_handoff_brake_error_rad_ = 0.0;
                  recovery_handoff_braking_ = false;
                  grasp_pressure_ = recovery_handoff_target_pressure_;
                  if (hybridLocomotionActive() && action_c_resume_pending_)
                    {
                      hybrid_grasp_pressure_ = grasp_pressure_;
                      hybrid_hold_pressure_ = true;
                    }
                  ROS_WARN("Action C balanced support transfer complete at roll %.2f deg; requesting %s recovery selection (rejected mask 0x%02x)",
                           roll_error * 180.0 / M_PI,
                           recovery_candidate_reselect_pending_
                               ? "contact-conditioned" : "fresh measured-roll",
                           recovery_rejected_candidate_mask_);
                  enterState(GRASP, now);
                  break;
                }
              const bool handoff_settled = pressure_ready && thrust_stopped &&
                  std::abs(recovery_roll_rate) <= roll_recovery_config_.settle_rate &&
                  abs_error <= active_handoff_settle_error;
              if (handoff_settled)
                {
                  if (ready_start_.isZero()) ready_start_ = now;
                  if ((now - ready_start_).toSec() >=
                      recovery_handoff_settle_sec_)
                    {
                      last_correction_end_ = now;
                      continuous_recovery_started_ = false;
                      recovery_rejected_candidate_mask_ = 0;
                      recovery_rejected_direction_sign_ = 0;
                      recovery_candidate_reselect_pending_ = false;
                      recovery_geometry_reestimate_attempted_ = false;
                      recovery_geometry_reestimate_pending_ = false;
                      recovery_circle_event_locked_ = false;
                      recovery_preferred_pair_locked_ = false;
                      // This branch is reached only after the final pressure,
                      // zero-thrust, low-rate and attitude dwell have all
                      // succeeded.  Do not carry the stricter trigger which
                      // originally requested recovery back into GRASP: the
                      // 2026-10-29 run completed at 4.66 deg, then immediately
                      // restarted because the stale 3-deg trigger remained
                      // armed.  GRASP may still start a new independent
                      // recovery at the ordinary 10-deg/hard-roll boundary.
                      continuous_recovery_waiting_for_settle_ = false;
                      action_c_requested_recovery_trigger_rad_ =
                          std::numeric_limits<double>::quiet_NaN();
                      recovery_handoff_integral_n_ = 0.0;
                      recovery_handoff_brake_error_rad_ = 0.0;
                      recovery_handoff_braking_ = false;
                      // Preserve the measured contact equilibrium through the
                      // deflated endpoint and the automatically resumed gait.
                      // A new operator command resets it to nominal pressure.
                      grasp_pressure_ = recovery_handoff_target_pressure_;
                      recovery_contact_grasp_active_ = true;
                      if (hybridLocomotionActive() && action_c_resume_pending_)
                        {
                          hybrid_grasp_pressure_ = grasp_pressure_;
                          hybrid_hold_pressure_ = true;
                        }
                      ROS_INFO("Action C recovery handoff complete: contact-conditioned grasp [%.1f %.1f %.1f %.1f] kPa with roll %.2f deg and rate %.3f rad/s",
                               grasp_pressure_[0], grasp_pressure_[1],
                               grasp_pressure_[2], grasp_pressure_[3],
                               roll_error * 180.0 / M_PI,
                               recovery_roll_rate);
                      enterState(GRASP, now);
                      break;
                    }
                }
              else
                ready_start_ = ros::Time(0);

              // Do not reopen a partly regrasped pair merely because the
              // normal 10-deg recovery threshold is crossed.  In MuJoCo this
              // 26 -> 0 kPa discontinuity, rather than the residual roll,
              // caused the physical fall.  Pressure is already frozen above
              // the acceptance band; retain the current contact geometry and
              // let the bounded rate PI bring roll back before continuing.
              if ((now - state_start_).toSec() >=
                  recovery_handoff_timeout_sec_)
                {
                  continuous_recovery_waiting_for_settle_ = true;
                  continuous_recovery_started_ = false;
                  resetRecoveryHandoffState();
                  prepare_shape_thrust_.fill(0.0);
                  prepare_pressure_ = grasp_pressure_;
                  publishCorrectionThrust(prepare_shape_thrust_);
                  publishPressureTarget(prepare_pressure_,
                      std::numeric_limits<double>::infinity());
                  enterState(GRASP, now);
                  ROS_WARN("Action C recovery handoff timed out after %.1f s; safe grasp commanded and automatic measured-roll retry retained",
                           recovery_handoff_timeout_sec_);
                  break;
                }
              ROS_INFO_THROTTLE(0.5,
                  "Action C closed-loop regrasp: roll %.2f deg rate %.3f rad/s, pressure [%.1f %.1f %.1f %.1f] kPa, pair thrust %.2f N, support_equalized=%d pressure_advance=%d ready=%d",
                  roll_error * 180.0 / M_PI, recovery_roll_rate,
                  prepare_pressure_[0], prepare_pressure_[1],
                  prepare_pressure_[2], prepare_pressure_[3],
                  handoff_target, support_transfer_equalized,
                  pressure_advance_allowed, pressure_ready);
              break;
            }

          // Restore contact first while removing rotor support gradually.
          // Sensor/IMU faults and angular-rate violations remain hard stops.
          if (!body_imu_fresh || !pressure_valid || !neurons_fresh ||
              (continuous_c_recovery && (!std::isfinite(recovery_roll_rate) ||
               std::abs(body_angular_velocity_[0]) >= correction_max_angular_rate_)) ||
              angular_rate >= correction_max_angular_rate_)
            prepare_shape_thrust_.fill(0.0);
          else
            for (double& thrust : prepare_shape_thrust_)
              thrust = std::max(0.0,
                  thrust - correction_thrust_down_rate_ * dt);
          publishCorrectionThrust(prepare_shape_thrust_);
          // Recovery must command the grasp pressure immediately. Applying
          // the PREPARE slew limit here can leave the old high-pressure
          // target active for several seconds even though thrust has stopped.
          publishPressureTarget(grasp_pressure_,
                                std::numeric_limits<double>::infinity());
          {
            bool recovered = pressure_valid;
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              recovered = recovered && std::abs(measured_pressure_[arm] - grasp_pressure_[arm]) <=
                                       pressure_tolerance_kpa_;
            if (continuous_c_recovery)
              recovered = pressure_valid && actionCPressureReady(grasp_pressure_, now);
            const bool thrust_stopped = std::all_of(
                prepare_shape_thrust_.begin(), prepare_shape_thrust_.end(),
                [](double thrust) { return thrust <= 0.01; });
            if (recovered && thrust_stopped)
              {
                last_correction_end_ = now;
                enterState(GRASP, now);
              }
            else if ((now - state_start_).toSec() >= recover_timeout_)
              ROS_WARN_THROTTLE(1.0,
                  "RECOVER waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                  grasp_pressure_[0], pressure_tolerance_kpa_);
          }
          break;
        }
      return;
    }

  const bool imu_fresh = imu_received_ &&
                         (now - imu_stamp_).toSec() <= imu_timeout_sec_;
  const bool thrust_fresh = thrust_received_ &&
                            (now - thrust_stamp_).toSec() <= thrust_timeout_sec_;
  if (!enabled_ || !target_received_ || !imu_fresh || !thrust_fresh)
    {
      if (enabled_ && (!imu_fresh || !thrust_fresh))
        {
          enabled_ = false;
          ROS_ERROR("Moment-based allocation latched off by IMU or thrust timeout");
        }
      return;
    }

  const Vector2 desired = desiredThrustMoment();
  const PressureArray allocated = allocatePressure(desired);
  std_msgs::Float32MultiArray target;
  target.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double change = std::max(-maximum_step,
          std::min(maximum_step, allocated[arm] - last_pressure_[arm]));
      last_pressure_[arm] = std::max(base_pressure_kpa_,
          std::min(maximum_pressure_kpa_, last_pressure_[arm] + change));
      target.data[arm] = last_pressure_[arm];
    }
  pressure_target_pub_.publish(target);

  const Vector2 achieved = thrustMoment(last_pressure_);
  geometry_msgs::Vector3Stamped desired_msg, achieved_msg;
  desired_msg.header.stamp = achieved_msg.header.stamp = now;
  desired_msg.vector.x = desired[0]; desired_msg.vector.y = desired[1];
  achieved_msg.vector.x = achieved[0]; achieved_msg.vector.y = achieved[1];
  desired_moment_pub_.publish(desired_msg);
  achieved_moment_pub_.publish(achieved_msg);
}

bool AttitudePressureController::correctionRollOnly() const
{
  // Preserve the roll-only Action-C handoff even in the optional two-axis
  // configuration. The default also covers GRASP before/after locomotion.
  return correction_roll_only_ || action_c_resume_pending_;
}

AttitudePressureController::Vector2
AttitudePressureController::desiredThrustMoment() const
{
  Vector2 desired{{
    roll_moment_kp_ * wrapAngle(target_roll_ - roll_) -
      roll_moment_kd_ * body_angular_velocity_[0],
    pitch_moment_kp_ * wrapAngle(target_pitch_ - pitch_) -
      pitch_moment_kd_ * body_angular_velocity_[1]
  }};
  if (gravity_compensation_)
    {
      Vector3 arm_to_com;
      Vector3 gravity_force;
      for (size_t axis = 0; axis < 3; ++axis)
        {
          arm_to_com[axis] = center_of_mass_[axis] - contact_position_[axis];
          gravity_force[axis] = mass_kg_ * gravity_body_[axis];
        }
      const Vector3 gravity_moment = cross(arm_to_com, gravity_force);
      desired[0] -= gravity_moment[0];
      desired[1] -= gravity_moment[1];
    }
  // Mask AFTER gravity compensation, including the PREPARE reallocation
  // path. Zero pitch gains alone would leave a gravity-induced pitch moment.
  if (correctionRollOnly()) desired[1] = 0.0;
  return desired;
}

AttitudePressureController::PressureArray
AttitudePressureController::allocatePressure(const Vector2& desired) const
{
  // A free four-variable least-squares solution can realize a roll or pitch
  // request with a single arm.  That is undesirable while perched: a pure
  // body-axis correction should load the two arms on the same side of the
  // human arm symmetrically.  Select the pair from the dominant attitude-error
  // axis and constrain both members to one common pressure.
  using ArmPair = std::array<size_t, 2>;
  const double roll_error = std::abs(wrapAngle(target_roll_ - roll_));
  const double pitch_error = std::abs(wrapAngle(target_pitch_ - pitch_));
  const bool use_roll_pairs = correctionRollOnly() || roll_error >= pitch_error;
  const std::array<ArmPair, 2> candidate_pairs = use_roll_pairs
      ? std::array<ArmPair, 2>{{ArmPair{{0, 1}}, ArmPair{{2, 3}}}}
      : std::array<ArmPair, 2>{{ArmPair{{0, 3}}, ArmPair{{1, 2}}}};

  PressureArray best{{base_pressure_kpa_, base_pressure_kpa_,
                      base_pressure_kpa_, base_pressure_kpa_}};
  double best_cost = std::numeric_limits<double>::infinity();
  const double pressure_step = std::max(0.05, allocation_pressure_epsilon_kpa_);

  for (const ArmPair& pair : candidate_pairs)
    {
      for (double common_pressure = base_pressure_kpa_;
           common_pressure <= maximum_pressure_kpa_ + 1e-9;
           common_pressure += pressure_step)
        {
          PressureArray pressure{{base_pressure_kpa_, base_pressure_kpa_,
                                  base_pressure_kpa_, base_pressure_kpa_}};
          pressure[pair[0]] = pressure[pair[1]] =
              std::min(common_pressure, maximum_pressure_kpa_);
          const Vector2 moment = thrustMoment(pressure);
          const double roll_residual = desired[0] - moment[0];
          const double pitch_residual = correctionRollOnly()
              ? 0.0 : desired[1] - moment[1];
          // The tiny final term only selects the lower-pressure solution when
          // two table samples produce effectively the same moment error.
          const double pressure_delta = common_pressure - base_pressure_kpa_;
          const double cost = roll_residual * roll_residual +
                              pitch_residual * pitch_residual +
                              1e-12 * pressure_delta * pressure_delta;
          if (cost < best_cost)
            {
              best_cost = cost;
              best = pressure;
            }
        }
    }
  return best;
}

AttitudePressureController::PressureArray
AttitudePressureController::allocateCorrectionThrust(
    const Vector2& desired, const PressureArray& pressure) const
{
  PressureArray bend_angle;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    bend_angle[arm] = bendAngle(arm, pressure[arm]);
  return allocateCorrectionThrustFromAngles(desired, bend_angle);
}

void AttitudePressureController::selectContinuousRecoveryPair(
    double roll_error)
{
  using Pair = std::array<size_t, 2>;
  const std::array<Pair, 2> pairs{{Pair{{0, 1}}, Pair{{2, 3}}}};
  struct Candidate {
    bool valid = false;
    bool rejected = false;
    size_t pair = 0;
    size_t pressure_index = 0;
    bool release = true;
    double pressure = 0.0;
    double release_fraction = 1.0;
    double lever = 0.0;
    double support_loss_moment = 0.0;
    double required_thrust = std::numeric_limits<double>::infinity();
    double depth = std::numeric_limits<double>::quiet_NaN();
    double score = std::numeric_limits<double>::infinity();
    Vector3 pivot{{0.0, 0.0, 0.0}};
  };

  recovery_selected_path_requires_zero_thrust_ = false;
  recovery_selected_path_min_lever_m_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_since_ = ros::Time(0);
  recovery_wrong_geometry_transition_bad_since_ = ros::Time(0);
  recovery_candidate_retained_worsening_since_ = ros::Time(0);
  recovery_wrong_geometry_pressure_settled_since_ = ros::Time(0);
  recovery_wrong_geometry_reference_error_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_reference_rate_rad_s_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_wrong_geometry_best_rate_rad_s_ =
      -std::numeric_limits<double>::infinity();
  recovery_wrong_geometry_response_observed_ = false;

  const bool circle_directly_fresh = recovery_circle_estimate_.valid &&
      recovery_circle_estimate_.confidence >= recovery_circle_min_confidence_ &&
      !recovery_circle_stamp_.isZero() &&
      (ros::Time::now() - recovery_circle_stamp_).toSec() <=
          recovery_circle_max_age_sec_;
  if (circle_directly_fresh) recovery_circle_event_locked_ = true;
  // The object radius does not change inside one recovery event.  A partial
  // load-bearing regrasp can temporarily make a new four-contact circle fit
  // ill-conditioned, but current arm bends and pivots are still recomputed at
  // every selection. Keep the radius estimate which was verified at event
  // entry instead of silently reverting to the fixed rotor mapping at 30 s.
  const bool circle_fresh = recovery_circle_estimate_.valid &&
      recovery_circle_estimate_.confidence >= recovery_circle_min_confidence_ &&
      (circle_directly_fresh || recovery_circle_event_locked_);
  std::array<double, ARM_COUNT> recovery_depth_angle =
      recovery_circle_estimate_.depth_angle;
  std::array<double, ARM_COUNT> downward_projection{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  if (circle_fresh)
    {
      const double gravity_norm = std::hypot(
          gravity_body_[1], gravity_body_[2]);
      if (std::isfinite(gravity_norm) && gravity_norm > 1.0e-6)
        {
          const double down_y = gravity_body_[1] / gravity_norm;
          const double down_z = gravity_body_[2] / gravity_norm;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              const double bend = measuredArmBendAngle(arm);
              if (!std::isfinite(bend)) continue;
              const Vector3 contact = armContactPointFromAngle(arm, bend);
              const double radial_y =
                  contact[1] - recovery_circle_estimate_.center_y;
              const double radial_z =
                  contact[2] - recovery_circle_estimate_.center_z;
              const double radial_norm = std::hypot(radial_y, radial_z);
              if (!std::isfinite(radial_norm) || radial_norm < 1.0e-6)
                continue;
              downward_projection[arm] =
                  (radial_y * down_y + radial_z * down_z) / radial_norm;
              // An IMU angle can be absent when its chamber is below the
              // contact-pressure gate even though the arm kinematics remain
              // available. Reconstruct that depth from the stored circle,
              // the current contact point and gravity instead of deciding
              // from the partner arm alone.
              if (!std::isfinite(recovery_depth_angle[arm]))
                recovery_depth_angle[arm] = std::acos(hugmy::circularClamp(
                    -downward_projection[arm], -1.0, 1.0));
            }
        }

      if (!recovery_preferred_pair_locked_)
        {
          // Hardware convention verified in the arm fixture: negative body
          // roll places arms 0+1 lower; positive roll places arms 2+3 lower.
          // This is used only when neither two-arm depth nor complete
          // gravity-projected contact evidence is available.
          const int roll_lower_pair = roll_ < 0.0 ? 0 : 1;
          const hugmy::CircularRecoveryPairChoice pair_choice =
              hugmy::chooseCircularRecoveryPair(
                  recovery_depth_angle, downward_projection,
                  roll_lower_pair, 0.5 * M_PI,
                  recovery_circle_pair_depth_ambiguity_rad_);
          if (pair_choice.valid)
            {
              recovery_preferred_pair_index_ = pair_choice.pair_index;
              recovery_preferred_pair_used_fallback_ =
                  pair_choice.used_gravity_or_roll_fallback;
              recovery_preferred_pair_locked_ = true;
              ROS_INFO("Circular recovery locked lower-side pair %zu+%zu for this event: pair depths [%.1f %.1f] deg, source=%s",
                       pair_choice.pair_index == 0 ? 0ul : 2ul,
                       pair_choice.pair_index == 0 ? 1ul : 3ul,
                       std::isfinite(pair_choice.pair_depth[0])
                           ? pair_choice.pair_depth[0] * 180.0 / M_PI : -1.0,
                       std::isfinite(pair_choice.pair_depth[1])
                           ? pair_choice.pair_depth[1] * 180.0 / M_PI : -1.0,
                       pair_choice.used_gravity_or_roll_fallback
                           ? "gravity/roll fallback" : "complete pair depth");
            }
        }
    }
  const double required_moment = roll_moment_kp_ * std::max(0.0,
      std::abs(roll_error) - roll_recovery_config_.stop);
  const double fixed_mapping_sign = correction_pair_reverse_spinal_mapping_
      ? -1.0 : 1.0;
  const double correction_sign = roll_error >= 0.0 ? 1.0 : -1.0;
  const int correction_direction = roll_error >= 0.0 ? 1 : -1;
  // Candidate rejection is evidence for one requested moment direction.
  // Regrasp can move the contacts across the cylinder side and reverse both
  // the roll error and the effective rotor moment. Carrying the old mask into
  // that new geometry can reject its only restoring candidate. Clear it only
  // after the required moment sign changes; same-direction retries still
  // cannot repeat a disproved pair/contact state.
  if (recovery_rejected_candidate_mask_ != 0 &&
      recovery_rejected_direction_sign_ != 0 &&
      recovery_rejected_direction_sign_ != correction_direction)
    {
      ROS_INFO("Circular recovery required-moment sign changed after regrasp; clearing directional rejection mask 0x%02x",
               recovery_rejected_candidate_mask_);
      recovery_rejected_candidate_mask_ = 0;
      recovery_rejected_direction_sign_ = 0;
    }
  const auto nominal_pressure = [&](size_t arm) {
    return hugmy::recoveryClamp(
        std::max(base_pressure_kpa_, grasp_pressure_[arm]),
        minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
  };
  const auto selected_pressure = [&](size_t arm, double requested) {
    return hugmy::recoveryClamp(requested,
        minimum_prepare_pressure_kpa_, nominal_pressure(arm));
  };
  auto evaluate = [&](size_t pair_index, size_t pressure_index) {
    Candidate candidate;
    candidate.pair = pair_index;
    candidate.pressure_index = pressure_index;
    candidate.pressure = recovery_circle_pressure_candidates_kpa_[pressure_index];
    double release_fraction_sum = 0.0;
    for (const size_t arm : pairs[pair_index])
      {
        const double nominal = nominal_pressure(arm);
        const double target = selected_pressure(arm, candidate.pressure);
        release_fraction_sum += (nominal - target) /
            std::max(1.0e-6, nominal - minimum_prepare_pressure_kpa_);
      }
    candidate.release_fraction = hugmy::recoveryClamp(
        release_fraction_sum / static_cast<double>(pairs[pair_index].size()),
        0.0, 1.0);
    candidate.release = candidate.release_fraction > 1.0e-3;
    const uint8_t candidate_bit = static_cast<uint8_t>(
        1u << (4u * pair_index + pressure_index));
    candidate.rejected =
        (recovery_rejected_candidate_mask_ & candidate_bit) != 0;
    std::array<bool, ARM_COUNT> selected_pair{{false, false, false, false}};
    for (const size_t arm : pairs[pair_index]) selected_pair[arm] = true;
    // Predict the contact load distribution at each pressure candidate.
    // Pressure is used only as a monotone proxy for normal load.  The CoP is
    // then the pressure-weighted mean of every load-bearing contact: at
    // 0 kPa this naturally becomes the opposite-pair anchor, whereas 10/20
    // kPa retain a partial contribution from the correcting pair.
    PressureArray candidate_pressure = measured_pressure_;
    PressureArray candidate_bend{};
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      candidate_bend[arm] = measuredArmBendAngle(arm);
    for (const size_t arm : pairs[pair_index])
      {
        candidate_pressure[arm] = selected_pressure(arm, candidate.pressure);
        const double model_bend = bend_model_.angleRad(
            candidate_pressure[arm],
            correction_planning_thrust_n_ + bend_model_thrust_offset_n_);
        if (std::isfinite(model_bend))
          candidate_bend[arm] = std::min(candidate_bend[arm], model_bend);
      }
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      if (!selected_pair[arm]) candidate_pressure[arm] = nominal_pressure(arm);
    candidate.pivot = recoveryContactPivot(
        selected_pair, candidate.release, &candidate_pressure, &candidate_bend);
    double minimum_selected_pressure = std::numeric_limits<double>::infinity();
    for (const size_t arm : pairs[pair_index])
      minimum_selected_pressure = std::min(
          minimum_selected_pressure, candidate_pressure[arm]);
    const bool selected_contact_retained = minimum_selected_pressure >=
        recovery_circle_contact_pressure_kpa_;
    // Releasing the geometrically lower pair also removes part of the
    // load-bearing contact reaction.  The rotor cross-product alone therefore
    // over-estimates the useful restoring moment and selected ~0.1-N commands
    // in the 2026-10-28 hardware trial.  Pressure is not a force sensor, but
    // its normalized removed share provides a bounded first-order estimate of
    // this adverse support-transfer moment.  Only the component opposing the
    // requested correction is charged to the candidate.
    if (candidate.release && recovery_circle_release_support_scale_ > 0.0)
      {
        double pressure_sum = 0.0;
        for (size_t arm = 0; arm < ARM_COUNT; ++arm)
          if (std::isfinite(measured_pressure_[arm]))
            pressure_sum += std::max(0.0, measured_pressure_[arm]);
        if (pressure_sum > 1.0e-6)
          {
            double projected_removed_support_moment = 0.0;
            for (const size_t arm : pairs[pair_index])
              {
                const double bend = measuredArmBendAngle(arm);
                if (!std::isfinite(bend) ||
                    !std::isfinite(measured_pressure_[arm]))
                  continue;
                const double release_target =
                    selected_pressure(arm, candidate.pressure);
                const double removed_share = std::max(
                    0.0, measured_pressure_[arm] - release_target) /
                    pressure_sum;
                const Vector3 contact = armContactPointFromAngle(arm, bend);
                Vector3 lever, removed_force;
                for (size_t axis = 0; axis < 3; ++axis)
                  {
                    lever[axis] = contact[axis] - candidate.pivot[axis];
                    // Loss of an upward support reaction is equivalent to a
                    // bounded added force in the measured gravity direction.
                    removed_force[axis] = mass_kg_ * gravity_body_[axis] *
                        removed_share * recovery_circle_release_support_scale_;
                  }
                projected_removed_support_moment += correction_sign *
                    cross(lever, removed_force)[0];
              }
            candidate.support_loss_moment = std::max(
                0.0, -projected_removed_support_moment);
          }
      }
    double depth_sum = 0.0;
    size_t depth_count = 0;
    for (const size_t arm : pairs[pair_index])
      {
        const double measured_bend = measuredArmBendAngle(arm);
        if (!std::isfinite(measured_bend)) return candidate;
        const double evaluated_bend = candidate_bend[arm];
        // A released arm uses the empirically verified actuator-axis mapping:
        // once straightened, it no longer has a loaded contact reaction that
        // can redirect its rotor force.  A flexed/grasped arm is different;
        // its contact location on the inferred circle determines the moment
        // sign, so applying the fixed mapping again inverted deep (>90-deg)
        // contacts.  Keep these two physical cases separate.
        const double actuator_sign = selected_contact_retained
            ? 1.0 : fixed_mapping_sign;
        candidate.lever += actuator_sign * correction_sign *
            recoveryRotorRawLeverAbout(
                arm, evaluated_bend, candidate.pivot);
        if (circle_fresh && std::isfinite(recovery_depth_angle[arm]))
          {
            depth_sum += recovery_depth_angle[arm];
            ++depth_count;
          }
      }
    candidate.depth = depth_count > 0
        ? depth_sum / static_cast<double>(depth_count)
        : std::numeric_limits<double>::quiet_NaN();
    const bool below_or_at_side = std::isfinite(candidate.depth) &&
        candidate.depth >= 0.5 * M_PI - recovery_circle_side_margin_rad_;
    // Retaining the full nominal grasp is allowed only for a small correction
    // from a geometrically useful below-side contact.  Partial 10/20-kPa
    // candidates remain in the search: their contact reaction and shifted
    // CoP are already represented in the signed lever, so rejecting them here
    // would collapse the new pressure search back to the old binary choice.
    const bool flexed_error_safe =
        std::abs(roll_error) <= recovery_circle_flexed_max_error_rad_;
    const bool full_grasp_candidate = candidate.release_fraction <= 1.0e-3;
    if (full_grasp_candidate && selected_contact_retained &&
        (!circle_fresh || !below_or_at_side || !flexed_error_safe))
      return candidate;
    const double usable_lever = std::max(
        recovery_circle_min_signed_lever_m_, roll_recovery_config_.min_lever);
    if (!std::isfinite(candidate.lever) ||
        candidate.lever < usable_lever) return candidate;
    candidate.required_thrust =
        (required_moment + candidate.support_loss_moment) /
        std::max(candidate.lever, usable_lever);
    candidate.score = candidate.required_thrust +
        recovery_circle_release_penalty_n_ * candidate.release_fraction;
    candidate.valid = !candidate.rejected &&
        std::isfinite(candidate.score) &&
        candidate.required_thrust <= 1.25 * roll_recovery_config_.limit;
    return candidate;
  };

  recovery_pair_selection_available_ = false;
  Candidate selected;
  if (recovery_circle_selection_enabled_ && circle_fresh)
    {
      for (size_t pair = 0; pair < pairs.size(); ++pair)
        {
          if (recovery_preferred_pair_locked_ &&
              pair != recovery_preferred_pair_index_)
            continue;
          for (size_t pressure_index = 0;
               pressure_index < recovery_circle_pressure_candidates_kpa_.size();
               ++pressure_index)
            {
              const Candidate candidate = evaluate(pair, pressure_index);
              ROS_INFO("Circular recovery candidate: pair %zu+%zu pressure=%.1f kPa (%s), depth %.1f deg, CoP yz=[%.1f %.1f] mm, signed lever %.4f m, support-loss %.3f Nm, initial FF %.2f N/rotor, rejected=%d, valid=%d, score %.2f",
                       pairs[pair][0], pairs[pair][1],
                       candidate.pressure,
                       candidate.release ? "partially released" : "grasped",
                       std::isfinite(candidate.depth) ?
                           candidate.depth * 180.0 / M_PI : -1.0,
                       1000.0 * candidate.pivot[1],
                       1000.0 * candidate.pivot[2],
                       candidate.lever,
                       candidate.support_loss_moment,
                       candidate.required_thrust,
                       candidate.rejected ? 1 : 0,
                       candidate.valid ? 1 : 0,
                       candidate.score);
              if (candidate.valid && candidate.score < selected.score)
                selected = candidate;
            }
        }
    }

  // The inferred lower-side pair is the preferred mechanical support, but it
  // is not allowed to become a dead end.  In the attached cylinder trial all
  // four pressure states of the locked pair had a negative signed lever, so
  // the old code aborted recovery without ever evaluating the other pair.
  // If (and only if) the preferred pair has no restoring candidate, evaluate
  // the opposite pair with the same CoP/support-loss model.  This is still a
  // model-checked choice, not an unconditional upper-pair switch.  Mid-pass
  // changes arrive here only after the existing zero-thrust, balanced-support
  // handoff has made pair switching safe.
  if (!selected.valid && recovery_circle_selection_enabled_ && circle_fresh &&
      recovery_preferred_pair_locked_)
    {
      const size_t alternate_pair = 1 - recovery_preferred_pair_index_;
      Candidate alternate;
      for (size_t pressure_index = 0;
           pressure_index < recovery_circle_pressure_candidates_kpa_.size();
           ++pressure_index)
        {
          const Candidate option = evaluate(alternate_pair, pressure_index);
          ROS_INFO("Circular recovery alternate candidate: pair %zu+%zu pressure=%.1f kPa, signed lever %.4f m, support-loss %.3f Nm, initial FF %.2f N/rotor, rejected=%d, valid=%d, score %.2f",
                   pairs[alternate_pair][0], pairs[alternate_pair][1],
                   option.pressure, option.lever,
                   option.support_loss_moment, option.required_thrust,
                   option.rejected ? 1 : 0, option.valid ? 1 : 0,
                   option.score);
          if (option.valid && option.score < alternate.score)
            alternate = option;
        }
      if (alternate.valid)
        {
          selected = alternate;
          recovery_preferred_pair_index_ = alternate_pair;
          recovery_preferred_pair_used_fallback_ = true;
          ROS_WARN("Locked lower-side pair had no restoring candidate; safely selecting model-restoring alternate pair %zu+%zu after support handoff",
                   pairs[alternate_pair][0], pairs[alternate_pair][1]);
        }
    }

  if (selected.valid)
    {
      if (continuous_recovery_use_opposite_pair_ &&
          !recovery_preferred_pair_locked_)
        {
          const size_t opposite = 1 - selected.pair;
          Candidate alternate;
          for (size_t pressure_index = 0;
               pressure_index < recovery_circle_pressure_candidates_kpa_.size();
               ++pressure_index)
            {
              const Candidate option = evaluate(opposite, pressure_index);
              if (option.valid && option.score < alternate.score)
                alternate = option;
            }
          if (alternate.valid) selected = alternate;
          else
            {
              selected.pair = opposite;
              selected.release = true;
              selected.pressure_index = 0;
              selected.pressure = recovery_circle_pressure_candidates_kpa_[0];
              selected.depth = std::numeric_limits<double>::quiet_NaN();
            }
          ROS_WARN("Continuous recovery uses the opposite circle-model pair after a measured wrong-way response");
        }
      // Check the complete discrete pressure path, not only the selected
      // endpoint.  The real chamber traverses 30 -> 20 -> 10 -> 0 kPa and
      // may temporarily put the rotor on the wrong side of its moment-arm
      // sign.  Keep pressure motion, but withhold rotor thrust until the
      // measured lever becomes restoring.  An endpoint whose own lever is
      // wrong was already rejected by evaluate().
      if (selected.release)
        {
          double current_pair_pressure = 0.0;
          size_t current_pressure_count = 0;
          for (const size_t arm : pairs[selected.pair])
            if (std::isfinite(measured_pressure_[arm]))
              {
                current_pair_pressure += measured_pressure_[arm];
                ++current_pressure_count;
              }
          if (current_pressure_count > 0)
            current_pair_pressure /=
                static_cast<double>(current_pressure_count);
          const double usable_lever = std::max(
              recovery_circle_min_signed_lever_m_,
              roll_recovery_config_.min_lever);
          double path_min_lever = selected.lever;
          for (size_t pressure_index = 0;
               pressure_index <
                   recovery_circle_pressure_candidates_kpa_.size();
               ++pressure_index)
            {
              const double pressure =
                  recovery_circle_pressure_candidates_kpa_[pressure_index];
              if (pressure + 1.0e-6 < selected.pressure ||
                  (current_pressure_count > 0 &&
                   pressure > current_pair_pressure +
                       roll_recovery_config_.stable_pressure))
                continue;
              const Candidate path_state =
                  evaluate(selected.pair, pressure_index);
              if (std::isfinite(path_state.lever))
                path_min_lever = std::min(
                    path_min_lever, path_state.lever);
              else
                path_min_lever =
                    -std::numeric_limits<double>::infinity();
            }
          recovery_selected_path_min_lever_m_ = path_min_lever;
          recovery_selected_path_requires_zero_thrust_ =
              !std::isfinite(path_min_lever) ||
              path_min_lever < usable_lever;
          if (recovery_selected_path_requires_zero_thrust_)
            ROS_WARN("Circular recovery pressure path for pair %zu+%zu crosses a non-restoring state (minimum signed lever %.4f m); exhaust with zero rotor thrust until measured geometry is restoring",
                     pairs[selected.pair][0], pairs[selected.pair][1],
                     path_min_lever);
        }
      prepare_arm_active_.fill(false);
      for (const size_t arm : pairs[selected.pair]) prepare_arm_active_[arm] = true;
      recovery_selected_pair_index_ = selected.pair;
      recovery_selected_pair_release_ = selected.release;
      recovery_selected_pair_pressure_kpa_ = selected.pressure;
      recovery_selected_pressure_candidate_index_ = selected.pressure_index;
      recovery_selected_contact_pivot_enabled_ = true;
      recovery_pair_selection_available_ = true;
      recovery_selected_pair_mean_depth_rad_ = selected.depth;
      recovery_selected_support_loss_moment_nm_ =
          selected.support_loss_moment;
      recovery_selected_release_start_pressure_ = measured_pressure_;
      ROS_INFO("Circular recovery selection: radius %.1f mm (rms %.1f mm, confidence %.2f), pair %zu+%zu, contact depth %.1f deg (%s side), target pressure %.1f kPa, CoP yz=[%.1f %.1f] mm, signed lever %.3f m, support-loss %.3f Nm",
               1000.0 * recovery_circle_estimate_.radius,
               1000.0 * recovery_circle_estimate_.rms,
               recovery_circle_estimate_.confidence,
               pairs[selected.pair][0], pairs[selected.pair][1],
               std::isfinite(selected.depth) ?
                   selected.depth * 180.0 / M_PI : -1.0,
               std::isfinite(selected.depth) && selected.depth >= 0.5 * M_PI
                   ? "below" : "above",
               selected.pressure,
               1000.0 * selected.pivot[1], 1000.0 * selected.pivot[2],
               selected.lever, selected.support_loss_moment);
      return;
    }

  // Neither pair has a model-valid restoring contact state.  Do not apply a
  // rotor command with the wrong signed lever; keep support and wait for a
  // fresh contact estimate instead.
  if (recovery_circle_selection_enabled_ && circle_fresh &&
      recovery_preferred_pair_locked_)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      ROS_ERROR("No restoring moment candidate remains in either pair after preferred pair %zu+%zu was exhausted",
                recovery_preferred_pair_index_ == 0 ? 0ul : 2ul,
                recovery_preferred_pair_index_ == 0 ? 1ul : 3ul);
      return;
    }

  // A fresh circular estimate plus a non-empty rejection set means the
  // measured-response supervisor has already disproved at least one model
  // candidate in this posture event.  If none of the remaining candidates
  // has a restoring signed lever, do not silently fall back to the fixed pair
  // and repeat an actuator state which has already failed.
  if (recovery_circle_selection_enabled_ && circle_fresh &&
      recovery_rejected_candidate_mask_ != 0)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      ROS_ERROR("No unused circular recovery candidate remains for rejected mask 0x%02x",
                recovery_rejected_candidate_mask_);
      return;
    }

  // If a circle cannot be inferred, do not blindly use the historical fixed
  // pair.  The attached hardware bag had valid bends but an invalid circle;
  // the nominal pair's instantaneous signed lever was negative throughout
  // its slow exhaust, while the opposite pair corrected immediately after
  // regrasp. Compare both measured pair levers and choose only a currently
  // restoring pair. If neither is restorative, preserve four-arm support and
  // wait for a new contact estimate instead of applying a wrong-sign force.
  std::array<double, 2> measured_pair_lever{{0.0, 0.0}};
  std::array<bool, 2> measured_pair_valid{{true, true}};
  for (size_t pair = 0; pair < pairs.size(); ++pair)
    for (const size_t arm : pairs[pair])
      {
        const double bend = measuredArmBendAngle(arm);
        if (!std::isfinite(bend))
          {
            measured_pair_valid[pair] = false;
            continue;
          }
        measured_pair_lever[pair] += fixed_mapping_sign *
            correction_axis_[0] * recoveryRotorRawLeverAbout(
                arm, bend, contact_position_);
      }
  const double usable_measured_lever = std::max(
      recovery_circle_min_signed_lever_m_, roll_recovery_config_.min_lever);
  size_t measured_pair = measured_pair_lever[1] > measured_pair_lever[0]
      ? 1 : 0;
  if (continuous_recovery_use_opposite_pair_)
    measured_pair = 1 - recovery_selected_pair_index_;
  const bool measured_pair_restoring = measured_pair_valid[measured_pair] &&
      std::isfinite(measured_pair_lever[measured_pair]) &&
      measured_pair_lever[measured_pair] >= usable_measured_lever;
  if (!measured_pair_restoring)
    {
      prepare_arm_active_.fill(false);
      recovery_selected_contact_pivot_enabled_ = false;
      recovery_selected_support_loss_moment_nm_ = 0.0;
      recovery_pair_selection_available_ = false;
      ROS_ERROR("Circle recovery estimate is unavailable and neither permitted instantaneous pair is restoring: lever 0+1 %.4f m (valid=%d), 2+3 %.4f m (valid=%d), requested retry=%d",
                measured_pair_lever[0], measured_pair_valid[0] ? 1 : 0,
                measured_pair_lever[1], measured_pair_valid[1] ? 1 : 0,
                continuous_recovery_use_opposite_pair_ ? 1 : 0);
      return;
    }
  prepare_arm_active_ = measured_pair == 0
      ? std::array<bool, ARM_COUNT>{{true, true, false, false}}
      : std::array<bool, ARM_COUNT>{{false, false, true, true}};
  recovery_selected_pair_release_ = true;
  recovery_selected_pair_pressure_kpa_ = recovery_release_pressure_kpa_;
  recovery_selected_pressure_candidate_index_ = 0;
  recovery_selected_contact_pivot_enabled_ = false;
  recovery_selected_pair_index_ = measured_pair;
  recovery_pair_selection_available_ = true;
  recovery_selected_pair_mean_depth_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  recovery_selected_support_loss_moment_nm_ = 0.0;
  recovery_selected_release_start_pressure_ = measured_pressure_;
  ROS_WARN("Circle recovery estimate unavailable; selected instantaneous restoring pair %zu+%zu from measured signed levers [%.4f %.4f] m",
           pairs[measured_pair][0], pairs[measured_pair][1],
           measured_pair_lever[0], measured_pair_lever[1]);
}

AttitudePressureController::PressureArray
AttitudePressureController::recoveryReselectionPressure() const
{
  PressureArray target{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double nominal = hugmy::recoveryClamp(
          std::max(base_pressure_kpa_, grasp_pressure_[arm]),
          recovery_handoff_min_pressure_kpa_, maximum_pressure_kpa_);
      // A flexed/grasped candidate is already load bearing: lowering it to
      // the generic support floor would release the only useful small-radius
      // contact merely because its rotor moment was ineffective. A released
      // candidate, in contrast, must be brought back only to the minimum
      // contact pressure before geometry is selected again. The opposite
      // pair remains at nominal pressure as the mechanical anchor.
      target[arm] = prepare_arm_active_[arm] &&
                    recovery_selected_pair_release_
          ? std::min(std::max(recoverySupportPressure(),
                              recoverySelectedPairPressure(arm)), nominal)
          : nominal;
    }
  return target;
}

AttitudePressureController::PressureArray
AttitudePressureController::recoveryMeasuredContactPressure() const
{
  PressureArray target{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      // At a measured target crossing, the current chamber pressure is the
      // only four-arm pressure state known to support the attitude reached
      // by the measured response.  Freeze *all* chambers there before
      // recomputing the contact circle.  Restoring the nominal 30-kPa anchor
      // on the non-selected pair changed the contact geometry during the
      // handoff; on the 130-mm fixture it kicked a 1.7-deg target crossing
      // back to a 5--6-deg residual before the new pair could be selected.
      // Keep only the ordinary load-bearing floor as a safety bound.
      target[arm] = hugmy::recoveryClamp(
          measured_pressure_[arm], recovery_handoff_min_pressure_kpa_,
          maximum_pressure_kpa_);
    }
  return target;
}

double AttitudePressureController::recoverySupportPressure() const
{
  const bool small_radius_release =
      recovery_selected_contact_pivot_enabled_ &&
      recovery_selected_pair_release_ &&
      recovery_circle_estimate_.valid &&
      recovery_circle_estimate_.radius <= recovery_circle_small_radius_m_;
  return hugmy::recoveryClamp(
      small_radius_release
          ? std::max(recovery_handoff_min_pressure_kpa_,
                     recovery_circle_small_radius_support_pressure_kpa_)
          : recovery_handoff_min_pressure_kpa_,
      minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
}

double AttitudePressureController::recoverySelectedPairPressure(
    size_t arm) const
{
  if (arm >= ARM_COUNT) return minimum_prepare_pressure_kpa_;
  const double nominal = hugmy::recoveryClamp(
      std::max(base_pressure_kpa_, grasp_pressure_[arm]),
      minimum_prepare_pressure_kpa_, maximum_pressure_kpa_);
  return hugmy::recoveryClamp(recovery_selected_pair_pressure_kpa_,
      minimum_prepare_pressure_kpa_, nominal);
}

void AttitudePressureController::updateCircularGraspGeometry(
    const ros::Time& now)
{
  std::array<std::array<double, 2>, ARM_COUNT> contact_yz{};
  std::array<bool, ARM_COUNT> contact_available{{false, false, false, false}};
  const bool simulation_contact_fresh = grip_state_valid_ &&
      !grip_state_stamp_.isZero() &&
      (now - grip_state_stamp_).toSec() <= 0.25;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bend = measuredArmBendAngle(arm);
      const Vector3 contact = armContactPointFromAngle(arm, bend);
      contact_yz[arm] = {{contact[1], contact[2]}};
      contact_available[arm] = std::isfinite(bend) &&
          std::isfinite(measured_pressure_[arm]) &&
          measured_pressure_[arm] >= recovery_circle_contact_pressure_kpa_ &&
          (!simulation_contact_fresh || grip_active_[arm]);
    }
  const auto estimate = hugmy::estimateCircularGrasp(
      contact_yz, contact_available,
      {{contact_position_[1], contact_position_[2]}},
      {{gravity_body_[1], gravity_body_[2]}},
      recovery_circle_min_radius_m_, recovery_circle_max_radius_m_,
      recovery_circle_max_rms_m_);
  if (estimate.valid && estimate.confidence >=
      recovery_circle_min_confidence_)
    {
      recovery_circle_estimate_ = estimate;
      recovery_circle_stamp_ = now;
    }

  const double age = recovery_circle_stamp_.isZero()
      ? std::numeric_limits<double>::infinity()
      : (now - recovery_circle_stamp_).toSec();
  const bool usable = recovery_circle_estimate_.valid &&
      age <= recovery_circle_max_age_sec_;
  // Layout: usable, radius, center y/z, RMS, confidence, sample count, age,
  // four depth angles, selected pair (0=01, 1=23, -1=none), release flag,
  // and selected correcting-pair pressure [kPa].
  std_msgs::Float32MultiArray msg;
  msg.data.resize(15, std::numeric_limits<float>::quiet_NaN());
  msg.data[0] = usable ? 1.0f : 0.0f;
  msg.data[1] = static_cast<float>(recovery_circle_estimate_.radius);
  msg.data[2] = static_cast<float>(recovery_circle_estimate_.center_y);
  msg.data[3] = static_cast<float>(recovery_circle_estimate_.center_z);
  msg.data[4] = static_cast<float>(recovery_circle_estimate_.rms);
  msg.data[5] = static_cast<float>(recovery_circle_estimate_.confidence);
  msg.data[6] = static_cast<float>(recovery_circle_estimate_.samples);
  msg.data[7] = static_cast<float>(age);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    msg.data[8 + arm] = static_cast<float>(
        recovery_circle_estimate_.depth_angle[arm]);
  int selected_pair = -1;
  if (prepare_arm_active_[0] && prepare_arm_active_[1]) selected_pair = 0;
  if (prepare_arm_active_[2] && prepare_arm_active_[3]) selected_pair = 1;
  msg.data[12] = static_cast<float>(selected_pair);
  msg.data[13] = recovery_selected_pair_release_ ? 1.0f : 0.0f;
  msg.data[14] = static_cast<float>(recovery_selected_pair_pressure_kpa_);
  recovery_circle_geometry_pub_.publish(msg);
}

AttitudePressureController::PressureArray
AttitudePressureController::allocateCorrectionThrustFromAngles(
    const Vector2& desired, const PressureArray& bend_angle) const
{
  std::array<Vector2, ARM_COUNT> columns;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const Vector3 position = rotorPositionFromAngle(arm, bend_angle[arm]);
      const Vector3 direction = rotorDirectionFromAngle(arm, bend_angle[arm]);
      Vector3 lever;
      for (size_t axis = 0; axis < 3; ++axis)
        lever[axis] = position[axis] - contact_position_[axis];
      const Vector3 unit_moment = cross(lever, direction);
      columns[arm] = {{unit_moment[0], unit_moment[1]}};
    }

  // Both rotors in one geometric pair receive a common nonnegative thrust.
  // Select the pair directly from the Spinal roll/pitch error convention
  // verified on the real vehicle:
  //   current +roll -> motors 2+3, current -roll -> motors 0+1
  //   current +pitch -> motors 0+3, current -pitch -> motors 1+2
  // The previous geometry-only choice selected motors 2+3 for a negative
  // Spinal roll in c7, which was the opposite of the measured correction
  // direction.
  using ArmPair = std::array<size_t, 2>;
  const double roll_error = wrapAngle(target_roll_ - roll_);
  const double pitch_error = wrapAngle(target_pitch_ - pitch_);
  const bool roll_only = correctionRollOnly();
  const bool pitch_dominant = !roll_only &&
                              std::abs(pitch_error) >=
                                  std::abs(roll_error);
  ArmPair correction_pair = pitch_dominant
      ? (pitch_error >= 0.0 ? ArmPair{{1, 2}} : ArmPair{{0, 3}})
      : (roll_error >= 0.0 ? ArmPair{{0, 1}} : ArmPair{{2, 3}});
  if (correction_pair_reverse_spinal_mapping_ && roll_only)
    {
      // Direct MuJoCo response tests show that its applied bidirectional
      // thrust axis produces the opposite roll sign to the real Spinal
      // convention (positive current roll: motors 0+1 restore it; negative:
      // motors 2+3). Do not change the real-vehicle mapping above.
      correction_pair = roll_error >= 0.0
          ? ArmPair{{2, 3}} : ArmPair{{0, 1}};
    }
  PressureArray result{{0.0, 0.0, 0.0, 0.0}};
  const Vector2 pair_column{{
      columns[correction_pair[0]][0] + columns[correction_pair[1]][0],
      roll_only ? 0.0 :
          columns[correction_pair[0]][1] + columns[correction_pair[1]][1]}};
  const double denominator = pair_column[0] * pair_column[0] +
                             pair_column[1] * pair_column[1] + 1.0e-8;
  double allocated_thrust =
      (pair_column[0] * desired[0] + pair_column[1] * desired[1]) /
      denominator;
  if (allocated_thrust <= 0.0)
    {
      // The pressure/angle model may still contain the old arm-sign
      // convention. Keep the experimentally verified pair and use the
      // requested moment magnitude rather than silently choosing the
      // opposite side of the vehicle.
      allocated_thrust = std::hypot(desired[0], roll_only ? 0.0 : desired[1]) /
          std::max(1.0e-4, std::hypot(pair_column[0], pair_column[1]));
      ROS_WARN_THROTTLE(1.0,
          "Correction geometry sign disagrees with requested moment; retaining %s motors %zu+%zu",
          correction_pair_reverse_spinal_mapping_ ? "MuJoCo-verified" : "hardware-verified",
          correction_pair[0], correction_pair[1]);
    }

  // A mathematically small command is too close to ESC idle to start the
  // real rotor, so retain the measured minimum useful thrust.
  const double common_thrust = std::min(correction_max_thrust_n_,
      std::max(correction_min_thrust_n_, allocated_thrust));
  result[correction_pair[0]] = result[correction_pair[1]] = common_thrust;
  ROS_INFO_THROTTLE(1.0,
      "Spinal attitude correction: error roll/pitch %.1f/%.1f deg -> motors %zu+%zu at %.2f N (%s)",
      roll_error * 180.0 / M_PI, pitch_error * 180.0 / M_PI,
      correction_pair[0], correction_pair[1], common_thrust,
      roll_only ? "roll only; pitch ignored" : "roll/pitch");
  return result;
}

double AttitudePressureController::measuredArmAngle(size_t arm) const
{
  Vector3 body_up{{-gravity_body_[0], -gravity_body_[1], -gravity_body_[2]}};
  const double body_norm = std::sqrt(body_up[0]*body_up[0] +
      body_up[1]*body_up[1] + body_up[2]*body_up[2]);
  Vector3 measured = neuron_acc_[arm];
  const double measured_norm = std::sqrt(measured[0]*measured[0] +
      measured[1]*measured[1] + measured[2]*measured[2]);
  if (body_norm < 1e-6 || measured_norm < 1e-6 ||
      !std::isfinite(body_norm) || !std::isfinite(measured_norm))
    return 0.0;
  for (double& value : body_up) value /= body_norm;
  for (double& value : measured) value /= measured_norm;

  const double c_yaw = std::cos(arm_yaw_[arm]);
  const double s_yaw = std::sin(arm_yaw_[arm]);
  const Vector3 local_up{{c_yaw*body_up[0] + s_yaw*body_up[1],
                          -s_yaw*body_up[0] + c_yaw*body_up[1],
                          body_up[2]}};
  // Find the rotation about local +Y that maps body-up into the Neuron
  // accelerometer frame. A constant sensor mounting offset cancels when this
  // value is subtracted from the GRASP baseline.
  const double dot_xz = local_up[0]*measured[0] + local_up[2]*measured[2];
  const double cross_xz = local_up[2]*measured[0] - local_up[0]*measured[2];
  return std::atan2(cross_xz, dot_xz);
}

double AttitudePressureController::measuredArmBendAngle(size_t arm) const
{
  // In MuJoCo use the exact Spinal-to-rotor/Neuron bend supplied by the
  // pneumatic plugin (joints 1--3 only). Joints 4--5 are distal to the rotor.
  // Hardware has no grip_state extension and retains the Neuron IMU fallback.
  if (arm < ARM_COUNT && std::isfinite(grip_joint_bend_rad_[arm]) &&
      !grip_state_stamp_.isZero() &&
      (ros::Time::now() - grip_state_stamp_).toSec() <= 0.25)
    return std::max(0.0, std::min(M_PI, grip_joint_bend_rad_[arm]));
  // After conversion into each arm's local frame, pneumatic bending makes the
  // signed Spinal--Neuron angle more negative.  Express this in the physical
  // convention used by Action C: straight = 0 deg, horizontal = 90 deg.
  return std::max(0.0, std::min(M_PI, -measuredArmAngle(arm)));
}

bool AttitudePressureController::armGeometryReady(
    const PressureArray& target, const ros::Time& now) const
{
  Vector3 body_up{{-gravity_body_[0], -gravity_body_[1], -gravity_body_[2]}};
  const double body_up_norm = std::sqrt(body_up[0]*body_up[0] +
      body_up[1]*body_up[1] + body_up[2]*body_up[2]);
  if (body_up_norm < 1e-6) return false;
  for (double& value : body_up) value /= body_up_norm;

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      if (neuron_stamp_[arm].isZero() ||
          (now - neuron_stamp_[arm]).toSec() > neuron_timeout_sec_)
        {
          ROS_WARN_THROTTLE(1.0, "PREPARE waiting: Neuron IMU %zu is stale", arm + 1);
          return false;
        }
      const double pressure_error = std::abs(measured_pressure_[arm] - target[arm]);
      if (pressure_error > pressure_tolerance_kpa_)
        {
          ROS_INFO_THROTTLE(1.0,
              "PREPARE waiting: arm %zu pressure %.2f/%.2f kPa (error %.2f > %.2f)",
              arm + 1, measured_pressure_[arm], target[arm], pressure_error,
              pressure_tolerance_kpa_);
          return false;
        }

      const double c_yaw = std::cos(arm_yaw_[arm]);
      const double s_yaw = std::sin(arm_yaw_[arm]);
      const Vector3 yaw_local{{c_yaw*body_up[0] + s_yaw*body_up[1],
                              -s_yaw*body_up[0] + c_yaw*body_up[1],
                              body_up[2]}};
      const double angle = bendAngle(arm, target[arm]);
      // A positive URDF joint angle rotates the child about local -Y.  The
      // accelerometer expresses body-up in that child frame, so the coordinate
      // transform uses its inverse rotation about +Y.
      const double c = std::cos(angle), s = std::sin(angle);
      Vector3 expected{{c*yaw_local[0] + s*yaw_local[2], yaw_local[1],
                        -s*yaw_local[0] + c*yaw_local[2]}};
      Vector3 measured = neuron_acc_[arm];
      const double measured_norm = std::sqrt(measured[0]*measured[0] +
          measured[1]*measured[1] + measured[2]*measured[2]);
      if (!std::isfinite(measured_norm) || measured_norm < 1e-6) return false;
      for (double& value : measured) value /= measured_norm;
      const double dot = std::max(-1.0, std::min(1.0,
          expected[0]*measured[0] + expected[1]*measured[1] + expected[2]*measured[2]));
      const double gyro_norm = std::sqrt(neuron_gyro_[arm][0]*neuron_gyro_[arm][0] +
          neuron_gyro_[arm][1]*neuron_gyro_[arm][1] + neuron_gyro_[arm][2]*neuron_gyro_[arm][2]);
      const double angle_error = std::acos(dot);
      if (angle_error > arm_angle_tolerance_ || gyro_norm > arm_gyro_tolerance_)
        {
          ROS_INFO_THROTTLE(1.0,
              "PREPARE waiting: arm %zu IMU angle error %.3f/%.3f rad, gyro %.3f/%.3f rad/s",
              arm + 1, angle_error, arm_angle_tolerance_, gyro_norm,
              arm_gyro_tolerance_);
          return false;
        }
    }
  return true;
}

void AttitudePressureController::publishPressureTarget(
    const PressureArray& requested, double maximum_step,
    double pressure_limit_kpa)
{
  const double effective_limit = pressure_limit_kpa >= 0.0
      ? std::min(pressure_safety_limit_kpa_, pressure_limit_kpa)
      : maximum_pressure_kpa_;
  std_msgs::Float32MultiArray target;
  target.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bounded = std::max(minimum_prepare_pressure_kpa_,
          std::min(effective_limit, requested[arm]));
      const double change = std::max(-maximum_step,
          std::min(maximum_step, bounded - last_pressure_[arm]));
      last_pressure_[arm] += change;
      target.data[arm] = last_pressure_[arm];
    }
  pressure_target_pub_.publish(target);
}

void AttitudePressureController::publishBottomTarget(double pressure_kpa)
{
  std_msgs::Float32 target;
  target.data = std::max(0.0, std::min(pressure_safety_limit_kpa_, pressure_kpa));
  bottom_target_pub_.publish(target);
}

void AttitudePressureController::publishBottomDirection(double direction_scale)
{
  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = "main_body";
  direction.vector.x = direction_scale * inchworm_direction_body_[0];
  direction.vector.y = direction_scale * inchworm_direction_body_[1];
  bottom_direction_pub_.publish(direction);
}

void AttitudePressureController::publishCorrectionThrust(
    const PressureArray& thrust)
{
  const ros::Time now = ros::Time::now();
  spinal::PerchingThrustCommand command;
  bool finite_request = true;
  for (const double value : thrust)
    finite_request = finite_request && std::isfinite(value);
  if (!finite_request && !correction_output_fault_latched_)
    {
      correction_output_fault_latched_ = true;
      ROS_FATAL("Non-finite rotor-thrust request detected; latching all four PWM outputs at neutral until the controller is restarted");
    }

  // Fail closed before any clamping/min/max operation. std::min(a, NaN)
  // returns `a` on common C++ implementations, which previously converted a
  // single invalid arm command into the 11.2-N calibration edge whenever a
  // second arm kept PWM-test mode active.
  if (correction_output_fault_latched_)
    {
      pwm_test_safe_thrust_.fill(0.0);
      pwm_test_safe_stamp_ = now;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        command.thrust[arm] = 0.0f;
      if (use_root_spinal_topics_ && root_correction_thrust_pub_)
        root_correction_thrust_pub_.publish(command);
      else
        correction_thrust_pub_.publish(command);
      if (use_pwm_test_for_correction_) publishNeutralPwmTest();
      return;
    }

  double forward_limit = use_pwm_test_for_correction_
      ? pwm_test_hard_forward_thrust_n_
      : std::min(pwm_test_hard_forward_thrust_n_,
                 correction_max_thrust_n_);
  if (continuous_recovery_started_ && state_ == PREPARE)
    forward_limit = std::min(forward_limit, roll_recovery_config_.limit);
  const double reverse_limit = pwm_test_hard_reverse_thrust_n_;
  const double raw_elapsed = pwm_test_safe_stamp_.isZero()
      ? 0.02 : (now - pwm_test_safe_stamp_).toSec();
  if (!pwm_test_safe_stamp_.isZero() &&
      (!std::isfinite(raw_elapsed) || raw_elapsed < 0.0 ||
       raw_elapsed > 0.25))
    {
      ROS_ERROR_THROTTLE(1.0,
          "Rotor-output timestamp gap/reset detected; discarding stale thrust state");
      pwm_test_safe_thrust_.fill(0.0);
    }
  const double elapsed = std::isfinite(raw_elapsed) && raw_elapsed >= 0.0 &&
      raw_elapsed <= 0.25 ? raw_elapsed : 0.02;
  // A delayed callback must not use the entire scheduling gap to jump from
  // neutral to full thrust in one published sample.
  const double dt = std::min(0.05, elapsed);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bounded = std::max(-reverse_limit,
          std::min(forward_limit, thrust[arm]));
      const bool increasing_magnitude =
          bounded * pwm_test_safe_thrust_[arm] >= 0.0 &&
          std::abs(bounded) > std::abs(pwm_test_safe_thrust_[arm]);
      const double slew = increasing_magnitude
          ? pwm_test_output_rise_n_s_ : pwm_test_output_fall_n_s_;
      const double maximum_change = slew * dt;
      pwm_test_safe_thrust_[arm] += std::max(-maximum_change,
          std::min(maximum_change, bounded - pwm_test_safe_thrust_[arm]));
      pwm_test_safe_thrust_[arm] = std::max(-reverse_limit,
          std::min(forward_limit, pwm_test_safe_thrust_[arm]));
      command.thrust[arm] = std::max(0.0, pwm_test_safe_thrust_[arm]);
    }
  pwm_test_safe_stamp_ = now;
  observeHybridThrust(now, pwm_test_safe_thrust_);

  if (use_pwm_test_for_correction_)
    {
      // Ensure the legacy perching pulse path is inactive, then drive only
      // the explicitly selected rotors through pwm_test. An all-zero request
      // publishes an empty PwmTest and exits test mode immediately.
      spinal::PerchingThrustCommand zero_command;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm) zero_command.thrust[arm] = 0.0f;
      if (use_root_spinal_topics_ && root_correction_thrust_pub_)
        root_correction_thrust_pub_.publish(zero_command);
      else
        correction_thrust_pub_.publish(zero_command);
      // The exact same finite, state-bounded and slew-bounded values are used
      // by both the diagnostic observer and the physical PWM conversion.
      publishPwmTest(pwm_test_safe_thrust_);
      return;
    }

  if (use_root_spinal_topics_ && root_correction_thrust_pub_)
    root_correction_thrust_pub_.publish(command);
  else
    correction_thrust_pub_.publish(command);
}

void AttitudePressureController::publishNeutralPwmTest()
{
  spinal::PwmTest command;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      command.motor_index.push_back(static_cast<uint8_t>(arm));
      command.pwms.push_back(static_cast<float>(pwm_test_neutral_pwm_));
    }
  pwm_test_pub_.publish(command);
  if (root_pwm_test_pub_) root_pwm_test_pub_.publish(command);
}

void AttitudePressureController::publishPwmTest(const PressureArray& thrust)
{
  spinal::PwmTest command;
  bool has_nonzero_thrust = false;
  for (const double arm_thrust : thrust)
    {
      if (!std::isfinite(arm_thrust))
        {
          correction_output_fault_latched_ = true;
          ROS_FATAL("Non-finite thrust reached publishPwmTest; forcing explicit neutral PWM and latching output fault");
          publishNeutralPwmTest();
          return;
        }
      has_nonzero_thrust = has_nonzero_thrust || std::abs(arm_thrust) > 0.01;
    }

  // Spinal's individual PwmTest callback enters one global flight-motor test
  // mode. Values for omitted motor indices remain cached, so a command for a
  // subset could unintentionally replay an old command on another rotor.
  // Whenever any flight rotor is active, explicitly command all four and set
  // every nominally inactive rotor to the bidirectional neutral duty.
  if (has_nonzero_thrust)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      {
        const double bounded_thrust = std::max(-pwm_test_hard_reverse_thrust_n_,
            std::min(pwm_test_hard_forward_thrust_n_, thrust[arm]));
        const double pwm = std::abs(bounded_thrust) <= 0.01
            ? pwm_test_neutral_pwm_ : signedThrustToPwm(bounded_thrust);
        if (!std::isfinite(pwm))
          {
            correction_output_fault_latched_ = true;
            ROS_FATAL("Non-finite PWM conversion detected; forcing explicit neutral PWM and latching output fault");
            publishNeutralPwmTest();
            return;
          }
        command.motor_index.push_back(static_cast<uint8_t>(arm));
        command.pwms.push_back(static_cast<float>(
            std::max(pwm_test_min_pwm_, std::min(pwm_test_max_pwm_, pwm))));
      }
  if (!command.motor_index.empty())
    {
      std::string description;
      for (size_t i = 0; i < command.motor_index.size(); ++i)
        description += (i ? ", " : "") + std::string("motor") +
            std::to_string(command.motor_index[i]) + "=" +
            std::to_string(command.pwms[i]);
      ROS_INFO_THROTTLE(0.1, "Publishing /pwm_test correction: %s",
                        description.c_str());
    }
  // In mixed bridge/bringup setups both /imu and /quadrotor/imu may be
  // visible. Do not let that input-route race redirect consecutive PWM
  // commands to different names. Publishing both is harmless when only one
  // Spinal route is connected, and keeps the stream continuous when remapped.
  pwm_test_pub_.publish(command);
  if (root_pwm_test_pub_) root_pwm_test_pub_.publish(command);
}

double AttitudePressureController::pwmModelForce(
    double pwm, bool reverse) const
{
  if (std::abs(pwm - pwm_test_neutral_pwm_) <= 1.0e-9) return 0.0;
  const auto& polynomial = reverse ? pwm_test_reverse_polynomial_
                                   : pwm_test_forward_polynomial_;
  const double percent = 100.0 * pwm;
  const double reference_force = polynomial[0] +
      (polynomial[1] * percent + polynomial[2] * percent * percent) / 10.0;
  const double voltage_ratio = pwm_test_reference_voltage_ > 0.0
      ? pwm_test_voltage_ / pwm_test_reference_voltage_ : 1.0;
  const double scaled_force =
      reference_force * voltage_ratio * voltage_ratio;
  return reverse ? -std::abs(scaled_force) : std::abs(scaled_force);
}

double AttitudePressureController::signedThrustToPwm(double thrust_n) const
{
  if (std::abs(thrust_n) <= 0.01) return pwm_test_neutral_pwm_;
  const bool reverse = thrust_n < 0.0;
  const bool positive = !reverse;
  const bool low_pwm_branch =
      positive == pwm_test_positive_thrust_below_neutral_;
  const double edge_pwm = low_pwm_branch ? pwm_test_min_pwm_
                                         : pwm_test_max_pwm_;
  const double target_magnitude = std::min(
      std::abs(thrust_n), std::abs(pwmModelForce(edge_pwm, reverse)));
  double lower_fraction = 0.0;
  double upper_fraction = 1.0;
  for (size_t iteration = 0; iteration < 50; ++iteration)
    {
      const double middle_fraction =
          0.5 * (lower_fraction + upper_fraction);
      const double middle_pwm = pwm_test_neutral_pwm_ + middle_fraction *
          (edge_pwm - pwm_test_neutral_pwm_);
      if (std::abs(pwmModelForce(middle_pwm, reverse)) < target_magnitude)
        lower_fraction = middle_fraction;
      else
        upper_fraction = middle_fraction;
    }
  const double fraction = 0.5 * (lower_fraction + upper_fraction);
  return pwm_test_neutral_pwm_ +
      fraction * (edge_pwm - pwm_test_neutral_pwm_);
}

void AttitudePressureController::publishArmShapeDebug(
    const PressureArray& current_relative_angle) const
{
  if (!grasp_baseline_valid_) return;
  std_msgs::Float32MultiArray current, target, error, active;
  current.data.resize(ARM_COUNT);
  target.data.resize(ARM_COUNT);
  error.data.resize(ARM_COUNT);
  active.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double target_relative = prepare_arm_active_[arm]
          ? -prepare_straighten_angle_by_arm_[arm] : 0.0;
      current.data[arm] = current_relative_angle[arm];
      target.data[arm] = target_relative;
      error.data[arm] = current_relative_angle[arm] - target_relative;
      active.data[arm] = prepare_arm_active_[arm] ? 1.0f : 0.0f;
    }
  arm_relative_angle_pub_.publish(current);
  arm_target_relative_angle_pub_.publish(target);
  arm_shape_error_pub_.publish(error);
  arm_shape_active_pub_.publish(active);
}

void AttitudePressureController::enterState(uint8_t state, const ros::Time& now)
{
  static const char* names[] = {
    "GRASP", "PREPARE", "THRUST_CORRECTION", "RECOVER",
    "FRONT_RELEASE", "FRONT_REACH", "FRONT_ANCHOR", "REAR_UNLOAD",
    "PULL", "REAR_ANCHOR", "SETTLE", "SUPPORT_RESTORE",
    "ACTION_B_FRONT_REACH", "ACTION_B_REGRASP", "ACTION_B_FRONT_REGRIP",
    "ACTION_C_PREPARE", "ACTION_C_HORIZONTAL_PULL", "RESERVED_17",
    "ACTION_C_REGRIP", "ROCK_BOTTOM_PRELOAD", "ROCK_FRONT_WINDUP",
    "ROCK_FRONT_LAND", "ROCK_REAR_UNLOAD", "ROCK_REAR_RECOVER",
    "ROCK_REAR_LAND", "ACTION_A_FRONT_PLACE", "ACTION_D_PREPARE",
    "ACTION_D_RAMP", "ACTION_D_REGRIP"
  };
  action_c_rear_release_gate_.reset();
  state_ = state;
  correction_entry_gate_.reset();
  action_c_resume_gate_.reset();
  action_c_recovery_gate_.reset();
  action_c_boundary_settle_gate_.reset();
  action_c_ppg_settle_gate_.reset();
  action_c_pressure_ready_gate_.reset();
  action_c_roll_trend_.reset();

  state_start_ = now;
  ready_start_ = ros::Time(0);
  if (state_ == PULL)
    {
      inchworm_motion_detected_ = false;
      inchworm_best_pull_travel_m_ = inchwormTravel();
      inchworm_motion_last_seen_ = now;
    }
  if (state_ == ACTION_B_FRONT_REGRIP)
    {
      const double lift_roll = wrapAngle(
          roll_ - inchworm_swing_start_roll_);
      const double lift_pitch = wrapAngle(
          pitch_ - inchworm_swing_start_pitch_);
      // Signed lift of the front side along the requested travel direction.
      // For +x travel, for example, only negative pitch raises the front.
      action_b_lift_tilt_rad_ =
          lift_roll * inchworm_direction_body_[1] -
          lift_pitch * inchworm_direction_body_[0];
      action_b_rear_contact_position_ = {{0.0, 0.0, contact_position_[2]}};
      size_t rear_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (!inchworm_front_[arm])
          {
            action_b_rear_contact_position_[0] += arm_base_position_[arm][0];
            action_b_rear_contact_position_[1] += arm_base_position_[arm][1];
            ++rear_count;
          }
      if (rear_count > 0)
        {
          action_b_rear_contact_position_[0] /= rear_count;
          action_b_rear_contact_position_[1] /= rear_count;
        }
      double front_thrust_sum = 0.0;
      size_t front_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          {
            front_thrust_sum += inchworm_thrust_[arm];
            ++front_count;
          }
      action_b_hold_thrust_n_ = front_count > 0
          ? front_thrust_sum / static_cast<double>(front_count) : 0.0;
      action_b_initial_hold_thrust_n_ = action_b_hold_thrust_n_;
      action_b_hold_tilt_integral_n_ = 0.0;
      action_b_pressure_limited_by_feedback_ = false;
      action_b_pulse_count_ = 0;
      action_b_pulse_on_ = false;
      action_b_pulse_phase_start_ = now;
      action_b_shape_phase_start_ = now;
      action_b_limit_start_ = ros::Time(0);
      action_b_lift_settle_start_ = ros::Time(0);
      action_b_lift_target_adapted_ = false;
      action_b_shape_phase_ = 0;
      action_b_shape_pressure_kpa_ = 0.0;
      action_b_best_pressure_kpa_ = 0.0;
      action_b_best_thrust_n_ = action_b_hold_thrust_n_;
      // The FRONT_REACH endpoint is the guaranteed initial candidate.  If
      // the stepped search later times out, return to the best settled pair
      // found so far instead of treating the unfinished current point as the
      // result or abandoning the action.
      action_b_best_horizontal_force_n_ = 0.0;
      double front_angle_sum = 0.0;
      size_t front_angle_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          {
            const double measured_angle = measuredArmAngle(arm);
            const double arm_angle = std::abs(wrapAngle(measured_angle));
            front_angle_sum += arm_angle;
            ++front_angle_count;
            action_b_shape_start_neuron_angle_rad_[arm] = measured_angle;
          }
      action_b_shape_start_angle_rad_ = front_angle_count > 0
          ? front_angle_sum / static_cast<double>(front_angle_count) : 0.0;
      action_b_last_bend_angle_rad_ = 0.0;
      action_b_bend_rate_rad_s_ = 0.0;
      action_b_last_bend_time_ = now;
      action_b_pulse_pressure_.fill(0.0);
      action_b_pre_pulse_angle_rad_.fill(0.0);
      Vector3 com_lever;
      Vector3 gravity_force;
      for (size_t axis = 0; axis < 3; ++axis)
        {
          com_lever[axis] = center_of_mass_[axis] -
              action_b_rear_contact_position_[axis];
          gravity_force[axis] = mass_kg_ * gravity_body_[axis];
        }
      const Vector3 gravity_moment = cross(com_lever, gravity_force);
      action_b_hold_reference_moment_ = {{-gravity_moment[0],
                                          -gravity_moment[1]}};
      action_b_hold_reference_vertical_force_n_ = 0.0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          {
            const double model_angle_at_transition = bend_model_.angleRad(
                std::max(0.0, measured_pressure_[arm]),
                action_b_hold_thrust_n_ + bend_model_thrust_offset_n_);
            action_b_shape_start_rotor_angle_rad_[arm] =
                model_angle_at_transition;
            action_b_neuron_angle_offset_[arm] = wrapAngle(
                model_angle_at_transition - measuredArmAngle(arm));
            const double rotor_angle = std::max(0.0, std::min(
                2.966, measuredArmAngle(arm) +
                       action_b_neuron_angle_offset_[arm]));
            const Vector3 direction = rotorDirectionFromAngle(arm, rotor_angle);
            // action_b_hold_reference_moment_ already contains the thrust
            // moment required to balance gravity about the rear contact.
            // Do not add the measured rotor moment here: that double-counted
            // support and commanded a large thrust jump at the state change.
            action_b_hold_reference_vertical_force_n_ +=
                action_b_hold_thrust_n_ * std::max(0.0, direction[2]);
          }

      // Select the pressure/thrust operating point directly from the measured
      // pressure-thrust-bend table. Reserve pulse headroom whenever the
      // measured lift thrust permits it; never select a holding thrust below
      // the thrust that first produced the lift posture.
      const double pressure_limit = std::min(
          maximum_pressure_kpa_, action_b_shape_max_pressure_kpa_);
      const double hold_limit = std::min(
          action_b_pre_pulse_max_thrust_n_,
          action_b_max_thrust_n_ - action_b_pulse_amplitude_n_);
      double best_residual = std::numeric_limits<double>::infinity();
      double best_vertical_error = std::numeric_limits<double>::infinity();
      double fallback_residual = std::numeric_limits<double>::infinity();
      double fallback_pressure = 0.0;
      double fallback_thrust = action_b_initial_hold_thrust_n_;
      double fallback_horizontal = 0.0;
      double fallback_vertical_error = std::numeric_limits<double>::infinity();
      if (action_b_initial_hold_thrust_n_ > hold_limit + 1.0e-6)
        ROS_WARN("Action B initial lift thrust %.2f N leaves only %.2f N pulse headroom under the %.2f N limit",
                 action_b_initial_hold_thrust_n_,
                 std::max(0.0, action_b_max_thrust_n_ -
                               action_b_initial_hold_thrust_n_),
                 action_b_max_thrust_n_);
      for (double pressure = 0.0; pressure <= pressure_limit + 1.0e-9;
           pressure += 0.5)
        for (double thrust = action_b_initial_hold_thrust_n_;
             thrust <= hold_limit + 1.0e-9; thrust += 0.05)
          {
            Vector2 predicted_moment{{0.0, 0.0}};
            double predicted_vertical_force = 0.0;
            double horizontal_force = 0.0;
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              if (inchworm_front_[arm])
                {
                  const double angle = bend_model_.angleRad(
                      pressure, thrust + bend_model_thrust_offset_n_);
                  const Vector3 position = rotorPositionFromAngle(arm, angle);
                  const Vector3 direction = rotorDirectionFromAngle(arm, angle);
                  Vector3 lever;
                  for (size_t axis = 0; axis < 3; ++axis)
                    lever[axis] = position[axis] -
                        action_b_rear_contact_position_[axis];
                  const Vector3 unit_moment = cross(lever, direction);
                  predicted_moment[0] += thrust * unit_moment[0];
                  predicted_moment[1] += thrust * unit_moment[1];
                  predicted_vertical_force +=
                      thrust * std::max(0.0, direction[2]);
                  horizontal_force += thrust * std::max(
                      0.0, direction[0] * inchworm_direction_body_[0] +
                           direction[1] * inchworm_direction_body_[1]);
                }
            const double residual = std::hypot(
                predicted_moment[0] - action_b_hold_reference_moment_[0],
                predicted_moment[1] - action_b_hold_reference_moment_[1]);
            const double vertical_error = std::abs(
                predicted_vertical_force -
                action_b_hold_reference_vertical_force_n_);
            if (residual < fallback_residual)
              {
                fallback_residual = residual;
                fallback_pressure = pressure;
                fallback_thrust = thrust;
                fallback_horizontal = horizontal_force;
                fallback_vertical_error = vertical_error;
              }
            const double reference_norm = std::max(1.0e-4, std::hypot(
                action_b_hold_reference_moment_[0],
                action_b_hold_reference_moment_[1]));
            const double vertical_reference = std::max(
                0.1, action_b_hold_reference_vertical_force_n_);
            // First preserve the initial upward component and moment. Among
            // feasible points choose the largest pressure; vertical error is
            // the tie-breaker at equal pressure.
            if (residual <= 0.15 * reference_norm &&
                vertical_error <= 0.05 * vertical_reference &&
                (pressure > action_b_best_pressure_kpa_ + 1.0e-9 ||
                 (std::abs(pressure - action_b_best_pressure_kpa_) <= 1.0e-9 &&
                  vertical_error < best_vertical_error)))
              {
                action_b_best_horizontal_force_n_ = horizontal_force;
                action_b_best_pressure_kpa_ = pressure;
                action_b_best_thrust_n_ = thrust;
                best_residual = residual;
                best_vertical_error = vertical_error;
              }
          }
      if (action_b_best_horizontal_force_n_ < 0.0)
        {
          if (std::isfinite(fallback_residual))
            {
              action_b_best_pressure_kpa_ = fallback_pressure;
              action_b_best_thrust_n_ = std::max(
                  action_b_initial_hold_thrust_n_, fallback_thrust);
              action_b_best_horizontal_force_n_ = fallback_horizontal;
              best_residual = fallback_residual;
              best_vertical_error = fallback_vertical_error;
            }
          else
            {
              // No +1 N pulse headroom exists even at the initial posture.
              // Preserve that posture instead of violating the thrust floor.
              action_b_best_pressure_kpa_ = 0.0;
              action_b_best_thrust_n_ = action_b_initial_hold_thrust_n_;
              action_b_best_horizontal_force_n_ = 0.0;
              best_residual = 0.0;
              best_vertical_error = 0.0;
            }
        }
      // Keep the actual thrust continuous at the state boundary. The selected
      // table result is diagnostic only.  The real operating point must be
      // found from body/Neuron feedback on the current grasped object.
      action_b_hold_thrust_n_ = action_b_initial_hold_thrust_n_;
      ROS_INFO("Action B lift/pull setup: rear pivot [%.3f, %.3f, %.3f] m, measured lift %.2f deg, gravity-balancing moment [%.4f, %.4f] Nm; table pressure %.1f kPa / %.2f N, horizontal %.2f N, pulse headroom %.2f N",
               action_b_rear_contact_position_[0],
               action_b_rear_contact_position_[1],
               action_b_rear_contact_position_[2],
               action_b_lift_tilt_rad_ * 180.0 / M_PI,
               action_b_hold_reference_moment_[0],
               action_b_hold_reference_moment_[1],
               action_b_best_pressure_kpa_, action_b_best_thrust_n_,
               action_b_best_horizontal_force_n_,
               std::max(0.0, action_b_max_thrust_n_ -
                                            action_b_hold_thrust_n_));

      // Do not jump directly to the table point.  Start the real closed-loop
      // search at the measured lift posture reached in FRONT_REACH.
      action_b_shape_phase_ = 0;
      action_b_shape_phase_start_ = now;
      action_b_shape_pressure_kpa_ = 0.0;
      action_b_best_pressure_kpa_ = 0.0;
      action_b_best_thrust_n_ = action_b_initial_hold_thrust_n_;
      action_b_best_horizontal_force_n_ = 0.0;
    }
  if (state_ == ROCK_FRONT_UNLOAD || state_ == ROCK_REAR_RECOVER)
    {
      rocking_phase_start_roll_ = roll_;
      rocking_phase_start_pitch_ = pitch_;
      if (state_ == ROCK_REAR_RECOVER)
        rocking_rear_recover_start_travel_m_ = inchwormTravel();
    }
  if (state_ == ROCK_FRONT_LAND)
    {
      rocking_swing_phase_ = ROCK_SWING_RELEASE;
      rocking_swing_phase_start_ = now;
      rocking_swing_hold_start_ = ros::Time(0);
    }
  else if (state_ == ROCK_REAR_INFLATE)
    rocking_swing_hold_start_ = ros::Time(0);
  ROS_INFO("Perching controller state: %s",
           names[std::min<size_t>(state_, sizeof(names) / sizeof(names[0]) - 1)]);
}

// --------------------------------------------------------------------------
// Inchworm locomotion state machine
// FRONT_REACH simultaneously unloads and extends one leading arm, then
// FRONT_ANCHOR -> SUPPORT_RESTORE is repeated for the second leading arm.
// Each placed arm keeps enough thrust to preserve its extended contact.  Once
// both are anchored, REAR_UNLOAD releases the trailing pair and PULL removes
// both front hold thrusts together while applying trailing reverse thrust.
// Faults use stopInchwormToGrasp(); RECOVER belongs to attitude correction.
// --------------------------------------------------------------------------
void AttitudePressureController::startInchwormStep(
    const Vector2& direction, double distance, const ros::Time& now)
{
  if (!action_c_preserve_adaptation_on_start_)
    recovery_contact_grasp_active_ = false;
  if (hybridLocomotionActive() && action_c_direction_sign_x_*(direction[0]>=0.?1.:-1.)<0.) {
    const bool search_enabled=hybrid_search_.enabled;
    const auto initial=hybrid_search_.initial;
    hybrid_search_.reset();hybrid_search_.enabled=search_enabled;
    hybrid_search_.initial=initial;hybrid_search_.current=initial;
    hybrid_planner_.reset(hybrid_config_); // new pair: do not swap learned left/right responses
  }
  inchworm_direction_body_ = direction;
  inchworm_direction_world_ = {{
      std::cos(yaw_) * direction[0] - std::sin(yaw_) * direction[1],
      std::sin(yaw_) * direction[0] + std::cos(yaw_) * direction[1]}};
  const Vector3 locomotion_position = locomotionPosition();
  inchworm_overall_start_position_ = locomotion_position;
  inchworm_total_target_distance_m_ = distance > 0.0 ? distance
                                                      : inchworm_default_distance_m_;
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
    action_c_direction_sign_x_ = direction[0] >= 0.0 ? 1.0 : -1.0;
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
      !action_c_preserve_adaptation_on_start_)
    {
      action_c_absolute_target_x_m_ = locomotion_position[0] +
          action_c_direction_sign_x_ * inchworm_total_target_distance_m_;
      action_c_resume_direction_ = direction;
      // A new operator command starts a new task; recovery preserves its goal.
      action_c_posture_recovery_count_ = 0;
    }
  inchworm_front_.fill(false);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double projection = arm_base_position_[arm][0] * direction[0] +
                                arm_base_position_[arm][1] * direction[1];
      inchworm_front_[arm] = projection > 0.0;
    }
  // Degenerate directions exactly between two arm pairs are resolved by
  // selecting the two largest projections.
  if (std::count(inchworm_front_.begin(), inchworm_front_.end(), true) != 2)
    {
      std::array<std::pair<double, size_t>, ARM_COUNT> order;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        order[arm] = {arm_base_position_[arm][0] * direction[0] +
                      arm_base_position_[arm][1] * direction[1], arm};
      std::sort(order.begin(), order.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });
      inchworm_front_.fill(false);
      inchworm_front_[order[0].second] = true;
      inchworm_front_[order[1].second] = true;
    }
  size_t front_index = 0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (inchworm_front_[arm] && front_index < inchworm_front_order_.size())
      inchworm_front_order_[front_index++] = arm;
  inchworm_active_ = true;
  inchworm_cycle_count_ = 0;
  startInchwormCycle(now);
  ROS_INFO("Inchworm motion started: body direction [%.2f, %.2f], total target %.1f mm, front arms [%d, %d, %d, %d]",
           direction[0], direction[1], 1000.0 * inchworm_total_target_distance_m_,
           inchworm_front_[0], inchworm_front_[1], inchworm_front_[2],
           inchworm_front_[3]);
  if (hand_relative_mocap_enabled_)
    {
      const double dx = hand_mocap_position_[0] - mocap_position_[0];
      const double dy = hand_mocap_position_[1] - mocap_position_[1];
      ROS_INFO("Human-arm relative goal: hand XY delta [%.3f, %.3f] m, distance %.1f mm -> %.1f mm",
               dx, dy, 1000.0 * std::hypot(dx, dy),
               1000.0 * std::max(0.0,
                   std::hypot(dx, dy) -
                   inchworm_total_target_distance_m_));
    }
}

void AttitudePressureController::startInchwormCycle(const ros::Time& now)
{
  const Vector3 locomotion_position = locomotionPosition();
  const double overall_travel =
      inchworm_action_c_enabled_ && !inchworm_action_d_enabled_
          ? action_c_direction_sign_x_ *
                (locomotion_position[0] - inchworm_overall_start_position_[0])
          : (locomotion_position[0] - inchworm_overall_start_position_[0]) *
                inchworm_direction_world_[0] +
            (locomotion_position[1] - inchworm_overall_start_position_[1]) *
                inchworm_direction_world_[1];
  const double remaining = std::max(0.0,
      inchworm_total_target_distance_m_ - overall_travel);
  inchworm_start_position_ = locomotion_position;
  inchworm_target_distance_m_ = inchworm_action_d_enabled_
      ? remaining : std::min(inchworm_stroke_distance_m_, remaining);
  // The recovery handoff may deliberately retain a contact-conditioned
  // pressure for one automatic restart even when long-term grasp-pressure
  // retention is disabled.  A fresh operator command still takes the normal
  // all-base-pressure branch.
  if (hybridLocomotionActive() && action_c_preserve_adaptation_on_start_)
    grasp_pressure_ = hybrid_grasp_pressure_;
  else
    grasp_pressure_.fill(base_pressure_kpa_);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
  {
    grasp_imu_angle_[arm] = measuredArmAngle(arm);
    // The Neuron IMU supplies the change from the grasp posture. Anchor that
    // change to the measured pressure/thrust model so FRONT_REACH can target
    // an absolute bend angle measured from a straight arm.
    grasp_model_angle_[arm] = bendAngle(arm, grasp_pressure_[arm]);
  }
  inchworm_pressure_ = grasp_pressure_;
  inchworm_thrust_.fill(0.0);
  inchworm_motion_detected_ = false;
  inchworm_motion_last_seen_ = now;
  inchworm_rate_stamp_ = mocap_stamp_;
  inchworm_last_travel_m_ = 0.0;
  inchworm_travel_speed_m_s_ = 0.0;
  inchworm_front_sequence_ = 0;
  inchworm_active_arm_ = inchworm_front_order_[0];
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    rocking_front_start_axis_m_[arm] = grip_state_valid_
        ? grip_axis_coordinate_m_[arm]
        : std::numeric_limits<double>::quiet_NaN();
  ++inchworm_cycle_count_;
  if (hybridLocomotionActive())
    {
      startHybridLocomotion(now);
      return;
    }
  if (rocking_gait_enabled_)
    {
      inchworm_swing_start_roll_ = roll_;
      inchworm_swing_start_pitch_ = pitch_;
      // The plugin applies its bottom force at -direction.  Sending the
      // opposite travel direction selects the front-side application point,
      // which creates a rearward (nose-up) preload before the swing.
      publishBottomDirection(-1.0);
      publishBottomTarget(rocking_bottom_low_kpa_);
      enterState(ROCK_FRONT_UNLOAD, now);
      ROS_WARN("Rocking gait started: rearward bottom preload -> front SWING with bottom exhaust -> rear recovery");
    }
  else if (inchworm_action_d_enabled_)
    {
      action_d_pressure_.fill(base_pressure_kpa_);
      action_d_pressure_bend_sign_.fill(1.0);
      action_d_front_thrust_n_ = 0.0;
      action_d_rear_thrust_n_ = 0.0;
      action_d_thrust_step_target_n_ = std::min(
          action_d_thrust_step_n_, action_d_max_front_thrust_n_);
      action_d_pulse_phase_ = ACTION_D_PULSE_RISE;
      action_d_pulse_start_travel_m_ = 0.0;
      action_d_locked_front_thrust_n_ = 0.0;
      action_d_locked_rear_thrust_n_ = 0.0;
      action_d_limit_locked_ = false;
      action_d_posture_limited_ = false;
      action_d_pulse_phase_start_ = ros::Time(0);
      action_d_motion_candidate_start_ = ros::Time(0);
      action_d_rapid_extension_start_.fill(ros::Time(0));
      action_d_filtered_extension_rate_rad_s_.fill(0.0);
      action_d_start_roll_ = roll_;
      action_d_start_pitch_ = pitch_;
      action_d_previous_angle_time_ = now;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (std::isfinite(measured_pressure_[arm]))
            action_d_pressure_[arm] = std::min(
                action_d_max_pressure_kpa_, measured_pressure_[arm]);
          action_d_initial_pressure_[arm] = action_d_pressure_[arm];
          action_d_previous_angle_rad_[arm] = measuredArmBendAngle(arm);
          action_d_ramp_reference_angle_rad_[arm] =
              action_d_previous_angle_rad_[arm];
          action_d_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
          action_d_probe_angle_rad_[arm] = action_d_previous_angle_rad_[arm];
          action_d_probe_pressure_kpa_[arm] = measured_pressure_[arm];
        }
      publishBottomDirection(1.0);
      publishBottomTarget(action_d_bottom_pressure_kpa_);
      enterState(ACTION_D_PREPARE, now);
      ROS_WARN("Action D started: front safety band %.1f--%.1f deg, rear extension margin %.1f deg (%.1f-deg pressure-FB target), bottom %.1f kPa; front %.1f-N steps to %.1f N with synchronized rear reverse to %.1f N, pulse ON %.1f s/OFF %.1f s, front/rear rise %.1f/%.1f N/s",
               action_d_hard_min_angle_rad_ * 180.0 / M_PI,
               action_d_hard_max_angle_rad_ * 180.0 / M_PI,
               action_d_rear_extension_margin_rad_ * 180.0 / M_PI,
               action_d_target_angle_rad_ * 180.0 / M_PI,
               action_d_bottom_pressure_kpa_, action_d_thrust_step_n_,
               action_d_max_front_thrust_n_, action_d_max_rear_thrust_n_,
               action_d_pulse_on_sec_, action_d_pulse_off_sec_,
               action_d_front_thrust_ramp_n_s_,
               action_d_rear_thrust_ramp_n_s_);
    }
  else if (inchworm_action_b_enabled_)
    {
      inchworm_swing_start_roll_ = roll_;
      inchworm_swing_start_pitch_ = pitch_;
      inchworm_angular_rate_fault_start_ = ros::Time(0);
      enterState(ACTION_B_FRONT_REACH, now);
      ROS_WARN("Action B test started: both front arms unload and thrust together");
    }
  else
    {
      publishBottomTarget(action_a_bottom_low_kpa_);
      beginInchwormArmReach(now);
    }
  ROS_INFO("Inchworm cycle %d/%d started: stroke target %.2f mm, remaining %.2f mm; swing order arm%zu then arm%zu",
           inchworm_cycle_count_, inchworm_max_cycles_,
           1000.0 * inchworm_target_distance_m_, 1000.0 * remaining,
           inchworm_front_order_[0], inchworm_front_order_[1]);
}

void AttitudePressureController::beginInchwormArmReach(
    const ros::Time& now)
{
  // First command the swinging chamber directly to zero and wait for its
  // measured pressure. FRONT_REACH starts the rotor only after this release
  // phase completes.
  const bool use_arm2_diagonal_assist =
      action_a_diagonal_assist_enabled_ && inchworm_active_arm_ == 0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const bool release_for_swing = arm == inchworm_active_arm_ ||
          (use_arm2_diagonal_assist && arm == 2);
      inchworm_pressure_[arm] = release_for_swing
          ? std::max(minimum_prepare_pressure_kpa_, inchworm_swing_pressure_kpa_)
          : (inchworm_front_[arm] && inchworm_front_sequence_ > 0
              ? inchworm_transfer_pressure_kpa_ : base_pressure_kpa_);
      inchworm_thrust_[arm] = 0.0;
    }
  grasp_imu_angle_[inchworm_active_arm_] =
      measuredArmAngle(inchworm_active_arm_);
  inchworm_swing_start_roll_ = roll_;
  inchworm_swing_start_pitch_ = pitch_;
  inchworm_angular_rate_fault_start_ = ros::Time(0);
  action_a_best_reach_bend_rad_ = M_PI;
  publishBottomTarget(action_a_bottom_low_kpa_);
  enterState(FRONT_RELEASE, now);
  ROS_INFO("Arm%zu release started%s: command pressure %.1f kPa before applying thrust; bend target %.1f deg",
           inchworm_active_arm_,
           use_arm2_diagonal_assist ? " with arm2 diagonal yaw assist" : "",
           inchworm_pressure_[inchworm_active_arm_],
           inchworm_target_bend_angle_rad_ * 180.0 / M_PI);
}

void AttitudePressureController::stopInchwormToGrasp(
    const ros::Time& now, const char* reason)
{
  if (hybridLocomotionActive() && hybrid_trial_active_)
    finishHybridLocomotion(now, false, false);
  continuous_recovery_waiting_for_settle_ = false;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_preferred_pair_locked_ = false;
  action_c_recovery_gate_.reset();
  const PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
  const bool target_reached=
      std::string(reason).find("hybrid target reached")!=std::string::npos;
  hybrid_hold_pressure_=hybridLocomotionActive() && hybrid_retain_grasp_pressure_ &&
      hybrid_hold_pressure_ && target_reached;
  // Repeated strokes are evaluated at the common inflated, four-arm support
  // boundary.  If no post-motion PPG dwell was requested, keep that already
  // stable boundary after reaching the target as well.  Immediately changing
  // 30 -> 0 kPa after the final successful stroke produced a new roll
  // transient, triggered posture recovery and could topple an otherwise
  // completed MuJoCo run.  A positive ppg_settle_sec remains an explicit
  // request for the old bottom-zero measurement boundary and therefore still
  // exhausts the bladder here.
  hybrid_hold_bottom_=hybridLocomotionActive() &&
      hybrid_keep_bottom_between_strokes_ && target_reached &&
      ppg_settle_sec_<=0.0;
  const PressureArray grasp_target=hybrid_hold_pressure_?hybrid_grasp_pressure_:
      PressureArray{{base_pressure_kpa_,base_pressure_kpa_,base_pressure_kpa_,base_pressure_kpa_}};
  inchworm_active_ = false;
  action_c_resume_pending_ = false;
  action_c_requested_recovery_trigger_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  action_c_preserve_adaptation_on_start_ = false;
  inchworm_thrust_ = zero_thrust;
  prepare_shape_thrust_ = zero_thrust;
  publishCorrectionThrust(zero_thrust);
  publishBottomTarget(hybrid_hold_bottom_?hybrid_single_config_.bottom_kpa:action_a_bottom_low_kpa_);
  publishPressureTarget(grasp_target,
                        std::numeric_limits<double>::infinity());
  last_correction_end_ = now;
  enterState(GRASP, now);
  ROS_WARN("Inchworm stopped and returned directly to GRASP: %s", reason);
}

void AttitudePressureController::requestActionCPostureRecovery(
    const ros::Time& now, const char* reason,
    double confirmed_trigger_rad)
{
  pauseHybridLocomotion(now);
  if (!target_received_)
    {
      stopInchwormToGrasp(now, "Action C cannot recover without a target attitude");
      return;
    }
  ++action_c_posture_recovery_count_;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_use_opposite_pair_ = false;
  continuous_recovery_retry_opposite_pending_ = false;
  continuous_recovery_pair_switches_ = 0;
  recovery_rejected_candidate_mask_ = 0;
  recovery_rejected_direction_sign_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  recovery_circle_event_locked_ = false;
  recovery_preferred_pair_locked_ = false;
  action_c_recovery_gate_.reset();
  action_c_resume_pending_ = true;
  action_c_requested_recovery_trigger_rad_ =
      std::isfinite(confirmed_trigger_rad) && confirmed_trigger_rad > 0.0
          ? confirmed_trigger_rad
          : action_c_posture_recovery_trigger_rad_;
  action_c_resume_direction_ = inchworm_direction_body_;
  inchworm_thrust_.fill(0.0);
  inchworm_pressure_.fill(base_pressure_kpa_);
  // Zero the gait output before handing ownership to shared roll recovery.
  publishCorrectionThrust(inchworm_thrust_);
  publishPressureTarget(inchworm_pressure_, std::numeric_limits<double>::infinity());
  publishBottomTarget(0.0);
  inchworm_active_ = false;
  enterState(GRASP, now);
  ROS_WARN("Action C paused for shared attitude recovery (attempt %d, no count stop): %s; saved goal x %.4f m, current x %.4f m",
      action_c_posture_recovery_count_, reason,
      action_c_absolute_target_x_m_, locomotionPosition()[0]);
}

bool AttitudePressureController::resumeActionCAfterRecovery(
    const ros::Time& now)
{
  if (!action_c_resume_pending_) return false;
  if (!hybridResumeAllowed(now)) return false;
  if (!locomotionMocapFresh(now))
    {
      ROS_WARN_THROTTLE(1.0, "Action C auto-resume waiting for fresh mocap; saved goal retained");
      return false;
    }
  if (hybrid_trial_active_ &&
      (!std::isfinite(bottom_pressure_kpa_) ||
       std::abs(bottom_pressure_kpa_) > action_c_bottom_deflation_tolerance_kpa_ ||
       !hybrid_endpoint_mocap_window_.stable(now.toSec())))
    {
      // Keep the correction itself fast, but do not evaluate/restart a gait
      // from an inflated or still-moving pose. This is the post-deflation end
      // paired with the pre-inflation start used by the goal conditioner.
      publishBottomTarget(0.0);
      ROS_INFO_THROTTLE(1.0,
          "Action C auto-resume waiting for stable deflated endpoint: bottom %.1f/%.1f kPa, mocap_stable=%d",
          bottom_pressure_kpa_, action_c_bottom_deflation_tolerance_kpa_,
          hybrid_endpoint_mocap_window_.stable(now.toSec()));
      return false;
    }
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_blocked_ = false;
  finishHybridRecovery(now);
  const Vector3 locomotion_position = locomotionPosition();
  const double remaining = action_c_direction_sign_x_ *
      (action_c_absolute_target_x_m_ - locomotion_position[0]);
  if (remaining <= inchworm_min_cycle_progress_m_)
    {
      action_c_resume_pending_ = false;
      action_c_requested_recovery_trigger_rad_ =
          std::numeric_limits<double>::quiet_NaN();
      continuous_recovery_use_opposite_pair_ = false;
      continuous_recovery_pair_switches_ = 0;
      recovery_rejected_candidate_mask_ = 0;
      recovery_candidate_reselect_pending_ = false;
      recovery_geometry_reestimate_attempted_ = false;
      recovery_geometry_reestimate_pending_ = false;
      ROS_INFO("Action C %s goal was reached during attitude recovery: x %.4f/%.4f m",
               action_c_direction_sign_x_ > 0.0 ? "+x" : "-x",
               locomotion_position[0], action_c_absolute_target_x_m_);
      return true;
    }

  // Keep the contact-conditioned pressure found by the recovery handoff.
  // The next single-pass preparation converts it directly to rear grasp and
  // front vent in one coordinated transition.  Restoring all four chambers
  // to nominal pressure first recreates the roll disturbance which recovery
  // has just removed and can trap the controller in a recovery loop.
  action_c_resume_pending_ = false;
  action_c_requested_recovery_trigger_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  continuous_recovery_use_opposite_pair_ = false;
  continuous_recovery_pair_switches_ = 0;
  recovery_rejected_candidate_mask_ = 0;
  recovery_candidate_reselect_pending_ = false;
  recovery_geometry_reestimate_attempted_ = false;
  recovery_geometry_reestimate_pending_ = false;
  action_c_preserve_adaptation_on_start_ = true;
  startInchwormStep(action_c_resume_direction_, remaining, now);
  action_c_preserve_adaptation_on_start_ = false;
  ROS_INFO("Action C resumed after attitude recovery: remaining %.2f mm", 1000.0 * remaining);
  return true;
}

bool AttitudePressureController::actionCRearReleased(const ros::Time& now)
{
  const double age = (now - measured_pressure_stamp_).toSec();
  const bool fresh = !measured_pressure_stamp_.isZero() && age >= 0.0 && age <= 0.5;
  return action_c_rear_release_gate_.update(measured_pressure_, inchworm_front_,
      now.toSec(), fresh, action_c_rear_release_max_kpa_, action_c_rear_release_hold_kpa_,
      action_c_rear_release_hard_kpa_, action_c_rear_release_grace_sec_);
}

bool AttitudePressureController::actionCPressureReady(
    const PressureArray& target, const ros::Time& now)
{
  PressureArray tolerance;
  tolerance.fill(pressure_tolerance_kpa_);
  // Recovery must not deadlock on regulator ripple. Still require all four
  // supports within the configured 5-kPa hold band, fresh and finite.
  if (action_c_continuous_recovery_enabled_ && inchworm_action_c_enabled_ &&
      !inchworm_action_d_enabled_ && (state_ == GRASP || state_ == RECOVER))
    tolerance.fill(std::max(pressure_tolerance_kpa_, action_c_pressure_hold_tolerance_kpa_));
  const double age = (now - measured_pressure_stamp_).toSec();
  return action_c_pressure_ready_gate_.update(measured_pressure_, target, tolerance,
      now.toSec(), !measured_pressure_stamp_.isZero() && age >= 0.0 && age <= 0.25,
      action_c_pressure_hold_tolerance_kpa_, action_c_pressure_dropout_sec_,
      action_c_pressure_max_deviation_kpa_);
}

bool AttitudePressureController::pressureReached(
    const PressureArray& target) const
{
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double tolerance = pressure_tolerance_kpa_;
      if (!std::isfinite(target[arm]) || !std::isfinite(measured_pressure_[arm]) ||
          std::abs(measured_pressure_[arm] - target[arm]) > tolerance)
        return false;
    }
  return true;
}

double AttitudePressureController::inchwormTravel() const
{
  const Vector3 position = locomotionPosition();
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
    {
      // Action C is longitudinal on the real fixture: arms 0/3 face +x and
      // arms 1/2 face -x. Apply the requested direction sign directly to raw
      // mocap x (or robot-minus-hand x in human-arm mode); Spinal yaw has an
      // unrelated marker mounting offset.
      return action_c_direction_sign_x_ *
          (position[0] - inchworm_start_position_[0]);
    }
  if (inchworm_action_d_enabled_)
    {
      // Action D is evaluated directly in the OptiTrack/mocap XY frame. The
      // real-machine marker yaw contains an approximately 180-deg mounting
      // offset (about 161 deg in d_11), so rotating the +x command by that yaw
      // incorrectly reported the clearly positive mocap-x displacement as
      // backward travel. For the usual {+1, 0} command this is exactly the
      // raw positive mocap-x displacement requested by the operator.
      return (position[0] - inchworm_start_position_[0]) *
                 inchworm_direction_body_[0] +
             (position[1] - inchworm_start_position_[1]) *
                 inchworm_direction_body_[1];
    }
  return (position[0] - inchworm_start_position_[0]) *
             inchworm_direction_world_[0] +
         (position[1] - inchworm_start_position_[1]) *
             inchworm_direction_world_[1];
}

void AttitudePressureController::updateInchworm(
    const ros::Time& now, double dt, bool, bool)
{
  if (hybridLocomotionActive())
    {
      updateHybridLocomotion(now, dt);
      return;
    }
  auto approach = [dt](double value, double target, double rate) {
    const double step = std::max(0.0, rate * dt);
    return value < target ? std::min(target, value + step)
                          : std::max(target, value - step);
  };
  auto updateActionDPressure = [this, &approach](size_t arm, double angle,
                                                 double) {
    double desired_pressure = action_d_initial_pressure_[arm];
    if (inchworm_front_[arm])
      {
        // Preload the front inflatable before the matching thrust arrives.
        // With the defaults this reaches 50 kPa near 6 N, two newtons before
        // the requested 8 N endpoint. Angle feedback is added around that
        // feed-forward value. Bend-angle convention is zero when straight and
        // increases with flexion, so an initially over-bent arm asks for less
        // pressure while an arm approaching extension asks for more.
        // Preserve pressure for the selected pulse amplitude during FALL and
        // OFF. Removing rotor thrust against this retained chamber pressure
        // produces the requested prompt re-bend.
        const double anticipated_thrust = std::min(
            action_d_max_front_thrust_n_,
            std::max(action_d_front_thrust_n_,
                     action_d_thrust_step_target_n_) +
                action_d_front_pressure_lead_n_);
        desired_pressure += action_d_front_pressure_per_thrust_kpa_n_ *
            anticipated_thrust;
        if (angle > action_d_soft_max_angle_rad_)
          desired_pressure -= action_d_pressure_kp_kpa_rad_ *
              (angle - action_d_soft_max_angle_rad_);
        else if (angle < action_d_soft_min_angle_rad_)
          desired_pressure += action_d_pressure_kp_kpa_rad_ *
              (action_d_soft_min_angle_rad_ - angle);
      }
    else
      {
        // Rear rotors apply reverse thrust. Their initial contact geometry is
        // already below 90 deg, so do not chase the common bend target. The
        // default drop gain is zero: keep the measured initial grasp pressure
        // while using reverse thrust to share propulsion and support posture.
        desired_pressure -= action_d_rear_pressure_drop_per_thrust_kpa_n_ *
            action_d_rear_thrust_n_;
        desired_pressure = std::max(action_d_rear_min_pressure_kpa_,
                                    desired_pressure);
      }
    desired_pressure = std::max(0.0, std::min(
        action_d_max_pressure_kpa_, desired_pressure));
    action_d_pressure_[arm] = approach(
        action_d_pressure_[arm], desired_pressure,
        action_d_pressure_rate_kpa_s_);
  };
  const double elapsed = (now - state_start_).toSec();
  const double pressure_step = pressure_rate_limit_kpa_s_ * dt;
  const double travel = inchwormTravel();
  auto frontAlreadyPlaced = [this](size_t arm) {
    for (size_t sequence = 0; sequence < inchworm_front_sequence_; ++sequence)
      if (inchworm_front_order_[sequence] == arm) return true;
    return false;
  };
  if (mocap_stamp_ != inchworm_rate_stamp_)
    {
      const double sample_dt = (mocap_stamp_ - inchworm_rate_stamp_).toSec();
      if (sample_dt > 1e-4)
        {
          const double raw_speed = (travel - inchworm_last_travel_m_) / sample_dt;
          inchworm_travel_speed_m_s_ += 0.25 *
              (raw_speed - inchworm_travel_speed_m_s_);
        }
      inchworm_last_travel_m_ = travel;
      inchworm_rate_stamp_ = mocap_stamp_;
    }

  switch (state_)
    {
    case ACTION_D_PREPARE:
      {
      inchworm_thrust_.fill(0.0);
      publishBottomDirection(1.0);
      publishBottomTarget(action_d_bottom_pressure_kpa_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          // Start correcting the front pair while the bottom chamber is being
          // prepared. In particular, a front arm that starts above the
          // 105-deg bend band is depressurized before rotor thrust begins.
          // Rear pressure remains referenced to its initial contact state and
          // is reduced only as reverse thrust is introduced.
          updateActionDPressure(arm, measuredArmBendAngle(arm),
                                action_d_target_angle_rad_);
        }
      publishPressureTarget(action_d_pressure_,
                            std::numeric_limits<double>::infinity(),
                            action_d_max_pressure_kpa_);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2,
          "Action D PREPARE: bend [%.1f %.1f %.1f %.1f] deg, pressure-FB band %.1f--%.1f deg, pressure target [%.1f %.1f %.1f %.1f], P->bend sign [%.0f %.0f %.0f %.0f], bottom %.1f/%.1f kPa",
          measuredArmBendAngle(0) * 180.0 / M_PI,
          measuredArmBendAngle(1) * 180.0 / M_PI,
          measuredArmBendAngle(2) * 180.0 / M_PI,
          measuredArmBendAngle(3) * 180.0 / M_PI,
          action_d_soft_min_angle_rad_ * 180.0 / M_PI,
          action_d_soft_max_angle_rad_ * 180.0 / M_PI,
          action_d_pressure_[0], action_d_pressure_[1],
          action_d_pressure_[2], action_d_pressure_[3],
          action_d_pressure_bend_sign_[0], action_d_pressure_bend_sign_[1],
          action_d_pressure_bend_sign_[2], action_d_pressure_bend_sign_[3],
          bottom_pressure_kpa_, action_d_bottom_pressure_kpa_);
      if (std::abs(bottom_pressure_kpa_ - action_d_bottom_pressure_kpa_) <=
              pressure_tolerance_kpa_)
        {
          if (ready_start_.isZero()) ready_start_ = now;
          if ((now - ready_start_).toSec() >= action_d_prepare_stable_sec_)
            {
              action_d_previous_angle_time_ = now;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  action_d_previous_angle_rad_[arm] = measuredArmBendAngle(arm);
                  action_d_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
                }
              // Pneumatic shaping and bottom inflation may move the body.
              // Action D measures only the subsequent thrust-driven slide.
              inchworm_start_position_ = mocap_position_;
              inchworm_rate_stamp_ = mocap_stamp_;
              inchworm_last_travel_m_ = 0.0;
              inchworm_travel_speed_m_s_ = 0.0;
              // Bottom inflation and pressure shaping intentionally change
              // the posture during PREPARE. Protect the subsequent thrust
              // motion relative to the actual RAMP starting posture.
              action_d_start_roll_ = roll_;
              action_d_start_pitch_ = pitch_;
              action_d_posture_limited_ = false;
              action_d_pulse_phase_ = ACTION_D_PULSE_RISE;
              action_d_pulse_phase_start_ = now;
              action_d_pulse_start_travel_m_ = 0.0;
              action_d_rapid_extension_start_.fill(ros::Time(0));
              action_d_filtered_extension_rate_rad_s_.fill(0.0);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  action_d_ramp_reference_angle_rad_[arm] =
                      measuredArmBendAngle(arm);
                  action_d_previous_angle_rad_[arm] =
                      action_d_ramp_reference_angle_rad_[arm];
                  action_d_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
                }
              enterState(ACTION_D_RAMP, now);
            }
        }
      else
        ready_start_ = ros::Time(0);
      if (state_ == ACTION_D_PREPARE &&
          elapsed >= action_d_prepare_timeout_sec_)
        stopInchwormToGrasp(now,
            "Action D could not establish the configured arm safety band");
      }
      break;

    case ACTION_D_RAMP:
      {
      publishBottomDirection(1.0);
      publishBottomTarget(action_d_bottom_pressure_kpa_);
      const double control_target = action_d_target_angle_rad_;
      double maximum_extension_rate = 0.0;
      size_t maximum_extension_arm = 0;
      double maximum_extension_arm_angle = 0.0;
      double maximum_extension_boundary = action_d_hard_min_angle_rad_;
      bool maximum_extension_arm_is_rear = false;
      bool rapid_extension_fault = false;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          const double angle = measuredArmBendAngle(arm);
          double bend_rate = 0.0;
          const bool new_neuron_sample =
              neuron_stamp_[arm] != action_d_previous_neuron_stamp_[arm];
          if (new_neuron_sample)
            {
              const double sample_dt = std::max(1.0e-4,
                  (neuron_stamp_[arm] -
                   action_d_previous_neuron_stamp_[arm]).toSec());
              bend_rate = (angle - action_d_previous_angle_rad_[arm]) /
                  sample_dt;
              action_d_previous_angle_rad_[arm] = angle;
              action_d_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
            }
          // Neuron bend angle is zero when straight and increases with
          // flexion. Extension is therefore a *decrease* in bend angle.
          const double extension_rate = -bend_rate;
          if (new_neuron_sample)
            action_d_filtered_extension_rate_rad_s_[arm] =
                action_d_rear_rate_filter_alpha_ * extension_rate +
                (1.0 - action_d_rear_rate_filter_alpha_) *
                action_d_filtered_extension_rate_rad_s_[arm];
          const bool front_arm = inchworm_front_[arm];
          const bool protected_rear_arm = !front_arm &&
              action_d_max_rear_thrust_n_ > 0.0 &&
              action_d_rear_pressure_drop_per_thrust_kpa_n_ <= 1.0e-6;
          const bool protected_arm = front_arm || protected_rear_arm;
          const double safety_extension_rate = front_arm ? extension_rate :
              action_d_filtered_extension_rate_rad_s_[arm];
          const double extension_boundary = front_arm ?
              action_d_hard_min_angle_rad_ : std::max(
                  0.0, action_d_ramp_reference_angle_rad_[arm] -
                  action_d_rear_extension_margin_rad_);
          const bool outside_extension_boundary =
              angle < extension_boundary;
          const bool extension_rate_high =
              safety_extension_rate > action_d_max_angle_rate_rad_s_;

          // Front arms retain the absolute 60-deg edge and immediate stop.
          // Rear-arm geometry can start below 60 deg, so its edge is relative
          // to the settled RAMP-start angle. Rear rate must remain high beyond
          // that relative edge for the confirmation interval; a one-sample
          // Neuron spike cannot stop the gait.
          const bool monitor_rate = front_arm ? extension_rate_high :
              (extension_rate_high && outside_extension_boundary);
          if (protected_arm && monitor_rate)
            {
              if (action_d_rapid_extension_start_[arm].isZero())
                action_d_rapid_extension_start_[arm] = now;
            }
          else
            action_d_rapid_extension_start_[arm] = ros::Time(0);
          const bool extension_rate_confirmed = protected_arm &&
              !action_d_rapid_extension_start_[arm].isZero() &&
              (now - action_d_rapid_extension_start_[arm]).toSec() >=
                  action_d_angle_rate_confirm_sec_;
          const bool extension_rate_immediate = front_arm &&
              safety_extension_rate > action_d_immediate_angle_rate_rad_s_;

          if (!rapid_extension_fault && protected_arm &&
              safety_extension_rate > maximum_extension_rate)
            {
              maximum_extension_rate = safety_extension_rate;
              maximum_extension_arm = arm;
              maximum_extension_arm_angle = angle;
              maximum_extension_boundary = extension_boundary;
              maximum_extension_arm_is_rear = !front_arm;
            }
          if (protected_arm && outside_extension_boundary &&
              (extension_rate_confirmed || extension_rate_immediate))
            {
              rapid_extension_fault = true;
              maximum_extension_rate = safety_extension_rate;
              maximum_extension_arm = arm;
              maximum_extension_arm_angle = angle;
              maximum_extension_boundary = extension_boundary;
              maximum_extension_arm_is_rear = !front_arm;
            }
          updateActionDPressure(arm, angle, control_target);
        }
      action_d_previous_angle_time_ = now;

      if (rapid_extension_fault)
        {
          ROS_ERROR("ACTION D HARD STOP: %s arm %zu extension %.1f deg/s at %.1f deg crossed %.1f-deg %s safety edge (limit %.1f deg/s for %.2f s%s)",
                    maximum_extension_arm_is_rear ? "rear" : "front",
                    maximum_extension_arm,
                    maximum_extension_rate * 180.0 / M_PI,
                    maximum_extension_arm_angle * 180.0 / M_PI,
                    maximum_extension_boundary * 180.0 / M_PI,
                    maximum_extension_arm_is_rear ? "RAMP-start-relative" :
                    "absolute straight-side",
                    action_d_max_angle_rate_rad_s_ * 180.0 / M_PI,
                    action_d_angle_rate_confirm_sec_,
                    maximum_extension_arm_is_rear ? "" :
                    ", or immediate front threshold");
          stopInchwormToGrasp(now, "Action D rapid arm-angle change");
          break;
        }

      const double body_tilt = std::hypot(
          wrapAngle(roll_ - action_d_start_roll_),
          wrapAngle(pitch_ - action_d_start_pitch_));
      const double body_rate = std::hypot(body_angular_velocity_[0],
                                          body_angular_velocity_[1]);
      const double predicted_body_tilt = body_tilt +
          action_d_prediction_horizon_sec_ * body_rate;
      if (predicted_body_tilt >= action_d_tilt_soft_limit_rad_)
        action_d_posture_limited_ = true;
      else if (predicted_body_tilt <= action_d_tilt_resume_rad_)
        action_d_posture_limited_ = false;

      // The successful-looking second trial stayed near 0.1 rad/s for 95%
      // of RAMP, then spiked to 2.32 rad/s as the posture collapsed. Leave
      // RAMP before that rotation develops further and lower thrust through
      // the controlled REGRIP path instead of cutting it instantaneously.
      if (body_tilt >= action_d_max_tilt_rad_ ||
          body_rate >= action_d_max_body_rate_rad_s_)
        {
          ROS_WARN("Action D posture guard: tilt %.1f/%.1f deg, roll/pitch rate %.2f/%.2f rad/s; entering controlled REGRIP at %.2f N",
                   body_tilt * 180.0 / M_PI,
                   action_d_max_tilt_rad_ * 180.0 / M_PI,
                   body_rate, action_d_max_body_rate_rad_s_,
                   action_d_front_thrust_n_);
          enterState(ACTION_D_REGRIP, now);
          break;
        }

      // Do not let mocap settling at the PREPARE->RAMP transition lock a
      // near-zero thrust. Require meaningful applied thrust, positive travel,
      // and sustained positive velocity before declaring that sliding began.
      const double action_d_pulse_travel =
          travel - action_d_pulse_start_travel_m_;
      const bool forward_motion_candidate =
          action_d_front_thrust_n_ >= action_d_motion_min_front_thrust_n_ &&
          action_d_pulse_travel >= action_d_motion_min_travel_m_ &&
          inchworm_travel_speed_m_s_ >= inchworm_motion_start_speed_m_s_;
      if (!action_d_limit_locked_ && forward_motion_candidate)
        {
          if (action_d_motion_candidate_start_.isZero())
            action_d_motion_candidate_start_ = now;
        }
      else
        action_d_motion_candidate_start_ = ros::Time(0);
      const bool forward_motion = !action_d_motion_candidate_start_.isZero() &&
          (now - action_d_motion_candidate_start_).toSec() >=
              action_d_motion_confirm_sec_;
      if (!action_d_limit_locked_ && forward_motion)
        {
          action_d_locked_front_thrust_n_ = action_d_front_thrust_n_;
          action_d_locked_rear_thrust_n_ = action_d_rear_thrust_n_;
          action_d_limit_locked_ = true;
          action_d_pulse_phase_ = ACTION_D_PULSE_FALL;
          action_d_pulse_phase_start_ = now;
          ROS_WARN("Action D forward motion detected at %.2f mm/s with front %.2f N, rear %.2f N; ending pulse and re-bending",
                   1000.0 * inchworm_travel_speed_m_s_,
                   action_d_locked_front_thrust_n_,
                   action_d_locked_rear_thrust_n_);
        }

      // A soft posture-limit event ends the current ON interval instead of
      // slowly bleeding thrust while it continues to rotate the body.
      if (action_d_posture_limited_ &&
          (action_d_pulse_phase_ == ACTION_D_PULSE_RISE ||
           action_d_pulse_phase_ == ACTION_D_PULSE_ON))
        {
          action_d_pulse_phase_ = ACTION_D_PULSE_FALL;
          action_d_pulse_phase_start_ = now;
          ROS_WARN("Action D pulse %.1f N ended early by posture soft limit; rapidly removing thrust for re-bend",
                   action_d_thrust_step_target_n_);
        }

      bool pulse_cycle_complete = false;
      const double rear_pulse_target = action_d_max_front_thrust_n_ > 1.0e-6
          ? action_d_max_rear_thrust_n_ *
                action_d_thrust_step_target_n_ /
                action_d_max_front_thrust_n_
          : 0.0;
      switch (action_d_pulse_phase_)
        {
        case ACTION_D_PULSE_RISE:
          action_d_front_thrust_n_ = approach(
              action_d_front_thrust_n_, action_d_thrust_step_target_n_,
              action_d_front_thrust_ramp_n_s_);
          action_d_rear_thrust_n_ = approach(
              action_d_rear_thrust_n_, rear_pulse_target,
              action_d_rear_thrust_ramp_n_s_);
          if (std::abs(action_d_front_thrust_n_ -
                       action_d_thrust_step_target_n_) <= 0.01 &&
              std::abs(action_d_rear_thrust_n_ -
                       rear_pulse_target) <= 0.01)
            {
              action_d_pulse_phase_ = ACTION_D_PULSE_ON;
              action_d_pulse_phase_start_ = now;
              ROS_INFO("Action D pulse %.1f N ON for %.2f s",
                       action_d_thrust_step_target_n_,
                       action_d_pulse_on_sec_);
            }
          break;

        case ACTION_D_PULSE_ON:
          action_d_front_thrust_n_ = action_d_thrust_step_target_n_;
          action_d_rear_thrust_n_ = rear_pulse_target;
          if ((now - action_d_pulse_phase_start_).toSec() >=
              action_d_pulse_on_sec_)
            {
              action_d_pulse_phase_ = ACTION_D_PULSE_FALL;
              action_d_pulse_phase_start_ = now;
              ROS_INFO("Action D pulse %.1f N OFF; retaining pressure for re-bend",
                       action_d_thrust_step_target_n_);
            }
          break;

        case ACTION_D_PULSE_FALL:
          action_d_front_thrust_n_ = approach(
              action_d_front_thrust_n_, 0.0,
              action_d_thrust_down_rate_n_s_);
          action_d_rear_thrust_n_ = approach(
              action_d_rear_thrust_n_, 0.0,
              action_d_thrust_down_rate_n_s_);
          if (action_d_front_thrust_n_ <= 0.01 &&
              action_d_rear_thrust_n_ <= 0.01)
            {
              action_d_front_thrust_n_ = 0.0;
              action_d_rear_thrust_n_ = 0.0;
              action_d_pulse_phase_ = ACTION_D_PULSE_OFF;
              action_d_pulse_phase_start_ = now;
              ROS_INFO("Action D pulse OFF interval %.2f s started at retained front pressure [%.1f %.1f] kPa",
                       action_d_pulse_off_sec_, action_d_pressure_[0],
                       action_d_pressure_[3]);
            }
          break;

        case ACTION_D_PULSE_OFF:
        default:
          action_d_front_thrust_n_ = 0.0;
          action_d_rear_thrust_n_ = 0.0;
          pulse_cycle_complete =
              (now - action_d_pulse_phase_start_).toSec() >=
                  action_d_pulse_off_sec_;
          break;
        }

      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_thrust_[arm] = inchworm_front_[arm]
            ? action_d_front_thrust_n_ : -action_d_rear_thrust_n_;
      publishPressureTarget(action_d_pressure_,
                            std::numeric_limits<double>::infinity(),
                            action_d_max_pressure_kpa_);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2,
          "Action D RAMP pulse phase %u: thrust [%.2f %.2f %.2f %.2f] N (amplitude %.1f/%.1f), bend [%.1f %.1f %.1f %.1f] deg, max arm rate %.1f deg/s, pressure-FB [%.1f %.1f %.1f %.1f] kPa, bottom %.1f kPa, pulse travel %.2f/%.2f mm, tilt %.1f deg (predicted %.1f), body rate %.2f rad/s, posture-limited %d, motion-candidate %d locked %d",
          action_d_pulse_phase_,
          inchworm_thrust_[0], inchworm_thrust_[1], inchworm_thrust_[2],
          inchworm_thrust_[3],
          action_d_thrust_step_target_n_, action_d_max_front_thrust_n_,
          measuredArmBendAngle(0) * 180.0 / M_PI,
          measuredArmBendAngle(1) * 180.0 / M_PI,
          measuredArmBendAngle(2) * 180.0 / M_PI,
          measuredArmBendAngle(3) * 180.0 / M_PI,
          maximum_extension_rate * 180.0 / M_PI,
          action_d_pressure_[0], action_d_pressure_[1],
          action_d_pressure_[2], action_d_pressure_[3],
          bottom_pressure_kpa_, 1000.0 * action_d_pulse_travel,
          1000.0 * inchworm_target_distance_m_,
          body_tilt * 180.0 / M_PI,
          predicted_body_tilt * 180.0 / M_PI,
          body_rate, action_d_posture_limited_,
          !action_d_motion_candidate_start_.isZero(), action_d_limit_locked_);
      if (pulse_cycle_complete)
        {
          const bool retained_motion = action_d_limit_locked_ ||
              action_d_pulse_travel >= action_d_motion_min_travel_m_;
          if (retained_motion)
            {
              ROS_WARN("Action D pulse %.1f N retained %.2f mm after OFF; regripping",
                       action_d_thrust_step_target_n_,
                       1000.0 * action_d_pulse_travel);
              enterState(ACTION_D_REGRIP, now);
            }
          else if (action_d_posture_limited_)
            {
              ROS_WARN("Action D posture did not recover during OFF at %.1f N; regripping without increasing amplitude",
                       action_d_thrust_step_target_n_);
              enterState(ACTION_D_REGRIP, now);
            }
          else if (action_d_thrust_step_target_n_ <
                   action_d_max_front_thrust_n_ - 1.0e-6)
            {
              action_d_thrust_step_target_n_ = std::min(
                  action_d_max_front_thrust_n_,
                  action_d_thrust_step_target_n_ + action_d_thrust_step_n_);
              action_d_pulse_phase_ = ACTION_D_PULSE_RISE;
              action_d_pulse_phase_start_ = now;
              action_d_pulse_start_travel_m_ = travel;
              action_d_motion_candidate_start_ = ros::Time(0);
              ROS_WARN("Action D no retained motion; next pulse amplitude %.1f N",
                       action_d_thrust_step_target_n_);
            }
          else
            {
              ROS_WARN("Action D reached the %.1f-N pulse limit without retained motion; regripping",
                       action_d_max_front_thrust_n_);
              enterState(ACTION_D_REGRIP, now);
            }
        }
      else if (!action_d_limit_locked_ &&
               elapsed >= action_d_ramp_timeout_sec_)
        enterState(ACTION_D_REGRIP, now);
      }
      break;

    case ACTION_D_REGRIP:
      publishBottomDirection(1.0);
      publishBottomTarget(action_d_bottom_pressure_kpa_);
      action_d_front_thrust_n_ = approach(
          action_d_front_thrust_n_, 0.0, action_d_thrust_down_rate_n_s_);
      action_d_rear_thrust_n_ = approach(
          action_d_rear_thrust_n_, 0.0, action_d_thrust_down_rate_n_s_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_thrust_[arm] = inchworm_front_[arm]
            ? action_d_front_thrust_n_ : -action_d_rear_thrust_n_;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        updateActionDPressure(arm, measuredArmBendAngle(arm),
                              action_d_target_angle_rad_);
      publishPressureTarget(action_d_pressure_,
                            std::numeric_limits<double>::infinity(),
                            action_d_max_pressure_kpa_);
      publishCorrectionThrust(inchworm_thrust_);
      if (action_d_front_thrust_n_ <= 0.01 &&
          action_d_rear_thrust_n_ <= 0.01)
        {
          publishBottomTarget(0.0);
          ROS_INFO("Action D complete: retained travel %.2f mm; locked front/rear thrust %.2f/%.2f N",
                   1000.0 * travel, action_d_locked_front_thrust_n_,
                   action_d_locked_rear_thrust_n_);
          inchworm_active_ = false;
          last_correction_end_ = now;
          enterState(GRASP, now);
        }
      break;

    case FRONT_RELEASE:
      inchworm_thrust_.fill(0.0);
      publishBottomTarget(action_a_bottom_low_kpa_);
      {
      const bool use_arm2_diagonal_assist =
          action_a_diagonal_assist_enabled_ && inchworm_active_arm_ == 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (arm == inchworm_active_arm_ ||
              (use_arm2_diagonal_assist && arm == 2))
            inchworm_pressure_[arm] = std::max(
                minimum_prepare_pressure_kpa_, inchworm_swing_pressure_kpa_);
          else if (frontAlreadyPlaced(arm))
            inchworm_pressure_[arm] = inchworm_transfer_pressure_kpa_;
          else
            inchworm_pressure_[arm] = base_pressure_kpa_;
        }
      publishCorrectionThrust(inchworm_thrust_);
      // The target itself changes immediately to zero. The pneumatic plant
      // still exhausts at its physical rate, and this state waits for the
      // measured pressure before starting the rotor.
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      ROS_INFO_THROTTLE(0.5,
          "FRONT_RELEASE waiting: pressure target [%.1f, %.1f, %.1f, %.1f], measured [%.1f, %.1f, %.1f, %.1f] kPa",
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3],
          measured_pressure_[0], measured_pressure_[1],
          measured_pressure_[2], measured_pressure_[3]);
      if (pressureReached(inchworm_pressure_))
        {
          // Keep the GRASP Neuron reference: pressure-driven extension is a
          // real part of the motion toward the absolute 5-deg bend target.
          inchworm_swing_start_roll_ = roll_;
          inchworm_swing_start_pitch_ = pitch_;
          action_a_reach_thrust_ceiling_n_ = inchworm_reach_max_thrust_n_;
          action_a_previous_body_tilt_rad_ = 0.0;
          action_a_best_reach_bend_rad_ = M_PI;
          action_a_yaw_start_rad_ = yaw_;
          action_a_yaw_assist_active_ = action_a_yaw_assist_enabled_;
          // Positive body yaw moves a point r by z x r. Choose the yaw sign
          // whose displacement has a positive projection on the requested
          // travel direction. For +x this is -yaw for arm 0 (+y), and +yaw
          // for arm 3 (-y).
          const double yaw_advance_per_rad =
              inchworm_direction_body_[1] *
                  arm_base_position_[inchworm_active_arm_][0] -
              inchworm_direction_body_[0] *
                  arm_base_position_[inchworm_active_arm_][1];
          action_a_yaw_signed_target_rad_ =
              action_a_yaw_assist_active_ &&
              std::abs(yaw_advance_per_rad) > 1.0e-6
                  ? std::copysign(action_a_yaw_target_rad_,
                                  yaw_advance_per_rad)
                  : 0.0;
          inchworm_angular_rate_fault_start_ = ros::Time(0);
          ROS_INFO("Arm%zu released at %.2f kPa; targeting %.1f deg absolute bend, %+.1f deg relative yaw, and bottom %.1f kPa",
                   inchworm_active_arm_,
                   measured_pressure_[inchworm_active_arm_],
                   inchworm_target_bend_angle_rad_ * 180.0 / M_PI,
                   action_a_yaw_signed_target_rad_ * 180.0 / M_PI,
                   action_a_bottom_high_kpa_);
          publishBottomTarget(action_a_bottom_high_kpa_);
          enterState(FRONT_REACH, now);
        }
      else if (elapsed >= inchworm_release_timeout_sec_)
        {
          ROS_ERROR("FRONT_RELEASE timed out");
          stopInchwormToGrasp(now, "FRONT_RELEASE timeout");
        }
      }
      break;

    case FRONT_REACH:
      {
      publishBottomTarget(action_a_bottom_high_kpa_);
      // Continue pressure reduction and thrust ramp concurrently. Reasserting
      // all four targets prevents unintended support arms from being unloaded.
      // During arm0 placement, arm2 is deliberately released as a diagonal
      // helper so the body can yaw without a large dedicated yaw thrust.
      const bool use_arm2_diagonal_assist =
          action_a_diagonal_assist_enabled_ && inchworm_active_arm_ == 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (arm == inchworm_active_arm_ ||
              (use_arm2_diagonal_assist && arm == 2))
            inchworm_pressure_[arm] = std::max(
                minimum_prepare_pressure_kpa_, inchworm_swing_pressure_kpa_);
          else if (frontAlreadyPlaced(arm))
            inchworm_pressure_[arm] = inchworm_transfer_pressure_kpa_;
          else
            inchworm_pressure_[arm] = base_pressure_kpa_;
        }
      PressureArray relative_angle{{0.0, 0.0, 0.0, 0.0}};
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        relative_angle[arm] = measuredArmAngle(arm) - grasp_imu_angle_[arm];
      // measuredArmAngle() is the Neuron attitude relative to the Spinal body
      // attitude.  Therefore zero is a straight arm and the magnitude is the
      // absolute bend angle; do not anchor this target to the pressure model
      // or to the bend at the beginning of GRASP.
      const double estimated_bend = std::abs(
          wrapAngle(measuredArmAngle(inchworm_active_arm_)));
      action_a_best_reach_bend_rad_ = std::min(
          action_a_best_reach_bend_rad_, estimated_bend);
      const double grasp_bend = std::abs(
          wrapAngle(grasp_imu_angle_[inchworm_active_arm_]));
      // The physical controller normally uses an absolute bend target.  A
      // positive relative target is available to simulation experiments,
      // where the initial bend changes with the randomized forearm radius.
      const double effective_target_bend =
          inchworm_target_straightening_angle_rad_ > 0.0
              ? std::max(0.0, grasp_bend -
                                  inchworm_target_straightening_angle_rad_)
              : inchworm_target_bend_angle_rad_;
      const double straightening = std::max(0.0,
          grasp_bend - estimated_bend);
      const double required_straightening = std::max(0.0,
          grasp_bend - effective_target_bend);
      const double body_tilt = std::hypot(
          wrapAngle(roll_ - inchworm_swing_start_roll_),
          wrapAngle(pitch_ - inchworm_swing_start_pitch_));
      const double body_tilt_rate = dt > 1.0e-4
          ? (body_tilt - action_a_previous_body_tilt_rad_) / dt : 0.0;
      action_a_previous_body_tilt_rad_ = body_tilt;
      // Anticipate a growing slip/rocking motion, then monotonically reduce
      // the available thrust for the rest of this swing.  Do not restore the
      // old high thrust after static friction has broken and tilt happens to
      // decrease again.
      const double predicted_body_tilt = body_tilt +
          action_a_tilt_prediction_horizon_sec_ *
          std::max(0.0, body_tilt_rate);
      const double tilt_range = std::max(
          1.0e-6, action_a_tilt_hard_limit_rad_ -
                    action_a_tilt_soft_limit_rad_);
      const double tilt_fraction = std::max(0.0, std::min(1.0,
          (predicted_body_tilt - action_a_tilt_soft_limit_rad_) /
              tilt_range));
      const double tilt_limited_thrust =
          inchworm_reach_max_thrust_n_ * (1.0 - tilt_fraction);
      action_a_reach_thrust_ceiling_n_ = std::min(
          action_a_reach_thrust_ceiling_n_, tilt_limited_thrust);
      grasp_baseline_valid_ = true;
      prepare_arm_active_.fill(false);
      prepare_arm_active_[inchworm_active_arm_] = true;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        prepare_straighten_angle_by_arm_[arm] = arm == inchworm_active_arm_
            ? required_straightening : 0.0;
      publishArmShapeDebug(relative_angle);
      const double angle_error = std::max(0.0,
          required_straightening - straightening);
      const bool arm_angle_changed = straightening >= prepare_angle_tolerance_;
      const bool bend_ready = arm_angle_changed && estimated_bend <=
          effective_target_bend + prepare_angle_tolerance_;
      const double yaw_target_sign = action_a_yaw_signed_target_rad_ > 0.0
          ? 1.0 : (action_a_yaw_signed_target_rad_ < 0.0 ? -1.0 : 0.0);
      const double yaw_delta = wrapAngle(yaw_ - action_a_yaw_start_rad_);
      const double yaw_progress = yaw_target_sign * yaw_delta;
      const double yaw_error = std::max(0.0,
          std::abs(action_a_yaw_signed_target_rad_) - yaw_progress);
      const bool yaw_ready = yaw_target_sign == 0.0 ||
          yaw_error <= action_a_yaw_tolerance_rad_;
      const double yaw_rate_toward_target =
          yaw_target_sign * body_angular_velocity_[2];
      if (yaw_target_sign != 0.0 &&
          yaw_progress < -action_a_yaw_wrong_way_limit_rad_)
        {
          // Yaw is only a placement aid. A wrong sign must not discard a
          // valid arm-extension attempt or prevent the state machine from
          // reaching FRONT_PLACE. Disable it for this swing and continue
          // with bend/tilt feedback. The real-machine trial used manual yaw,
          // so it cannot be used to infer the rotor-to-yaw sign.
          ROS_WARN("FRONT_REACH yaw assist moved the wrong way: target %+.1f deg, measured %+.1f deg; disabling yaw assist for this swing",
                   action_a_yaw_signed_target_rad_ * 180.0 / M_PI,
                   yaw_delta * 180.0 / M_PI);
          action_a_yaw_assist_active_ = false;
          action_a_yaw_signed_target_rad_ = 0.0;
        }
      double yaw_reverse_target_n = 0.0;
      if (action_a_yaw_assist_active_ && !yaw_ready &&
          yaw_rate_toward_target < action_a_yaw_max_rate_rad_s_)
        yaw_reverse_target_n = std::min(
            action_a_yaw_max_reverse_thrust_n_,
            action_a_yaw_kp_n_rad_ * yaw_error);
      // Action A places a contact point with the arm itself. Body tilt is a
      // diagnostic here, not a completion condition: requiring tilt caused
      // the thrust to remain at its maximum even after the arm passed through
      // the requested near-straight angle.
      const double adaptive_thrust = std::max(
          0.0, action_a_reach_thrust_ceiling_n_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (arm == inchworm_active_arm_)
            {
              // Once the bend target is reached, do not keep increasing the
              // extension thrust while the slower yaw motion finishes.
              const double target_thrust = bend_ready && !yaw_ready
                  ? std::min(adaptive_thrust, inchworm_thrust_[arm])
                  : adaptive_thrust;
              const double rate = target_thrust < inchworm_thrust_[arm]
                  ? action_a_tilt_thrust_down_rate_n_s_
                  : inchworm_thrust_ramp_n_s_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], target_thrust, rate);
            }
          else if (use_arm2_diagonal_assist && arm == 2)
            {
              // Arm2 is opposite arm0. Extend it gently in the same phase;
              // this removes the diagonal contact constraint and supplies a
              // small yaw moment without the former 1+3 reverse-thrust pair.
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm],
                  action_a_diagonal_assist_thrust_n_,
                  action_a_diagonal_assist_thrust_ramp_n_s_);
            }
          else
            {
              // Around the approximately 90-deg grasp posture, equal reverse
              // thrust on 1+3 creates -yaw, while 0+2 creates +yaw. The
              // active swing arm is not part of either assisting pair for
              // the normal +x Action-A order (arm 0 then arm 3).
              const bool yaw_assist_rotor = yaw_target_sign < 0.0
                  ? (arm == 1 || arm == 3)
                  : (yaw_target_sign > 0.0 && (arm == 0 || arm == 2));
              const double target_thrust = yaw_assist_rotor
                  ? -yaw_reverse_target_n : 0.0;
              const double rate = std::abs(target_thrust) <
                                       std::abs(inchworm_thrust_[arm])
                  ? action_a_yaw_thrust_down_rate_n_s_
                  : action_a_yaw_thrust_ramp_n_s_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], target_thrust, rate);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2, "Inchworm FRONT_REACH arm%zu bend %.1f/%.1f deg (best %.1f), body tilt %.2f deg (predicted %.2f), yaw %+.1f/%+.1f deg, extension thrust %.2f/%.2f N, arm2 diagonal thrust %.2f N, legacy yaw reverse %.2f N",
                        inchworm_active_arm_,
                        estimated_bend * 180.0 / M_PI,
                        effective_target_bend * 180.0 / M_PI,
                        action_a_best_reach_bend_rad_ * 180.0 / M_PI,
                        body_tilt * 180.0 / M_PI,
                        predicted_body_tilt * 180.0 / M_PI,
                        yaw_delta * 180.0 / M_PI,
                        action_a_yaw_signed_target_rad_ * 180.0 / M_PI,
                        inchworm_thrust_[inchworm_active_arm_],
                        adaptive_thrust,
                        use_arm2_diagonal_assist ? inchworm_thrust_[2] : 0.0,
                        yaw_reverse_target_n);
      const bool front_pressure_following =
          measured_pressure_[inchworm_active_arm_] <=
              inchworm_pressure_[inchworm_active_arm_] + pressure_tolerance_kpa_;
      ROS_INFO_THROTTLE(0.5,
          "FRONT_REACH pressure target [%.1f, %.1f, %.1f, %.1f], measured [%.1f, %.1f, %.1f, %.1f], bottom %.1f/%.1f kPa, thrust [%.2f, %.2f, %.2f, %.2f]",
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3],
          measured_pressure_[0], measured_pressure_[1],
          measured_pressure_[2], measured_pressure_[3],
          bottom_pressure_kpa_, action_a_bottom_high_kpa_,
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3]);
      // On the real arm the body-tilt limiter deliberately reduced thrust at
      // about 8 deg. In a2, arm0 had already reached 38 deg bend and advanced
      // mocap x by about 12 mm, but the controller kept waiting for the ideal
      // 20-deg target and timed out. Preserve the best bend reached during
      // the swing and accept it once the soft tilt boundary confirms that the
      // safe thrust ceiling, rather than lack of extension, is now limiting.
      const bool tilt_limited_extension_ready =
          predicted_body_tilt >= action_a_tilt_soft_limit_rad_ &&
          action_a_best_reach_bend_rad_ <=
              action_a_tilt_limited_bend_target_rad_;
      const bool tilt_requires_placement = tilt_limited_extension_ready ||
          (predicted_body_tilt >= action_a_tilt_hard_limit_rad_ &&
           straightening >= inchworm_min_usable_reach_angle_rad_);
      // The yaw target is intentionally not a completion gate. The contact
      // can be placed as soon as the arm reaches its bend target; otherwise a
      // slow or incorrectly signed yaw response keeps thrust on after the arm
      // has already reached its best extension and causes a timeout.
      if ((bend_ready || tilt_requires_placement) &&
          front_pressure_following)
        {
          action_a_place_start_thrust_n_ =
              inchworm_thrust_[inchworm_active_arm_];
          action_a_place_start_bend_rad_ = estimated_bend;
          ROS_INFO("FRONT_REACH achieved on arm%zu: bend %.1f deg (best %.1f) with %.2f N, body tilt %.1f deg, yaw %+.1f/%+.1f deg%s; lowering to place contact",
                   inchworm_active_arm_, estimated_bend * 180.0 / M_PI,
                   action_a_best_reach_bend_rad_ * 180.0 / M_PI,
                   action_a_place_start_thrust_n_,
                   body_tilt * 180.0 / M_PI,
                   yaw_delta * 180.0 / M_PI,
                   action_a_yaw_signed_target_rad_ * 180.0 / M_PI,
                   tilt_requires_placement ? " (safe tilt-limited reach)" : "");
          enterState(ACTION_A_FRONT_PLACE, now);
        }
      else if (std::hypot(body_angular_velocity_[0],
                          body_angular_velocity_[1]) >=
               correction_max_angular_rate_)
        {
          if (inchworm_angular_rate_fault_start_.isZero())
            inchworm_angular_rate_fault_start_ = now;
          else if ((now - inchworm_angular_rate_fault_start_).toSec() >=
                   inchworm_angular_rate_fault_delay_sec_)
            {
              ROS_ERROR("FRONT_REACH angular-rate limit persisted for %.2f s before the angle/tilt targets; restoring grasp",
                        inchworm_angular_rate_fault_delay_sec_);
              stopInchwormToGrasp(now,
                  "sustained angular-rate limit during FRONT_REACH");
            }
        }
      else
        inchworm_angular_rate_fault_start_ = ros::Time(0);
      if (state_ == FRONT_REACH &&
          elapsed >= inchworm_reach_duration_sec_)
        {
          ROS_ERROR("FRONT_REACH timed out: bend %.1f/%.1f deg (best %.1f), body tilt %.2f deg (diagnostic only); restoring grasp",
                    estimated_bend * 180.0 / M_PI,
                    effective_target_bend * 180.0 / M_PI,
                    action_a_best_reach_bend_rad_ * 180.0 / M_PI,
                    body_tilt * 180.0 / M_PI);
          stopInchwormToGrasp(now, "FRONT_REACH timeout");
        }
      }
      break;

    case ACTION_A_FRONT_PLACE:
      {
      // Deflate the bottom support at the same instant that lowering starts.
      publishBottomTarget(action_a_bottom_low_kpa_);
      // Re-pressurize the arm at the same instant that its extension thrust
      // starts to fall.  Pressure supplies the new grasp progressively while
      // the rotor lowers the arm onto the contact surface.
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (arm == inchworm_active_arm_)
            {
              // Arm0 is the first new contact. Curl it at 40 kPa while its
              // extension thrust falls so it pulls the body toward the new
              // contact rather than merely resting there at transfer pressure.
              inchworm_pressure_[arm] = inchworm_active_arm_ == 0
                  ? action_a_arm0_rebend_pressure_kpa_
                  : inchworm_transfer_pressure_kpa_;
              const double target_thrust = elapsed < action_a_place_hold_sec_
                  ? action_a_place_start_thrust_n_ : 0.0;
              const double rate = elapsed < action_a_place_hold_sec_
                  ? inchworm_thrust_ramp_n_s_
                  : action_a_place_thrust_down_rate_n_s_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], target_thrust, rate);
            }
          else if (frontAlreadyPlaced(arm))
            {
              inchworm_pressure_[arm] = inchworm_transfer_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  action_a_yaw_thrust_down_rate_n_s_);
            }
          else
            {
              // This also returns diagonal helper arm2 to normal grasp at the
              // exact instant arm0 begins its high-pressure re-bend.
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  action_a_diagonal_assist_thrust_down_rate_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);

      const double placed_bend = measuredArmBendAngle(inchworm_active_arm_);
      const bool bend_ready = placed_bend >=
          action_a_place_bend_target_rad_ - prepare_angle_tolerance_;
      const bool grip_telemetry_fresh = grip_state_valid_ &&
          !grip_state_stamp_.isZero() &&
          (now - grip_state_stamp_).toSec() <= 0.25;
      const bool contact_ready = grip_telemetry_fresh &&
          std::isfinite(grip_surface_gap_m_[inchworm_active_arm_]) &&
          grip_surface_gap_m_[inchworm_active_arm_] <=
              action_a_place_contact_gap_m_ &&
          placed_bend >= action_a_place_start_bend_rad_ +
              action_a_place_min_bend_change_rad_;
      ROS_INFO_THROTTLE(0.2,
          "Action A FRONT_PLACE arm%zu bend %.1f/%.1f deg, thrust %.2f N (start %.2f), pressure %.1f/%.1f kPa, bottom %.1f/%.1f kPa, gap %.1f mm, contact %d",
          inchworm_active_arm_, placed_bend * 180.0 / M_PI,
          action_a_place_bend_target_rad_ * 180.0 / M_PI,
          inchworm_thrust_[inchworm_active_arm_],
          action_a_place_start_thrust_n_,
          last_pressure_[inchworm_active_arm_],
          inchworm_pressure_[inchworm_active_arm_],
          bottom_pressure_kpa_, action_a_bottom_low_kpa_,
          std::isfinite(grip_surface_gap_m_[inchworm_active_arm_])
              ? 1000.0 * grip_surface_gap_m_[inchworm_active_arm_] : -1.0,
          contact_ready);
      if (elapsed >= action_a_place_hold_sec_ &&
          (bend_ready || contact_ready))
        {
          ROS_INFO("Arm%zu placed at %.1f deg with %.2f N; pressurizing new contact",
                   inchworm_active_arm_, placed_bend * 180.0 / M_PI,
                   inchworm_thrust_[inchworm_active_arm_]);
          enterState(FRONT_ANCHOR, now);
        }
      else if (elapsed >= action_a_place_timeout_sec_)
        {
          ROS_ERROR("ACTION_A_FRONT_PLACE timed out: arm%zu bend %.1f/%.1f deg, gap %.1f mm",
                    inchworm_active_arm_, placed_bend * 180.0 / M_PI,
                    action_a_place_bend_target_rad_ * 180.0 / M_PI,
                    std::isfinite(grip_surface_gap_m_[inchworm_active_arm_])
                        ? 1000.0 * grip_surface_gap_m_[inchworm_active_arm_]
                        : -1.0);
          stopInchwormToGrasp(now, "ACTION_A_FRONT_PLACE timeout");
        }
      }
      break;

    case FRONT_ANCHOR:
      publishBottomTarget(action_a_bottom_low_kpa_);
      // Pressurize the new contact, then remove its remaining landing thrust
      // completely before the other front arm is allowed to extend.
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (frontAlreadyPlaced(arm) || arm == inchworm_active_arm_)
            {
              inchworm_pressure_[arm] = arm == 0 && inchworm_active_arm_ == 0
                  ? action_a_arm0_rebend_pressure_kpa_
                  : inchworm_transfer_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  action_a_place_thrust_down_rate_n_s_);
            }
          else
            {
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0, inchworm_thrust_ramp_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) &&
          std::abs(inchworm_thrust_[inchworm_active_arm_]) <= 0.01)
        {
          ROS_INFO("Front arm%zu anchored at %.1f kPa with thrust fully stopped",
                   inchworm_active_arm_, inchworm_pressure_[inchworm_active_arm_]);
          enterState(SUPPORT_RESTORE, now);
        }
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          ROS_ERROR("FRONT_ANCHOR timed out");
          stopInchwormToGrasp(now, "FRONT_ANCHOR timeout");
        }
      break;

    case SUPPORT_RESTORE:
      publishBottomTarget(action_a_bottom_low_kpa_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (frontAlreadyPlaced(arm) || arm == inchworm_active_arm_)
            {
              inchworm_pressure_[arm] = arm == 0 && inchworm_active_arm_ == 0
                  ? action_a_arm0_rebend_pressure_kpa_
                  : inchworm_transfer_pressure_kpa_;
              inchworm_thrust_[arm] = 0.0;
            }
          else
            {
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0, inchworm_thrust_ramp_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) &&
          std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                      [](double value) { return std::abs(value) <= 0.01; }))
        {
          if (inchworm_front_sequence_ + 1 < inchworm_front_order_.size())
            {
              ++inchworm_front_sequence_;
              inchworm_active_arm_ = inchworm_front_order_[inchworm_front_sequence_];
              ROS_INFO("All supports restored; beginning arm%zu swing",
                       inchworm_active_arm_);
              beginInchwormArmReach(now);
            }
          else
            {
              // The real-machine bag showed that rear reverse thrust moved
              // the body backwards.  Action A now keeps the rear pair grasped
              // and goes directly to the all-arm anchor/settle phase.
              ROS_INFO("Both front contacts placed; rear reverse-thrust phase disabled");
              enterState(REAR_ANCHOR, now);
            }
        }
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          ROS_ERROR("SUPPORT_RESTORE timed out");
          stopInchwormToGrasp(now, "SUPPORT_RESTORE timeout");
        }
      break;

    case REAR_UNLOAD:
      publishBottomTarget(action_a_bottom_low_kpa_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (inchworm_front_[arm])
            {
              inchworm_pressure_[arm] = inchworm_transfer_pressure_kpa_;
              inchworm_thrust_[arm] = 0.0;
            }
          else
            {
              inchworm_pressure_[arm] = std::max(
                  minimum_prepare_pressure_kpa_,
                  base_pressure_kpa_ - inchworm_rear_unload_kpa_);
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0, inchworm_thrust_ramp_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_)) enterState(PULL, now);
      else if (elapsed >= inchworm_release_timeout_sec_)
        {
          ROS_ERROR("REAR_UNLOAD timed out");
          stopInchwormToGrasp(now, "REAR_UNLOAD timeout");
        }
      break;

    case PULL:
      {
      // The central bottom support assists the body while the rear pair is
      // applying reverse thrust.  It is exhausted together with thrust in
      // REAR_ANCHOR, rather than before the pulling impulse is complete.
      publishBottomTarget(action_a_bottom_high_kpa_);
      double rear_reverse_thrust = 0.0;
      size_t rear_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (!inchworm_front_[arm])
          {
            rear_reverse_thrust += -inchworm_thrust_[arm];
            ++rear_count;
          }
      rear_reverse_thrust /= std::max<size_t>(1, rear_count);
      const bool meaningful_progress =
          travel >= inchworm_best_pull_travel_m_ + inchworm_progress_epsilon_m_;
      if (meaningful_progress &&
          rear_reverse_thrust >= inchworm_rear_reverse_thrust_n_ - 0.05)
        {
          inchworm_motion_detected_ = true;
          inchworm_best_pull_travel_m_ = travel;
          inchworm_motion_last_seen_ = now;
        }
      else if (inchworm_motion_detected_)
        {
          const bool regressed =
              travel < inchworm_best_pull_travel_m_ - inchworm_progress_regression_m_;
          const bool stalled = (now - inchworm_motion_last_seen_).toSec() >=
                               inchworm_motion_stop_timeout_sec_;
          if (regressed || stalled)
            {
              inchworm_motion_detected_ = false;
              ROS_INFO("Inchworm PULL progress stopped; resuming thrust search");
            }
        }
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (inchworm_front_[arm])
            {
              // Both front arms re-bend together: keep their new contacts
              // pressurized and remove the extension-hold thrust at the same
              // configured ramp rate.
              inchworm_pressure_[arm] = std::max(
                  base_pressure_kpa_, inchworm_front_pull_pressure_kpa_);
              inchworm_thrust_[arm] = approach(inchworm_thrust_[arm], 0.0,
                                                inchworm_thrust_ramp_n_s_);
            }
          else
            {
              // Low rear pressure reduces friction while reverse thrust on
              // the trailing pair assists the body toward the two front
              // anchors. Reverse thrust is deliberately fixed and bounded;
              // it is not increased merely because mocap reports no motion.
              inchworm_pressure_[arm] = std::max(
                  minimum_prepare_pressure_kpa_, inchworm_pull_pressure_kpa_);
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], -inchworm_rear_reverse_thrust_n_,
                  inchworm_thrust_ramp_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2, "Action A PULL travel %.2f/%.2f mm, speed %.2f mm/s, thrust [%.2f %.2f %.2f %.2f] N, rear reverse %.2f N, bottom %.1f/%.1f kPa, motion %d",
                        1000.0 * inchwormTravel(),
                        1000.0 * inchworm_target_distance_m_,
                        1000.0 * inchworm_travel_speed_m_s_,
                        inchworm_thrust_[0], inchworm_thrust_[1],
                        inchworm_thrust_[2], inchworm_thrust_[3],
                        rear_reverse_thrust,
                        bottom_pressure_kpa_, action_a_bottom_high_kpa_,
                        inchworm_motion_detected_);
      if (inchwormTravel() >= inchworm_target_distance_m_ ||
          elapsed >= inchworm_pull_timeout_sec_)
        enterState(REAR_ANCHOR, now);
      }
      break;

    case REAR_ANCHOR:
      // Restore rear grasp pressure, remove reverse thrust, and exhaust the
      // bottom support together.  A dedicated faster ramp avoids dragging
      // the body after the pulling phase has finished.
      publishBottomTarget(action_a_bottom_low_kpa_);
      inchworm_pressure_.fill(base_pressure_kpa_);
      for (double& thrust : inchworm_thrust_)
        thrust = approach(thrust, 0.0,
                          action_a_rear_thrust_down_rate_n_s_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2,
          "Action A REAR_ANCHOR thrust [%.2f %.2f %.2f %.2f] N, pressure target %.1f kPa, bottom %.1f/%.1f kPa",
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3],
          base_pressure_kpa_, bottom_pressure_kpa_, action_a_bottom_low_kpa_);
      if (pressureReached(inchworm_pressure_) &&
          std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                      [](double value) { return std::abs(value) <= 0.01; }))
        enterState(SETTLE, now);
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          inchworm_thrust_.fill(0.0);
          enterState(SETTLE, now);
        }
      break;

    case ROCK_FRONT_UNLOAD:
      {
      // Establish the rearward bottom posture before touching the front-arm
      // grasp.  Keeping all four chambers at base pressure preserves the
      // already bent front pair; v5 released it here and let it settle at the
      // zero-angle mechanical stop before any useful rotor thrust arrived.
      inchworm_thrust_.fill(0.0);
      inchworm_pressure_.fill(base_pressure_kpa_);
      publishBottomDirection(-1.0);
      publishBottomTarget(rocking_bottom_high_kpa_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      const double delta_roll = wrapAngle(roll_ - rocking_phase_start_roll_);
      const double delta_pitch = wrapAngle(pitch_ - rocking_phase_start_pitch_);
      const double rocking_angle =
          delta_roll * inchworm_direction_body_[1] -
          delta_pitch * inchworm_direction_body_[0];
      const double total_tilt = std::hypot(
          wrapAngle(roll_ - inchworm_swing_start_roll_),
          wrapAngle(pitch_ - inchworm_swing_start_pitch_));
      ROS_INFO_THROTTLE(0.2,
          "ROCK bottom PRELOAD: bottom %.1f/%.1f kPa, signed tilt %.2f/%.2f deg, front pressure %.1f kPa",
          bottom_pressure_kpa_, rocking_bottom_high_kpa_,
          rocking_angle * 180.0 / M_PI,
          rocking_front_tilt_target_rad_ * 180.0 / M_PI,
          measured_pressure_[inchworm_front_order_[0]]);
      if (total_tilt >= rocking_max_tilt_rad_)
        stopInchwormToGrasp(now, "rocking bottom-preload tilt safety limit");
      else if (rocking_angle >= rocking_front_tilt_target_rad_ &&
               bottom_pressure_kpa_ >= rocking_bottom_high_kpa_ -
                                           pressure_tolerance_kpa_)
        enterState(ROCK_REAR_INFLATE, now);
      else if (elapsed >= rocking_phase_timeout_sec_)
        stopInchwormToGrasp(now, "rocking bottom-preload timeout");
      }
      break;

    case ROCK_REAR_INFLATE:
      {
      // Deliberately prepare a large bend while the bottom preload is held.
      // The forward sweep starts only after both front arms reach this
      // wind-up angle; reverse rotor thrust is not used to bend a straight
      // chain against its mechanical stop.
      double minimum_front_bend = M_PI;
      double maximum_front_bend = 0.0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          inchworm_pressure_[arm] = inchworm_front_[arm]
              ? rocking_front_windup_pressure_kpa_ : base_pressure_kpa_;
          inchworm_thrust_[arm] = approach(
              inchworm_thrust_[arm], 0.0, rocking_thrust_ramp_n_s_);
          if (inchworm_front_[arm])
            {
              const double bend = measuredArmBendAngle(arm);
              minimum_front_bend = std::min(minimum_front_bend, bend);
              maximum_front_bend = std::max(maximum_front_bend, bend);
            }
        }
      publishBottomDirection(-1.0);
      publishBottomTarget(rocking_bottom_high_kpa_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      const double total_tilt = std::hypot(
          wrapAngle(roll_ - inchworm_swing_start_roll_),
          wrapAngle(pitch_ - inchworm_swing_start_pitch_));
      ROS_INFO_THROTTLE(0.2,
          "ROCK front WINDUP: bend %.1f..%.1f/%.1f deg, pressure %.1f/%.1f kPa, bottom %.1f/%.1f kPa",
          minimum_front_bend * 180.0 / M_PI,
          maximum_front_bend * 180.0 / M_PI,
          rocking_front_windup_target_rad_ * 180.0 / M_PI,
          measured_pressure_[inchworm_front_order_[0]],
          rocking_front_windup_pressure_kpa_,
          bottom_pressure_kpa_, rocking_bottom_high_kpa_);
      if (total_tilt >= rocking_max_tilt_rad_)
        stopInchwormToGrasp(now, "rocking front-windup tilt safety limit");
      else if (minimum_front_bend >=
                   rocking_front_windup_target_rad_ -
                       rocking_front_angle_tolerance_rad_ &&
               pressureReached(inchworm_pressure_))
        {
          if (rocking_swing_hold_start_.isZero())
            rocking_swing_hold_start_ = now;
          if ((now - rocking_swing_hold_start_).toSec() >=
              rocking_front_windup_hold_sec_)
            enterState(ROCK_FRONT_LAND, now);
        }
      else
        rocking_swing_hold_start_ = ros::Time(0);
      if (state_ == ROCK_REAR_INFLATE &&
          elapsed >= rocking_phase_timeout_sec_)
        stopInchwormToGrasp(now, "rocking pressure WINDUP angle timeout");
      }
      break;

    case ROCK_FRONT_LAND:
      {
      const double swing_elapsed =
          (now - rocking_swing_phase_start_).toSec();
      double minimum_front_bend = M_PI;
      double maximum_front_bend = 0.0;
      double mean_front_advance = 0.0;
      double maximum_front_gap = 0.0;
      size_t front_count = 0;
      bool front_grip_active = true;
      bool front_grip_inactive = true;
      bool advance_available = grip_state_valid_ &&
          (now - grip_state_stamp_).toSec() <= 0.25 &&
          std::abs(inchworm_direction_world_[0]) >= 0.5;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          {
            const double bend = measuredArmBendAngle(arm);
            minimum_front_bend = std::min(minimum_front_bend, bend);
            maximum_front_bend = std::max(maximum_front_bend, bend);
            front_grip_active = front_grip_active && grip_active_[arm];
            front_grip_inactive = front_grip_inactive && !grip_active_[arm];
            maximum_front_gap = std::max(
                maximum_front_gap, grip_surface_gap_m_[arm]);
            if (advance_available &&
                std::isfinite(rocking_front_start_axis_m_[arm]))
              mean_front_advance += inchworm_direction_world_[0] *
                  (grip_axis_coordinate_m_[arm] -
                   rocking_front_start_axis_m_[arm]);
            else
              advance_available = false;
            ++front_count;
          }
      if (front_count > 0 && advance_available)
        mean_front_advance /= static_cast<double>(front_count);

      // Keep the bottom preload while the released front pair is swept toward
      // straight.  Exhaust starts only after both measured bend angles cross
      // the EXTEND target and the state advances to PLACE.
      publishBottomDirection(-1.0);

      const char* swing_name = "RELEASE";
      double bottom_target_kpa = rocking_bottom_high_kpa_;
      bool phase_complete = false;
      bool phase_timeout = false;
      switch (rocking_swing_phase_)
        {
        case ROCK_SWING_RELEASE:
          // Exhaust the front chambers before applying sweep thrust, but keep
          // the bottom posture bias until the arm pair is straight.
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              inchworm_pressure_[arm] = inchworm_front_[arm]
                  ? rocking_front_release_kpa_ : base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  rocking_thrust_ramp_n_s_);
            }
          phase_complete = front_grip_inactive &&
              measured_pressure_[inchworm_front_order_[0]] <=
                  rocking_front_release_kpa_ + pressure_tolerance_kpa_;
          phase_timeout = swing_elapsed >= rocking_phase_timeout_sec_;
          break;

        case ROCK_SWING_EXTEND:
          swing_name = "EXTEND";
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              inchworm_pressure_[arm] = inchworm_front_[arm]
                  ? rocking_front_release_kpa_ : base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], inchworm_front_[arm]
                      ? rocking_front_extend_thrust_n_ : 0.0,
                  rocking_thrust_ramp_n_s_);
            }
          // 0 rad is mechanically straight.  Do not add the general bend-angle
          // tolerance here: doing so made a 20-deg target complete at 35 deg
          // and exhausted bottom before the visual straight posture.
          if (maximum_front_bend <= rocking_front_extend_target_rad_)
            {
              if (rocking_swing_hold_start_.isZero())
                rocking_swing_hold_start_ = now;
              phase_complete =
                  (now - rocking_swing_hold_start_).toSec() >=
                      rocking_front_extend_hold_sec_;
            }
          else
            rocking_swing_hold_start_ = ros::Time(0);
          phase_timeout = swing_elapsed >= rocking_phase_timeout_sec_;
          break;

        case ROCK_SWING_PLACE:
          swing_name = "PLACE";
          bottom_target_kpa = rocking_bottom_low_kpa_;
          // Sweep back slowly from the fully extended posture.  Pressure is
          // introduced during the return, so the first new cylinder contact
          // becomes a real anchor rather than a time-based assumed landing.
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              inchworm_pressure_[arm] = inchworm_front_[arm]
                  ? rocking_front_place_pressure_kpa_ : base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  rocking_front_lower_rate_n_s_);
            }
          {
          const bool bend_recovered = minimum_front_bend >=
              rocking_front_place_bend_target_rad_ -
                  rocking_front_angle_tolerance_rad_;
          const bool advanced_contact = advance_available &&
              front_grip_active &&
              mean_front_advance >= rocking_front_min_advance_m_;
          // grip_state is MuJoCo-only. On hardware, use the measured bend and
          // pressure as the conservative fallback until contact sensing exists.
          const bool hardware_fallback = !advance_available &&
              bend_recovered && pressureReached(inchworm_pressure_);
          phase_complete = advanced_contact || hardware_fallback;
          }
          phase_timeout = swing_elapsed >= rocking_phase_timeout_sec_;
          break;

        case ROCK_SWING_REGRIP:
          swing_name = "REGRIP";
          bottom_target_kpa = rocking_bottom_low_kpa_;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0,
                  rocking_front_lower_rate_n_s_);
            }
          phase_complete = pressureReached(inchworm_pressure_) &&
              std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                  [](double value) { return std::abs(value) <= 0.01; }) &&
              (!advance_available || front_grip_active) &&
              swing_elapsed >= rocking_land_settle_sec_;
          phase_timeout = swing_elapsed >= rocking_phase_timeout_sec_;
          break;
        }

      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      publishBottomTarget(bottom_target_kpa);
      ROS_INFO_THROTTLE(0.2,
          "ROCK SWING %s: bend %.1f..%.1f deg, thrust %.2f N, "
          "pressure %.1f kPa, grip %d, advance %s%.1f mm, gap %.1f mm, "
          "bottom %.1f->%.1f kPa",
          swing_name, minimum_front_bend * 180.0 / M_PI,
          maximum_front_bend * 180.0 / M_PI,
          inchworm_thrust_[inchworm_front_order_[0]],
          measured_pressure_[inchworm_front_order_[0]], front_grip_active,
          advance_available ? "" : "n/a ",
          advance_available ? 1000.0 * mean_front_advance : 0.0,
          1000.0 * maximum_front_gap,
          bottom_pressure_kpa_, bottom_target_kpa);
      const double total_tilt = std::hypot(
          wrapAngle(roll_ - inchworm_swing_start_roll_),
          wrapAngle(pitch_ - inchworm_swing_start_pitch_));
      if (total_tilt >= rocking_max_tilt_rad_)
        stopInchwormToGrasp(now, "rocking front-swing tilt safety limit");
      else if (phase_complete)
        {
          rocking_swing_hold_start_ = ros::Time(0);
          rocking_swing_phase_start_ = now;
          if (rocking_swing_phase_ < ROCK_SWING_REGRIP)
            {
              ++rocking_swing_phase_;
              ROS_INFO("ROCK SWING entered internal phase %u",
                       static_cast<unsigned>(rocking_swing_phase_));
            }
          else
            enterState(ROCK_REAR_UNLOAD, now);
        }
      else if (phase_timeout)
        {
          if (rocking_swing_phase_ == ROCK_SWING_RELEASE)
            stopInchwormToGrasp(now,
                "rocking RELEASE pressure/contact timeout");
          else if (rocking_swing_phase_ == ROCK_SWING_EXTEND)
            stopInchwormToGrasp(now, "rocking EXTEND angle timeout");
          else if (rocking_swing_phase_ == ROCK_SWING_PLACE)
            stopInchwormToGrasp(now,
                "rocking PLACE new-contact/advance timeout");
          else
            stopInchwormToGrasp(now, "rocking REGRIP timeout");
        }
      }
      break;

    case ROCK_REAR_UNLOAD:
      inchworm_thrust_.fill(0.0);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_pressure_[arm] = inchworm_front_[arm]
            ? base_pressure_kpa_ : rocking_rear_release_kpa_;
      publishBottomDirection(-1.0);
      publishBottomTarget(rocking_bottom_low_kpa_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_))
        enterState(ROCK_REAR_RECOVER, now);
      else if (elapsed >= rocking_phase_timeout_sec_)
        stopInchwormToGrasp(now, "rocking rear-unload timeout");
      break;

    case ROCK_REAR_RECOVER:
      {
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          inchworm_pressure_[arm] = inchworm_front_[arm]
              ? base_pressure_kpa_ : rocking_rear_release_kpa_;
          inchworm_thrust_[arm] = approach(
              inchworm_thrust_[arm],
              inchworm_front_[arm] ? 0.0 : -rocking_rear_reverse_thrust_n_,
              rocking_thrust_ramp_n_s_);
        }
      publishBottomDirection(-1.0);
      publishBottomTarget(rocking_bottom_low_kpa_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      const double delta_roll = wrapAngle(roll_ - rocking_phase_start_roll_);
      const double delta_pitch = wrapAngle(pitch_ - rocking_phase_start_pitch_);
      const double rear_recovery = std::abs(
          delta_roll * inchworm_direction_body_[1] -
          delta_pitch * inchworm_direction_body_[0]);
      // On the cylindrical target, a well-anchored front pair can translate
      // the body/rear tips forward with very little attitude change. Accept
      // that directly measured recovery as well as the intended rocking angle;
      // otherwise a useful reverse pulse runs until timeout merely because the
      // front anchor suppressed pitch.
      const double rear_forward_shift =
          travel - rocking_rear_recover_start_travel_m_;
      const double total_tilt = std::hypot(
          wrapAngle(roll_ - inchworm_swing_start_roll_),
          wrapAngle(pitch_ - inchworm_swing_start_pitch_));
      ROS_INFO_THROTTLE(0.2,
          "ROCK rear contact shift: rocking %.2f/%.2f deg, rear shift %.2f/%.2f mm, rear reverse thrust %.2f N, travel %.2f mm",
          rear_recovery * 180.0 / M_PI,
          rocking_recovery_tilt_target_rad_ * 180.0 / M_PI,
          1000.0 * rear_forward_shift,
          1000.0 * rocking_front_min_advance_m_,
          rocking_rear_reverse_thrust_n_, 1000.0 * travel);
      if (total_tilt >= rocking_max_tilt_rad_)
        stopInchwormToGrasp(now, "rocking rear-recovery tilt safety limit");
      else if (rear_recovery >= rocking_recovery_tilt_target_rad_ ||
               rear_forward_shift >= rocking_front_min_advance_m_)
        enterState(ROCK_REAR_LAND, now);
      else if (elapsed >= rocking_phase_timeout_sec_)
        stopInchwormToGrasp(now, "rocking rear-recovery timeout");
      }
      break;

    case ROCK_REAR_LAND:
      {
      inchworm_pressure_.fill(base_pressure_kpa_);
      for (double& thrust : inchworm_thrust_)
        thrust = approach(thrust, 0.0, rocking_thrust_ramp_n_s_);
      publishBottomDirection(-1.0);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      const bool rear_landed = pressureReached(inchworm_pressure_) &&
          std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                      [](double value) { return std::abs(value) <= 0.01; });
      publishBottomTarget(rocking_bottom_low_kpa_);
      if (rear_landed && std::isfinite(bottom_pressure_kpa_) &&
          std::abs(bottom_pressure_kpa_ - rocking_bottom_low_kpa_) <=
              pressure_tolerance_kpa_ &&
          elapsed >= rocking_land_settle_sec_)
        enterState(SETTLE, now);
      else if (elapsed >= rocking_phase_timeout_sec_)
        stopInchwormToGrasp(now, "rocking rear-landing timeout");
      }
      break;

    case ACTION_B_FRONT_REACH:
      {
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (inchworm_front_[arm])
            {
              inchworm_pressure_[arm] = 0.0;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], std::max(0.0, std::min(
                      action_b_pre_pulse_max_thrust_n_,
                      action_b_max_thrust_n_ - action_b_pulse_amplitude_n_)),
                  action_b_front_thrust_ramp_up_n_s_);
            }
          else
            {
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(
                  inchworm_thrust_[arm], 0.0, action_b_thrust_ramp_n_s_);
            }
        }
      // Exhaust the front pair at maximum duty while thrust rises gradually.
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      const double delta_roll = wrapAngle(
          roll_ - inchworm_swing_start_roll_);
      const double delta_pitch = wrapAngle(
          pitch_ - inchworm_swing_start_pitch_);
      const double body_tilt =
          delta_roll * inchworm_direction_body_[1] -
          delta_pitch * inchworm_direction_body_[0];
      const double signed_lift_rate =
          body_angular_velocity_[0] * inchworm_direction_body_[1] -
          body_angular_velocity_[1] * inchworm_direction_body_[0];
      const double body_angular_rate = std::abs(signed_lift_rate);
      const double predicted_tilt = body_tilt +
          0.30 * std::max(0.0, signed_lift_rate);
      ROS_INFO_THROTTLE(0.2,
          "Action B front pair: tilt %.2f/%.2f deg, angular rate %.2f rad/s, thrust [%.2f, %.2f, %.2f, %.2f], pressure [%.1f, %.1f, %.1f, %.1f]",
          body_tilt * 180.0 / M_PI,
          action_b_front_transition_tilt_rad_ * 180.0 / M_PI,
          body_angular_rate,
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3],
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3]);
      bool front_at_max = true;
      const double lift_hold_limit = std::max(0.0, std::min(
          action_b_pre_pulse_max_thrust_n_,
          action_b_max_thrust_n_ - action_b_pulse_amplitude_n_));
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          front_at_max = front_at_max &&
              inchworm_thrust_[arm] >= lift_hold_limit - 0.05;
      // Stop as soon as a small but measurable lift is obtained.  Five
      // degrees was only a legacy attitude target and allowed the front pair
      // to keep accelerating the body long after contact unloading began.
      if (body_tilt >= action_b_min_lift_tilt_rad_)
        {
          ROS_INFO("Action B lift phase complete (measured %.2f deg, predicted %.2f deg, minimum %.2f deg, angular rate %.2f rad/s, hold limit reached %d); preserving pulse headroom",
                   body_tilt * 180.0 / M_PI,
                   predicted_tilt * 180.0 / M_PI,
                   action_b_min_lift_tilt_rad_ * 180.0 / M_PI,
                   body_angular_rate, front_at_max);
          enterState(ACTION_B_FRONT_REGRIP, now);
        }
      else if (elapsed >= action_b_front_duration_sec_)
        stopInchwormToGrasp(now, "Action B front-pair maximum-duration timeout");
      }
      break;

    case ACTION_B_FRONT_REGRIP:
      {
      // Keep the rear pair anchored at its root-side contact midpoint. Search
      // front pressure while thrust feedback preserves the lift posture that
      // was actually measured in FRONT_REACH. Neuron IMUs provide rotor
      // direction, so pressure may increase horizontal pull only while the
      // vertical/moment component continues supporting the body weight.
      const double delta_roll = wrapAngle(
          roll_ - inchworm_swing_start_roll_);
      const double delta_pitch = wrapAngle(
          pitch_ - inchworm_swing_start_pitch_);
      const double body_tilt =
          delta_roll * inchworm_direction_body_[1] -
          delta_pitch * inchworm_direction_body_[0];
      const double body_angular_rate = std::abs(
          body_angular_velocity_[0] * inchworm_direction_body_[1] -
          body_angular_velocity_[1] * inchworm_direction_body_[0]);
      const double total_body_tilt = std::hypot(delta_roll, delta_pitch);
      if (body_tilt > 30.0 * M_PI / 180.0 ||
          total_body_tilt > 30.0 * M_PI / 180.0)
        {
          // A perched lift is intentionally small.  Do not let model error or
          // delayed mechanics turn it into a takeoff/flip: remove thrust in
          // the same control cycle and restore the four-arm grasp.
          inchworm_thrust_.fill(0.0);
          publishCorrectionThrust(inchworm_thrust_);
          ROS_ERROR("Action B lift overshoot %.2f/%.2f deg: cutting front thrust immediately",
                    body_tilt * 180.0 / M_PI,
                    action_b_lift_tilt_rad_ * 180.0 / M_PI);
          enterState(ACTION_B_REGRASP, now);
          break;
        }
      const double hold_thrust_limit = std::max(0.0, std::min(
          action_b_pre_pulse_max_thrust_n_,
          action_b_max_thrust_n_ - action_b_pulse_amplitude_n_));

      // The threshold is crossed while the body still has momentum.  Once
      // that motion has settled, adopt the actually attained small lift angle
      // once instead of forcing the body back to the threshold-crossing angle.
      if (action_b_shape_phase_ >= 1 && !action_b_lift_target_adapted_ &&
          body_tilt > action_b_lift_tilt_rad_ +
                          action_b_hold_tilt_tolerance_rad_ &&
          body_angular_rate <= 0.03)
        {
          if (action_b_lift_settle_start_.isZero())
            action_b_lift_settle_start_ = now;
          if ((now - action_b_lift_settle_start_).toSec() >=
              action_b_hold_stable_duration_sec_)
            {
              ROS_INFO("Action B adopted settled lift posture %.2f deg (threshold crossing %.2f deg) at %.2f N",
                       body_tilt * 180.0 / M_PI,
                       action_b_lift_tilt_rad_ * 180.0 / M_PI,
                       action_b_hold_thrust_n_);
              action_b_lift_tilt_rad_ = body_tilt;
              action_b_lift_target_adapted_ = true;
              action_b_lift_settle_start_ = ros::Time(0);
            }
        }
      else if (!action_b_lift_target_adapted_)
        action_b_lift_settle_start_ = ros::Time(0);

      // Phase 0 has its own Neuron-angle-to-thrust law below. Running the
      // attitude-hold PI at the same time made the two controllers fight:
      // the PI reduced thrust at up to 20 N/s while the shape law could add
      // only 1 N/s, so a valid 7-N angle target produced about 5 N. Enable
      // attitude hold only after the 30-kPa shape ramp has completed.
      if (!action_b_pulse_on_ && action_b_shape_phase_ >= 2)
        {
          Vector2 current_unit_moment{{0.0, 0.0}};
          double front_angle_sum = 0.0;
          size_t front_angle_count = 0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                // Pressure bends the real arm while measuredArmAngle()
                // becomes more negative. Convert the change from the saved
                // 0-kPa lift posture, not the absolute IMU mounting angle.
                const double rotor_angle = std::max(0.0, std::min(
                    2.966, action_b_shape_start_rotor_angle_rad_[arm] -
                        wrapAngle(measuredArmAngle(arm) -
                                  action_b_shape_start_neuron_angle_rad_[arm])));
                // Display the actual Spinal-relative Neuron bend, not the
                // model angle after zero-clamping.
                front_angle_sum += std::abs(
                    wrapAngle(measuredArmAngle(arm)));
                ++front_angle_count;
                const Vector3 position = rotorPositionFromAngle(arm, rotor_angle);
                const Vector3 direction = rotorDirectionFromAngle(arm, rotor_angle);
                Vector3 lever;
                for (size_t axis = 0; axis < 3; ++axis)
                  lever[axis] = position[axis] -
                      action_b_rear_contact_position_[axis];
                const Vector3 unit_moment = cross(lever, direction);
                current_unit_moment[0] += unit_moment[0];
                current_unit_moment[1] += unit_moment[1];
              }
          if (front_angle_count > 0)
            action_b_current_front_angle_rad_ =
                front_angle_sum / static_cast<double>(front_angle_count);
          const double moment_denominator =
              current_unit_moment[0] * current_unit_moment[0] +
              current_unit_moment[1] * current_unit_moment[1];
          double feedforward_thrust = action_b_hold_thrust_n_;
          if (moment_denominator > 1.0e-8)
            feedforward_thrust =
                (current_unit_moment[0] * action_b_hold_reference_moment_[0] +
                 current_unit_moment[1] * action_b_hold_reference_moment_[1]) /
                moment_denominator;
          const double tilt_error =
              action_b_lift_tilt_rad_ - body_tilt;
          action_b_hold_tilt_integral_n_ = std::max(
              0.0, std::min(3.0, action_b_hold_tilt_integral_n_ +
                  action_b_hold_tilt_ki_n_rad_s_ * tilt_error * dt));
          const double tilt_feedback =
              action_b_hold_tilt_kp_n_rad_ * tilt_error +
              action_b_hold_tilt_integral_n_;
          double desired_hold_thrust = std::max(
              action_b_initial_hold_thrust_n_,
              std::min(hold_thrust_limit,
                       feedforward_thrust + tilt_feedback));
          // During pressure/thrust search and selected-point convergence,
          // use the measured lift directly.  The model feedforward can have a
          // large offset; previously it reset every small thrust increment to
          // about 5.15 N on the following cycle.  Below the required lift,
          // keep ramping from the current value toward the 6.5 N limit.
          bool operating_pressure_ready = true;
          if (action_b_shape_phase_ <= 1)
            for (size_t arm = 0; arm < ARM_COUNT; ++arm)
              if (inchworm_front_[arm])
                operating_pressure_ready = operating_pressure_ready &&
                    std::abs(measured_pressure_[arm] -
                             action_b_shape_pressure_kpa_) <=
                        pressure_tolerance_kpa_;
          if (action_b_shape_phase_ <= 1 && operating_pressure_ready &&
              body_tilt < action_b_lift_tilt_rad_)
            desired_hold_thrust = hold_thrust_limit;
          // Once the measured lift is at or above its recorded value, model
          // feedforward must never increase thrust.  Brake the accumulated
          // angular motion instead of waiting for a slow PI response.
          if (body_tilt >= action_b_lift_tilt_rad_)
            desired_hold_thrust = std::min(
                desired_hold_thrust, action_b_hold_thrust_n_);
          if (body_tilt > action_b_lift_tilt_rad_ +
                              action_b_hold_tilt_tolerance_rad_ &&
              body_angular_rate > 0.03)
            desired_hold_thrust = std::max(
                0.0, action_b_hold_thrust_n_ -
                         2.0 * body_angular_rate);
          action_b_hold_thrust_n_ = approach(
              action_b_hold_thrust_n_, desired_hold_thrust,
              desired_hold_thrust >= action_b_hold_thrust_n_
                  ? action_b_hold_thrust_up_rate_n_s_
                  : action_b_hold_thrust_down_rate_n_s_);
        }

      const double shape_pressure_limit = std::min(
          maximum_pressure_kpa_, action_b_shape_max_pressure_kpa_);
      if (action_b_shape_phase_ == 0)
        {
          double measured_front_pressure = 0.0;
          double raw_neuron_angle_sum = 0.0;
          double bend_angle_sum = 0.0;
          double vertical_fraction_sum = 0.0;
          size_t front_count = 0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                measured_front_pressure += measured_pressure_[arm];
                const double raw_neuron_angle = measuredArmAngle(arm);
                raw_neuron_angle_sum += std::abs(
                    wrapAngle(raw_neuron_angle));
                // Treat the Neuron orientation at the 0-kPa/3-deg lift point
                // as zero bend. This removes the fixed IMU mounting angle and
                // remains valid regardless of the raw angle sign.
                const double bend_angle = std::abs(wrapAngle(
                    raw_neuron_angle -
                    action_b_shape_start_neuron_angle_rad_[arm]));
                bend_angle_sum += bend_angle;
                vertical_fraction_sum += std::max(
                    0.0, std::cos(bend_angle));
                ++front_count;
              }
          const double mean_raw_neuron_angle = front_count > 0
              ? raw_neuron_angle_sum / static_cast<double>(front_count) : 0.0;
          const double mean_bend_angle = front_count > 0
              ? bend_angle_sum / static_cast<double>(front_count) : 0.0;
          const double mean_vertical_fraction = front_count > 0
              ? vertical_fraction_sum / static_cast<double>(front_count)
              : 1.0;
          if (front_count > 0)
            measured_front_pressure /= static_cast<double>(front_count);
          action_b_shape_pressure_kpa_ = approach(
              action_b_shape_pressure_kpa_, shape_pressure_limit,
              action_b_shape_pressure_rate_kpa_s_);

          action_b_current_front_angle_rad_ = mean_bend_angle;

          if (!action_b_last_bend_time_.isZero())
            {
              const double bend_dt = (now - action_b_last_bend_time_).toSec();
              if (bend_dt > 1.0e-4 && bend_dt < 0.2)
                {
                  const double measured_bend_rate = std::max(-2.0, std::min(
                      2.0, (mean_bend_angle - action_b_last_bend_angle_rad_) /
                               bend_dt));
                  // Low-pass the differentiated IMU angle before prediction.
                  action_b_bend_rate_rad_s_ += 0.15 *
                      (measured_bend_rate - action_b_bend_rate_rad_s_);
                }
            }
          action_b_last_bend_angle_rad_ = mean_bend_angle;
          action_b_last_bend_time_ = now;
          const double predicted_bend_angle = std::max(0.0, std::min(
              0.5 * M_PI - 0.05,
              mean_bend_angle + action_b_prediction_horizon_sec_ *
                                    action_b_bend_rate_rad_s_));
          const double predicted_vertical_fraction = std::max(
              0.10, std::cos(predicted_bend_angle));

          // Preserve the vertical component that first produced the 3-deg
          // lift at 0 kPa. theta is the bend relative to that initial rotor
          // orientation, hence T*cos(theta) = T0. Add a measured body-tilt
          // trim because pressure also changes the moment arm/contact state,
          // effects which the vertical-component relation does not model.
          const double shape_thrust_limit = std::min(
              action_b_shape_target_thrust_n_,
              std::min(action_b_max_thrust_n_ - action_b_pulse_amplitude_n_,
                       action_b_max_thrust_n_));
          const double vertical_thrust_target =
              action_b_initial_hold_thrust_n_ /
              predicted_vertical_fraction;
          // The 5-deg posture is a guide, not a reason to reduce the vertical
          // support found at 0 kPa. Attitude feedback may add thrust when the
          // lift is insufficient, but can never subtract from the vertical-
          // component feedforward.
          const double tilt_trim = std::max(
              0.0, action_b_hold_tilt_kp_n_rad_ *
                       (action_b_lift_tilt_rad_ - body_tilt));
          const double angle_thrust_target = std::max(0.0, std::min(
              shape_thrust_limit, vertical_thrust_target + tilt_trim));
          const double bend_for_eight_n = std::acos(std::max(
              0.0, std::min(1.0,
                  action_b_initial_hold_thrust_n_ /
                      std::max(0.1, shape_thrust_limit))));
          action_b_hold_thrust_n_ = approach(
              action_b_hold_thrust_n_, angle_thrust_target,
              angle_thrust_target >= action_b_hold_thrust_n_
                  ? action_b_hold_thrust_up_rate_n_s_
                  : action_b_hold_thrust_down_rate_n_s_);

          ROS_INFO_THROTTLE(0.5,
              "Action B predictive shape: pressure %.1f/%.1f kPa, bend %.2f deg (8N bend %.2f), rate %+.2f deg/s, predicted %.2f deg, raw Neuron %.2f deg, vertical ratio measured/predicted %.3f/%.3f, vertical FF %.2f N, positive tilt trim +%.2f N, thrust %.2f/%.2f N, tilt %.2f/%.2f deg",
              measured_front_pressure, action_b_shape_pressure_kpa_,
              mean_bend_angle * 180.0 / M_PI,
              bend_for_eight_n * 180.0 / M_PI,
              action_b_bend_rate_rad_s_ * 180.0 / M_PI,
              predicted_bend_angle * 180.0 / M_PI,
              mean_raw_neuron_angle * 180.0 / M_PI,
              mean_vertical_fraction,
              predicted_vertical_fraction,
              vertical_thrust_target, tilt_trim,
              action_b_hold_thrust_n_, angle_thrust_target,
              body_tilt * 180.0 / M_PI,
              action_b_lift_tilt_rad_ * 180.0 / M_PI);

          bool pressure_at_limit = true;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              pressure_at_limit = pressure_at_limit &&
                  std::abs(measured_pressure_[arm] - shape_pressure_limit) <=
                      pressure_tolerance_kpa_;
          const bool shape_timed_out =
              (now - action_b_shape_phase_start_).toSec() >=
              action_b_shape_search_timeout_sec_;
          if (pressure_at_limit || shape_timed_out)
            {
              action_b_best_pressure_kpa_ = shape_pressure_limit;
              action_b_best_thrust_n_ = action_b_hold_thrust_n_;
              action_b_shape_phase_ = 1;
              action_b_shape_phase_start_ = now;
              ready_start_ = ros::Time(0);
              action_b_shape_pressure_kpa_ = action_b_best_pressure_kpa_;
              ROS_INFO("Action B shape ramp complete at %.1f kPa, %.2f N, bend %.2f deg, raw Neuron %.2f deg (pressure reached %d, timeout %d)",
                       action_b_best_pressure_kpa_,
                       action_b_best_thrust_n_,
                       mean_bend_angle * 180.0 / M_PI,
                       mean_raw_neuron_angle * 180.0 / M_PI,
                       pressure_at_limit, shape_timed_out);
            }
        }
      else if (action_b_shape_phase_ == 1)
        {
          // Phase 0 has already brought the pair to the fixed 30-kPa shape
          // target while coupling thrust to the measured Neuron angle. Hold
          // that endpoint briefly before starting the existing pulses.
          action_b_pressure_limited_by_feedback_ = false;
          action_b_shape_pressure_kpa_ = approach(
              action_b_shape_pressure_kpa_, action_b_best_pressure_kpa_,
              action_b_shape_pressure_rate_kpa_s_);
          // At 30 kPa, keep the selected vertical-component feedforward and
          // use the remaining authority only to recover the 3-deg lift. Do
          // not hand control back to the old model feedforward, which reduced
          // a valid ~7 N endpoint to ~5 N in the previous experiment.
          const double endpoint_target = std::max(
              action_b_best_thrust_n_, std::min(
                  action_b_shape_target_thrust_n_,
                  action_b_best_thrust_n_ +
                      action_b_hold_tilt_kp_n_rad_ *
                          (action_b_lift_tilt_rad_ - body_tilt)));
          action_b_hold_thrust_n_ = approach(
              action_b_hold_thrust_n_, endpoint_target,
              endpoint_target >= action_b_hold_thrust_n_
                  ? action_b_hold_thrust_up_rate_n_s_
                  : action_b_hold_thrust_down_rate_n_s_);
          bool pressure_ready = true;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              pressure_ready = pressure_ready &&
                  std::abs(measured_pressure_[arm] -
                           action_b_best_pressure_kpa_) <=
                      pressure_tolerance_kpa_;
          const double body_angular_rate = std::hypot(
              body_angular_velocity_[0], body_angular_velocity_[1]);
          const bool lift_posture_ready =
              body_tilt >= action_b_lift_tilt_rad_ &&
              body_tilt <= action_b_lift_tilt_rad_ +
                               action_b_hold_tilt_tolerance_rad_ &&
              body_angular_rate <= 0.10;
          if (pressure_ready && lift_posture_ready)
            {
              if (ready_start_.isZero()) ready_start_ = now;
              if ((now - ready_start_).toSec() >=
                  action_b_hold_stable_duration_sec_)
                {
                  action_b_shape_phase_ = 2;
                  action_b_pulse_phase_start_ = now;
                  ready_start_ = ros::Time(0);
                  ROS_INFO("Action B lift posture %.2f deg is stable at %.1f kPa / %.2f N; starting pull pulses with %.2f N headroom",
                           body_tilt * 180.0 / M_PI,
                           action_b_best_pressure_kpa_,
                           action_b_hold_thrust_n_,
                           action_b_max_thrust_n_ -
                               action_b_hold_thrust_n_);
                }
            }
          else
            ready_start_ = ros::Time(0);
          if (action_b_shape_phase_ == 1 &&
              (now - action_b_shape_phase_start_).toSec() >=
                  action_b_shape_settle_timeout_sec_)
            {
              ROS_ERROR("Action B selected operating point did not maintain the measured lift posture within %.1f s (tilt %.2f/%.2f deg, pressure target %.1f kPa, hold %.2f N); cancelling pulses and restoring grasp",
                        action_b_shape_settle_timeout_sec_,
                        body_tilt * 180.0 / M_PI,
                        action_b_lift_tilt_rad_ * 180.0 / M_PI,
                        action_b_best_pressure_kpa_, action_b_hold_thrust_n_);
              enterState(ACTION_B_REGRASP, now);
            }
        }
      else if (!action_b_pulse_on_ &&
               action_b_pulse_count_ < action_b_pulse_repetitions_ &&
               (now - action_b_pulse_phase_start_).toSec() >=
                   action_b_pulse_off_duration_sec_)
        {
          action_b_pulse_on_ = true;
          action_b_pulse_phase_start_ = now;
          ready_start_ = ros::Time(0);
          const double pulse_target = std::min(
              action_b_max_thrust_n_, action_b_hold_thrust_n_ +
              action_b_pulse_amplitude_n_);
          // Capture the actual Neuron angle immediately before every pulse.
          // Invert the measured pressure/thrust/bend table and increase
          // pressure together with thrust so the rotor direction does not
          // change merely because the pulse straightens the arm.
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              action_b_pulse_pressure_[arm] = action_b_best_pressure_kpa_;
              if (!inchworm_front_[arm]) continue;
              const double neuron_angle = measuredArmAngle(arm);
              action_b_pre_pulse_angle_rad_[arm] = neuron_angle;
              // Use the model only for the angle change caused by the thrust
              // step.  The raw Neuron angle remains the feedback reference;
              // this avoids an IMU/model zero offset corrupting the inverse.
              const double target_model_angle = bend_model_.angleRad(
                  action_b_best_pressure_kpa_,
                  action_b_hold_thrust_n_ + bend_model_thrust_offset_n_);
              double best_pressure = action_b_best_pressure_kpa_;
              double best_error = std::numeric_limits<double>::infinity();
              for (double pressure = action_b_best_pressure_kpa_;
                   pressure <= action_b_shape_max_pressure_kpa_ + 1.0e-9;
                   pressure += 0.1)
                {
                  const double predicted_angle = bend_model_.angleRad(
                      pressure,
                      pulse_target + bend_model_thrust_offset_n_,
                      hugmy::PressureThrustBendModel::Trend::INCREASING,
                      hugmy::PressureThrustBendModel::Trend::INCREASING);
                  const double error = std::abs(
                      predicted_angle - target_model_angle);
                  if (error < best_error)
                    {
                      best_error = error;
                      best_pressure = pressure;
                    }
                }
              action_b_pulse_pressure_[arm] = std::min(
                  maximum_pressure_kpa_, best_pressure);
              ROS_INFO("ACTION_B pulse angle hold arm%zu: saved Neuron %.2f deg, model angle %.2f deg, pressure %.1f -> %.1f kPa for thrust %.2f -> %.2f N (model error %.2f deg)",
                       arm, neuron_angle * 180.0 / M_PI,
                       target_model_angle * 180.0 / M_PI,
                       action_b_best_pressure_kpa_,
                       action_b_pulse_pressure_[arm],
                       action_b_hold_thrust_n_, pulse_target,
                       best_error * 180.0 / M_PI);
            }
          const double pulse_off = action_b_hold_thrust_n_ >=
                  action_b_max_thrust_n_ - 0.05
              ? std::max(0.0, action_b_max_thrust_n_ -
                                  action_b_pulse_amplitude_n_)
              : action_b_hold_thrust_n_;
          ROS_ERROR("ACTION_B PULSE %d/%d ON: tilt error %.2f deg, OFF %.2f N, ON %.2f N, selected pressure %.1f kPa, headroom %.2f N",
                    action_b_pulse_count_ + 1, action_b_pulse_repetitions_,
                    (body_tilt - action_b_lift_tilt_rad_) *
                        180.0 / M_PI,
                    pulse_off, pulse_target,
                    action_b_best_pressure_kpa_,
                    std::max(0.0, action_b_max_thrust_n_ -
                                  action_b_hold_thrust_n_));
        }

      // End a pulse as soon as it has produced a measurable additional body
      // lift.  Waiting for the full thrust step made the pneumatic angle
      // compensation arrive only after the body had already jumped several
      // degrees.
      const bool pulse_motion_reached = action_b_pulse_on_ &&
          body_tilt >= action_b_lift_tilt_rad_ +
                       action_b_hold_tilt_tolerance_rad_;
      if (action_b_shape_phase_ == 2 && action_b_pulse_on_ &&
          (pulse_motion_reached ||
           (now - action_b_pulse_phase_start_).toSec() >=
               action_b_pulse_on_duration_sec_))
        {
          ROS_INFO("ACTION_B pulse %d OFF: %s at thrust %.2f N, tilt %.2f/%.2f deg",
                   action_b_pulse_count_ + 1,
                   pulse_motion_reached ? "body motion reached" : "duration reached",
                   inchworm_thrust_[0], body_tilt * 180.0 / M_PI,
                   action_b_lift_tilt_rad_ * 180.0 / M_PI);
          action_b_pulse_on_ = false;
          ++action_b_pulse_count_;
          action_b_pulse_phase_start_ = now;
        }

      // OFF always remains the lift-supporting thrust. ON adds the reserved
      // pulse so vertical support and horizontal pulling occur together.
      const double pulse_off_thrust = action_b_hold_thrust_n_;
      const double front_target_thrust = action_b_pulse_on_
          ? std::min(action_b_max_thrust_n_,
                     pulse_off_thrust + action_b_pulse_amplitude_n_)
          : pulse_off_thrust;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (!inchworm_front_[arm])
            inchworm_pressure_[arm] = base_pressure_kpa_;
          else if (action_b_shape_phase_ <= 1)
            inchworm_pressure_[arm] = action_b_shape_pressure_kpa_;
          else if (action_b_pulse_on_)
            {
              // measuredArmAngle() becomes more negative as pressure bends
              // this arm.  Therefore pressure must be added when the pulse
              // has made the measured angle larger (straighter) than the
              // saved angle.  The previous saved-minus-measured sign added
              // pressure after the arm had already bent too far and formed
              // positive feedback.
              const double angle_error = wrapAngle(
                  measuredArmAngle(arm) -
                  action_b_pre_pulse_angle_rad_[arm]);
              inchworm_pressure_[arm] = std::max(
                  action_b_best_pressure_kpa_,
                  std::min(maximum_pressure_kpa_,
                      action_b_pulse_pressure_[arm] +
                      action_b_pulse_angle_pressure_gain_kpa_rad_ *
                          angle_error));
            }
          else
            inchworm_pressure_[arm] = action_b_best_pressure_kpa_;
          const double front_thrust_rate =
              action_b_shape_phase_ == 0 &&
                      front_target_thrust < inchworm_thrust_[arm]
                  ? action_b_hold_thrust_down_rate_n_s_
                  : action_b_thrust_ramp_n_s_;
          inchworm_thrust_[arm] = approach(
              inchworm_thrust_[arm],
              inchworm_front_[arm] ? front_target_thrust : 0.0,
              inchworm_front_[arm] ? front_thrust_rate
                                   : action_b_thrust_ramp_n_s_);
        }
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);

      ROS_INFO_THROTTLE(0.1,
          "Action B shape/hold/pulse: phase %d, tilt %.2f/%.2f deg, front angle %.1f deg, hold %.2f N, tilt-I %.2f N, pressure cap %.1f kPa, pressure-limited %d, horizontal %.2f N, pulse %d/%d %s, thrust [%.2f, %.2f, %.2f, %.2f], pressure [%.1f, %.1f, %.1f, %.1f]",
          action_b_shape_phase_,
          body_tilt * 180.0 / M_PI,
          action_b_lift_tilt_rad_ * 180.0 / M_PI,
          action_b_current_front_angle_rad_ * 180.0 / M_PI,
          action_b_hold_thrust_n_, action_b_hold_tilt_integral_n_,
          action_b_best_pressure_kpa_,
          action_b_pressure_limited_by_feedback_,
          std::max(0.0, action_b_best_horizontal_force_n_),
          action_b_pulse_count_,
          action_b_pulse_repetitions_, action_b_pulse_on_ ? "ON" : "OFF",
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3],
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3]);
      if (action_b_shape_phase_ == 2 && action_b_pulse_on_)
        {
          ROS_INFO_THROTTLE(0.1,
              "ACTION_B pulse angle feedback: arm0 saved/current/error %.1f/%.1f/%.1f deg -> %.1f kPa; arm3 saved/current/error %.1f/%.1f/%.1f deg -> %.1f kPa",
              action_b_pre_pulse_angle_rad_[0] * 180.0 / M_PI,
              measuredArmAngle(0) * 180.0 / M_PI,
              wrapAngle(measuredArmAngle(0) -
                        action_b_pre_pulse_angle_rad_[0]) * 180.0 / M_PI,
              inchworm_pressure_[0],
              action_b_pre_pulse_angle_rad_[3] * 180.0 / M_PI,
              measuredArmAngle(3) * 180.0 / M_PI,
              wrapAngle(measuredArmAngle(3) -
                        action_b_pre_pulse_angle_rad_[3]) * 180.0 / M_PI,
              inchworm_pressure_[3]);
        }
      if (action_b_shape_phase_ == 2 &&
          action_b_pulse_count_ >= action_b_pulse_repetitions_ &&
          !action_b_pulse_on_ &&
          (now - action_b_pulse_phase_start_).toSec() >=
              action_b_pulse_off_duration_sec_)
        {
          ROS_INFO("Action B completed %d front thrust pulses; restoring four-arm grasp",
                   action_b_pulse_count_);
          enterState(ACTION_B_REGRASP, now);
        }
      else if (elapsed >= action_b_front_regrip_timeout_sec_)
        {
          ROS_ERROR("Action B FRONT_REGRIP overall timeout: elapsed %.1f/%.1f s, phase %d, pulses %d/%d; restoring four-arm grasp",
                    elapsed, action_b_front_regrip_timeout_sec_,
                    action_b_shape_phase_, action_b_pulse_count_,
                    action_b_pulse_repetitions_);
          enterState(ACTION_B_REGRASP, now);
        }
      }
      break;

    case ACTION_B_REGRASP:
      inchworm_pressure_.fill(base_pressure_kpa_);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_thrust_[arm] = approach(
            inchworm_thrust_[arm], 0.0,
            inchworm_front_[arm] ? inchworm_thrust_ramp_n_s_
                                 : action_b_thrust_ramp_n_s_);
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      if ((pressureReached(inchworm_pressure_) &&
           std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                       [](double value) { return value <= 0.01; })) ||
          elapsed >= inchworm_anchor_timeout_sec_)
        {
          ROS_INFO("Action B complete; all arms commanded to %.1f kPa GRASP",
                   base_pressure_kpa_);
          enterState(SETTLE, now);
        }
      break;

    case SETTLE:
      inchworm_pressure_.fill(base_pressure_kpa_);
      inchworm_thrust_.fill(0.0);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (elapsed >= inchworm_settle_duration_sec_)
        {
          const double cycle_travel = inchwormTravel();
          const double overall_travel =
              inchworm_action_c_enabled_ && !inchworm_action_d_enabled_
                  ? action_c_direction_sign_x_ *
                        (mocap_position_[0] -
                         inchworm_overall_start_position_[0])
                  : (mocap_position_[0] -
                         inchworm_overall_start_position_[0]) *
                        inchworm_direction_world_[0] +
                    (mocap_position_[1] -
                         inchworm_overall_start_position_[1]) *
                        inchworm_direction_world_[1];
          ROS_INFO("Inchworm cycle %d finished: cycle %.2f mm, total %.2f/%.2f mm",
                   inchworm_cycle_count_, 1000.0 * cycle_travel,
                   1000.0 * overall_travel,
                   1000.0 * inchworm_total_target_distance_m_);
          if (inchworm_action_b_enabled_)
            {
              ROS_INFO("Action B test is single-shot; returning to GRASP after one cycle");
              inchworm_active_ = false;
              last_correction_end_ = now;
              enterState(GRASP, now);
              break;
            }
          const bool target_reached =
              overall_travel >= inchworm_total_target_distance_m_;
          const bool no_progress = cycle_travel < inchworm_min_cycle_progress_m_;
          const bool cycle_limit = inchworm_cycle_count_ >= inchworm_max_cycles_;
          if (!target_reached && !cycle_limit && !no_progress)
            {
              startInchwormCycle(now);
            }
          else
            {
              if (no_progress && !target_reached)
                ROS_WARN("Inchworm cycle retained only %.2f mm%s",
                         1000.0 * cycle_travel,
                         "; repetition stopped");
              if (cycle_limit && !target_reached)
                ROS_WARN("Inchworm repetition reached the %d-cycle limit",
                         inchworm_max_cycles_);
              inchworm_active_ = false;
              last_correction_end_ = now;
              enterState(GRASP, now);
            }
        }
      break;

    default:
      stopInchwormToGrasp(now, "unexpected locomotion state");
      break;
    }
}

AttitudePressureController::Vector2
AttitudePressureController::thrustMoment(const PressureArray& pressure) const
{
  Vector2 moment{{0.0, 0.0}};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const Vector3 position = rotorPosition(arm, pressure[arm]);
      const Vector3 direction = rotorDirection(arm, pressure[arm]);
      Vector3 lever, force;
      for (size_t axis = 0; axis < 3; ++axis)
        {
          lever[axis] = position[axis] - contact_position_[axis];
          force[axis] = thrust_n_[arm] * direction[axis];
        }
      const Vector3 rotor_moment = cross(lever, force);
      moment[0] += rotor_moment[0];
      moment[1] += rotor_moment[1];
    }
  return moment;
}

hugmy::RollRecoveryModel AttitudePressureController::measuredRecoveryModel() const
{
  hugmy::RollRecoveryModel result;
  result.valid = true;
  // Perched recovery asks for ADDITIONAL restoring moment, not the full
  // gravity moment: contacts already support the body, and their reaction
  // moments are not known. Do not inherit gravity_compensation_ from the
  // legacy/other-mode allocator. Rate PI handles residual contact/friction.
  // The old PD derivative is not stacked with the inner velocity PI either.
  result.moment = roll_moment_kp_ * std::max(0.0,
      wrapAngle(target_roll_ - roll_) * correction_axis_[0] - roll_recovery_config_.stop);
  if (recovery_selected_pair_release_ &&
      recovery_selected_support_loss_moment_nm_ > 0.0)
    {
      double release_progress = 0.0;
      size_t release_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (!prepare_arm_active_[arm] ||
              !std::isfinite(measured_pressure_[arm]) ||
              !std::isfinite(recovery_selected_release_start_pressure_[arm]))
            continue;
          const double release_target = recoverySelectedPairPressure(arm);
          const double release_span =
              recovery_selected_release_start_pressure_[arm] - release_target;
          if (release_span <= 1.0e-6) continue;
          release_progress += hugmy::recoveryClamp(
              (recovery_selected_release_start_pressure_[arm] -
               measured_pressure_[arm]) / release_span, 0.0, 1.0);
          ++release_count;
        }
      if (release_count > 0)
        result.moment += recovery_selected_support_loss_moment_nm_ *
            release_progress / static_cast<double>(release_count);
    }
  result.context[4] = bottom_pressure_kpa_;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bend = measuredArmBendAngle(arm);
      result.context[arm] = measured_pressure_[arm];
      result.context[5 + arm] = bend;
      result.valid = result.valid && std::isfinite(bend) && std::isfinite(measured_pressure_[arm]);
      if (!prepare_arm_active_[arm]) continue;
      result.lever += recoveryRotorLever(arm,bend);
    }
  result.valid = result.valid && std::isfinite(result.context[4]);
  // selectContinuousRecoveryPair() accepts only a candidate whose predicted
  // signed lever points toward the target.  Keep that sign here: taking the
  // absolute value allowed a pair whose moment reversed after flexion to
  // receive an increasing feed-forward command.
  return result;
}

double AttitudePressureController::recoveryRotorRawLever(
    size_t arm, double bend) const
{
  return recoveryRotorRawLeverAbout(arm, bend, contact_position_);
}

AttitudePressureController::Vector3
AttitudePressureController::recoveryContactPivot(
    const std::array<bool, ARM_COUNT>& selected_pair,
    bool selected_pair_release,
    const PressureArray* pressure_override,
    const PressureArray* bend_override) const
{
  Vector3 pivot{{0.0, 0.0, 0.0}};
  double weight_sum = 0.0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double pressure = pressure_override
          ? (*pressure_override)[arm] : measured_pressure_[arm];
      // Chamber pressure is not a contact-force sensor, but above the common
      // pneumatic floor it is a useful monotone load proxy.  Its weighted
      // mean gives a continuous CoP as support transfers between pairs,
      // rather than jumping the pivot between two arithmetic means.
      const double weight = std::isfinite(pressure)
          ? std::max(0.0, pressure - minimum_prepare_pressure_kpa_) : 0.0;
      if (weight <= 1.0e-6) continue;
      const double bend = bend_override
          ? (*bend_override)[arm] : measuredArmBendAngle(arm);
      if (!std::isfinite(bend)) continue;
      const Vector3 contact = armContactPointFromAngle(arm, bend);
      if (!std::isfinite(contact[0]) || !std::isfinite(contact[1]) ||
          !std::isfinite(contact[2])) continue;
      for (size_t axis = 0; axis < 3; ++axis)
        pivot[axis] += weight * contact[axis];
      weight_sum += weight;
    }
  if (weight_sum <= 1.0e-6)
    {
      // Preserve the old contact-topology fallback for missing pressure. It
      // is used only when no load proxy is available at all.
      size_t count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          const bool pivot_arm = selected_pair_release
              ? !selected_pair[arm] : selected_pair[arm];
          if (!pivot_arm) continue;
          const double bend = bend_override
              ? (*bend_override)[arm] : measuredArmBendAngle(arm);
          if (!std::isfinite(bend)) continue;
          const Vector3 contact = armContactPointFromAngle(arm, bend);
          if (!std::isfinite(contact[0]) || !std::isfinite(contact[1]) ||
              !std::isfinite(contact[2])) continue;
          for (size_t axis = 0; axis < 3; ++axis) pivot[axis] += contact[axis];
          ++count;
        }
      if (count == 0) return contact_position_;
      weight_sum = static_cast<double>(count);
    }
  for (double& value : pivot) value /= weight_sum;
  return pivot;
}

double AttitudePressureController::recoveryRotorRawLeverAbout(
    size_t arm, double bend, const Vector3& pivot) const
{
      const Vector3 position = rotorPositionFromAngle(arm, bend);
      // Same local-X cant and rotor signs as hugmy2.urdf.xacro. Neuron bend
      // is measured BELOW the rotor, so add cant here, not to the sensor angle.
      const double direction = (arm == 0 || arm == 2) ? 1.0 : -1.0;
      const Vector3 unit_force = rotateLocalToBody(rotateAboutLocalY(
          {{0.0, direction * std::sin(recovery_rotor_cant_rad_),
            std::cos(recovery_rotor_cant_rad_)}}, std::max(0.0, std::min(2.966, bend))), arm_yaw_[arm]);
      Vector3 lever;
      for (size_t axis = 0; axis < 3; ++axis)
        lever[axis] = position[axis] - pivot[axis];
      return cross(lever, unit_force)[0];
}

double AttitudePressureController::recoveryRotorLever(
    size_t arm, double bend) const
{
  // Loaded circle contacts use the physical contact-dependent moment.
  // Released arms, like the no-circle fallback, use the empirically verified
  // actuator-axis mapping after they have straightened away from contact.
  const bool flexed_circle_contact =
      recovery_selected_contact_pivot_enabled_ &&
      recovery_selected_pair_pressure_kpa_ >=
          recovery_circle_contact_pressure_kpa_;
  const double mapping_sign = flexed_circle_contact
      ? 1.0 : (correction_pair_reverse_spinal_mapping_ ? -1.0 : 1.0);
  const Vector3 pivot = recovery_selected_contact_pivot_enabled_
      ? recoveryContactPivot(
          prepare_arm_active_, recovery_selected_pair_release_)
      : contact_position_;
  return recoveryRotorRawLeverAbout(arm, bend, pivot) *
      correction_axis_[0] * mapping_sign;
}

AttitudePressureController::Vector3
AttitudePressureController::rotorPosition(size_t arm, double pressure) const
{
  return rotorPositionFromAngle(arm, bendAngle(arm, pressure));
}

AttitudePressureController::Vector3
AttitudePressureController::rotorPositionFromAngle(
    size_t arm, double bend_angle) const
{
  const double one_third = std::max(0.0, bend_angle) / 3.0;
  const std::array<double, 3> joint_angle{{
      std::min(0.8727, one_third), std::min(1.047, one_third),
      std::min(1.047, one_third)}};
  Vector3 local{{0.0, 0.0, 0.0}};
  const std::array<double, 2> segment_length{{0.023, 0.023}};
  double accumulated_angle = 0.0;
  for (size_t segment = 0; segment < segment_length.size(); ++segment)
    {
      accumulated_angle += joint_angle[segment];
      const Vector3 offset = rotateAboutLocalY(
          {{-segment_length[segment], 0.0, 0.0}}, accumulated_angle);
      for (size_t axis = 0; axis < 3; ++axis) local[axis] += offset[axis];
    }
  accumulated_angle += joint_angle[2];
  const Vector3 propeller_offset = rotateAboutLocalY(
      {{-0.020, 0.0, 0.0369}}, accumulated_angle);
  for (size_t axis = 0; axis < 3; ++axis) local[axis] += propeller_offset[axis];
  const Vector3 body_offset = rotateLocalToBody(local, arm_yaw_[arm]);
  Vector3 result;
  for (size_t axis = 0; axis < 3; ++axis)
    result[axis] = arm_base_position_[arm][axis] + body_offset[axis];
  return result;
}

AttitudePressureController::Vector3
AttitudePressureController::armContactPointFromAngle(
    size_t arm, double bend_angle) const
{
  // The Neuron is below the rotor and measures joints 1--3.  For the two
  // distal pneumatic joints, use the same mean joint curvature as the
  // measured proximal chain.  This is an explicit first-order approximation;
  // disagreement among the four inferred radii appears in the circle RMS and
  // lowers confidence rather than being hidden.
  const double mean_joint = std::max(0.0, bend_angle) / 3.0;
  const std::array<double, 4> joint_angle{{
      std::min(0.8727, mean_joint),
      std::min(1.047, mean_joint),
      std::min(1.047, mean_joint),
      std::min(1.047, mean_joint)}};
  const std::array<double, 4> segment_length{{0.023, 0.023, 0.040, 0.023}};
  Vector3 local{{0.0, 0.0, 0.0}};
  double accumulated_angle = 0.0;
  for (size_t segment = 0; segment < segment_length.size(); ++segment)
    {
      accumulated_angle += joint_angle[segment];
      const Vector3 offset = rotateAboutLocalY(
          {{-segment_length[segment], 0.0, 0.0}}, accumulated_angle);
      for (size_t axis = 0; axis < 3; ++axis) local[axis] += offset[axis];
    }
  const Vector3 body_offset = rotateLocalToBody(local, arm_yaw_[arm]);
  Vector3 result;
  for (size_t axis = 0; axis < 3; ++axis)
    result[axis] = arm_base_position_[arm][axis] + body_offset[axis];
  return result;
}

AttitudePressureController::Vector3
AttitudePressureController::rotorDirection(size_t arm, double pressure) const
{
  return rotorDirectionFromAngle(arm, bendAngle(arm, pressure));
}

AttitudePressureController::Vector3
AttitudePressureController::rotorDirectionFromAngle(
    size_t arm, double bend_angle) const
{
  const double angle = std::max(0.0, std::min(2.966, bend_angle));
  return rotateLocalToBody(rotateAboutLocalY({{0.0, 0.0, 1.0}}, angle),
                           arm_yaw_[arm]);
}

double AttitudePressureController::bendAngle(size_t arm, double pressure) const
{
  // Keep the same nonnegative thrust convention as joint_model.cpp. Fixed
  // rotors cannot realize negative thrust; the callback rejects it.
  const double table_thrust = std::max(0.0, thrust_n_[arm]) + bend_model_thrust_offset_n_;
  using Trend = hugmy::PressureThrustBendModel::Trend;
  Trend pressure_trend = Trend::NEUTRAL;
  if (std::isfinite(measured_pressure_[arm]))
    {
      if (pressure > measured_pressure_[arm] + 0.05)
        pressure_trend = Trend::INCREASING;
      else if (pressure < measured_pressure_[arm] - 0.05)
        pressure_trend = Trend::DECREASING;
    }
  // The allocator evaluates hypothetical thrust without a time history, so
  // use the center of the measured thrust hysteresis while retaining the
  // known pressure direction.
  return bend_model_.angleRad(pressure, table_thrust, pressure_trend,
                              Trend::NEUTRAL);
}

std::array<double, 3> AttitudePressureController::jointAngles(
    size_t arm, double pressure) const
{
  const double one_third = bendAngle(arm, pressure) / 3.0;
  // Same limits and distribution used by joint_model.cpp and robot_ver2 URDF.
  return {{std::max(0.0, std::min(0.8727, one_third)),
           std::max(0.0, std::min(1.047, one_third)),
           std::max(0.0, std::min(1.047, one_third))}};
}

AttitudePressureController::Vector3 AttitudePressureController::cross(
    const Vector3& lhs, const Vector3& rhs)
{
  return {{lhs[1]*rhs[2] - lhs[2]*rhs[1],
           lhs[2]*rhs[0] - lhs[0]*rhs[2],
           lhs[0]*rhs[1] - lhs[1]*rhs[0]}};
}

AttitudePressureController::Vector3
AttitudePressureController::rotateLocalToBody(const Vector3& value, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {{c*value[0] - s*value[1], s*value[0] + c*value[1], value[2]}};
}

AttitudePressureController::Vector3
AttitudePressureController::rotateAboutLocalY(const Vector3& value, double angle)
{
  // URDF joint axis is local -Y.
  const double c = std::cos(angle), s = std::sin(angle);
  return {{c*value[0] - s*value[2], value[1], s*value[0] + c*value[2]}};
}

double AttitudePressureController::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
