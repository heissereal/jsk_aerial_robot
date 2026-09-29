#include <hugmy/control/attitude_pressure_controller.h>
#include <hugmy/control/action_c_feedback.h>

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
  pnh_.param("allocation_iterations", allocation_iterations_, 6);
  pnh_.param("allocation_damping", allocation_damping_, 1e-5);
  pnh_.param("allocation_step_limit_kpa", allocation_step_limit_kpa_, 5.0);
  pnh_.param("allocation_pressure_epsilon_kpa", allocation_pressure_epsilon_kpa_, 0.25);
  pnh_.param("allocation_bias_weight", allocation_bias_weight_, 0.01);
  pnh_.param("state_machine_enabled", state_machine_enabled_, true);
  pnh_.param("action_c_reference_pulse_mode", action_c_reference_pulse_mode_, false);
  pnh_.param("action_c_reference_single_pulse", action_c_reference_single_pulse_, false);
  pnh_.param("action_c_pulse_search_enabled", action_c_pulse_search_enabled_, false);
  pnh_.param("action_c_pulse_search_step_n", action_c_pulse_search_step_n_, 0.5);
  pnh_.param("action_c_pulse_search_progress_m", action_c_pulse_search_progress_m_, 0.002);
  action_c_pulse_search_step_n_ = std::max(0.0, std::min(0.5, action_c_pulse_search_step_n_));
  action_c_pulse_search_progress_m_ = std::max(0.0005, action_c_pulse_search_progress_m_);
  pnh_.param("action_c_reference_off_sec", action_c_reference_off_sec_, 1.0);
  pnh_.param("action_c_reference_thrust_n", action_c_reference_thrust_n_, 3.0);
  action_c_reference_off_sec_ = std::max(0.1, action_c_reference_off_sec_);
  pnh_.param("action_c_pressure_pull_enabled", action_c_pressure_pull_enabled_, true);
  pnh_.param("action_c_propulsion_enabled", action_c_propulsion_enabled_, true);
  pnh_.param("action_c_pressure_pull_ratio", action_c_pressure_pull_ratio_, 0.35);
  pnh_.param("action_c_pressure_pull_max_n", action_c_pressure_pull_max_n_, 1.5);
  pnh_.param("action_c_pressure_pull_timeout_sec", action_c_pressure_pull_timeout_sec_, 5.0);
  pnh_.param("action_c_powered_regrip_timeout_sec", action_c_powered_regrip_timeout_sec_, 3.0);
  pnh_.param("action_c_flexion_lookahead_sec", action_c_flexion_lookahead_sec_, 0.08);
  if (!std::isfinite(action_c_pressure_pull_timeout_sec_) ||
      !std::isfinite(action_c_powered_regrip_timeout_sec_) || !std::isfinite(action_c_flexion_lookahead_sec_))
    throw std::invalid_argument("Nonfinite Action C powered-pull timing");
  pnh_.param("action_c_rear_release_max_kpa", action_c_rear_release_max_kpa_, 8.0);
  pnh_.param("action_c_rear_release_hold_kpa", action_c_rear_release_hold_kpa_, 10.0);
  pnh_.param("action_c_rear_release_hard_kpa", action_c_rear_release_hard_kpa_, 12.0);
  pnh_.param("action_c_rear_release_grace_sec", action_c_rear_release_grace_sec_, 0.10);
  if (!std::isfinite(action_c_rear_release_max_kpa_) ||
      !std::isfinite(action_c_rear_release_hold_kpa_) ||
      !std::isfinite(action_c_rear_release_hard_kpa_) ||
      !std::isfinite(action_c_rear_release_grace_sec_) ||
      action_c_rear_release_max_kpa_ < 0.0 || action_c_rear_release_max_kpa_ > 10.0 ||
      action_c_rear_release_hold_kpa_ < action_c_rear_release_max_kpa_ ||
      action_c_rear_release_hard_kpa_ < action_c_rear_release_hold_kpa_ ||
      action_c_rear_release_hard_kpa_ > 12.0 ||
      action_c_rear_release_grace_sec_ < 0.0 || action_c_rear_release_grace_sec_ > 0.3)
    throw std::invalid_argument("Invalid Action C rear release hysteresis limits");
  pnh_.param("action_c_rear_release_timeout_sec", action_c_rear_release_timeout_sec_, 3.0);
  pnh_.param("action_c_abort_reverse_travel_m", action_c_abort_reverse_travel_m_, 0.03);
  // Bound the new powered phase independently of the main pulse amplitude.
  action_c_pressure_pull_ratio_ = std::max(0.0, std::min(0.5, action_c_pressure_pull_ratio_));
  action_c_pressure_pull_max_n_ = std::max(0.0, std::min(1.5, action_c_pressure_pull_max_n_));
  action_c_pressure_pull_timeout_sec_ = std::max(0.1, std::min(6.0, action_c_pressure_pull_timeout_sec_));
  action_c_powered_regrip_timeout_sec_ = std::max(0.2, std::min(5.0, action_c_powered_regrip_timeout_sec_));
  action_c_flexion_lookahead_sec_ = std::max(0.02, std::min(0.15, action_c_flexion_lookahead_sec_));
  action_c_rear_release_max_kpa_ = std::max(0.0, std::min(10.0, action_c_rear_release_max_kpa_));
  action_c_rear_release_timeout_sec_ = std::max(0.1, std::min(5.0, action_c_rear_release_timeout_sec_));
  action_c_abort_reverse_travel_m_ = std::max(0.005, std::min(0.03, action_c_abort_reverse_travel_m_));
  pnh_.param("correction_roll_only", correction_roll_only_, true);
  pnh_.param("action_c_continuous_recovery_enabled",
             action_c_continuous_recovery_enabled_, true);
  // These settings affect continuous Action-C posture recovery ONLY.
  auto recovery_param = [&](const char* name, double& value, double low, double high) {
    double configured = value;
    pnh_.param(name, configured, value);
    if (std::isfinite(configured)) value = hugmy::recoveryClamp(configured, low, high);
  };
  recovery_param("recovery_angle_gain_s_inv", roll_recovery_config_.angle_gain, 0.1, 3.0);
  pnh_.param("recovery_response_learning_enabled", roll_recovery_config_.learn_response, true);
  recovery_param("recovery_target_rate_rad_s", roll_recovery_config_.max_target_rate, 0.02, 0.3);
  recovery_param("recovery_rate_kp_n_s_rad", roll_recovery_config_.rate_kp, 0.1, 10.0);
  recovery_param("recovery_rate_ki_n_rad", roll_recovery_config_.rate_ki, 0.0, 10.0);
  recovery_param("recovery_model_min_lever_m", roll_recovery_config_.min_lever, 0.005, 0.1);
  recovery_param("recovery_model_fallback_thrust_n", roll_recovery_config_.fallback_thrust, 0.0, 1.0);
  recovery_param("recovery_response_delay_sec", roll_recovery_config_.response_delay, 0.0, 1.0);
  recovery_param("recovery_response_window_sec", roll_recovery_config_.response_window, 0.2, 2.0);
  recovery_param("recovery_progress_window_sec", roll_recovery_config_.progress_window, 0.5, 2.0);
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
  pnh_.param("action_c_recovery_no_progress_timeout_sec",
             action_c_recovery_no_progress_timeout_sec_, 10.0);
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
  pnh_.param("pwm_test_positive_thrust_below_neutral",
             pwm_test_positive_thrust_below_neutral_, true);
  loadFixedVector(pnh_, "pwm_test_forward_polynomial",
                  pwm_test_forward_polynomial_);
  loadFixedVector(pnh_, "pwm_test_reverse_polynomial",
                  pwm_test_reverse_polynomial_);
  pnh_.param("prepare_pressure_ramp_rate_kpa_s", prepare_pressure_ramp_rate_, 10.0);
  pnh_.param("minimum_prepare_pressure_kpa", minimum_prepare_pressure_kpa_, 0.0);
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
  pnh_.param("use_mocap_attitude_for_inchworm",
             use_mocap_attitude_for_inchworm_, false);
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
  pnh_.param("action_c_min_thrust_n", action_c_min_thrust_n_, 3.0);
  pnh_.param("action_c_max_thrust_n", action_c_max_thrust_n_, 5.0);
  pnh_.param("action_c_thrust_step_n", action_c_thrust_step_n_, 0.5);
  pnh_.param("action_c_target_angle_rad", action_c_target_angle_rad_, M_PI / 2.0);
  pnh_.param("action_c_angle_tolerance_rad", action_c_angle_tolerance_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("action_c_pressure_gain_kpa_rad", action_c_pressure_gain_kpa_rad_, 8.0);
  pnh_.param("action_c_pulse_duration_sec", action_c_pulse_duration_sec_, 0.60);
  pnh_.param("action_c_thrust_ramp_n_s", action_c_thrust_ramp_n_s_, 8.0);
  pnh_.param("action_c_thrust_down_rate_n_s",
             action_c_thrust_down_rate_n_s_, 30.0);
  pnh_.param("action_c_extension_target_rad",
             action_c_extension_target_rad_, 8.0 * M_PI / 180.0);
  pnh_.param("action_c_max_extension_rad",
             action_c_max_extension_rad_, 20.0 * M_PI / 180.0);
  pnh_.param("action_c_max_extension_mismatch_rad",
             action_c_max_extension_mismatch_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("action_c_min_safe_angle_rad",
             action_c_min_safe_angle_rad_, 55.0 * M_PI / 180.0);
  pnh_.param("action_c_prepare_min_bend_rad",
             action_c_prepare_min_bend_rad_, 40.0 * M_PI / 180.0);
  pnh_.param("action_c_hard_min_bend_rad",
             action_c_hard_min_bend_rad_, 30.0 * M_PI / 180.0);
  pnh_.param("action_c_start_extension_margin_rad",
             action_c_start_extension_margin_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_pulse_roll_handoff_enabled", action_c_pulse_roll_handoff_enabled_, true);
  pnh_.param("action_c_pulse_roll_handoff_lookahead_sec", action_c_pulse_roll_handoff_lookahead_sec_, 0.10);
  pnh_.param("action_c_pulse_roll_handoff_margin_rad", action_c_pulse_roll_handoff_margin_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_extension_rate_limit_rad_s",
             action_c_extension_rate_limit_rad_s_, 180.0 * M_PI / 180.0);
  pnh_.param("action_c_rebend_target_rad",
             action_c_rebend_target_rad_, 6.0 * M_PI / 180.0);
  pnh_.param("action_c_rebend_pressure_boost_kpa",
             action_c_rebend_pressure_boost_kpa_, 5.0);
  pnh_.param("action_c_front_pulse_pressure_kpa",
             action_c_front_pulse_pressure_kpa_, 5.0);
  pnh_.param("action_c_recovery_timeout_sec",
             action_c_recovery_timeout_sec_, 1.5);
  pnh_.param("action_c_recovery_settle_sec",
             action_c_recovery_settle_sec_, 0.10);
  pnh_.param("action_c_max_reverse_travel_m",
             action_c_max_reverse_travel_m_, 0.005);
  pnh_.param("action_c_max_body_tilt_change_rad",
             action_c_max_body_tilt_change_rad_, 10.0 * M_PI / 180.0);
  pnh_.param("action_c_pulse_pressure_gain_kpa_rad",
             action_c_pulse_pressure_gain_kpa_rad_, 8.0);
  pnh_.param("action_c_pulse_pressure_rate_kpa_s",
             action_c_pulse_pressure_rate_kpa_s_, 10.0);
  pnh_.param("action_c_pulse_angle_deadband_rad",
             action_c_pulse_angle_deadband_rad_, 3.0 * M_PI / 180.0);
  pnh_.param("action_c_max_pressure_kpa", action_c_max_pressure_kpa_, 50.0);
  pnh_.param("action_c_success_confirm_sec", action_c_success_confirm_sec_, 0.15);
  pnh_.param("action_c_max_angle_retries", action_c_max_angle_retries_, 2);
  pnh_.param("action_c_max_error_before_thrust_increase_rad",
             action_c_max_error_before_thrust_increase_rad_, M_PI / 4.0);
  pnh_.param("action_c_regrip_min_retained_ratio",
             action_c_regrip_min_retained_ratio_, 0.75);
  pnh_.param("action_c_gradient_min_pressure_delta_kpa",
             action_c_gradient_min_pressure_delta_kpa_, 1.0);
  pnh_.param("action_c_gradient_min_angle_delta_rad",
             action_c_gradient_min_angle_delta_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_model_update_alpha", action_c_model_update_alpha_, 0.25);
  pnh_.param("action_c_max_feedforward_pressure_step_kpa",
             action_c_max_feedforward_pressure_step_kpa_, 10.0);
  pnh_.param("action_c_angle_thrust_kp_n_rad",
             action_c_angle_thrust_kp_n_rad_, 2.0);
  pnh_.param("action_c_angle_thrust_kd_n_rad_s",
             action_c_angle_thrust_kd_n_rad_s_, 0.08);
  pnh_.param("action_c_angle_thrust_max_trim_n",
             action_c_angle_thrust_max_trim_n_, 2.0);
  pnh_.param("action_c_extension_sync_kp_n_rad",
             action_c_extension_sync_kp_n_rad_, 0.0);
  pnh_.param("action_c_extension_sync_deadband_rad",
             action_c_extension_sync_deadband_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_extension_prediction_sec",
             action_c_extension_prediction_sec_, 0.04);
  pnh_.param("action_c_individual_extension_enabled",
             action_c_individual_extension_enabled_, false);
  pnh_.param("action_c_extension_trajectory_sec", action_c_extension_trajectory_sec_, 0.45);
  pnh_.param("action_c_individual_kp_n_rad", action_c_individual_kp_n_rad_, 6.0);
  pnh_.param("action_c_individual_kd_n_rad_s", action_c_individual_kd_n_rad_s_, 0.12);
  pnh_.param("action_c_individual_assist_max_n", action_c_individual_assist_max_n_, 0.25);
  pnh_.param("action_c_sync_brake_band_rad", action_c_sync_brake_band_rad_, 4.0 * M_PI / 180.0);
  pnh_.param("action_c_arm_learning_step_n", action_c_arm_learning_step_n_, 0.25);
  pnh_.param("action_c_angle_rate_filter_alpha",
             action_c_angle_rate_filter_alpha_, 0.25);
  pnh_.param("action_c_online_pressure_adaptation_enabled",
             action_c_online_pressure_adaptation_enabled_, false);
  pnh_.param("action_c_pulse_roll_kp_n_rad",
             action_c_pulse_roll_kp_n_rad_, 1.5);
  pnh_.param("action_c_pulse_roll_kd_n_rad_s",
             action_c_pulse_roll_kd_n_rad_s_, 0.05);
  pnh_.param("action_c_pulse_roll_max_trim_n",
             action_c_pulse_roll_max_trim_n_, 0.3);
  pnh_.param("action_c_roll_target_rad", action_c_roll_target_rad_, 0.0);
  pnh_.param("action_c_pre_roll_deadband_rad",
             action_c_pre_roll_deadband_rad_, 2.0 * M_PI / 180.0);
  pnh_.param("action_c_roll_recorrect_threshold_rad",
             action_c_roll_recorrect_threshold_rad_,
             10.0 * M_PI / 180.0);
  pnh_.param("action_c_pre_roll_rate_tolerance_rad_s",
             action_c_pre_roll_rate_tolerance_rad_s_, 10.0 * M_PI / 180.0);
  pnh_.param("action_c_pre_roll_kp_n_rad",
             action_c_pre_roll_kp_n_rad_, 8.0);
  pnh_.param("action_c_pre_roll_min_thrust_n",
             action_c_pre_roll_min_thrust_n_, 1.0);
  pnh_.param("action_c_pre_roll_max_thrust_n",
             action_c_pre_roll_max_thrust_n_, 2.5);
  pnh_.param("action_c_pre_roll_thrust_ramp_n_s",
             action_c_pre_roll_thrust_ramp_n_s_, 6.0);
  pnh_.param("action_c_pre_roll_settle_sec",
             action_c_pre_roll_settle_sec_, 0.10);
  pnh_.param("action_c_pre_roll_pressure_drop_kpa",
             action_c_pre_roll_pressure_drop_kpa_, 40.0);
  pnh_.param("action_c_pre_roll_min_pressure_kpa",
             action_c_pre_roll_min_pressure_kpa_, 0.0);
  pnh_.param("action_c_prepare_timeout_sec",
             action_c_prepare_timeout_sec_, 20.0);
  pnh_.param("action_c_auto_roll_correction_enabled",
             action_c_auto_roll_correction_enabled_, false);
  pnh_.param("action_c_adaptive_burst_enabled",
             action_c_adaptive_burst_enabled_, true);
  pnh_.param("action_c_optimizer_enabled",
             action_c_adaptive_burst_enabled_,
             action_c_adaptive_burst_enabled_);
  pnh_.param("action_c_optimizer_reset_on_command",
             action_c_optimizer_reset_on_command_, false);
  pnh_.param("action_c_thrust_reprobe_safe_pulses",
             action_c_thrust_reprobe_safe_pulses_, 3);
  pnh_.param("action_c_thrust_reprobe_step_n",
             action_c_thrust_reprobe_step_n_, 0.25);
  pnh_.param("action_c_thrust_reprobe_max_mismatch_rad",
             action_c_thrust_reprobe_max_mismatch_rad_, 12.0 * M_PI / 180.0);
  pnh_.param("action_c_optimize_bottom_enabled",
             action_c_optimize_bottom_enabled_, true);
  pnh_.param("action_c_bottom_initial_kpa",
             action_c_bottom_initial_kpa_, 30.0);
  pnh_.param("action_c_bottom_min_kpa",
             action_c_bottom_min_kpa_, 20.0);
  pnh_.param("action_c_bottom_max_kpa",
             action_c_bottom_max_kpa_, 30.0);
  pnh_.param("action_c_bottom_step_kpa",
             action_c_bottom_step_kpa_, 5.0);
  pnh_.param("action_c_adaptive_thrust_step_n",
             action_c_adaptive_thrust_step_n_, 0.25);
  pnh_.param("action_c_optimizer_thrust_step_n",
             action_c_adaptive_thrust_step_n_,
             action_c_adaptive_thrust_step_n_);
  pnh_.param("action_c_objective_progress_weight",
             action_c_objective_progress_weight_, 1.0);
  pnh_.param("action_c_objective_tilt_weight",
             action_c_objective_tilt_weight_, 0.10);
  pnh_.param("action_c_objective_rate_weight",
             action_c_objective_rate_weight_, 0.001);
  pnh_.param("action_c_objective_thrust_energy_weight",
             action_c_objective_thrust_energy_weight_, 0.0001);
  pnh_.param("action_c_objective_recovery_weight",
             action_c_objective_recovery_weight_, 0.030);
  pnh_.param("action_c_objective_angle_mismatch_weight",
             action_c_objective_angle_mismatch_weight_, 0.10);
  pnh_.param("action_c_objective_failed_pulse_weight",
             action_c_objective_failed_pulse_weight_, 0.005);
  pnh_.param("action_c_optimizer_initial_burst_pulses",
             action_c_optimizer_initial_burst_pulses_, 1);
  pnh_.param("action_c_optimizer_min_burst_pulses",
             action_c_optimizer_min_burst_pulses_, 1);
  pnh_.param("action_c_optimizer_max_burst_pulses",
             action_c_optimizer_max_burst_pulses_, 5);
  pnh_.param("action_c_posture_soft_limit_rad",
             action_c_posture_soft_limit_rad_, 8.0 * M_PI / 180.0);
  pnh_.param("action_c_posture_recovery_trigger_rad",
             action_c_posture_recovery_trigger_rad_,
             8.0 * M_PI / 180.0);
  pnh_.param("action_c_posture_resume_rad",
             action_c_posture_resume_rad_, 3.0 * M_PI / 180.0);
  pnh_.param("action_c_posture_settle_sec", action_c_posture_settle_sec_, 0.5);
  action_c_posture_settle_sec_ = std::max(0.0, action_c_posture_settle_sec_);
  pnh_.param("action_c_posture_rate_trigger_rad_s",
             action_c_posture_rate_trigger_rad_s_, 0.8);
  pnh_.param("action_c_tilt_growth_alpha",
             action_c_tilt_growth_alpha_, 0.5);
  pnh_.param("action_c_max_posture_recoveries",
             action_c_max_posture_recoveries_, 5);
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
  action_c_min_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_thrust_n_, action_c_min_thrust_n_));
  action_c_max_thrust_n_ = std::max(action_c_min_thrust_n_, std::min(
      pwm_test_max_thrust_n_, action_c_max_thrust_n_));
  action_c_thrust_step_n_ = std::max(0.0, action_c_thrust_step_n_);
  action_c_thrust_ramp_n_s_ = std::max(0.0, action_c_thrust_ramp_n_s_);
  action_c_thrust_down_rate_n_s_ = std::max(
      action_c_thrust_ramp_n_s_, action_c_thrust_down_rate_n_s_);
  action_c_extension_target_rad_ = std::max(0.0,
      action_c_extension_target_rad_);
  action_c_max_extension_rad_ = std::max(action_c_extension_target_rad_,
      action_c_max_extension_rad_);
  action_c_max_extension_mismatch_rad_ = std::max(0.0,
      action_c_max_extension_mismatch_rad_);
  action_c_min_safe_angle_rad_ = std::max(0.0, std::min(
      action_c_target_angle_rad_, action_c_min_safe_angle_rad_));
  if (!std::isfinite(action_c_prepare_min_bend_rad_) ||
      !std::isfinite(action_c_hard_min_bend_rad_) ||
      !std::isfinite(action_c_start_extension_margin_rad_) ||
      action_c_prepare_min_bend_rad_ < 0.0 || action_c_prepare_min_bend_rad_ > M_PI ||
      action_c_hard_min_bend_rad_ < 0.0 ||
      action_c_hard_min_bend_rad_ > action_c_min_safe_angle_rad_ ||
      action_c_start_extension_margin_rad_ <= 0.0 ||
      action_c_hard_min_bend_rad_ + action_c_extension_target_rad_ +
          action_c_start_extension_margin_rad_ > M_PI)
    throw std::invalid_argument("Invalid Action C preparation/pulse bend limits");
  if (!std::isfinite(action_c_pulse_roll_handoff_lookahead_sec_) ||
      action_c_pulse_roll_handoff_lookahead_sec_ < 0.0 ||
      action_c_pulse_roll_handoff_lookahead_sec_ > 0.25 ||
      !std::isfinite(action_c_pulse_roll_handoff_margin_rad_) ||
      action_c_pulse_roll_handoff_margin_rad_ < 0.0 ||
      action_c_pulse_roll_handoff_margin_rad_ > 5.0 * M_PI / 180.0)
    throw std::invalid_argument("Invalid Action C pulse roll handoff prediction limits");
  action_c_extension_rate_limit_rad_s_ = std::max(0.0,
      action_c_extension_rate_limit_rad_s_);
  action_c_rebend_target_rad_ = std::max(0.0,
      action_c_rebend_target_rad_);
  action_c_rebend_pressure_boost_kpa_ = std::max(0.0,
      action_c_rebend_pressure_boost_kpa_);
  action_c_front_pulse_pressure_kpa_ = std::max(0.0, std::min(
      std::min(maximum_pressure_kpa_, action_c_max_pressure_kpa_),
      action_c_front_pulse_pressure_kpa_));
  action_c_recovery_timeout_sec_ = std::max(0.1,
      action_c_recovery_timeout_sec_);
  action_c_recovery_settle_sec_ = std::max(0.0,
      action_c_recovery_settle_sec_);
  action_c_max_reverse_travel_m_ = std::max(0.0,
      action_c_max_reverse_travel_m_);
  action_c_max_body_tilt_change_rad_ = std::max(0.0,
      action_c_max_body_tilt_change_rad_);
  action_c_angle_thrust_kp_n_rad_ = std::max(0.0,
      action_c_angle_thrust_kp_n_rad_);
  action_c_angle_thrust_kd_n_rad_s_ = std::max(0.0,
      action_c_angle_thrust_kd_n_rad_s_);
  action_c_angle_thrust_max_trim_n_ = std::max(0.0,
      action_c_angle_thrust_max_trim_n_);
  action_c_extension_sync_kp_n_rad_ = std::max(0.0,
      action_c_extension_sync_kp_n_rad_);
  action_c_extension_sync_deadband_rad_ = std::max(0.0,
      action_c_extension_sync_deadband_rad_);
  action_c_extension_prediction_sec_ = std::max(0.0, std::min(
      0.08, action_c_extension_prediction_sec_));
  action_c_extension_trajectory_sec_ = std::max(0.05, std::min(
      action_c_pulse_duration_sec_, action_c_extension_trajectory_sec_));
  action_c_individual_kp_n_rad_ = std::max(0.0, action_c_individual_kp_n_rad_);
  action_c_individual_kd_n_rad_s_ = std::max(0.0, action_c_individual_kd_n_rad_s_);
  action_c_individual_assist_max_n_ = std::max(0.0, std::min(0.5, action_c_individual_assist_max_n_));
  action_c_sync_brake_band_rad_ = std::max(0.01, action_c_sync_brake_band_rad_);
  action_c_arm_learning_step_n_ = std::max(0.0, std::min(0.25, action_c_arm_learning_step_n_));
  action_c_angle_rate_filter_alpha_ = std::max(0.0, std::min(
      1.0, action_c_angle_rate_filter_alpha_));
  pnh_.param("action_c_coordination_enabled", action_c_coordination_enabled_, true);
  pnh_.param("action_c_coordination_assist_max_n", action_c_coordination_.max_assist, 0.3);
  pnh_.param("action_c_coordination_lag_kp_n_rad", action_c_coordination_.lag_kp, 6.0);
  action_c_coordination_.lag_kp = std::max(0.0, std::min(10.0, action_c_coordination_.lag_kp));
  pnh_.param("action_c_coordination_pressure_lead_max_sec", action_c_coordination_.max_lead, 0.12);
  pnh_.param("action_c_coordination_learning_trials", action_c_coordination_.learning_rounds, 3);
  action_c_coordination_.max_assist = std::max(0.0, std::min(0.3, action_c_coordination_.max_assist));
  action_c_coordination_.max_lead = std::max(0.0, std::min(0.12, action_c_coordination_.max_lead));
  action_c_coordination_.learning_rounds = std::max(0, std::min(5, action_c_coordination_.learning_rounds));
  pnh_.param("action_c_efficiency_objective_enabled", action_c_efficiency_objective_enabled_, true);
  pnh_.param("action_c_front_release_enabled", action_c_front_release_enabled_, true);
  pnh_.param("action_c_front_release_tolerance_kpa", action_c_front_release_tolerance_kpa_, 5.0);
  pnh_.param("action_c_bottom_deflation_tolerance_kpa", action_c_bottom_deflation_tolerance_kpa_, 5.0);
  pnh_.param("action_c_bottom_post_pulse_drop_kpa", action_c_bottom_post_pulse_drop_kpa_, 3.0);
  pnh_.param("action_c_bottom_sync_enabled", action_c_bottom_sync_enabled_, true);
  pnh_.param("action_c_bottom_wait_kpa", action_c_bottom_wait_kpa_, 5.0);
  action_c_bottom_wait_kpa_ = std::max(0.0, std::min(30.0, action_c_bottom_wait_kpa_));
  // Reference comparisons retain their explicit, fixed bottom waveform.
  if (action_c_reference_pulse_mode_) action_c_bottom_sync_enabled_ = false;
  action_c_bottom_post_pulse_drop_kpa_ = std::max(0.0, std::min(5.0, action_c_bottom_post_pulse_drop_kpa_));
  pnh_.param("action_c_roll_trend_window_sec", action_c_roll_trend_.window_sec, 0.5);
  pnh_.param("action_c_roll_trend_growth_rad", action_c_roll_trend_.growth_rad, M_PI / 180.0);
  pnh_.param("action_c_roll_trend_rate_rad_s", action_c_roll_trend_.outward_rate_rad_s, M_PI / 180.0);
  pnh_.param("action_c_posture_hard_roll_rad", action_c_posture_hard_roll_rad_, 15.0 * M_PI / 180.0);
  action_c_roll_trend_.window_sec = std::max(0.2, std::min(1.0, action_c_roll_trend_.window_sec));
  action_c_roll_trend_.growth_rad = std::max(0.1 * M_PI / 180.0, action_c_roll_trend_.growth_rad);
  action_c_roll_trend_.outward_rate_rad_s = std::max(0.0, action_c_roll_trend_.outward_rate_rad_s);
  action_c_posture_hard_roll_rad_ = std::max(action_c_posture_recovery_trigger_rad_,
      std::min(15.0 * M_PI / 180.0, action_c_posture_hard_roll_rad_));
  pnh_.param("action_c_pressure_hold_tolerance_kpa", action_c_pressure_hold_tolerance_kpa_, 5.0);
  pnh_.param("action_c_pressure_dropout_sec", action_c_pressure_dropout_sec_, 0.10);
  pnh_.param("action_c_pressure_max_deviation_kpa", action_c_pressure_max_deviation_kpa_, 8.0);
  action_c_pressure_hold_tolerance_kpa_ = std::max(0.0, std::min(5.0, action_c_pressure_hold_tolerance_kpa_));
  action_c_pressure_dropout_sec_ = std::max(0.0, std::min(0.10, action_c_pressure_dropout_sec_));
  action_c_pressure_max_deviation_kpa_ = std::max(action_c_pressure_hold_tolerance_kpa_,
      std::min(8.0, action_c_pressure_max_deviation_kpa_));
  action_c_front_release_tolerance_kpa_ = std::max(0.0, std::min(5.0, action_c_front_release_tolerance_kpa_));
  action_c_bottom_deflation_tolerance_kpa_ = std::max(0.0, std::min(5.0, action_c_bottom_deflation_tolerance_kpa_));
  pnh_.param("action_c_efficiency_shortfall_weight", action_c_efficiency_shortfall_weight_, 0.010);
  action_c_efficiency_shortfall_weight_ = std::max(0.0, action_c_efficiency_shortfall_weight_);
  pnh_.param("action_c_efficiency_thrust_weight", action_c_efficiency_thrust_weight_, 0.0001);
  pnh_.param("action_c_efficiency_recovery_time_weight", action_c_efficiency_recovery_time_weight_, 0.001);
  action_c_efficiency_thrust_weight_ = std::max(0.0, action_c_efficiency_thrust_weight_);
  action_c_efficiency_recovery_time_weight_ = std::max(0.0, action_c_efficiency_recovery_time_weight_);
  if (action_c_front_release_enabled_ && !action_c_reference_pulse_mode_ &&
      action_c_online_pressure_adaptation_enabled_)
    {
      action_c_online_pressure_adaptation_enabled_ = false;
      ROS_WARN("Action C front release owns pulse pressure; disabling conflicting online pressure-sign adaptation");
    }
  if (action_c_coordination_enabled_ &&
      (action_c_reference_pulse_mode_ || action_c_individual_extension_enabled_ ||
       action_c_online_pressure_adaptation_enabled_ || !action_c_adaptive_burst_enabled_))
    {
      action_c_coordination_enabled_ = false;
      ROS_WARN("Action C residual coordination disabled: experimental controller or non-trial mode owns the actuators");
    }
  if (action_c_propulsion_enabled_ &&
      (action_c_reference_pulse_mode_ || action_c_individual_extension_enabled_ ||
       action_c_online_pressure_adaptation_enabled_ || !action_c_adaptive_burst_enabled_ ||
       !action_c_pressure_pull_enabled_))
    {
      action_c_propulsion_enabled_ = false;
      ROS_WARN("Action C propulsion adaptation disabled: requires normal C powered pull with exclusive trial ownership");
    }
  if (inchworm_action_c_enabled_)
    ROS_INFO("Action C online coordination=%d: pair assist <= %.2f N, pressure lead <= %.3f s, learning <= %d valid trials/task; efficiency objective=%d",
             action_c_coordination_enabled_, action_c_coordination_.max_assist,
             action_c_coordination_.max_lead, action_c_coordination_.learning_rounds,
             action_c_efficiency_objective_enabled_);
  if (action_c_reference_pulse_mode_)
    {
      // Preserve the proven posture supervisor, but freeze gait exploration.
      action_c_individual_extension_enabled_ = false;
      action_c_online_pressure_adaptation_enabled_ = false;
      action_c_auto_roll_correction_enabled_ = false;
      action_c_reference_thrust_n_ = std::max(action_c_min_thrust_n_, std::min(
          action_c_max_thrust_n_, std::isfinite(action_c_reference_thrust_n_)
              ? action_c_reference_thrust_n_ : action_c_min_thrust_n_));
      ROS_WARN("Action C timed pulse: initial %.2f N per front arm, base ON %.2f s / OFF >= %.2f s, equal pair/no angle trim; single=%d search=%d",
               action_c_reference_thrust_n_, action_c_pulse_duration_sec_,
               action_c_reference_off_sec_, action_c_reference_single_pulse_,
               action_c_pulse_search_enabled_ && !action_c_reference_single_pulse_);
    }
  else if (inchworm_action_c_enabled_)
    ROS_WARN("Action C optimizer: front release=%d (0 kPa when enabled); rear exhaust at pulse ON / regrip at OFF; J-based [thrust,bottom,burst] search=%d, bottom search=%d, online pressure adaptation=%d, individual assist=%d",
             action_c_front_release_enabled_,
             action_c_adaptive_burst_enabled_, action_c_optimize_bottom_enabled_,
             action_c_online_pressure_adaptation_enabled_, action_c_individual_extension_enabled_);
  action_c_pulse_roll_kp_n_rad_ = std::max(0.0,
      action_c_pulse_roll_kp_n_rad_);
  action_c_pulse_roll_kd_n_rad_s_ = std::max(0.0,
      action_c_pulse_roll_kd_n_rad_s_);
  action_c_pulse_roll_max_trim_n_ = std::max(0.0,
      action_c_pulse_roll_max_trim_n_);
  action_c_pre_roll_deadband_rad_ = std::max(0.0,
      action_c_pre_roll_deadband_rad_);
  action_c_roll_recorrect_threshold_rad_ = std::max(
      action_c_pre_roll_deadband_rad_,
      action_c_roll_recorrect_threshold_rad_);
  action_c_pre_roll_rate_tolerance_rad_s_ = std::max(0.0,
      action_c_pre_roll_rate_tolerance_rad_s_);
  action_c_pre_roll_kp_n_rad_ = std::max(0.0,
      action_c_pre_roll_kp_n_rad_);
  action_c_pre_roll_min_thrust_n_ = std::max(0.0, std::min(
      pwm_test_max_thrust_n_, action_c_pre_roll_min_thrust_n_));
  action_c_pre_roll_max_thrust_n_ = std::max(
      action_c_pre_roll_min_thrust_n_, std::min(
          pwm_test_max_thrust_n_, action_c_pre_roll_max_thrust_n_));
  action_c_pre_roll_thrust_ramp_n_s_ = std::max(0.0,
      action_c_pre_roll_thrust_ramp_n_s_);
  action_c_pre_roll_settle_sec_ = std::max(0.0,
      action_c_pre_roll_settle_sec_);
  action_c_pre_roll_pressure_drop_kpa_ = std::max(0.0,
      action_c_pre_roll_pressure_drop_kpa_);
  action_c_pre_roll_min_pressure_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, action_c_pre_roll_min_pressure_kpa_));
  action_c_prepare_timeout_sec_ = std::max(1.0,
      action_c_prepare_timeout_sec_);
  action_c_bottom_min_kpa_ = std::max(0.0, std::min(
      pressure_safety_limit_kpa_, action_c_bottom_min_kpa_));
  action_c_bottom_max_kpa_ = std::max(action_c_bottom_min_kpa_, std::min(
      pressure_safety_limit_kpa_, action_c_bottom_max_kpa_));
  action_c_bottom_initial_kpa_ = std::max(action_c_bottom_min_kpa_, std::min(
      action_c_bottom_max_kpa_, action_c_bottom_initial_kpa_));
  action_c_bottom_step_kpa_ = std::max(0.0,
      action_c_bottom_step_kpa_);
  action_c_adaptive_thrust_step_n_ = std::max(0.0,
      action_c_adaptive_thrust_step_n_);
  action_c_thrust_reprobe_safe_pulses_ = std::max(1,
      action_c_thrust_reprobe_safe_pulses_);
  action_c_thrust_reprobe_step_n_ = std::max(0.0, std::min(
      0.25, action_c_thrust_reprobe_step_n_));
  action_c_thrust_reprobe_max_mismatch_rad_ = std::max(0.0, std::min(
      0.8 * action_c_max_extension_mismatch_rad_,
      action_c_thrust_reprobe_max_mismatch_rad_));
  action_c_objective_progress_weight_ = std::max(0.0,
      action_c_objective_progress_weight_);
  action_c_objective_tilt_weight_ = std::max(0.0,
      action_c_objective_tilt_weight_);
  action_c_objective_rate_weight_ = std::max(0.0,
      action_c_objective_rate_weight_);
  action_c_objective_thrust_energy_weight_ = std::max(0.0,
      action_c_objective_thrust_energy_weight_);
  action_c_objective_recovery_weight_ = std::max(0.0,
      action_c_objective_recovery_weight_);
  action_c_objective_angle_mismatch_weight_ = std::max(0.0,
      action_c_objective_angle_mismatch_weight_);
  action_c_objective_failed_pulse_weight_ = std::max(0.0,
      action_c_objective_failed_pulse_weight_);
  action_c_optimizer_min_burst_pulses_ = std::max(1,
      action_c_optimizer_min_burst_pulses_);
  action_c_optimizer_max_burst_pulses_ = std::max(
      action_c_optimizer_min_burst_pulses_,
      action_c_optimizer_max_burst_pulses_);
  action_c_optimizer_initial_burst_pulses_ = std::max(
      action_c_optimizer_min_burst_pulses_, std::min(
          action_c_optimizer_max_burst_pulses_,
          action_c_optimizer_initial_burst_pulses_));
  action_c_posture_soft_limit_rad_ = std::max(0.0,
      action_c_posture_soft_limit_rad_);
  action_c_posture_recovery_trigger_rad_ = std::max(
      action_c_posture_soft_limit_rad_,
      action_c_posture_recovery_trigger_rad_);
  action_c_posture_resume_rad_ = std::max(0.0, std::min(
      action_c_posture_recovery_trigger_rad_,
      action_c_posture_resume_rad_));
  action_c_posture_rate_trigger_rad_s_ = std::max(0.0,
      action_c_posture_rate_trigger_rad_s_);
  action_c_tilt_growth_alpha_ = std::max(0.0, std::min(
      1.0, action_c_tilt_growth_alpha_));
  action_c_max_posture_recoveries_ = std::max(1,
      action_c_max_posture_recoveries_);
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
  action_c_pulse_bend_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>(
      "action_c_pulse_bend_angle_deg", 1);
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
      action_c_bottom_timing_.observe(msg->data, measured_bottom_pressure_stamp_.toSec(),
                                     action_c_bottom_hold_kpa_);
    }
}

void AttitudePressureController::targetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  if (!std::isfinite(msg->vector.x) || !std::isfinite(msg->vector.y)) return;
  target_roll_ = msg->vector.x;
  target_pitch_ = msg->vector.y;
  target_received_ = true;
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
  const Vector2 direction{{msg->vector.x / norm, msg->vector.y / norm}};
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
      std::abs(direction[0]) < 0.9)
    {
      ROS_ERROR("Action C currently supports longitudinal commands only; use body direction [+1, 0] or [-1, 0]");
      return;
    }
  if (!mocap_received_)
    {
      ROS_ERROR("Cannot start inchworm motion before receiving mocap/pose");
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
  action_c_resume_pending_ = false;
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
  continuous_recovery_blocked_ = false;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  grasp_baseline_valid_ = false;
  inchworm_active_ = false;
  action_c_recovery_gate_.reset();
  action_c_resume_pending_ = false;
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

  const double dt = std::max(0.0, std::min(0.1, (event.current_real - event.last_real).toSec()));
  const double maximum_step = pressure_rate_limit_kpa_s_ * dt;

  // Use the full ROS-time interval (not the slew limiter's clipped dt).
  // The trial remains active during shared recovery, so its time is charged.
  if (action_c_optimizer_trial_active_)
    {
      const double clock_step = now.toSec() - action_c_optimizer_last_clock_sec_;
      if (!std::isfinite(clock_step) || clock_step < 0)
        action_c_optimizer_clock_valid_ = false;
      else
        {
          action_c_optimizer_elapsed_sec_ += clock_step;
          if (action_c_optimizer_recovery_pending_)
            action_c_optimizer_recovery_sec_ += clock_step;
        }
      action_c_optimizer_last_clock_sec_ = now.toSec();
    }

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
          action_c_pressure_ready_gate_.reset();
          action_c_rear_release_gate_.reset();
          action_c_roll_trend_.reset();
          action_c_boundary_ready_ = false;
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
          const bool mocap_fresh = mocap_received_ &&
              (now - mocap_stamp_).toSec() <= inchworm_mocap_timeout_sec_;
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
              state_ != ACTION_C_EVALUATE &&
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
                  "mocap=%d age=%.3f/%.3f s, angular_rate=%.3f/%.3f rad/s, "
                  "max_pressure=%.2f/%.2f kPa over_limit_for=%.3f/%.3f s",
                  state_, body_imu_fresh, body_imu_age, imu_timeout_sec_,
                  pressure_valid, neurons_fresh, oldest_neuron_age,
                  neuron_timeout_sec_, mocap_fresh, mocap_age,
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
      if (action_c_optimizer_trial_active_ &&
          action_c_optimizer_recovery_pending_)
        {
          // Complete the candidate with the actual post-recovery state. The
          // separate recovery indicator penalizes invoking this controller,
          // while these terms also retain its peak posture and thrust cost.
          action_c_optimizer_peak_tilt_rad_ = std::max(
              action_c_optimizer_peak_tilt_rad_, attitude_error);
          action_c_optimizer_peak_rate_rad_s_ = std::max(
              action_c_optimizer_peak_rate_rad_s_, angular_rate);
          double recovery_thrust_squared = 0.0;
          const PressureArray& active_recovery_thrust =
              state_ == THRUST_CORRECTION
                  ? correction_thrust_ : prepare_shape_thrust_;
          for (double thrust : active_recovery_thrust)
            recovery_thrust_squared += thrust * thrust;
          action_c_optimizer_thrust_energy_n2s_ +=
              recovery_thrust_squared * std::max(0.0, dt);
        }
      const double recovery_stop_angle = (action_c_resume_pending_ || model_c_recovery)
          ? action_c_posture_resume_rad_ : correction_stop_angle_;
      const bool continuous_c_recovery = action_c_continuous_recovery_enabled_ &&
          inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !fixed_arm_test_enabled_ && roll_only_recovery;
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
            publishBottomTarget(action_c_resume_pending_ ? 0.0 : rocking_bottom_low_kpa_);
          publishPressureTarget(PressureArray{{base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_}}, maximum_step);
          {
          bool grasp_pressure_ready = pressure_valid;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            grasp_pressure_ready = grasp_pressure_ready &&
                std::abs(measured_pressure_[arm] - base_pressure_kpa_) <=
                pressure_tolerance_kpa_;
          if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
            grasp_pressure_ready = pressure_valid && actionCPressureReady(
                PressureArray{{base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_}}, now);
          if (!grasp_pressure_ready)
            ROS_INFO_THROTTLE(1.0,
                "GRASP waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                base_pressure_kpa_, pressure_tolerance_kpa_);
          const double active_correction_trigger = action_c_resume_pending_
              ? action_c_posture_recovery_trigger_rad_ : correction_trigger_angle_;
          // Continuous C resumes on fresh pressure + stable posture, not
          // atmospheric pressure. The next PREPARE owns the low bottom target.
          const bool recovery_bottom_ready = !action_c_resume_pending_ ||
              (continuous_c_recovery ? recovery_pressure_fresh :
               (std::isfinite(bottom_pressure_kpa_) &&
                std::abs(bottom_pressure_kpa_) <= action_c_bottom_deflation_tolerance_kpa_));
          const bool c_trend_mode = inchworm_action_c_enabled_ && !inchworm_action_d_enabled_;
          if (c_trend_mode)
            action_c_roll_trend_.update(body_imu_fresh && pressure_valid && neurons_fresh &&
                (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready)),
                wrapAngle(roll_ - target_roll_), recovery_roll_rate, now.toSec());
          const bool c_hard_roll = c_trend_mode &&
              std::abs(wrapAngle(roll_ - target_roll_)) >= action_c_posture_hard_roll_rad_;
          const bool stable_c_roll = c_trend_mode && !c_hard_roll &&
              action_c_roll_trend_.ready && !action_c_roll_trend_.worsening;
          const bool c_recovery_requested = !c_trend_mode || c_hard_roll || action_c_roll_trend_.worsening;
          // A handoff already confirmed the recovery request. Do not demand
          // another pressure/settling/trend cycle before correcting it.
          const bool continuous_recovery_requested = continuous_c_recovery &&
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              recovery_pressure_fresh && !continuous_recovery_blocked_ &&
              angular_rate < correction_max_angular_rate_ &&
              attitude_error > active_correction_trigger &&
              ((action_c_resume_pending_ && !continuous_recovery_started_) || c_hard_roll);
          const bool recovery_settled = action_c_boundary_settle_gate_.update(
              body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && recovery_bottom_ready &&
              angular_rate <= action_c_pre_roll_rate_tolerance_rad_s_,
              now.toSec(), action_c_posture_settle_sec_);
          if (action_c_resume_pending_ && !recovery_settled)
            ROS_INFO_THROTTLE(1.0,
                "Action C auto-resume waiting: bottom %.1f kPa (zero not required in continuous mode), regrasp=%d, roll rate %.3f rad/s, blocked=%d; saved goal retained",
                bottom_pressure_kpa_, grasp_pressure_ready, angular_rate, continuous_recovery_blocked_);
          if (action_c_resume_pending_ && !continuous_c_recovery && !recovery_settled && !continuous_recovery_requested &&
              (now - state_start_).toSec() >= action_c_prepare_timeout_sec_)
            {
              stopInchwormToGrasp(now, "Action C recovery deflation/regrasp settling timeout");
              break;
            }
          // C correction succeeds inside 3 deg. At regrasp, also accept a stable
          // offset below the hard limit, so a completed recovery does not
          // immediately trigger again solely because roll remains above 8 deg.
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
          if (resume_ready)
            {
              if (resumeActionCAfterRecovery(now)) break;
            }
          const bool correction_confirmed = correction_entry_gate_.update(
              target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              (!continuous_c_recovery || (!continuous_recovery_blocked_ &&
               recovery_pressure_fresh && angular_rate < correction_max_angular_rate_)) &&
              (continuous_c_recovery || (grasp_pressure_ready && recovery_bottom_ready &&
               (!action_c_resume_pending_ || recovery_settled))) &&
              attitude_error > active_correction_trigger && c_recovery_requested,
              now.toSec(), correction_trigger_confirm_sec_);
          if ((correction_confirmed || continuous_recovery_requested) &&
              (continuous_recovery_requested || last_correction_end_.isZero() ||
               (now - last_correction_end_).toSec() >= cooldown_duration_))
            {
              ROS_INFO("Posture recovery accepted: error %.2f deg > %.2f deg; confirmation %.2f s, confirmed handoff=%d; stop at %.2f deg",
                       attitude_error * 180.0 / M_PI,
                       active_correction_trigger * 180.0 / M_PI,
                       correction_trigger_confirm_sec_,
                       continuous_recovery_requested,
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
                  // Preserve the commanded GRASP pressure, not a transient
                  // measurement taken while an arm is still inflating.
                  grasp_pressure_[arm] = base_pressure_kpa_;
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
                  continuous_recovery_started_ = true;
                  // Use the verified Spinal pair sign independently of model
                  // magnitude; measured geometry sets magnitude in PREPARE.
                  correction_axis_ = {{roll_error >= 0.0 ? 1.0 : -1.0, 0.0}};
                  prepare_arm_active_ = roll_error >= 0.0
                      ? std::array<bool, ARM_COUNT>{{true, true, false, false}}
                      : std::array<bool, ARM_COUNT>{{false, false, true, true}};
                  continuous_roll_recovery_.reset(attitude_error, now.toSec());
                  recovery_predictive_.reset();
                  roll_recovery_config_.stop = recovery_stop_angle;
                  roll_recovery_config_.horizon = correction_prediction_horizon_sec_;
                  roll_recovery_config_.limit = std::min(4.0,
                      std::min(prepare_max_thrust_n_, correction_max_thrust_n_));
                  roll_recovery_config_.rise = prepare_thrust_ramp_rate_;
                  roll_recovery_config_.fall = correction_thrust_down_rate_;
                  roll_recovery_config_.integral_rate = correction_stall_thrust_ramp_n_s_;
                  roll_recovery_config_.start_rate = correction_motion_start_rate_;
                  roll_recovery_config_.detect_thrust = correction_motion_detection_min_thrust_n_;
                  roll_recovery_config_.confirm_sec = correction_motion_confirm_sec_;
                  roll_recovery_config_.confirm_angle = correction_motion_min_error_reduction_;
                  roll_recovery_config_.progress_angle = correction_progress_epsilon_;
                  roll_recovery_config_.stall_sec = correction_stall_timeout_sec_;
                  roll_recovery_config_.abort_sec = action_c_recovery_no_progress_timeout_sec_;
                  roll_recovery_config_.max_rate = correction_max_angular_rate_;
                  roll_recovery_config_.max_target_rate = std::min(
                      roll_recovery_config_.max_target_rate, 0.5 * correction_max_angular_rate_);
                  prepare_shape_thrust_.fill(0.0);
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (prepare_arm_active_[arm])
                      prepare_pressure_[arm] = std::max(minimum_prepare_pressure_kpa_,
                          std::min(grasp_pressure_[arm], recovery_release_pressure_kpa_));
                  publishBottomTarget(0.0);
                  publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
                  ROS_INFO("Action C continuous roll recovery: exhaust bottom and selected pair in parallel; no zero-pressure wait; limit %.2f N per rotor",
                           std::min(prepare_max_thrust_n_, correction_max_thrust_n_));
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
                  if (!prepare_arm_active_[arm]) prepare_pressure_[arm] = grasp_pressure_[arm];
                  else if ((!recovery_predictive_config_.enabled || recovery_predictive_config_.shadow) &&
                           continuous_roll_recovery_.stalled &&
                           continuous_roll_recovery_.thrust >= 0.8 * roll_recovery_config_.limit)
                    prepare_pressure_[arm] = std::max(minimum_prepare_pressure_kpa_,
                        prepare_pressure_[arm] - recovery_pressure_down_rate_ * std::max(0.0, dt));
                }
              const double signed_error = roll_error * correction_axis_[0];
              const double omega = recovery_roll_rate * correction_axis_[0];
              const auto model = measuredRecoveryModel();
              auto& response = roll_recovery_response_[correction_axis_[0] > 0.0 ? 0 : 1];
              const bool recovery_inputs_valid = recovery_pressure_fresh &&
                  std::isfinite(bottom_pressure_kpa_) && bottom_pressure_kpa_ < pressure_safety_limit_kpa_ &&
                  std::isfinite(body_angular_velocity_[0]) &&
                  std::abs(body_angular_velocity_[0]) < correction_max_angular_rate_;
              double predictive_target = std::numeric_limits<double>::quiet_NaN();
              if (recovery_predictive_config_.enabled && recovery_inputs_valid && model.valid &&
                  std::isfinite(omega) && std::abs(omega) < correction_max_angular_rate_ &&
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
                  config.pressure_min=minimum_prepare_pressure_kpa_;
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
                      // No feasible prediction is NOT permission to fall back
                      // to an increasing thrust. The standard fault exit regrips.
                      continuous_roll_recovery_.fault=true;
                      ROS_ERROR_THROTTLE(1., "Action C predictive recovery infeasible/invalid; stopping correction and regrasping");
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
              publishPressureTarget(prepare_pressure_, std::numeric_limits<double>::infinity());
              const double thrust = continuous_roll_recovery_.update(
                  recovery_inputs_valid ? signed_error : std::numeric_limits<double>::quiet_NaN(),
                  omega, now.toSec(), dt, model, roll_recovery_config_, response, predictive_target);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                prepare_shape_thrust_[arm] = prepare_arm_active_[arm] ? thrust : 0.0;
              publishCorrectionThrust(prepare_shape_thrust_);
              ROS_INFO_THROTTLE(0.25,
                  "Action C model-rate recovery: roll error %.2f deg, rate %.3f/%.3f rad/s, pair thrust %.2f/%.2f N (command/target), FF %.2f N, I %.2f N, breakaway %.2f N, moment %.3f Nm, lever %.3f m, model=%d learned=%d gain %.3f samples=%u, moving=%d stalled=%d speed_brake=%d predicted_brake=%d horizon=%.3f s, gravity_ff=0, bottom %.1f kPa (exhaust, no wait), recent_progress %.2f deg / %.2f s, progress_age %.2f s",
                  signed_error * 180.0 / M_PI, omega, continuous_roll_recovery_.target_rate, thrust,
                  continuous_roll_recovery_.target_thrust,
                  continuous_roll_recovery_.feedforward, continuous_roll_recovery_.boost,
                  continuous_roll_recovery_.hold, model.moment, model.lever,
                  continuous_roll_recovery_.model_valid, continuous_roll_recovery_.learned_response,
                  response.gain, response.samples,
                  continuous_roll_recovery_.moving, continuous_roll_recovery_.stalled,
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
                  ROS_WARN("Action C continuous recovery ended: finished=%d fault=%d elapsed=%.2f s; restoring grasp",
                           continuous_roll_recovery_.finished, continuous_roll_recovery_.fault,
                           (now - state_start_).toSec());
                  if (!continuous_roll_recovery_.finished)
                    {
                      const bool progress_timeout = continuous_roll_recovery_.timed_out ||
                          (!continuous_roll_recovery_.fault && (now - state_start_).toSec() >= prepare_timeout_);
                      if (progress_timeout && action_c_resume_pending_)
                        {
                          // End evaluation before any hand-assisted displacement.
                          finishActionCOptimizerTrial(true, "recovery progress timeout; waiting for stable posture");
                          continuous_recovery_blocked_ = true;
                          continuous_recovery_waiting_for_settle_ = true;
                          prepare_shape_thrust_.fill(0.0);
                          publishCorrectionThrust(prepare_shape_thrust_);
                          publishBottomTarget(0.0);
                          publishPressureTarget(grasp_pressure_, std::numeric_limits<double>::infinity());
                          action_c_resume_gate_.reset();
                          action_c_boundary_settle_gate_.reset();
                          enterState(RECOVER, now);
                          ROS_WARN("Action C recovery paused: motors stopped, goal retained; resume after roll within %.1f deg, rate <=0.05 rad/s for 1 s, fresh sensors/mocap and regrasp. A zero motion command cancels continuation.",
                                   action_c_posture_resume_rad_ * 180.0 / M_PI);
                          break;
                        }
                      // No repeated timed search pulses after a fault/stall.
                      // Re-arm only on a new operator command or enable cycle.
                      continuous_recovery_blocked_ = true;
                      stopInchwormToGrasp(now, "continuous recovery fault/timeout; new operator command required");
                      break;
                    }
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
  const ArmPair correction_pair = pitch_dominant
      ? (pitch_error >= 0.0 ? ArmPair{{1, 2}} : ArmPair{{0, 3}})
      : (roll_error >= 0.0 ? ArmPair{{0, 1}} : ArmPair{{2, 3}});
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
          "Correction geometry sign disagrees with Spinal attitude; retaining verified motors %zu+%zu",
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
  spinal::PerchingThrustCommand command;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    command.thrust[arm] = std::max(0.0, std::min(correction_max_thrust_n_, thrust[arm]));

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
      publishPwmTest(thrust);
      return;
    }

  if (use_root_spinal_topics_ && root_correction_thrust_pub_)
    root_correction_thrust_pub_.publish(command);
  else
    correction_thrust_pub_.publish(command);
}

void AttitudePressureController::publishPwmTest(const PressureArray& thrust)
{
  spinal::PwmTest command;
  bool has_nonzero_thrust = false;
  for (const double arm_thrust : thrust)
    has_nonzero_thrust = has_nonzero_thrust || std::abs(arm_thrust) > 0.01;

  // Spinal's individual PwmTest callback enters one global flight-motor test
  // mode. Values for omitted motor indices remain cached, so a command for a
  // subset could unintentionally replay an old command on another rotor.
  // Whenever any flight rotor is active, explicitly command all four and set
  // every nominally inactive rotor to the bidirectional neutral duty.
  if (has_nonzero_thrust)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      {
        const double bounded_thrust = std::max(-pwm_test_max_reverse_thrust_n_,
            std::min(pwm_test_max_thrust_n_, thrust[arm]));
        const double pwm = std::abs(bounded_thrust) <= 0.01
            ? pwm_test_neutral_pwm_ : signedThrustToPwm(bounded_thrust);
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
    "ACTION_C_PREPARE", "ACTION_C_HORIZONTAL_PULL", "ACTION_C_EVALUATE",
    "ACTION_C_REGRIP", "ROCK_BOTTOM_PRELOAD", "ROCK_FRONT_WINDUP",
    "ROCK_FRONT_LAND", "ROCK_REAR_UNLOAD", "ROCK_REAR_RECOVER",
    "ROCK_REAR_LAND", "ACTION_A_FRONT_PLACE", "ACTION_D_PREPARE",
    "ACTION_D_RAMP", "ACTION_D_REGRIP"
  };
  // Preserve the same release latch across ON -> powered flexion only.
  // A new attempt, regrip, disable or recovery must acquire release anew.
  if (!(state_ == ACTION_C_HORIZONTAL_PULL && state == ACTION_C_EVALUATE &&
        action_c_pressure_pull_active_))
    action_c_rear_release_gate_.reset();
  state_ = state;
  if (state_ == ACTION_C_PREPARE) action_c_bottom_post_pulse_ = false;
  if (state_ == ACTION_C_PREPARE) action_c_bottom_timing_.beginCycle();
  if (state_ == GRASP || state_ == RECOVER) action_c_bottom_timing_.beginCycle();
  correction_entry_gate_.reset();
  action_c_resume_gate_.reset();
  action_c_recovery_gate_.reset();
  action_c_boundary_settle_gate_.reset();
  action_c_pressure_ready_gate_.reset();
  action_c_roll_trend_.reset();
  action_c_boundary_ready_ = false;
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
  inchworm_direction_body_ = direction;
  inchworm_direction_world_ = {{
      std::cos(yaw_) * direction[0] - std::sin(yaw_) * direction[1],
      std::sin(yaw_) * direction[0] + std::cos(yaw_) * direction[1]}};
  inchworm_overall_start_position_ = mocap_position_;
  inchworm_total_target_distance_m_ = distance > 0.0 ? distance
                                                      : inchworm_default_distance_m_;
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
    action_c_direction_sign_x_ = direction[0] >= 0.0 ? 1.0 : -1.0;
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
      !action_c_preserve_adaptation_on_start_)
    {
      action_c_absolute_target_x_m_ = mocap_position_[0] +
          action_c_direction_sign_x_ * inchworm_total_target_distance_m_;
      action_c_resume_direction_ = direction;
      // A new operator task permits a bounded recalibration block. Shared
      // posture recovery is NOT a new task and must not restart learning.
      action_c_coordination_.rounds = 0;
      if (action_c_adaptive_burst_enabled_)
        {
          const bool optimizer_direction_changed =
              action_c_optimizer_direction_sign_x_ != 0.0 &&
              action_c_optimizer_direction_sign_x_ !=
                  action_c_direction_sign_x_;
          if (!action_c_optimizer_initialized_ ||
              action_c_optimizer_reset_on_command_ ||
              optimizer_direction_changed)
            {
              initializeActionCOptimizer();
              if (optimizer_direction_changed)
                ROS_WARN("Action C travel direction changed; reset optimizer instead of reusing the opposite-direction candidate");
            }
          action_c_optimizer_direction_sign_x_ =
              action_c_direction_sign_x_;
          pnh_.setParam("action_c_optimizer_direction_sign_x",
                        action_c_optimizer_direction_sign_x_);
          // An operator command starts a fresh measurement window. If the
          // previous command ended mid-candidate, discard that incomplete
          // window but retain all completed trial results and the candidate.
          action_c_optimizer_trial_active_ = false;
          beginActionCOptimizerTrial(now);
        }
      else
        action_c_bottom_hold_kpa_ = action_c_bottom_initial_kpa_;
      action_c_burst_pulse_count_ = 0;
      action_c_posture_recovery_count_ = 0;
      action_c_estimated_tilt_growth_rad_ = 0.0;
      action_c_last_retained_progress_m_ = 0.0;
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
  // This latch belongs to the complete requested move, not to one short
  // stroke.  Once roll has been restored, Action C may issue consecutive
  // pulses until the wider re-correction threshold is crossed.
  action_c_posture_ready_latched_ = false;
  startInchwormCycle(now);
  ROS_INFO("Inchworm motion started: body direction [%.2f, %.2f], total target %.1f mm, front arms [%d, %d, %d, %d]",
           direction[0], direction[1], 1000.0 * inchworm_total_target_distance_m_,
           inchworm_front_[0], inchworm_front_[1], inchworm_front_[2],
           inchworm_front_[3]);
}

void AttitudePressureController::startInchwormCycle(const ros::Time& now)
{
  const double overall_travel =
      inchworm_action_c_enabled_ && !inchworm_action_d_enabled_
          ? action_c_direction_sign_x_ *
                (mocap_position_[0] - inchworm_overall_start_position_[0])
          : (mocap_position_[0] - inchworm_overall_start_position_[0]) *
                inchworm_direction_world_[0] +
            (mocap_position_[1] - inchworm_overall_start_position_[1]) *
                inchworm_direction_world_[1];
  const double remaining = std::max(0.0,
      inchworm_total_target_distance_m_ - overall_travel);
  inchworm_start_position_ = mocap_position_;
  inchworm_target_distance_m_ = inchworm_action_d_enabled_
      ? remaining : std::min(inchworm_stroke_distance_m_, remaining);
  // Freeze the burst goal before the first pulse. Neither early recovery nor
  // fewer completed pulses can shrink the denominator of the shortfall cost.
  if (action_c_optimizer_trial_active_ && action_c_optimizer_target_progress_m_ <= 0.0)
    action_c_optimizer_target_progress_m_ = std::min(remaining,
        inchworm_stroke_distance_m_ * action_c_optimizer_burst_pulses_);
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
  else if (inchworm_action_c_enabled_)
    {
      action_c_bottom_post_pulse_ = false;
      // Keep the useful pulse level and locally learned pressure/angle model
      // through consecutive short strokes in one requested move.  Repeating
      // the ineffective 2-N search pulse before every stroke made the former
      // "continuous" behavior stop-and-search each time.
      if (inchworm_cycle_count_ == 1 &&
          !action_c_preserve_adaptation_on_start_)
        {
          if (!action_c_adaptive_burst_enabled_)
            action_c_thrust_n_ = action_c_reference_pulse_mode_
                ? action_c_reference_thrust_n_ : action_c_min_thrust_n_;
          action_c_pressure_bias_kpa_.fill(0.0);
          action_c_pressure_bend_sign_.fill(1.0);
          action_c_dtheta_dpressure_.fill(0.02);
          action_c_dtheta_dthrust_.fill(0.0);
          action_c_pressure_gradient_initialized_ = false;
          action_c_last_pulse_valid_ = false;
          action_c_last_pulse_thrust_n_ = 0.0;
        }
      else
        action_c_thrust_n_ = std::max(action_c_min_thrust_n_, std::min(
            action_c_max_thrust_n_, action_c_thrust_n_));
      action_c_angle_retry_ = 0;
      action_c_pulse_started_ = false;
      action_c_pressure_pull_active_ = false;
      action_c_success_ = false;
      action_c_pulse_angle_rad_ = 0.0;
      action_c_front_angle_rad_.fill(0.0);
      action_c_front_pressure_.fill(base_pressure_kpa_);
      action_c_pulse_start_angle_rad_.fill(0.0);
      action_c_pulse_min_angle_rad_.fill(0.0);
      action_c_rate_previous_angle_rad_.fill(0.0);
      action_c_filtered_bend_rate_rad_s_.fill(0.0);
      action_c_previous_neuron_stamp_.fill(ros::Time(0));
      action_c_command_thrust_n_ = 0.0;
      action_c_pulse_start_travel_m_ = 0.0;
      action_c_pulse_start_roll_ = roll_;
      action_c_pulse_start_pitch_ = pitch_;
      action_c_pre_roll_thrust_n_ = 0.0;
      action_c_pre_roll_pair_sign_ = 0;
      action_c_pre_roll_ready_start_ = ros::Time(0);
      action_c_recovery_gate_.reset();
      action_c_rate_time_ = now;
      action_c_recovery_ready_start_ = ros::Time(0);
      action_c_pulse_had_extension_ = false;
      action_c_pulse_overextended_ = false;
      action_c_optimizer_cycle_accounted_ = false;
      action_c_cycle_start_tilt_rad_ =
          std::abs(wrapAngle(roll_ - target_roll_));
      action_c_cycle_peak_tilt_rad_ = action_c_cycle_start_tilt_rad_;
      action_c_cycle_peak_rate_rad_s_ =
          std::abs(body_angular_velocity_[0]);
      // As with the rocking gait, apply the bottom force on the front side.
      // This creates the rearward preload requested for Action C and reduces
      // the chance that loss of front contact launches the whole body back.
      publishBottomDirection(-1.0);
      enterState(ACTION_C_PREPARE, now);
      publishBottomTarget(actionCBottomTarget());
      ROS_WARN("Action C started: direction %s, active front arms [%d %d %d %d], optimizer candidate [T %.2f N, bottom %.1f kPa, burst %d], bounded T %.1f--%.1f N / bottom %.1f--%.1f kPa; target extension %.1f deg; travel uses signed mocap x",
               action_c_direction_sign_x_ > 0.0 ? "+x" : "-x",
               inchworm_front_[0], inchworm_front_[1],
               inchworm_front_[2], inchworm_front_[3],
               action_c_thrust_n_, action_c_bottom_hold_kpa_,
               action_c_optimizer_burst_pulses_,
               action_c_min_thrust_n_, action_c_max_thrust_n_,
               action_c_bottom_min_kpa_, action_c_bottom_max_kpa_,
               action_c_extension_target_rad_ * 180.0 / M_PI);
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
  continuous_recovery_waiting_for_settle_ = false;
  action_c_recovery_gate_.reset();
  if (action_c_optimizer_trial_active_)
    finishActionCOptimizerTrial(true, reason);
  const PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
  const PressureArray grasp_target{{base_pressure_kpa_, base_pressure_kpa_,
                                    base_pressure_kpa_, base_pressure_kpa_}};
  inchworm_active_ = false;
  action_c_resume_pending_ = false;
  action_c_preserve_adaptation_on_start_ = false;
  inchworm_thrust_ = zero_thrust;
  prepare_shape_thrust_ = zero_thrust;
  publishCorrectionThrust(zero_thrust);
  publishBottomTarget(action_a_bottom_low_kpa_);
  publishPressureTarget(grasp_target,
                        std::numeric_limits<double>::infinity());
  last_correction_end_ = now;
  enterState(GRASP, now);
  ROS_WARN("Inchworm stopped and returned directly to GRASP: %s", reason);
}

bool AttitudePressureController::finishActionCPropulsionTrial(bool recovery_required, double retained_dx)
{
  if (!action_c_propulsion_enabled_ || !action_c_pressure_pull_enabled_ || action_c_reference_pulse_mode_) return false;
  const bool safe = !recovery_required && !action_c_optimizer_trial_had_extension_guard_ &&
      action_c_optimizer_clock_valid_ &&
      action_c_optimizer_valid_pulses_ == action_c_optimizer_completed_pulses_ &&
      action_c_optimizer_peak_tilt_rad_ < action_c_posture_recovery_trigger_rad_ &&
      action_c_optimizer_peak_rate_rad_s_ < action_c_posture_rate_trigger_rad_s_;
  const double base = std::min(action_c_pressure_pull_max_n_, action_c_pressure_pull_ratio_ * action_c_thrust_n_);
  // Pressure-only identification is allowed without successful locomotion.
  // A recovery changes contact/pressure ownership, so do not commit that window.
  const int changed = !recovery_required && action_c_propulsion_.commitTiming(inchworm_front_) ? 1 :
      action_c_propulsion_.finish(safe, action_c_optimizer_completed_pulses_,
          retained_dx, action_c_optimizer_target_progress_m_, base, action_c_pressure_pull_max_n_, inchworm_front_);
  pnh_.setParam("action_c_pull_forward_impulse_ns", action_c_propulsion_.trial_forward_impulse);
  pnh_.setParam("action_c_pull_extra_n", action_c_propulsion_.extra_n);
  // This learner owns timing. Existing coordination may still update left/
  // right offsets on a later boundary, but must not also update pressure lead.
  action_c_coordination_.lag_count.fill(0);
  if (!changed) return false;
  // Re-evaluate the SAME T/P/burst; previous-waveform scores are no longer comparable.
  action_c_optimizer_best_score_ = -std::numeric_limits<double>::infinity();
  action_c_optimizer_direction_index_ = 0;
  action_c_optimizer_sweep_improved_ = false;
  action_c_optimizer_reprobe_pending_ = false;
  pnh_.setParam("action_c_optimizer_best_score", action_c_optimizer_best_score_);
  ROS_WARN("Action C PROPULSION commit: %s, pull extra %.2f N, leads [%.3f %.3f %.3f %.3f] s; same T/P/burst re-evaluation, outer search held",
      changed==1 ? "pressure timing only" : "pull thrust only", action_c_propulsion_.extra_n,
      action_c_propulsion_.response[0].lead, action_c_propulsion_.response[1].lead,
      action_c_propulsion_.response[2].lead, action_c_propulsion_.response[3].lead);
  return true;
}

void AttitudePressureController::observeActionCPropulsion(const ros::Time& now)
{
  if (!action_c_propulsion_enabled_ || !action_c_pressure_pull_enabled_ || action_c_reference_pulse_mode_) return;
  const double age=(now-measured_pressure_stamp_).toSec();
  const bool pressure_fresh=!measured_pressure_stamp_.isZero() && age>=0 && age<=.25;
  double forward=0.;
  bool both_powered=true;
  for (size_t arm=0; arm<ARM_COUNT; ++arm) if (inchworm_front_[arm])
    {
      auto& response=action_c_propulsion_.response[arm];
      auto& observer=action_c_pulse_observer_[arm];
      observer.update(hugmy::relativeBendRate(neuron_gyro_[arm][1], body_angular_velocity_[0],
          body_angular_velocity_[1], arm_yaw_[arm]), neuron_stamp_[arm].toSec());
      const bool simulation_bend = std::isfinite(grip_joint_bend_rad_[arm]) &&
          !grip_state_stamp_.isZero() && (now-grip_state_stamp_).toSec()>=0 &&
          (now-grip_state_stamp_).toSec()<=.25;
      const double bend=simulation_bend ? measuredArmBendAngle(arm) : observer.bend;
      const double imu_age=(now-neuron_stamp_[arm]).toSec();
      const bool fresh=pressure_fresh && (observer.valid || simulation_bend) && imu_age>=0 && imu_age<=imu_timeout_sec_;
      if (pressure_fresh && inchworm_pressure_[arm]>=action_c_coordination_rebend_target_[arm]-.01)
        response.command(now.toSec(), measured_pressure_[arm], inchworm_pressure_[arm], bend);
      response.observe(now.toSec(), measured_pressure_stamp_.toSec(), measured_pressure_[arm],
          bend, observer.rate, pressure_fresh, fresh);
      const double projection=hugmy::actionCForwardProjection(bend, arm_yaw_[arm],
          recovery_rotor_cant_rad_, (arm==0 || arm==2) ? 1. : -1., roll_, pitch_,
          wrapAngle(yaw_-action_c_propulsion_heading_rad_), action_c_direction_sign_x_);
      if (fresh) forward+=projection*inchworm_thrust_[arm];
      both_powered = both_powered && fresh && projection>0. && inchworm_thrust_[arm]>.05;
    }
  if (action_c_pressure_pull_active_ && pressure_fresh)
    action_c_propulsion_.force(now.toSec(), forward, both_powered);
  ROS_INFO_THROTTLE(.2, "Action C PROPULSION: estimated commanded forward force %.3f N (initial body-x horizontal frame), pull impulse %.4f Ns, pressure delay [%.3f %.3f %.3f %.3f] s, rise [%.1f %.1f %.1f %.1f] kPa/s",
      forward, action_c_propulsion_.forward_impulse,
      action_c_propulsion_.response[0].delay, action_c_propulsion_.response[1].delay,
      action_c_propulsion_.response[2].delay, action_c_propulsion_.response[3].delay,
      action_c_propulsion_.response[0].rate, action_c_propulsion_.response[1].rate,
      action_c_propulsion_.response[2].rate, action_c_propulsion_.response[3].rate);
}

void AttitudePressureController::initializeActionCOptimizer()
{
  action_c_propulsion_ = hugmy::ActionCPropulsion();
  action_c_propulsion_heading_rad_ = yaw_;
  action_c_coordination_.offset.fill(0.0);
  action_c_coordination_.lead.fill(0.0);
  action_c_coordination_.rounds = 0;
  action_c_coordination_.clearTrial();
  action_c_coordination_.startPulse();
  action_c_optimizer_initialized_ = true;
  action_c_optimizer_trial_active_ = false;
  action_c_optimizer_recovery_pending_ = false;
  action_c_optimizer_sweep_improved_ = false;
  action_c_optimizer_trial_index_ = 0;
  action_c_optimizer_direction_index_ = 0;
  action_c_optimizer_best_score_ =
      -std::numeric_limits<double>::infinity();
  action_c_optimizer_thrust_step_n_ =
      action_c_adaptive_thrust_step_n_;
  action_c_optimizer_bottom_step_kpa_ = action_c_bottom_step_kpa_;
  action_c_thrust_n_ = action_c_reference_pulse_mode_
      ? action_c_reference_thrust_n_ : action_c_min_thrust_n_;
  action_c_bottom_hold_kpa_ = action_c_bottom_initial_kpa_;
  action_c_optimizer_burst_pulses_ =
      action_c_optimizer_initial_burst_pulses_;
  action_c_optimizer_best_thrust_n_ = action_c_thrust_n_;
  action_c_optimizer_best_bottom_kpa_ = action_c_bottom_hold_kpa_;
  action_c_optimizer_best_burst_pulses_ =
      action_c_optimizer_burst_pulses_;
  action_c_optimizer_safe_thrust_max_n_ = action_c_max_thrust_n_;
  action_c_thrust_safety_.reset(action_c_max_thrust_n_);
  action_c_arm_thrust_offset_n_.fill(0.0);
  action_c_arm_calibration_changed_ = false;
  action_c_optimizer_reprobe_pending_ = false;
  pnh_.setParam("action_c_optimizer_hard_thrust_max_n", action_c_max_thrust_n_);
  pnh_.setParam("action_c_optimizer_pair_thrust_max_n", action_c_max_thrust_n_);
  pnh_.setParam("action_c_optimizer_reprobe_safe_pulses", 0);
  pnh_.setParam("action_c_optimizer_last_guard_reasons", 0);
  pnh_.setParam("action_c_optimizer_safe_thrust_max_n",
                action_c_optimizer_safe_thrust_max_n_);
  ROS_WARN("Action C optimizer reset: initial candidate T=%.2f N, bottom=%.1f kPa, burst=%d",
           action_c_thrust_n_, action_c_bottom_hold_kpa_,
           action_c_optimizer_burst_pulses_);
}

void AttitudePressureController::beginActionCOptimizerTrial(
    const ros::Time& now)
{
  // Explicit fixed/single diagnostics do not run the optimizer. Timed-pulse
  // search uses the same measured pulse-to-regrip progress and safety record.
  if (!action_c_adaptive_burst_enabled_ ||
      (action_c_reference_pulse_mode_ &&
       (!action_c_pulse_search_enabled_ || action_c_reference_single_pulse_))) return;
  action_c_optimizer_trial_active_ = true;
  action_c_propulsion_.beginTrial();
  action_c_coordination_.clearTrial();
  action_c_optimizer_elapsed_sec_ = 0.0;
  action_c_optimizer_recovery_sec_ = 0.0;
  action_c_optimizer_last_clock_sec_ = now.toSec();
  action_c_optimizer_clock_valid_ = true;
  action_c_optimizer_recovery_pending_ = false;
  action_c_optimizer_trial_start_x_m_ = mocap_position_[0];
  action_c_optimizer_retained_progress_m_ = 0.0;
  action_c_optimizer_target_progress_m_ = 0.0;
  action_c_optimizer_peak_tilt_rad_ =
      std::abs(wrapAngle(roll_ - target_roll_));
  action_c_optimizer_peak_rate_rad_s_ =
      std::abs(body_angular_velocity_[0]);
  action_c_optimizer_peak_extension_mismatch_rad_ = 0.0;
  action_c_optimizer_thrust_energy_n2s_ = 0.0;
  action_c_optimizer_completed_pulses_ = 0;
  action_c_optimizer_successful_pulses_ = 0;
  action_c_optimizer_valid_pulses_ = 0;
  action_c_optimizer_tracking_pulses_ = 0;
  action_c_optimizer_failed_pulses_ = 0;
  action_c_optimizer_trial_had_extension_guard_ = false;
  action_c_optimizer_trial_guard_reasons_ = 0;
  action_c_optimizer_trial_required_recovery_ = false;
  action_c_burst_pulse_count_ = 0;
  ++action_c_optimizer_trial_index_;
  pnh_.setParam("action_c_optimizer_current_trial",
                action_c_optimizer_trial_index_);
  pnh_.setParam("action_c_optimizer_current_thrust_n",
                action_c_thrust_n_);
  pnh_.setParam("action_c_optimizer_current_bottom_kpa",
                action_c_bottom_hold_kpa_);
  pnh_.setParam("action_c_optimizer_current_burst_pulses",
                action_c_optimizer_burst_pulses_);
  ROS_WARN("Action C optimizer trial %d START %s: T=%.2f N, bottom=%.1f kPa, burst=%d",
           action_c_optimizer_trial_index_,
           action_c_direction_sign_x_ > 0.0 ? "+x" : "-x",
           action_c_thrust_n_,
           action_c_bottom_hold_kpa_, action_c_optimizer_burst_pulses_);
}

void AttitudePressureController::selectNextActionCCandidate()
{
  if (action_c_reference_pulse_mode_)
    {
      action_c_optimizer_burst_pulses_ = 1;
      action_c_optimizer_reprobe_pending_ = false;
      if (action_c_pulse_search_enabled_ && !action_c_reference_single_pulse_ &&
          action_c_adaptive_burst_enabled_)
        {
          const double previous = action_c_thrust_n_;
          const double retained = action_c_optimizer_retained_progress_m_ /
              std::max(1, action_c_optimizer_valid_pulses_);
          action_c_thrust_n_ = hugmy::actionCNextPulseAmplitude(
              previous, action_c_min_thrust_n_,
              std::min(action_c_max_thrust_n_, action_c_optimizer_safe_thrust_max_n_),
              action_c_pulse_search_step_n_, retained, action_c_pulse_search_progress_m_,
              action_c_optimizer_valid_pulses_ > 0,
              action_c_optimizer_trial_had_extension_guard_,
              action_c_optimizer_trial_required_recovery_, action_c_pulse_started_);
          action_c_bottom_hold_kpa_ = action_c_bottom_initial_kpa_;
          ROS_INFO("Action C PULSE SEARCH: retained %.2f/%.2f mm, valid=%d guard=%d recovery=%d; next %.2f -> %.2f N, ceiling %.2f N, bottom %.1f kPa",
                   1000.0*retained, 1000.0*action_c_pulse_search_progress_m_,
                   action_c_optimizer_valid_pulses_, action_c_optimizer_trial_had_extension_guard_,
                   action_c_optimizer_trial_required_recovery_, previous, action_c_thrust_n_,
                   action_c_optimizer_safe_thrust_max_n_, action_c_bottom_hold_kpa_);
        }
      else
        ROS_INFO("Action C REFERENCE: candidate frozen at %.2f N / %.1f kPa; no automatic thrust increase",
                 action_c_thrust_n_, action_c_bottom_hold_kpa_);
      return;
    }
  if (!action_c_adaptive_burst_enabled_)
    return;

  if (action_c_individual_extension_enabled_ && action_c_arm_calibration_changed_)
    {
      // Changing individual feed-forward changes the candidate: scores from
      // the old arm calibration are not comparable. Test one pulse first.
      action_c_arm_calibration_changed_ = false;
      action_c_optimizer_reprobe_pending_ = false;
      action_c_optimizer_best_score_ = -std::numeric_limits<double>::infinity();
      action_c_optimizer_direction_index_ = 0;
      action_c_optimizer_burst_pulses_ = 1;
      action_c_thrust_n_ = std::min(action_c_thrust_n_, action_c_optimizer_safe_thrust_max_n_);
      ROS_WARN("Action C ARM CALIBRATION: next offsets [%.2f %.2f %.2f %.2f] N; base %.2f N, one pulse, old score invalidated",
               action_c_arm_thrust_offset_n_[0], action_c_arm_thrust_offset_n_[1],
               action_c_arm_thrust_offset_n_[2], action_c_arm_thrust_offset_n_[3], action_c_thrust_n_);
      return;
    }

  if (action_c_individual_extension_enabled_ && action_c_optimizer_tracking_pulses_ == 0)
    {
      // Do not defeat individual calibration by raising BOTH arms just
      // because a non-synchronized pulse failed to make forward progress.
      action_c_optimizer_reprobe_pending_ = false;
      action_c_optimizer_burst_pulses_ = 1;
      action_c_thrust_n_ = std::min(action_c_thrust_n_, action_c_optimizer_safe_thrust_max_n_);
      ROS_INFO("Action C TRACKING FIRST: base %.2f N unchanged; one pulse with individual assist until both extensions track the target",
               action_c_thrust_n_);
      return;
    }

  if (action_c_optimizer_reprobe_pending_)
    {
      // Test the newly unlocked increment exactly once, even if the current
      // best remains at the old ceiling. Do not change bottom at the same time.
      action_c_optimizer_reprobe_pending_ = false;
      action_c_thrust_n_ = std::min(action_c_max_thrust_n_,
          action_c_optimizer_safe_thrust_max_n_);
      action_c_optimizer_burst_pulses_ = 1;
      ROS_WARN("Action C optimizer REPROBE: one pulse at %.2f N / bottom %.1f kPa after verified safe pulses; all pulse-stop guards remain active",
               action_c_thrust_n_, action_c_bottom_hold_kpa_);
      return;
    }

  const bool have_safe_best =
      std::isfinite(action_c_optimizer_best_score_);
  if (!have_safe_best)
    {
      // No successful baseline: never wander from 30 to 20 kPa or increase
      // burst count around an invalid current point. Change only thrust,
      // one pulse at a time, and only after a safe completed measurement.
      action_c_bottom_hold_kpa_ = action_c_bottom_initial_kpa_;
      action_c_optimizer_burst_pulses_ = action_c_optimizer_min_burst_pulses_;
      const double next_thrust = std::min(
          std::min(action_c_max_thrust_n_,
                   action_c_optimizer_safe_thrust_max_n_),
          action_c_thrust_n_ + action_c_optimizer_thrust_step_n_);
      if (!action_c_optimizer_trial_had_extension_guard_ &&
          !action_c_optimizer_trial_required_recovery_ &&
          action_c_optimizer_valid_pulses_ > 0 &&
          next_thrust > action_c_thrust_n_ + 1.0e-6)
        {
          action_c_thrust_n_ = next_thrust;
        }
      action_c_thrust_n_ = std::min(action_c_thrust_n_,
          action_c_optimizer_safe_thrust_max_n_);
      ROS_INFO("Action C optimizer BASELINE: no positive safe baseline yet; T=%.2f N, bottom=%.1f kPa, burst=%d (safety ceiling %.2f N; no bottom/burst exploration)",
               action_c_thrust_n_, action_c_bottom_hold_kpa_,
               action_c_optimizer_burst_pulses_,
               action_c_optimizer_safe_thrust_max_n_);
      return;
    }
  const double base_thrust = std::min(
      action_c_optimizer_safe_thrust_max_n_, have_safe_best
          ? action_c_optimizer_best_thrust_n_ : action_c_thrust_n_);
  const double base_bottom = action_c_optimize_bottom_enabled_
      ? action_c_optimizer_best_bottom_kpa_ : action_c_bottom_initial_kpa_;
  const int base_burst = have_safe_best
      ? action_c_optimizer_best_burst_pulses_
      : action_c_optimizer_burst_pulses_;
  bool candidate_found = false;
  double candidate_thrust = base_thrust;
  double candidate_bottom = base_bottom;
  int candidate_burst = base_burst;

  // Bounded coordinate pattern search around the best safe candidate seen so
  // far: +P, -P, +T, -T, +N, -N. Bottom support is explored before asking
  // the human-contact rotors for more thrust. Continuous steps halve after a full
  // sweep without improvement; burst length remains an integer step of one.
  for (int attempt = 0; attempt < 6 && !candidate_found; ++attempt)
    {
      const int direction = action_c_optimizer_direction_index_;
      action_c_optimizer_direction_index_ =
          (action_c_optimizer_direction_index_ + 1) % 6;
      candidate_thrust = base_thrust;
      candidate_bottom = base_bottom;
      candidate_burst = base_burst;
      if (direction < 2 && !action_c_optimize_bottom_enabled_)
        continue;
      if (direction == 0)
        candidate_bottom += action_c_optimizer_bottom_step_kpa_;
      else if (direction == 1)
        candidate_bottom -= action_c_optimizer_bottom_step_kpa_;
      else if (direction == 2)
        {
          if (action_c_optimizer_valid_pulses_ > 0 &&
              !action_c_optimizer_trial_had_extension_guard_ &&
              !action_c_optimizer_trial_required_recovery_)
            candidate_thrust += action_c_optimizer_thrust_step_n_;
        }
      else if (direction == 3)
        candidate_thrust -= action_c_optimizer_thrust_step_n_;
      else if (direction == 4)
        {
          if (action_c_optimizer_successful_pulses_ > 0 &&
              !action_c_optimizer_trial_required_recovery_ &&
              !action_c_optimizer_trial_had_extension_guard_)
            ++candidate_burst;
        }
      else
        --candidate_burst;

      candidate_thrust = std::max(action_c_min_thrust_n_, std::min(
          std::min(action_c_max_thrust_n_,
                   action_c_optimizer_safe_thrust_max_n_),
          candidate_thrust));
      candidate_bottom = std::max(action_c_bottom_min_kpa_, std::min(
          action_c_bottom_max_kpa_, candidate_bottom));
      candidate_burst = std::max(action_c_optimizer_min_burst_pulses_,
          std::min(action_c_optimizer_max_burst_pulses_, candidate_burst));
      candidate_found =
          std::abs(candidate_thrust - base_thrust) > 1.0e-6 ||
          std::abs(candidate_bottom - base_bottom) > 1.0e-6 ||
          candidate_burst != base_burst;

      if (action_c_optimizer_direction_index_ == 0)
        {
          if (!action_c_optimizer_sweep_improved_)
            {
              action_c_optimizer_thrust_step_n_ = std::max(
                  0.125, 0.5 * action_c_optimizer_thrust_step_n_);
              action_c_optimizer_bottom_step_kpa_ = std::max(
                  1.25, 0.5 * action_c_optimizer_bottom_step_kpa_);
            }
          action_c_optimizer_sweep_improved_ = false;
        }
    }

  action_c_thrust_n_ = candidate_found ? candidate_thrust : base_thrust;
  action_c_bottom_hold_kpa_ = candidate_found ? candidate_bottom : base_bottom;
  action_c_optimizer_burst_pulses_ = candidate_found
      ? candidate_burst : base_burst;
  ROS_INFO("Action C optimizer NEXT: T=%.2f N, bottom=%.1f kPa, burst=%d; best J=%.6f at [%.2f N, %.1f kPa, %d]",
           action_c_thrust_n_, action_c_bottom_hold_kpa_,
           action_c_optimizer_burst_pulses_, action_c_optimizer_best_score_,
           base_thrust, base_bottom, base_burst);
}

void AttitudePressureController::finishActionCOptimizerTrial(
    bool recovery_required, const char* reason)
{
  if (!action_c_adaptive_burst_enabled_ ||
      !action_c_optimizer_trial_active_)
    return;
  const double raw_trial_dx =
      action_c_direction_sign_x_ *
          (mocap_position_[0] - action_c_optimizer_trial_start_x_m_);
  // Only motion generated between pulse ON and the completed regrip is gait
  // progress. PREPARE drift and attitude-recovery motion must not make an
  // otherwise failed candidate look good.
  const double recovery_dx = action_c_direction_sign_x_ *
      (mocap_position_[0] - action_c_optimizer_recovery_start_x_m_);
  const double retained_dx = action_c_efficiency_objective_enabled_
      ? std::min(raw_trial_dx, hugmy::actionCRetainedAfterRecovery(
          action_c_optimizer_retained_progress_m_,
          action_c_optimizer_recovery_pending_, recovery_dx))
      : action_c_optimizer_retained_progress_m_;
  const double progress_term =
      action_c_objective_progress_weight_ * retained_dx;
  const double tilt_cost = action_c_objective_tilt_weight_ *
      action_c_optimizer_peak_tilt_rad_ *
      action_c_optimizer_peak_tilt_rad_;
  const double rate_cost = action_c_objective_rate_weight_ *
      action_c_optimizer_peak_rate_rad_s_ *
      action_c_optimizer_peak_rate_rad_s_;
  const double thrust_cost = action_c_objective_thrust_energy_weight_ *
      action_c_optimizer_thrust_energy_n2s_;
  const double recovery_cost = recovery_required
      ? action_c_objective_recovery_weight_ : 0.0;
  const double angle_mismatch_cost =
      action_c_objective_angle_mismatch_weight_ *
      action_c_optimizer_peak_extension_mismatch_rad_ *
      action_c_optimizer_peak_extension_mismatch_rad_;
  const double failed_pulse_cost =
      action_c_objective_failed_pulse_weight_ *
      static_cast<double>(action_c_optimizer_failed_pulses_);
  const double legacy_score = progress_term - tilt_cost - rate_cost -
      thrust_cost - recovery_cost - angle_mismatch_cost - failed_pulse_cost;
  const double shortfall_ratio = action_c_optimizer_target_progress_m_ > 0.0
      ? std::max(0.0, 1.0 - retained_dx / action_c_optimizer_target_progress_m_) : 1.0;
  const double shortfall_cost = action_c_efficiency_shortfall_weight_ * shortfall_ratio;
  // Stability is enforced by the unchanged guards/supervisor. The online
  // objective rewards retained speed, not tiny motion with zero recoveries.
  const double score = action_c_efficiency_objective_enabled_
      ? (action_c_optimizer_clock_valid_ ? hugmy::actionCEfficiencyScore(
          retained_dx, action_c_optimizer_elapsed_sec_,
          action_c_optimizer_thrust_energy_n2s_, action_c_efficiency_thrust_weight_,
          action_c_optimizer_recovery_sec_, action_c_efficiency_recovery_time_weight_) - shortfall_cost
         : -std::numeric_limits<double>::infinity())
      : legacy_score;

  const bool eligible = action_c_optimizer_valid_pulses_ > 0 &&
      action_c_optimizer_successful_pulses_ > 0 &&
      retained_dx > 0.0 && std::isfinite(score) &&
      !action_c_optimizer_trial_had_extension_guard_;
  const bool improved = eligible &&
      score > action_c_optimizer_best_score_;
  pnh_.setParam("action_c_optimizer_last_score", score);
  pnh_.setParam("action_c_optimizer_target_progress_m", action_c_optimizer_target_progress_m_);
  pnh_.setParam("action_c_optimizer_shortfall_cost", shortfall_cost);
  ROS_INFO("Action C progress objective: goal %.2f mm, retained %.2f mm, deficit %.1f%%, shortfall cost %.6f; established successes %d/%d",
           1000.0 * action_c_optimizer_target_progress_m_, 1000.0 * retained_dx,
           100.0 * shortfall_ratio, shortfall_cost,
           action_c_optimizer_successful_pulses_, action_c_optimizer_completed_pulses_);
  pnh_.setParam("action_c_optimizer_elapsed_sec", action_c_optimizer_elapsed_sec_);
  pnh_.setParam("action_c_optimizer_recovery_sec", action_c_optimizer_recovery_sec_);
  ROS_INFO("Action C efficiency: retained %.2f mm / total %.3f s (recovery %.3f s), objective=%s J=%.6f; legacy diagnostic J=%.6f",
           1000.0*retained_dx, action_c_optimizer_elapsed_sec_, action_c_optimizer_recovery_sec_,
           action_c_efficiency_objective_enabled_ ? "retained-speed" : "legacy", score, legacy_score);
  pnh_.setParam("action_c_optimizer_last_retained_dx_m", retained_dx);
  pnh_.setParam("action_c_optimizer_last_raw_trial_dx_m", raw_trial_dx);
  pnh_.setParam("action_c_optimizer_last_peak_tilt_rad",
                action_c_optimizer_peak_tilt_rad_);
  pnh_.setParam("action_c_optimizer_last_peak_rate_rad_s",
                action_c_optimizer_peak_rate_rad_s_);
  pnh_.setParam("action_c_optimizer_last_thrust_energy_n2s",
                action_c_optimizer_thrust_energy_n2s_);
  pnh_.setParam("action_c_optimizer_last_recovery_required",
                recovery_required);
  pnh_.setParam("action_c_optimizer_last_peak_extension_mismatch_rad",
                action_c_optimizer_peak_extension_mismatch_rad_);
  pnh_.setParam("action_c_optimizer_last_failed_pulses",
                action_c_optimizer_failed_pulses_);
  pnh_.setParam("action_c_optimizer_last_valid_pulses",
                action_c_optimizer_valid_pulses_);
  ROS_WARN("Action C optimizer trial %d RESULT %s (%s): u=[T %.2f N, bottom %.1f kPa, burst %d], credited retained dx=%.2f mm (raw trial %.2f mm), success/fail=%d/%d, peak roll=%.1f deg, peak roll rate=%.3f rad/s, peak extension mismatch=%.1f deg, integral T^2 dt=%.3f N^2s, recovery=%d, legacy_terms=[+%.6f -%.6f -%.6f -%.6f -%.6f -%.6f -%.6f], selected J=%.6f%s%s",
           action_c_optimizer_trial_index_,
           action_c_direction_sign_x_ > 0.0 ? "+x" : "-x", reason,
           action_c_thrust_n_, action_c_bottom_hold_kpa_,
           action_c_optimizer_burst_pulses_, 1000.0 * retained_dx,
           1000.0 * raw_trial_dx,
           action_c_optimizer_successful_pulses_,
           action_c_optimizer_failed_pulses_,
           action_c_optimizer_peak_tilt_rad_ * 180.0 / M_PI,
           action_c_optimizer_peak_rate_rad_s_,
           action_c_optimizer_peak_extension_mismatch_rad_ * 180.0 / M_PI,
           action_c_optimizer_thrust_energy_n2s_, recovery_required,
           progress_term, tilt_cost, rate_cost, thrust_cost, recovery_cost,
           angle_mismatch_cost, failed_pulse_cost,
           score, improved ? " NEW_BEST" : "",
           eligible ? "" : " INELIGIBLE");
  if (improved)
    {
      action_c_optimizer_best_score_ = score;
      action_c_optimizer_best_thrust_n_ = action_c_thrust_n_;
      action_c_optimizer_best_bottom_kpa_ = action_c_bottom_hold_kpa_;
      action_c_optimizer_best_burst_pulses_ =
          action_c_optimizer_burst_pulses_;
      action_c_optimizer_sweep_improved_ = true;
      pnh_.setParam("action_c_optimizer_best_score",
                    action_c_optimizer_best_score_);
      pnh_.setParam("action_c_optimizer_best_thrust_n",
                    action_c_optimizer_best_thrust_n_);
      pnh_.setParam("action_c_optimizer_best_bottom_kpa",
                    action_c_optimizer_best_bottom_kpa_);
      pnh_.setParam("action_c_optimizer_best_burst_pulses",
                    action_c_optimizer_best_burst_pulses_);
    }
  const bool reprobe_qualified = !recovery_required &&
      !action_c_optimizer_trial_had_extension_guard_ &&
      action_c_optimizer_completed_pulses_ > 0 &&
      action_c_optimizer_valid_pulses_ == action_c_optimizer_completed_pulses_ &&
      std::isfinite(retained_dx) && retained_dx >= 0.0 &&
      action_c_optimizer_peak_extension_mismatch_rad_ <=
          action_c_thrust_reprobe_max_mismatch_rad_ &&
      action_c_optimizer_peak_tilt_rad_ < action_c_posture_recovery_trigger_rad_ &&
      action_c_optimizer_peak_rate_rad_s_ < action_c_posture_rate_trigger_rad_s_;
  action_c_optimizer_reprobe_pending_ = action_c_thrust_safety_.observeTrial(
      action_c_thrust_n_, action_c_min_thrust_n_, action_c_thrust_reprobe_step_n_,
      action_c_optimizer_trial_guard_reasons_,
      reprobe_qualified ? action_c_optimizer_valid_pulses_ : 0,
      action_c_thrust_reprobe_safe_pulses_);
  action_c_optimizer_safe_thrust_max_n_ = action_c_thrust_safety_.limit();
  pnh_.setParam("action_c_optimizer_safe_thrust_max_n",
                action_c_optimizer_safe_thrust_max_n_);
  pnh_.setParam("action_c_optimizer_hard_thrust_max_n",
                action_c_thrust_safety_.hard_max_n);
  pnh_.setParam("action_c_optimizer_pair_thrust_max_n",
                action_c_thrust_safety_.pair_max_n);
  pnh_.setParam("action_c_optimizer_reprobe_safe_pulses",
                action_c_thrust_safety_.safe_pulses);
  pnh_.setParam("action_c_optimizer_last_guard_reasons",
                static_cast<int>(action_c_optimizer_trial_guard_reasons_));
  ROS_INFO("Action C optimizer LIMITS: effective %.2f N, hard %.2f N, pair %.2f N; verified pulses %d/%d, eligible=%d, guard mask=%u%s",
           action_c_optimizer_safe_thrust_max_n_, action_c_thrust_safety_.hard_max_n,
           action_c_thrust_safety_.pair_max_n, action_c_thrust_safety_.safe_pulses,
           action_c_thrust_reprobe_safe_pulses_, reprobe_qualified,
           action_c_optimizer_trial_guard_reasons_,
           action_c_optimizer_reprobe_pending_ ? "; re-probe unlocked" : "");
  action_c_optimizer_trial_active_ = false;
  action_c_optimizer_recovery_pending_ = false;
  action_c_optimizer_trial_required_recovery_ = recovery_required;
  if (finishActionCPropulsionTrial(recovery_required, retained_dx)) return;
  if (action_c_coordination_enabled_ && action_c_coordination_.finishTrial(
          eligible && !recovery_required && !action_c_optimizer_trial_had_extension_guard_ &&
          action_c_optimizer_clock_valid_ && action_c_optimizer_valid_pulses_ > 0 &&
          action_c_optimizer_successful_pulses_ == action_c_optimizer_completed_pulses_))
    {
      // Calibration and optimizer NEVER change inputs at the same boundary.
      // Re-measure the same T/P, one pulse, under the new fixed calibration.
      action_c_optimizer_best_score_ = -std::numeric_limits<double>::infinity();
      action_c_optimizer_direction_index_ = 0;
      action_c_optimizer_sweep_improved_ = false;
      action_c_optimizer_reprobe_pending_ = false;
      action_c_optimizer_burst_pulses_ = 1;
      pnh_.setParam("action_c_optimizer_best_score", action_c_optimizer_best_score_);
      pnh_.setParam("action_c_coordination_learning_round", action_c_coordination_.rounds);
      ROS_INFO("Action C COORDINATION commit %d/%d: offsets [%.3f %.3f %.3f %.3f] N, leads [%.3f %.3f %.3f %.3f] s; same T/P re-evaluation, optimizer held for this boundary",
               action_c_coordination_.rounds, action_c_coordination_.learning_rounds,
               action_c_coordination_.offset[0], action_c_coordination_.offset[1],
               action_c_coordination_.offset[2], action_c_coordination_.offset[3],
               action_c_coordination_.lead[0], action_c_coordination_.lead[1],
               action_c_coordination_.lead[2], action_c_coordination_.lead[3]);
      return;
    }
  selectNextActionCCandidate();
}

void AttitudePressureController::requestActionCPostureRecovery(
    const ros::Time& now, const char* reason)
{
  if (!action_c_adaptive_burst_enabled_ || !target_received_)
    {
      stopInchwormToGrasp(now,
          target_received_ ? reason :
              "Action C cannot invoke attitude recovery without a target attitude");
      return;
    }
  if (action_c_optimizer_trial_active_)
    {
      action_c_optimizer_recovery_pending_ = true;
      action_c_optimizer_recovery_start_x_m_ = mocap_position_[0];
    }
  if (action_c_posture_recovery_count_ >=
      action_c_max_posture_recoveries_)
    {
      finishActionCOptimizerTrial(true, "posture recovery limit");
      stopInchwormToGrasp(now,
          "Action C reached the posture-recovery limit");
      return;
    }

  ++action_c_posture_recovery_count_;
  continuous_recovery_started_ = false;
  continuous_recovery_waiting_for_settle_ = false;
  action_c_recovery_gate_.reset();
  action_c_resume_pending_ = true;
  action_c_pressure_pull_active_ = false;
  action_c_bottom_timing_.filling = false;
  action_c_bottom_timing_.peak_at = -1.;
  action_c_resume_direction_ = inchworm_direction_body_;
  action_c_posture_ready_latched_ = false;
  action_c_burst_pulse_count_ = 0;
  action_c_command_thrust_n_ = 0.0;
  inchworm_thrust_.fill(0.0);
  inchworm_pressure_.fill(base_pressure_kpa_);
  publishCorrectionThrust(inchworm_thrust_);
  publishPressureTarget(inchworm_pressure_,
                        std::numeric_limits<double>::infinity());
  // The bottom bladder makes the correction less stable, so exhaust it while
  // the proven shared attitude-recovery sequence owns the robot.
  publishBottomTarget(0.0);
  inchworm_active_ = false;
  enterState(GRASP, now);
  ROS_WARN("Action C paused for shared attitude recovery (%d/%d): %s; saved goal x %.4f m, current x %.4f m, next pulse %.2f N / bottom %.1f kPa",
           action_c_posture_recovery_count_,
           action_c_max_posture_recoveries_, reason,
           action_c_absolute_target_x_m_, mocap_position_[0],
           action_c_thrust_n_, action_c_bottom_hold_kpa_);
}

bool AttitudePressureController::resumeActionCAfterRecovery(
    const ros::Time& now)
{
  if (!action_c_resume_pending_) return false;
  if (!mocap_received_ || (now - mocap_stamp_).toSec() < 0.0 ||
      (now - mocap_stamp_).toSec() > inchworm_mocap_timeout_sec_)
    {
      ROS_WARN_THROTTLE(1.0, "Action C auto-resume waiting for fresh mocap; saved goal retained");
      return false;
    }
  continuous_recovery_waiting_for_settle_ = false;
  continuous_recovery_blocked_ = false;
  if (action_c_optimizer_recovery_pending_)
    finishActionCOptimizerTrial(true, "posture recovery");
  const double remaining = action_c_direction_sign_x_ *
      (action_c_absolute_target_x_m_ - mocap_position_[0]);
  if (remaining <= inchworm_min_cycle_progress_m_)
    {
      action_c_resume_pending_ = false;
      ROS_INFO("Action C %s goal was reached during attitude recovery: x %.4f/%.4f m",
               action_c_direction_sign_x_ > 0.0 ? "+x" : "-x",
               mocap_position_[0], action_c_absolute_target_x_m_);
      return true;
    }

  action_c_resume_pending_ = false;
  beginActionCOptimizerTrial(now);
  action_c_preserve_adaptation_on_start_ = true;
  startInchwormStep(action_c_resume_direction_, remaining, now);
  action_c_preserve_adaptation_on_start_ = false;
  // The shared recovery has already certified the wider resume band. Do not
  // immediately demand the old +/-2-deg manual gate on the next tick.
  action_c_posture_ready_latched_ = true;
  ROS_INFO("Action C resumed after attitude recovery: remaining %.2f mm, pulse %.2f N, bottom hold %.1f kPa",
           1000.0 * remaining, action_c_thrust_n_,
           action_c_bottom_hold_kpa_);
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

void AttitudePressureController::updateActionCPressurePull(const ros::Time& now)
{
  const double elapsed = (now - state_start_).toSec();
  const bool powered_regrip = action_c_propulsion_enabled_ && action_c_propulsion_.regrip_start>=0.;
  const double pressure_age=(now-measured_pressure_stamp_).toSec();
  if (action_c_propulsion_enabled_ && (measured_pressure_stamp_.isZero() || pressure_age<0. || pressure_age>.25 ||
      !std::all_of(measured_pressure_.begin(), measured_pressure_.end(), [](double p){return std::isfinite(p);})))
    {
      action_c_pressure_pull_active_=false;
      stopInchwormToGrasp(now, "stale or invalid pressure during powered pull/regrip");
      return;
    }
  if (!powered_regrip && !actionCRearReleased(now))
    {
      action_c_pressure_pull_active_ = false;
      stopInchwormToGrasp(now, "Action C pressure pull lost rear release");
      return;
    }
  // Ordinary roll assessment belongs after recoil/regrip. Hard posture and
  // impending bend-limit handoffs below may interrupt powered flexion.
  if (std::abs(body_angular_velocity_[0]) >= action_c_posture_rate_trigger_rad_s_)
    {
      action_c_pressure_pull_active_ = false;
      inchworm_thrust_.fill(0.0);
      action_c_command_thrust_n_ = 0.0;
      publishCorrectionThrust(inchworm_thrust_);
      requestActionCPostureRecovery(now, "roll limit during pressure-driven pull");
      return;
    }
  bool front_ready = true, both_rebent = true, both_reached_extension = true;
  unsigned int guards = 0;
  double predicted_bend_margin = std::numeric_limits<double>::infinity();
  double min_extension = std::numeric_limits<double>::infinity();
  double max_extension = -std::numeric_limits<double>::infinity();
  std_msgs::Float32MultiArray diagnostic;
  diagnostic.data.resize(ARM_COUNT, std::numeric_limits<float>::quiet_NaN());
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      inchworm_pressure_[arm] = inchworm_front_[arm] ? action_c_front_pressure_[arm] :
          (powered_regrip ? base_pressure_kpa_ : 0.0);
      if (!inchworm_front_[arm]) { inchworm_thrust_[arm] = 0.0; continue; }
      auto& observer = action_c_pulse_observer_[arm];
      observer.update(hugmy::relativeBendRate(neuron_gyro_[arm][1], body_angular_velocity_[0],
          body_angular_velocity_[1], arm_yaw_[arm]), neuron_stamp_[arm].toSec());
      const bool simulation_bend = std::isfinite(grip_joint_bend_rad_[arm]) &&
          !grip_state_stamp_.isZero() && (now - grip_state_stamp_).toSec() <= 0.25;
      const double bend = simulation_bend ? measuredArmBendAngle(arm) : observer.bend;
      if ((!simulation_bend && !observer.valid) || !std::isfinite(bend))
        guards |= hugmy::ACTION_C_ANGLE_UNRELIABLE;
      const double extension = action_c_pulse_start_angle_rad_[arm] - bend;
      const double floor = std::max(hugmy::actionCPulseBendFloor(
          action_c_pulse_start_angle_rad_[arm], action_c_min_safe_angle_rad_,
          action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
          action_c_start_extension_margin_rad_),
          action_c_pulse_start_angle_rad_[arm] - action_c_max_extension_rad_);
      const double predicted_margin = bend - floor + std::min(0.0, observer.rate) *
          action_c_pulse_roll_handoff_lookahead_sec_;
      if (std::isfinite(predicted_margin))
        predicted_bend_margin = std::min(predicted_bend_margin, predicted_margin);
      min_extension = std::min(min_extension, extension);
      max_extension = std::max(max_extension, extension);
      if (extension >= action_c_max_extension_rad_) guards |= hugmy::ACTION_C_OVER_EXTENSION;
      if (bend <= hugmy::actionCPulseBendFloor(action_c_pulse_start_angle_rad_[arm],
          action_c_min_safe_angle_rad_, action_c_hard_min_bend_rad_,
          action_c_extension_target_rad_, action_c_start_extension_margin_rad_))
        guards |= hugmy::ACTION_C_LOW_BEND;
      if (observer.rate <= -action_c_extension_rate_limit_rad_s_) guards |= hugmy::ACTION_C_EXTENSION_RATE;
      action_c_front_angle_rad_[arm] = bend;
      action_c_pulse_min_angle_rad_[arm] = std::min(action_c_pulse_min_angle_rad_[arm], bend);
      both_reached_extension = both_reached_extension &&
          action_c_pulse_start_angle_rad_[arm]-action_c_pulse_min_angle_rad_[arm]>=action_c_extension_target_rad_;
      const double rebound = bend - action_c_pulse_min_angle_rad_[arm];
      both_rebent = both_rebent && rebound >= action_c_rebend_target_rad_;
      front_ready = front_ready && std::isfinite(measured_pressure_[arm]) &&
          measured_pressure_[arm] >= inchworm_pressure_[arm] - pressure_tolerance_kpa_;
      inchworm_thrust_[arm] = action_c_propulsion_enabled_
          ? std::max(0.0, std::min(inchworm_thrust_[arm], action_c_pressure_pull_ceiling_n_[arm]))
          : hugmy::actionCPressurePullThrust(inchworm_thrust_[arm],
              action_c_pressure_pull_ceiling_n_[arm], rebound, action_c_rebend_target_rad_, true);
      diagnostic.data[arm] = bend * 180.0 / M_PI;
    }
  action_c_pulse_bend_pub_.publish(diagnostic);
  action_c_pulse_had_extension_ = action_c_pulse_had_extension_ || both_reached_extension;
  // During intentional asynchronous flexion, compare attained extensions,
  // not one arm's rebound against the other's extension. Individual hard
  // bend/rate limits and the measured roll supervisor are unchanged.
  const double mismatch = action_c_propulsion_enabled_
      ? hugmy::actionCPeakExtensionMismatch(inchworm_front_, action_c_pulse_start_angle_rad_, action_c_pulse_min_angle_rad_)
      : max_extension - min_extension;
  action_c_optimizer_peak_extension_mismatch_rad_ = std::max(action_c_optimizer_peak_extension_mismatch_rad_, mismatch);
  if (max_extension >= action_c_extension_target_rad_ && mismatch >= action_c_max_extension_mismatch_rad_)
    guards |= hugmy::ACTION_C_PAIR_MISMATCH;
  if (inchwormTravel() - action_c_pulse_start_travel_m_ <= -action_c_abort_reverse_travel_m_)
    guards |= hugmy::ACTION_C_REVERSE_TRAVEL;
  if (std::hypot(wrapAngle(roll_ - action_c_pulse_start_roll_),
                 wrapAngle(pitch_ - action_c_pulse_start_pitch_)) >= action_c_max_body_tilt_change_rad_)
    guards |= hugmy::ACTION_C_BODY_TILT;
  if (action_c_pulse_roll_handoff_enabled_ &&
      !(guards & hugmy::ACTION_C_ANGLE_UNRELIABLE) &&
      hugmy::actionCPulseRollHandoff(wrapAngle(roll_ - target_roll_),
          wrapAngle(action_c_pulse_start_roll_ - target_roll_), body_angular_velocity_[0],
          predicted_bend_margin, action_c_pulse_roll_handoff_margin_rad_,
          action_c_posture_recovery_trigger_rad_, action_c_posture_hard_roll_rad_,
          action_c_posture_rate_trigger_rad_s_, action_c_roll_trend_.growth_rad,
          action_c_roll_trend_.outward_rate_rad_s))
    {
      action_c_pulse_overextended_ = true;
      action_c_optimizer_trial_had_extension_guard_ |= guards != 0;
      action_c_optimizer_trial_guard_reasons_ |= guards;
      action_c_pressure_pull_active_ = false;
      requestActionCPostureRecovery(now, "roll/bend limit during pressure-driven pull");
      return;
    }
  if (guards)
    {
      action_c_pulse_overextended_ = true;
      action_c_optimizer_trial_had_extension_guard_ = true;
      action_c_optimizer_trial_guard_reasons_ |= guards;
      action_c_pressure_pull_active_ = false;
      ROS_WARN("Action C PRESSURE PULL guard mask=%u", guards);
      stopInchwormToGrasp(now, "safety guard during pressure-driven pull");
      return;
    }
  observeActionCPropulsion(now);
  const bool timed_out = elapsed >= action_c_pressure_pull_timeout_sec_;
  bool powered_complete=false;
  if (action_c_propulsion_enabled_)
    {
      if (!powered_regrip && front_ready && both_rebent && elapsed>=.1)
        {
          action_c_propulsion_.regrip_start=now.toSec();
          for (size_t arm=0;arm<ARM_COUNT;++arm)
            if (!inchworm_front_[arm]) inchworm_pressure_[arm]=base_pressure_kpa_;
          ROS_INFO("Action C POWERED REGRIP start: both arms flexed; retain bounded front thrust while rear pressure returns");
        }
      else if (!powered_regrip && timed_out)
        {
          action_c_pressure_pull_active_=false;
          stopInchwormToGrasp(now, "powered flexion timeout: pressure or two-arm rebound incomplete");
          return;
        }
      if (powered_regrip)
        {
          bool ready=front_ready && both_rebent;
          for(size_t arm=0;arm<ARM_COUNT;++arm) if(!inchworm_front_[arm])
            ready=ready && std::abs(measured_pressure_[arm]-base_pressure_kpa_)<=pressure_tolerance_kpa_;
          powered_complete=action_c_propulsion_.regripReady(now.toSec(),ready);
          if (!powered_complete && now.toSec()-action_c_propulsion_.regrip_start>=action_c_powered_regrip_timeout_sec_)
            {
              action_c_pressure_pull_active_=false;
              stopInchwormToGrasp(now, "powered rear regrip timed out");
              return;
            }
        }
    }
  if (!action_c_propulsion_enabled_ && timed_out && !front_ready)
    {
      action_c_pressure_pull_active_ = false;
      stopInchwormToGrasp(now, "front pressure did not recover during bounded pressure pull");
      return;
    }
  if (powered_complete || (!action_c_propulsion_enabled_ && front_ready && ((both_rebent && elapsed >= 0.1) || timed_out)))
    {
      action_c_propulsion_.complete(both_rebent && action_c_pulse_had_extension_ &&
          (action_c_propulsion_enabled_ ? powered_complete && action_c_propulsion_.both_powered_sec>=.1 : !timed_out));
      action_c_pressure_pull_active_ = false;
      inchworm_thrust_.fill(0.0);
      inchworm_pressure_.fill(base_pressure_kpa_);
      action_c_recovery_ready_start_ = ros::Time(0);
      // Start the full OFF/regrip timer AFTER powered pull, not at ON end.
      enterState(ACTION_C_EVALUATE, now);
      ROS_INFO("Action C PRESSURE PULL done: rebound=%d, duration=%.2f s; powered rear regrip=%d, joint forward-thrust time=%.2f s",
          both_rebent, elapsed, powered_complete, action_c_propulsion_.both_powered_sec);
    }
  action_c_command_thrust_n_ = *std::max_element(inchworm_thrust_.begin(), inchworm_thrust_.end());
  observeActionCPropulsion(now);
  publishBottomTarget(actionCBottomTarget());
  publishPressureTarget(inchworm_pressure_, std::numeric_limits<double>::infinity());
  publishCorrectionThrust(inchworm_thrust_);
  ROS_INFO_THROTTLE(0.1, "Action C PRESSURE PULL: residual [%.2f %.2f %.2f %.2f] N, front pressure ready=%d, rebent=%d, rear release enter/hold/hard %.1f/%.1f/%.1f kPa, grace %.2f s, elapsed %.2f/%.2f s",
      inchworm_thrust_[0], inchworm_thrust_[1], inchworm_thrust_[2], inchworm_thrust_[3],
      front_ready, both_rebent, action_c_rear_release_max_kpa_, action_c_rear_release_hold_kpa_,
      action_c_rear_release_hard_kpa_, action_c_rear_release_grace_sec_,
      elapsed, action_c_pressure_pull_timeout_sec_);
}

double AttitudePressureController::actionCBottomTarget() const
{
  if (action_c_bottom_sync_enabled_ &&
      (state_ == SETTLE || (state_ == ACTION_C_PREPARE && !action_c_bottom_timing_.filling)))
    return std::min(action_c_bottom_hold_kpa_, action_c_bottom_wait_kpa_);
  return std::max(0.0, action_c_bottom_hold_kpa_ -
      (action_c_bottom_post_pulse_ ? action_c_bottom_post_pulse_drop_kpa_ : 0.0));
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
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (state_ == ACTION_C_PREPARE && action_c_front_release_enabled_ &&
        !action_c_reference_pulse_mode_ && inchworm_front_[arm] && std::abs(target[arm]) <= 1e-6)
      tolerance[arm] = action_c_front_release_tolerance_kpa_;
  const double age = (now - measured_pressure_stamp_).toSec();
  return action_c_pressure_ready_gate_.update(measured_pressure_, target, tolerance,
      now.toSec(), !measured_pressure_stamp_.isZero() && age >= 0.0 && age <= 0.25,
      action_c_pressure_hold_tolerance_kpa_, action_c_pressure_dropout_sec_,
      action_c_pressure_max_deviation_kpa_);
}

double AttitudePressureController::actionCBottomPeakLead() const
{
  // Nominal peak timing only: an early extension/safety cutoff always wins.
  const double level = std::max(0.0, std::min(action_c_thrust_n_, action_c_max_thrust_n_));
  return std::min(action_c_pulse_duration_sec_, level / std::max(0.01, action_c_thrust_ramp_n_s_));
}

bool AttitudePressureController::pressureReached(
    const PressureArray& target) const
{
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      // Only the released front pair in C PREPARE gets the exhaust tolerance.
      // Keep grasp/regrip, rear supports and other actions at their normal tolerance.
      const double tolerance = state_ == ACTION_C_PREPARE &&
          action_c_front_release_enabled_ && !action_c_reference_pulse_mode_ &&
          inchworm_front_[arm] && std::abs(target[arm]) <= 1.0e-6
          ? action_c_front_release_tolerance_kpa_ : pressure_tolerance_kpa_;
      if (!std::isfinite(target[arm]) || !std::isfinite(measured_pressure_[arm]) ||
          std::abs(measured_pressure_[arm] - target[arm]) > tolerance)
        return false;
    }
  return true;
}

double AttitudePressureController::inchwormTravel() const
{
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
    {
      // Action C is longitudinal on the real fixture: arms 0/3 face +x and
      // arms 1/2 face -x. Apply the requested direction sign directly to raw
      // mocap x; Spinal yaw has an unrelated marker mounting offset.
      return action_c_direction_sign_x_ *
          (mocap_position_[0] - inchworm_start_position_[0]);
    }
  if (inchworm_action_d_enabled_)
    {
      // Action D is evaluated directly in the OptiTrack/mocap XY frame. The
      // real-machine marker yaw contains an approximately 180-deg mounting
      // offset (about 161 deg in d_11), so rotating the +x command by that yaw
      // incorrectly reported the clearly positive mocap-x displacement as
      // backward travel. For the usual {+1, 0} command this is exactly the
      // raw positive mocap-x displacement requested by the operator.
      return (mocap_position_[0] - inchworm_start_position_[0]) *
                 inchworm_direction_body_[0] +
             (mocap_position_[1] - inchworm_start_position_[1]) *
                 inchworm_direction_body_[1];
    }
  return (mocap_position_[0] - inchworm_start_position_[0]) *
             inchworm_direction_world_[0] +
         (mocap_position_[1] - inchworm_start_position_[1]) *
             inchworm_direction_world_[1];
}

void AttitudePressureController::updateInchworm(
    const ros::Time& now, double dt, bool, bool)
{
  auto approach = [dt](double value, double target, double rate) {
    const double step = std::max(0.0, rate * dt);
    return value < target ? std::min(target, value + step)
                          : std::max(target, value - step);
  };
  if (action_c_coordination_enabled_ && inchworm_action_c_enabled_ &&
      !inchworm_action_d_enabled_ && action_c_pulse_started_)
    action_c_coordination_.observePressure(now.toSec(), measured_pressure_,
        !measured_pressure_stamp_.isZero() &&
        (now - measured_pressure_stamp_).toSec() >= 0.0 &&
        (now - measured_pressure_stamp_).toSec() <= 0.25);
  // Continue identification through zero-thrust re-inflation even after a
  // guarded pulse. Never borrow samples from the shared recovery controller.
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ && action_c_pulse_started_ &&
      !action_c_pressure_pull_active_ &&
      (state_==ACTION_C_EVALUATE || state_==ACTION_C_REGRIP || state_==SETTLE))
    observeActionCPropulsion(now);
  if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
    {
      // Bottom pressure is an optimized Action-C input and intentionally
      // changes pitch. Posture handoff and its optimizer terms therefore use
      // only Spinal roll and roll rate. Pitch still remains covered by the
      // per-pulse relative-tilt safety guard below.
      const double tilt_error =
          std::abs(wrapAngle(roll_ - target_roll_));
      const double body_rate = std::abs(body_angular_velocity_[0]);
      action_c_cycle_peak_tilt_rad_ = std::max(
          action_c_cycle_peak_tilt_rad_, tilt_error);
      action_c_cycle_peak_rate_rad_s_ = std::max(
          action_c_cycle_peak_rate_rad_s_, body_rate);
      if (action_c_optimizer_trial_active_)
        {
          action_c_optimizer_peak_tilt_rad_ = std::max(
              action_c_optimizer_peak_tilt_rad_, tilt_error);
          action_c_optimizer_peak_rate_rad_s_ = std::max(
              action_c_optimizer_peak_rate_rad_s_, body_rate);
          double thrust_squared = 0.0;
          for (double thrust : inchworm_thrust_)
            thrust_squared += thrust * thrust;
          action_c_optimizer_thrust_energy_n2s_ +=
              thrust_squared * std::max(0.0, dt);
        }

      // A pulse is allowed to finish recoil and regrip. Only count residual
      // roll at a quiet PREPARE/SETTLE boundary, with the selected bottom
      // pressure already reached. Never carry pulse/inflation excursions
      // into the ordinary recovery confirmation timer.
      const bool posture_boundary = state_ == ACTION_C_PREPARE || state_ == SETTLE;
      const bool pneumatic_settled = posture_boundary && actionCPressureReady(inchworm_pressure_, now) &&
          std::isfinite(bottom_pressure_kpa_) &&
          std::abs(bottom_pressure_kpa_ - actionCBottomTarget()) <=
              pressure_tolerance_kpa_;
      const bool boundary_quiet = posture_boundary && pneumatic_settled &&
          action_c_command_thrust_n_ <= 0.01 &&
          std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                      [](double thrust) { return std::abs(thrust) <= 0.01; }) &&
          body_rate <= action_c_pre_roll_rate_tolerance_rad_s_;
      action_c_roll_trend_.update(boundary_quiet, wrapAngle(roll_ - target_roll_),
          body_angular_velocity_[0], now.toSec());
      const bool hard_roll = tilt_error >= action_c_posture_hard_roll_rad_;
      const bool boundary_settled = action_c_boundary_settle_gate_.update(boundary_quiet,
          now.toSec(), action_c_posture_settle_sec_);
      const bool posture_confirmed = action_c_recovery_gate_.update(
          boundary_settled && tilt_error > action_c_posture_recovery_trigger_rad_ &&
          action_c_roll_trend_.worsening,
          now.toSec(), correction_trigger_confirm_sec_);
      action_c_boundary_ready_ = boundary_settled && !hard_roll &&
          (tilt_error <= action_c_posture_recovery_trigger_rad_ ||
           (action_c_roll_trend_.ready && !action_c_roll_trend_.worsening));
      // The low-pressure quiet boundary was certified BEFORE fill began.
      // Do not demand high-pressure settling again during just-in-time fill.
      // Absolute roll/rate guards below remain active in either direction.
      if (action_c_bottom_sync_enabled_ && state_ == ACTION_C_PREPARE &&
          action_c_bottom_timing_.filling)
        action_c_boundary_ready_ = !hard_roll &&
            body_rate <= action_c_pre_roll_rate_tolerance_rad_s_;
      if (posture_boundary)
        ROS_INFO_THROTTLE(0.5,
            "Action C roll boundary: error %.1f deg, trend ready=%d worsening=%d, hard limit %.1f deg, next pulse ready=%d; bottom %.1f/%.1f kPa",
            tilt_error * 180.0 / M_PI, action_c_roll_trend_.ready, action_c_roll_trend_.worsening,
            action_c_posture_hard_roll_rad_ * 180.0 / M_PI, action_c_boundary_ready_,
            bottom_pressure_kpa_, actionCBottomTarget());
      const bool safe_to_handoff =
          (state_ != ACTION_C_HORIZONTAL_PULL || !action_c_pulse_started_) &&
          action_c_command_thrust_n_ <= 0.01;
      if (action_c_adaptive_burst_enabled_ && safe_to_handoff &&
          (posture_confirmed || hard_roll ||
           body_rate >= action_c_posture_rate_trigger_rad_s_))
        {
          if (action_c_reference_pulse_mode_ && action_c_pulse_started_ &&
              (action_c_reference_single_pulse_ || action_c_pulse_overextended_))
            {
              stopInchwormToGrasp(now, "reference single/guarded pulse ended with posture limit; no automatic pulse retry");
              return;
            }
          // The candidate remains unchanged until its objective is measured
          // after recovery. Mutating it here would mix two different u values
          // in one optimization trial.
          requestActionCPostureRecovery(now,
              body_rate >= action_c_posture_rate_trigger_rad_s_
                  ? "Spinal roll-rate limit (immediate)"
                  : (hard_roll ? "absolute Spinal roll safety limit"
                               : "worsening residual Spinal roll (angle growth and gyro confirmed)"));
          return;
        }
    }
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

    case ACTION_C_PREPARE:
      {
      publishBottomDirection(-1.0);
      publishBottomTarget(actionCBottomTarget());
      bool front_shape_ready = true;
      using Trend = hugmy::PressureThrustBendModel::Trend;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          inchworm_thrust_[arm] = 0.0;
          if (!inchworm_front_[arm])
            {
              inchworm_pressure_[arm] = base_pressure_kpa_;
              continue;
            }
          double best_pressure = 0.0;
          double best_error = std::numeric_limits<double>::infinity();
          const double action_c_pressure_limit = std::min(
              maximum_pressure_kpa_, action_c_max_pressure_kpa_);
          for (double pressure = 0.0;
               !action_c_reference_pulse_mode_ && !action_c_front_release_enabled_ &&
               pressure <= action_c_pressure_limit + 1.0e-9; pressure += 0.1)
            {
              const double predicted = bend_model_.angleRad(
                  pressure, action_c_thrust_n_ + bend_model_thrust_offset_n_,
                  Trend::INCREASING, Trend::INCREASING);
              const double error = std::abs(predicted - action_c_target_angle_rad_);
              if (error < best_error)
                {
                  best_error = error;
                  best_pressure = pressure;
                }
            }
          // The successful gait releases the front before extension. A static
          // 90-degree-at-thrust table counteracts the intended pulse extension.
          // Keep that table only for explicit legacy comparisons.
          double selected_pressure = action_c_reference_pulse_mode_
              ? action_c_front_pulse_pressure_kpa_
              : action_c_front_release_enabled_ ? 0.0
              : best_pressure + (action_c_online_pressure_adaptation_enabled_
                    ? action_c_pressure_bias_kpa_[arm] : 0.0);
          // Once one pulse has been observed, use the local measured model
          // to prepare the next operating point.  This compensates the
          // expected arm-angle change caused by a new thrust level before
          // that thrust is applied.  The table remains the safe initial
          // guess for the first pulse.
          if (!action_c_front_release_enabled_ && action_c_online_pressure_adaptation_enabled_ &&
              action_c_last_pulse_valid_ &&
              std::abs(action_c_dtheta_dpressure_[arm]) > 1.0e-4)
            {
              const double thrust_change = action_c_thrust_n_ -
                  action_c_last_pulse_thrust_n_;
              const double pressure_to_target =
                  (action_c_target_angle_rad_ -
                   action_c_last_pulse_bend_angle_rad_[arm]) /
                  action_c_dtheta_dpressure_[arm];
              const double pressure_for_thrust = -
                  action_c_dtheta_dthrust_[arm] /
                  action_c_dtheta_dpressure_[arm] * thrust_change;
              selected_pressure = action_c_last_pulse_pressure_kpa_[arm] +
                  std::max(-action_c_max_feedforward_pressure_step_kpa_,
                      std::min(action_c_max_feedforward_pressure_step_kpa_,
                          pressure_to_target + pressure_for_thrust));
            }
          action_c_front_pressure_[arm] = std::max(0.0, std::min(
              action_c_pressure_limit, selected_pressure));
          action_c_front_angle_rad_[arm] = measuredArmBendAngle(arm);
          // Legacy table mode corrects its preparation shape. Release mode
          // must not silently reinflate to 30+ kPa; the measured minimum-bend
          // gate below still prevents starting from an already-straight arm.
          if (!action_c_front_release_enabled_ && !action_c_reference_pulse_mode_ && action_c_front_angle_rad_[arm] <
              action_c_target_angle_rad_ - action_c_angle_tolerance_rad_)
            {
              const double angle_error = action_c_target_angle_rad_ -
                  action_c_front_angle_rad_[arm];
              action_c_front_pressure_[arm] = std::min(
                  action_c_pressure_limit,
                  std::max(base_pressure_kpa_,
                      action_c_front_pressure_[arm]) +
                      action_c_pressure_gain_kpa_rad_ * angle_error);
            }
          const bool arm_shape_ready = hugmy::actionCStartBendReady(
              action_c_front_angle_rad_[arm], action_c_prepare_min_bend_rad_,
              action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
              action_c_start_extension_margin_rad_);
          front_shape_ready = front_shape_ready && arm_shape_ready;
          if (!arm_shape_ready)
            ROS_WARN_THROTTLE(1.0, "Action C PREPARE bend blocked: arm%zu %.1f deg; start minimum %.1f deg, absolute floor %.1f deg + extension %.1f deg + margin %.1f deg",
                arm, action_c_front_angle_rad_[arm] * 180.0 / M_PI,
                action_c_prepare_min_bend_rad_ * 180.0 / M_PI,
                action_c_hard_min_bend_rad_ * 180.0 / M_PI,
                action_c_extension_target_rad_ * 180.0 / M_PI,
                action_c_start_extension_margin_rad_ * 180.0 / M_PI);
          if (action_c_reference_pulse_mode_ || action_c_pressure_pull_enabled_)
            {
              // Fail closed at unsafe starting bend; do not silently replace
              // the requested 5 kPa with 30+ kPa to chase an absolute 90 deg.
              const double gyro_norm = std::sqrt(
                  neuron_gyro_[arm][0]*neuron_gyro_[arm][0] +
                  neuron_gyro_[arm][1]*neuron_gyro_[arm][1] +
                  neuron_gyro_[arm][2]*neuron_gyro_[arm][2]);
              front_shape_ready = front_shape_ready &&
                  std::abs(neuron_acc_norm_[arm] - 9.80665) <= 1.5 && gyro_norm <= 0.20;
              if (!front_shape_ready)
                ROS_WARN_THROTTLE(1.0, "Action C waiting for safe settled bend/IMU: arm%zu bend %.1f deg, acc %.2f m/s^2, gyro %.2f rad/s; front target stays %.1f kPa",
                    arm, action_c_front_angle_rad_[arm]*180.0/M_PI,
                    neuron_acc_norm_[arm], gyro_norm, action_c_front_pressure_[arm]);
            }
          inchworm_pressure_[arm] = action_c_front_pressure_[arm];
        }
      const bool nominal_arm_pressure_ready = actionCPressureReady(inchworm_pressure_, now);
      const bool bottom_ready = std::isfinite(bottom_pressure_kpa_) &&
          (!action_c_bottom_sync_enabled_ || action_c_bottom_timing_.fresh(now.toSec())) &&
          ((action_c_bottom_sync_enabled_ && action_c_bottom_timing_.filling) ||
           std::abs(bottom_pressure_kpa_ - actionCBottomTarget()) <= pressure_tolerance_kpa_);
      // Requiring nominal pressure before the first correction is safe, but
      // do not turn the correction off merely because its intentional
      // pressure reduction moved the active pair away from that nominal
      // target. Otherwise thrust and pressure would chatter every cycle.
      const bool correction_allowed = action_c_auto_roll_correction_enabled_ &&
          action_c_roll_trend_.worsening &&
          bottom_ready && front_shape_ready &&
          (action_c_pre_roll_pair_sign_ != 0 || nominal_arm_pressure_ready);

      // Correct roll before releasing the rear chambers or starting the
      // forward pulse. Use the explicitly verified real-machine mapping:
      //   positive roll -> positive thrust on arms 2+3
      //   negative roll -> positive thrust on arms 0+1
      // Keep the currently active pair while ramping down so crossing zero
      // cannot instantaneously transfer thrust to the opposite side.
      const double roll_offset = wrapAngle(roll_ - action_c_roll_target_rad_);
      if (action_c_posture_ready_latched_ &&
          std::abs(roll_offset) >= action_c_roll_recorrect_threshold_rad_)
        {
          action_c_posture_ready_latched_ = false;
          action_c_pre_roll_ready_start_ = ros::Time(0);
          ROS_WARN("Action C continuous-pulse roll limit reached: %.1f deg (limit %.1f deg); pausing pulses for posture recovery",
                   roll_offset * 180.0 / M_PI,
                   action_c_roll_recorrect_threshold_rad_ * 180.0 / M_PI);
        }
      int desired_pair_sign = 0;
      // Hysteresis is intentional.  Before the first good posture (or after
      // exceeding the wide limit), restore roll to the narrow deadband. Once
      // latched, tolerate normal pulse-to-pulse roll motion and do not run a
      // correction before every pulse.
      if (!action_c_posture_ready_latched_)
        {
          if (roll_offset > action_c_pre_roll_deadband_rad_)
            desired_pair_sign = 1;
          else if (roll_offset < -action_c_pre_roll_deadband_rad_)
            desired_pair_sign = -1;
        }
      if (correction_allowed && desired_pair_sign != 0)
        {
          if (action_c_pre_roll_pair_sign_ == 0 &&
              action_c_pre_roll_thrust_n_ <= 0.01)
            action_c_pre_roll_pair_sign_ = desired_pair_sign;
        }

      // Release only the active roll pair, but do not gate rotor thrust on
      // measured pressure. In c3 arm 2 fell from 30 to only 22--23 kPa during
      // the entire eight-second PREPARE window; the former +/-3 kPa arrival
      // gate therefore kept /pwm_test empty for every trial. The pressure
      // command and bounded rotor ramp now start together.
      const bool pressure_release_requested = correction_allowed &&
          desired_pair_sign != 0 &&
          action_c_pre_roll_pair_sign_ == desired_pair_sign;
      const double active_pressure_drop =
          action_c_pre_roll_pressure_drop_kpa_;
      auto weakenPreRollArm = [this, active_pressure_drop](size_t arm) {
        inchworm_pressure_[arm] = std::max(
            action_c_pre_roll_min_pressure_kpa_,
            inchworm_pressure_[arm] - active_pressure_drop);
      };
      if (pressure_release_requested && action_c_pre_roll_pair_sign_ > 0)
        {
          weakenPreRollArm(2);
          weakenPreRollArm(3);
          // Arms 0+1 are the support pair during positive-roll correction.
          // Do not leave front arm 0 at the model's low 14-kPa preparation
          // target while the opposite pair is released.
          inchworm_pressure_[0] = std::max(
              inchworm_pressure_[0], base_pressure_kpa_);
          inchworm_pressure_[1] = std::max(
              inchworm_pressure_[1], base_pressure_kpa_);
        }
      else if (pressure_release_requested &&
               action_c_pre_roll_pair_sign_ < 0)
        {
          weakenPreRollArm(0);
          weakenPreRollArm(1);
          // Arms 2+3 support the body during negative-roll correction.
          inchworm_pressure_[2] = std::max(
              inchworm_pressure_[2], base_pressure_kpa_);
          inchworm_pressure_[3] = std::max(
              inchworm_pressure_[3], base_pressure_kpa_);
        }
      double desired_pre_roll_thrust = 0.0;
      if (correction_allowed && pressure_release_requested &&
          action_c_pre_roll_pair_sign_ == desired_pair_sign)
        desired_pre_roll_thrust = std::min(
            action_c_pre_roll_max_thrust_n_, std::max(
                action_c_pre_roll_min_thrust_n_,
                action_c_pre_roll_kp_n_rad_ *
                    (std::abs(roll_offset) -
                     action_c_pre_roll_deadband_rad_)));
      action_c_pre_roll_thrust_n_ = approach(
          action_c_pre_roll_thrust_n_, desired_pre_roll_thrust,
          action_c_pre_roll_thrust_ramp_n_s_);
      if (action_c_pre_roll_thrust_n_ <= 0.01 &&
          desired_pair_sign == 0)
        action_c_pre_roll_pair_sign_ = 0;
      else if (action_c_pre_roll_thrust_n_ <= 0.01 &&
               desired_pair_sign != 0 &&
               action_c_pre_roll_pair_sign_ != desired_pair_sign)
        action_c_pre_roll_pair_sign_ = desired_pair_sign;
      inchworm_thrust_.fill(0.0);
      if (action_c_pre_roll_pair_sign_ > 0)
        inchworm_thrust_[2] = inchworm_thrust_[3] =
            action_c_pre_roll_thrust_n_;
      else if (action_c_pre_roll_pair_sign_ < 0)
        inchworm_thrust_[0] = inchworm_thrust_[1] =
            action_c_pre_roll_thrust_n_;

      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.5,
          "Action C PREPARE: pulse %.1f N, roll %.1f/%.1f deg, posture latch %d (hard limit %.1f deg, auto correction %d), pre-roll pair %s at %.2f N, pressure target [%.1f %.1f %.1f %.1f] kPa, bottom %.1f/%.1f kPa, local model %d, learned front bias [%.1f %.1f %.1f %.1f] kPa, rear pulse target 0 kPa",
          action_c_thrust_n_, roll_ * 180.0 / M_PI,
          action_c_roll_target_rad_ * 180.0 / M_PI,
          action_c_posture_ready_latched_ ? 1 : 0,
          action_c_posture_hard_roll_rad_ * 180.0 / M_PI,
          action_c_auto_roll_correction_enabled_ ? 1 : 0,
          action_c_pre_roll_pair_sign_ > 0 ? "2+3" :
              (action_c_pre_roll_pair_sign_ < 0 ? "0+1" : "off"),
          action_c_pre_roll_thrust_n_,
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3],
          bottom_pressure_kpa_,
          action_c_bottom_hold_kpa_,
          action_c_online_pressure_adaptation_enabled_ &&
              action_c_last_pulse_valid_ ? 1 : 0,
          action_c_pressure_bias_kpa_[0],
          action_c_pressure_bias_kpa_[1], action_c_pressure_bias_kpa_[2],
          action_c_pressure_bias_kpa_[3]);
      const bool pneumatic_ready = actionCPressureReady(inchworm_pressure_, now) &&
          bottom_ready && front_shape_ready;
      const double allowed_roll_error = action_c_posture_ready_latched_
          ? action_c_roll_recorrect_threshold_rad_
          : (action_c_auto_roll_correction_enabled_
                 ? action_c_pre_roll_deadband_rad_
                 : action_c_posture_recovery_trigger_rad_);
      const bool roll_ready =
          (action_c_boundary_ready_ || std::abs(roll_offset) < allowed_roll_error) &&
          std::abs(body_angular_velocity_[0]) <=
              action_c_pre_roll_rate_tolerance_rad_s_ &&
          action_c_pre_roll_thrust_n_ <= 0.01;
      if (pneumatic_ready && roll_ready && action_c_boundary_ready_)
        {
          if (action_c_pre_roll_ready_start_.isZero())
            action_c_pre_roll_ready_start_ = now;
        }
      else
        action_c_pre_roll_ready_start_ = ros::Time(0);
      const bool pre_roll_complete =
          !action_c_pre_roll_ready_start_.isZero() &&
          (now - action_c_pre_roll_ready_start_).toSec() >=
              action_c_pre_roll_settle_sec_;
      if (action_c_bottom_sync_enabled_ && action_c_bottom_timing_.filling &&
          (!nominal_arm_pressure_ready || !front_shape_ready ||
           !action_c_bottom_timing_.fresh(now.toSec())))
        {
          action_c_bottom_timing_.beginCycle();
          publishBottomTarget(actionCBottomTarget());
        }
      if (pre_roll_complete && action_c_bottom_sync_enabled_)
        {
          if (!action_c_bottom_timing_.filling)
            {
              ROS_INFO("Action C BOTTOM fill start: low %.1f kPa, predicted precharge duration %.3f s from learned rate/lag; launch remains gated by fresh measured rise",
                       bottom_pressure_kpa_, std::max(0.0,
                         action_c_bottom_timing_.plannedFillLead(action_c_bottom_hold_kpa_, bottom_pressure_kpa_) -
                         actionCBottomPeakLead()));
              action_c_bottom_timing_.startFill(now.toSec());
            }
          publishBottomTarget(actionCBottomTarget());
          ROS_INFO_THROTTLE(0.25,
              "Action C BOTTOM SYNC: actual %.1f -> peak target %.1f kPa, rise %.1f kPa/s, delay %.3f s, ETA %.3f s / thrust peak lead %.3f s; previous peak error %.1f kPa",
              bottom_pressure_kpa_, action_c_bottom_hold_kpa_, action_c_bottom_timing_.rate,
              action_c_bottom_timing_.delay,
              action_c_bottom_timing_.eta(action_c_bottom_hold_kpa_, pressure_tolerance_kpa_, now.toSec()),
              actionCBottomPeakLead(), action_c_bottom_timing_.peak_error);
        }
      if (pre_roll_complete && (!action_c_bottom_sync_enabled_ ||
          action_c_bottom_timing_.ready(action_c_bottom_hold_kpa_, pressure_tolerance_kpa_,
                                       actionCBottomPeakLead(), now.toSec())))
        {
          if (!action_c_posture_ready_latched_)
            {
              action_c_posture_ready_latched_ = true;
              ROS_INFO("Action C posture accepted at roll %.1f deg; stable offsets allowed, worsening roll requests recovery; hard limit %.1f deg",
                       roll_offset * 180.0 / M_PI,
                       action_c_posture_hard_roll_rad_ * 180.0 / M_PI);
            }
          action_c_pulse_started_ = false;
          action_c_pulse_angle_rad_ = 0.0;
          action_c_command_thrust_n_ = 0.0;
          action_c_pulse_start_travel_m_ = travel;
          action_c_pulse_start_roll_ = roll_;
          action_c_pulse_start_pitch_ = pitch_;
          action_c_pre_roll_ready_start_ = ros::Time(0);
          enterState(ACTION_C_HORIZONTAL_PULL, now);
        }
      else if (elapsed >= action_c_prepare_timeout_sec_)
        {
          action_c_success_ = false;
          ROS_ERROR("ACTION C FAILED safely in PREPARE: pressure/start bend (ready=%d, minimum %.1f deg plus absolute-floor headroom)/roll %.1f-deg target was not ready before the pulse",
                    front_shape_ready, action_c_prepare_min_bend_rad_ * 180.0 / M_PI,
                    action_c_roll_target_rad_ * 180.0 / M_PI);
          enterState(ACTION_C_REGRIP, now);
        }
      }
      break;

    case ACTION_C_HORIZONTAL_PULL:
      {
      publishBottomDirection(-1.0);
      // Bottom fill began in PREPARE; keep its peak target during the
      // unchanged thrust ramp. Rear pressure still releases at the ON edge.
      publishBottomTarget(actionCBottomTarget());
      const double rear_target = 0.0;
      double pulse_roll_trim_n = 0.0;
      PressureArray angle_thrust_reduction_n{{0.0, 0.0, 0.0, 0.0}};
      double front_extension_mismatch_rad = 0.0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (inchworm_front_[arm])
            inchworm_pressure_[arm] = action_c_front_pressure_[arm];
          else
            inchworm_pressure_[arm] = rear_target;
          if (!inchworm_front_[arm] || !action_c_pulse_started_)
            inchworm_thrust_[arm] = 0.0;
        }
      if (!action_c_pulse_started_)
        {
          // An exhaust command is not evidence of low measured pressure.
          // Start the pulse clock only AFTER rear pressure is low, so peak
          // thrust cannot arrive while the rear is still gripping strongly.
          if (action_c_pressure_pull_enabled_ &&
              !actionCRearReleased(now))
            {
              inchworm_thrust_.fill(0.0);
              publishPressureTarget(inchworm_pressure_, std::numeric_limits<double>::infinity());
              publishCorrectionThrust(inchworm_thrust_);
              ROS_INFO_THROTTLE(0.2, "Action C REAR RELEASE: waiting for rear <= %.1f kPa before pulse", action_c_rear_release_max_kpa_);
              if (elapsed >= action_c_rear_release_timeout_sec_)
                stopInchwormToGrasp(now, "Action C rear exhaust timeout; no pulse emitted");
              break;
            }
          // The grasp can change during the rear-release wait after PREPARE.
          // Recheck before latching ON or issuing any pulse thrust.
          bool start_shape_ready = true;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              start_shape_ready = start_shape_ready && hugmy::actionCStartBendReady(
                  measuredArmBendAngle(arm), action_c_prepare_min_bend_rad_,
                  action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
                  action_c_start_extension_margin_rad_);
          if (!start_shape_ready)
            {
              stopInchwormToGrasp(now, "Action C start bend/headroom lost before pulse ON");
              break;
            }
          action_c_pulse_started_ = true;
          action_c_pulse_start_ = now;
          if (action_c_bottom_sync_enabled_)
            {
              action_c_bottom_timing_.peak_at = now.toSec() + actionCBottomPeakLead();
              action_c_bottom_timing_.peak_error = std::numeric_limits<double>::quiet_NaN();
              // Learn pneumatic response before body/arm motion, not from
              // pressure spikes caused by contact loading during the pulse.
              action_c_bottom_timing_.filling = false;
            }
          action_c_coordination_.startPulse();
          action_c_propulsion_.beginPulse();
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            action_c_coordination_rebend_target_[arm] = std::min(
                std::min(maximum_pressure_kpa_, action_c_max_pressure_kpa_),
                std::max(base_pressure_kpa_, std::max(action_c_front_pressure_[arm],
                    measured_pressure_[arm]) + action_c_rebend_pressure_boost_kpa_));
          action_c_pressure_feedback_time_ = now;
          action_c_pulse_peak_thrust_n_.fill(0.0);
          action_c_pulse_terminal_extension_rad_.fill(0.0);
          action_c_pulse_arm_ceiling_n_.fill(0.0);
          action_c_pulse_arm_nominal_n_.fill(0.0);
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                const double hard_limit = action_c_adaptive_burst_enabled_
                    ? action_c_thrust_safety_.hard_max_n : action_c_max_thrust_n_;
                action_c_pulse_arm_nominal_n_[arm] = std::max(0.0, std::min(
                    std::min(action_c_max_thrust_n_, hard_limit),
                    action_c_thrust_n_ + (action_c_individual_extension_enabled_
                        ? action_c_arm_thrust_offset_n_[arm] : 0.0)));
                action_c_pulse_arm_ceiling_n_[arm] = std::min(
                    std::min(action_c_max_thrust_n_, hard_limit),
                    action_c_pulse_arm_nominal_n_[arm] + (action_c_individual_extension_enabled_
                        ? action_c_individual_assist_max_n_ : 0.0));
                action_c_previous_pressure_kpa_[arm] = measured_pressure_[arm];
                const double bend = measuredArmBendAngle(arm);
                action_c_previous_bend_angle_rad_[arm] = bend;
                action_c_pulse_start_angle_rad_[arm] = bend;
                ROS_INFO("Action C bend limits arm%zu: start %.1f deg, pulse floor %.1f deg, relative extension limit %.1f deg",
                    arm, bend * 180.0 / M_PI,
                    hugmy::actionCPulseBendFloor(bend, action_c_min_safe_angle_rad_,
                        action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
                        action_c_start_extension_margin_rad_) * 180.0 / M_PI,
                    action_c_max_extension_rad_ * 180.0 / M_PI);
                action_c_pulse_observer_[arm].seed(bend,
                    hugmy::relativeBendRate(neuron_gyro_[arm][1], body_angular_velocity_[0],
                        body_angular_velocity_[1], arm_yaw_[arm]), neuron_stamp_[arm].toSec());
                action_c_pulse_min_angle_rad_[arm] = bend;
                action_c_rate_previous_angle_rad_[arm] = bend;
                action_c_filtered_bend_rate_rad_s_[arm] = 0.0;
                action_c_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
              }
          action_c_command_thrust_n_ = 0.0;
          action_c_pulse_start_travel_m_ = travel;
          action_c_pulse_start_roll_ = roll_;
          action_c_pulse_start_pitch_ = pitch_;
          publishBottomTarget(actionCBottomTarget());
          action_c_rate_time_ = now;
          action_c_pulse_had_extension_ = false;
          action_c_pulse_overextended_ = false;
          action_c_pulse_recoil_complete_ = false;
          action_c_pressure_gradient_initialized_ = true;
          ready_start_ = ros::Time(0);
          ROS_INFO("Action C pulse ON: base front-pair level %.1f N (individual control may add bounded assist), rear command %.1f kPa (measured release required=%d), relative extension target %.1f deg, corrected start roll %.1f deg",
                   action_c_thrust_n_, rear_target,
                   action_c_pressure_pull_enabled_,
                   action_c_extension_target_rad_ * 180.0 / M_PI,
                   roll_ * 180.0 / M_PI);
          if (action_c_individual_extension_enabled_)
            ROS_INFO("Action C individual ceilings [%.2f %.2f %.2f %.2f] N; shared relative extension 0 -> %.1f deg in %.2f s (base %.2f N, pair ceiling limits base only)",
                     action_c_pulse_arm_ceiling_n_[0], action_c_pulse_arm_ceiling_n_[1],
                     action_c_pulse_arm_ceiling_n_[2], action_c_pulse_arm_ceiling_n_[3],
                     action_c_extension_target_rad_ * 180.0 / M_PI,
                     action_c_extension_trajectory_sec_, action_c_thrust_n_);
        }
      if (action_c_pulse_started_)
        {
          action_c_command_thrust_n_ = approach(
              action_c_command_thrust_n_, action_c_thrust_n_,
              action_c_thrust_ramp_n_s_);
          double angle_sum = 0.0;
          size_t angle_count = 0;
          bool both_extended = true;
          bool extension_guard = false;
          unsigned int guard_reasons = 0;
          double predicted_bend_margin = std::numeric_limits<double>::infinity();
          size_t limiting_bend_arm = ARM_COUNT;
          double minimum_extension = std::numeric_limits<double>::infinity();
          double maximum_extension = -std::numeric_limits<double>::infinity();
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                // Do not fuse the pressure model into this feedback angle.
                // The model is useful for the initial pressure guess, but
                // under thrust/contact it can have a large bias.  Control
                // each arm from its directly measured Spinal--Neuron bend.
                action_c_front_angle_rad_[arm] = measuredArmBendAngle(arm);
                if (action_c_reference_pulse_mode_ || action_c_pressure_pull_enabled_)
                  {
                    auto& observer = action_c_pulse_observer_[arm];
                    observer.update(hugmy::relativeBendRate(neuron_gyro_[arm][1],
                        body_angular_velocity_[0], body_angular_velocity_[1], arm_yaw_[arm]),
                        neuron_stamp_[arm].toSec());
                    const bool simulation_bend = std::isfinite(grip_joint_bend_rad_[arm]) &&
                        !grip_state_stamp_.isZero() && (now-grip_state_stamp_).toSec() <= 0.25;
                    if (!simulation_bend)
                      {
                        if (!observer.valid) guard_reasons |= hugmy::ACTION_C_ANGLE_UNRELIABLE;
                        action_c_front_angle_rad_[arm] = observer.bend;
                      }
                    ROS_INFO_THROTTLE(0.2, "Action C ANGLE CHECK arm%zu: pulse estimate %.1f deg, accelerometer %.1f deg, acc norm %.2f m/s^2; valid=%d",
                        arm, action_c_front_angle_rad_[arm]*180.0/M_PI,
                        measuredArmBendAngle(arm)*180.0/M_PI, neuron_acc_norm_[arm], observer.valid);
                  }
                action_c_pulse_min_angle_rad_[arm] = std::min(
                    action_c_pulse_min_angle_rad_[arm],
                    action_c_front_angle_rad_[arm]);
                const double extension = action_c_pulse_start_angle_rad_[arm] -
                    action_c_front_angle_rad_[arm];
                minimum_extension = std::min(minimum_extension, extension);
                maximum_extension = std::max(maximum_extension, extension);
                // Only differentiate a genuinely new Neuron sample. The main
                // control loop is faster than the IMU transport, and treating
                // repeated samples as new data made the short-pulse derivative
                // alternate between zero and a large spike.
                if (!neuron_stamp_[arm].isZero() &&
                    neuron_stamp_[arm] !=
                        action_c_previous_neuron_stamp_[arm])
                  {
                    const double sample_dt = std::max(1.0e-4,
                        (neuron_stamp_[arm] -
                         action_c_previous_neuron_stamp_[arm]).toSec());
                    const double raw_rate =
                        (action_c_front_angle_rad_[arm] -
                         action_c_rate_previous_angle_rad_[arm]) / sample_dt;
                    action_c_filtered_bend_rate_rad_s_[arm] =
                        (1.0 - action_c_angle_rate_filter_alpha_) *
                            action_c_filtered_bend_rate_rad_s_[arm] +
                        action_c_angle_rate_filter_alpha_ * raw_rate;
                    action_c_rate_previous_angle_rad_[arm] =
                        action_c_front_angle_rad_[arm];
                    action_c_previous_neuron_stamp_[arm] = neuron_stamp_[arm];
                  }
                if ((action_c_reference_pulse_mode_ || action_c_pressure_pull_enabled_) &&
                    !(std::isfinite(grip_joint_bend_rad_[arm]) &&
                      !grip_state_stamp_.isZero() && (now-grip_state_stamp_).toSec() <= 0.25))
                  action_c_filtered_bend_rate_rad_s_[arm] = action_c_pulse_observer_[arm].rate;
                const double bend_rate =
                    action_c_filtered_bend_rate_rad_s_[arm];
                const double bend_floor = hugmy::actionCPulseBendFloor(
                    action_c_pulse_start_angle_rad_[arm], action_c_min_safe_angle_rad_,
                    action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
                    action_c_start_extension_margin_rad_);
                // Include both absolute/adjusted bend floor and relative
                // extension limit. Re-flexion must not hide a current limit.
                const double effective_floor = std::max(bend_floor,
                    action_c_pulse_start_angle_rad_[arm] - action_c_max_extension_rad_);
                const double predicted_margin = action_c_front_angle_rad_[arm] - effective_floor +
                    std::min(0.0, bend_rate) * action_c_pulse_roll_handoff_lookahead_sec_;
                if (std::isfinite(predicted_margin) && predicted_margin < predicted_bend_margin)
                  {
                    predicted_bend_margin = predicted_margin;
                    limiting_bend_arm = arm;
                  }
                both_extended = both_extended &&
                    (action_c_propulsion_enabled_ && action_c_pressure_pull_enabled_ && !action_c_reference_pulse_mode_
                         ? action_c_pulse_start_angle_rad_[arm]-action_c_pulse_min_angle_rad_[arm]
                         : extension) >= action_c_extension_target_rad_;
                if (extension >= action_c_max_extension_rad_)
                  guard_reasons |= hugmy::ACTION_C_OVER_EXTENSION;
                if (action_c_front_angle_rad_[arm] <= hugmy::actionCPulseBendFloor(
                    action_c_pulse_start_angle_rad_[arm], action_c_min_safe_angle_rad_,
                    action_c_hard_min_bend_rad_, action_c_extension_target_rad_,
                    action_c_start_extension_margin_rad_))
                  guard_reasons |= hugmy::ACTION_C_LOW_BEND;
                if (bend_rate <= -action_c_extension_rate_limit_rad_s_)
                  guard_reasons |= hugmy::ACTION_C_EXTENSION_RATE;

                // The optimizer selects the common pulse ceiling. Angle
                // feedback may only reduce an individual rotor from that
                // ceiling: it must never add thrust above the candidate. This
                // lets the arm that straightens first slow down without the
                // other arm receiving the same (wrong) correction.
                const double desired_bend =
                    action_c_pulse_start_angle_rad_[arm] -
                    action_c_extension_target_rad_;
                const double bend_error =
                    action_c_front_angle_rad_[arm] - desired_bend;
                const double raw_angle_trim =
                    action_c_angle_thrust_kp_n_rad_ * bend_error +
                    action_c_angle_thrust_kd_n_rad_s_ * bend_rate;
                angle_thrust_reduction_n[arm] = std::max(0.0, std::min(
                    action_c_angle_thrust_max_trim_n_, -raw_angle_trim));
                angle_sum += action_c_front_angle_rad_[arm];
                ++angle_count;
              }
          action_c_rate_time_ = now;
          if (angle_count > 1 &&
              std::isfinite(minimum_extension) &&
              std::isfinite(maximum_extension))
            front_extension_mismatch_rad =
                maximum_extension - minimum_extension;
          if (action_c_propulsion_enabled_ && std::any_of(action_c_propulsion_.flexing.begin(),
              action_c_propulsion_.flexing.end(), [](bool flexing){return flexing;}))
            front_extension_mismatch_rad = hugmy::actionCPeakExtensionMismatch(inchworm_front_,
                action_c_pulse_start_angle_rad_, action_c_pulse_min_angle_rad_);
          action_c_optimizer_peak_extension_mismatch_rad_ = std::max(
              action_c_optimizer_peak_extension_mismatch_rad_,
              front_extension_mismatch_rad);
          // IMU-based bend can jump in both directions under rotor/body
          // acceleration. A pair mismatch is unsafe only when at least one
          // arm has genuinely reached the requested extension. Do not turn a
          // common re-bend/no-extension sample into a permanent thrust cap.
          const bool extension_imbalance_guard = angle_count > 1 &&
              maximum_extension >= action_c_extension_target_rad_ &&
              front_extension_mismatch_rad >=
                  action_c_max_extension_mismatch_rad_;
          if (extension_imbalance_guard)
            guard_reasons |= hugmy::ACTION_C_PAIR_MISMATCH;
          // Hard posture limits must not wait for pulse OFF. For smaller
          // excursions, recover only when worsening roll threatens an arm's
          // bend budget. Keep all actual angle guards and their accounting.
          if (action_c_pulse_roll_handoff_enabled_ &&
              !(guard_reasons & hugmy::ACTION_C_ANGLE_UNRELIABLE) &&
              hugmy::actionCPulseRollHandoff(wrapAngle(roll_ - target_roll_),
                  wrapAngle(action_c_pulse_start_roll_ - target_roll_), body_angular_velocity_[0],
                  predicted_bend_margin, action_c_pulse_roll_handoff_margin_rad_,
                  action_c_posture_recovery_trigger_rad_, action_c_posture_hard_roll_rad_,
                  action_c_posture_rate_trigger_rad_s_, action_c_roll_trend_.growth_rad,
                  action_c_roll_trend_.outward_rate_rad_s))
            {
              action_c_pulse_overextended_ = true; // interrupted, not a successful pulse
              action_c_pulse_had_extension_ = false;
              action_c_optimizer_trial_had_extension_guard_ |= guard_reasons != 0;
              action_c_optimizer_trial_guard_reasons_ |= guard_reasons;
              ROS_WARN("Action C pulse roll handoff: roll %.1f -> %.1f deg, gyro %.1f deg/s, limiting arm%zu predicted bend margin %.1f deg (lead %.2f s), actual guard mask=%u; cut pulse before shared recovery",
                  wrapAngle(action_c_pulse_start_roll_ - target_roll_) * 180.0 / M_PI,
                  wrapAngle(roll_ - target_roll_) * 180.0 / M_PI,
                  body_angular_velocity_[0] * 180.0 / M_PI, limiting_bend_arm,
                  predicted_bend_margin * 180.0 / M_PI,
                  action_c_pulse_roll_handoff_lookahead_sec_, guard_reasons);
              // requestActionCPostureRecovery publishes zero thrust FIRST,
              // preserves the goal and then hands pressure/bottom to recovery.
              requestActionCPostureRecovery(now, "worsening pulse roll near bend limit or hard posture limit");
              return; // no later pulse output may overwrite recovery commands
            }
          const double common_front_thrust = std::max(0.0, std::min(
              action_c_thrust_n_, action_c_command_thrust_n_));
          const double pulse_elapsed = std::max(0.0, (now - action_c_pulse_start_).toSec());
          const double shared_extension_reference = hugmy::actionCExtensionReference(
              pulse_elapsed, action_c_extension_trajectory_sec_, action_c_extension_target_rad_);

          // Never increase either rotor above the learned/selected common
          // pulse magnitude for roll correction. Reduce the selected front
          // rotor on the matching Y side; this trades a small amount of
          // longitudinal thrust for posture instead of increasing load.
          const double roll_error = wrapAngle(
              roll_ - action_c_roll_target_rad_);
          const double raw_roll_trim =
              action_c_pulse_roll_kp_n_rad_ * roll_error +
              action_c_pulse_roll_kd_n_rad_s_ *
                  body_angular_velocity_[0];
          const double roll_trim_capacity = std::max(0.0, std::min(
              action_c_pulse_roll_max_trim_n_, common_front_thrust));
          pulse_roll_trim_n = std::max(-roll_trim_capacity, std::min(
              roll_trim_capacity, raw_roll_trim));
          // The reference run identifies the paired pulse itself. Keep both
          // commands equal; the roll supervisor and hard tilt guards remain.
          if (action_c_reference_pulse_mode_) pulse_roll_trim_n = 0.0;
          PressureArray relative_extension{}, coordinated_assist{};
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            relative_extension[arm] = action_c_pulse_start_angle_rad_[arm] - action_c_front_angle_rad_[arm];
          if (action_c_coordination_enabled_)
            coordinated_assist = action_c_coordination_.assist(inchworm_front_, relative_extension,
                action_c_extension_target_rad_, roll_error, body_angular_velocity_[0],
                std::hypot(wrapAngle(roll_ - action_c_pulse_start_roll_),
                           wrapAngle(pitch_ - action_c_pulse_start_pitch_)),
                action_c_max_body_tilt_change_rad_, pulse_elapsed, action_c_thrust_ramp_n_s_);
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            {
              const double previous_thrust = inchworm_thrust_[arm];
              inchworm_thrust_[arm] = 0.0;
              if (!inchworm_front_[arm]) continue;
              const double extension = action_c_pulse_start_angle_rad_[arm] -
                  action_c_front_angle_rad_[arm];
              // Brake the leading arm BEFORE it reaches its 8-deg target.
              // Never speed up the lagging rotor above the common candidate.
              const double sync_reduction = hugmy::actionCExtensionSyncReduction(
                  extension, minimum_extension,
                  action_c_filtered_bend_rate_rad_s_[arm],
                  action_c_extension_target_rad_,
                  action_c_extension_prediction_sec_,
                  action_c_extension_sync_deadband_rad_,
                  action_c_extension_sync_kp_n_rad_,
                  action_c_angle_thrust_max_trim_n_);
              angle_thrust_reduction_n[arm] = std::max(
                  angle_thrust_reduction_n[arm], sync_reduction);
              if (action_c_reference_pulse_mode_) angle_thrust_reduction_n[arm] = 0.0;
              // Roll trim follows rotor Y position, not travel direction.
              // Positive roll reduces the +Y front rotor; negative roll
              // reduces the -Y rotor. Thus the same rule works for the +x
              // pair 0/3 and the -x pair 1/2.
              const bool positive_y = arm_base_position_[arm][1] > 0.0;
              const double roll_reduction = positive_y
                  ? std::max(0.0, pulse_roll_trim_n)
                  : std::max(0.0, -pulse_roll_trim_n);
              const double reduction = std::min(common_front_thrust,
                  (action_c_reference_pulse_mode_ ? 0.0 : angle_thrust_reduction_n[arm]) + roll_reduction);
              // Releasing a trim must not introduce a thrust step either.
              inchworm_thrust_[arm] = std::min(
                  std::max(0.0, common_front_thrust - reduction),
                  previous_thrust + action_c_thrust_ramp_n_s_ * std::max(0.0, dt));
              if (action_c_reference_pulse_mode_)
                inchworm_thrust_[arm] = std::min(inchworm_thrust_[arm],
                    action_c_pulse_arm_ceiling_n_[arm]);
              if (action_c_individual_extension_enabled_)
                {
                  const double requested = hugmy::actionCIndividualThrust(
                      action_c_pulse_arm_nominal_n_[arm], action_c_pulse_arm_ceiling_n_[arm], pulse_elapsed,
                      action_c_thrust_ramp_n_s_, shared_extension_reference,
                      extension, maximum_extension, action_c_filtered_bend_rate_rad_s_[arm],
                      action_c_individual_kp_n_rad_, action_c_individual_kd_n_rad_s_,
                      action_c_extension_prediction_sec_, action_c_extension_sync_deadband_rad_,
                      action_c_sync_brake_band_rad_, action_c_extension_target_rad_);
                  const double target = std::max(0.0, requested - roll_reduction);
                  inchworm_thrust_[arm] = approach(previous_thrust, target,
                      target < previous_thrust ? action_c_thrust_down_rate_n_s_
                                              : action_c_thrust_ramp_n_s_);
                  inchworm_thrust_[arm] = std::min(inchworm_thrust_[arm],
                      action_c_pulse_arm_ceiling_n_[arm]);
                  angle_thrust_reduction_n[arm] = std::max(0.0,
                      action_c_pulse_arm_ceiling_n_[arm] - requested);
                }
              if (action_c_coordination_enabled_)
                {
                  // Existing angle/roll brakes have priority over both FF
                  // and live assist; all output is still ramp/ceiling bounded.
                  const double extra = reduction > 1e-6 ? 0.0 : coordinated_assist[arm];
                  const double ceiling = std::min(action_c_max_thrust_n_,
                      action_c_thrust_safety_.hard_max_n);
                  // Pair-mismatch limits constrain the COMMON candidate, not
                  // the small lagging-arm correction needed to remove mismatch.
                  // Hard tilt/overextension limits still cap every rotor.
                  const double requested = std::min(ceiling,
                      std::max(0.0, common_front_thrust + extra - reduction));
                  inchworm_thrust_[arm] = approach(previous_thrust, requested,
                      requested < previous_thrust ? action_c_thrust_down_rate_n_s_ : action_c_thrust_ramp_n_s_);
                  inchworm_thrust_[arm] = std::min(ceiling, inchworm_thrust_[arm]);
                  if (!(action_c_propulsion_enabled_ && action_c_pressure_pull_enabled_ && !action_c_reference_pulse_mode_) &&
                      action_c_coordination_.earlyPressure(arm, pulse_elapsed,
                          action_c_pulse_duration_sec_, extension, action_c_extension_target_rad_,
                          action_c_filtered_bend_rate_rad_s_[arm]))
                    {
                      if (action_c_coordination_.pressure_start[arm] < 0)
                        ROS_INFO("Action C pressure lead arm%zu: %.3f s, measured extension %.1f deg; target %.1f kPa",
                                 arm, action_c_coordination_.lead[arm], extension*180.0/M_PI,
                                 action_c_coordination_rebend_target_[arm]);
                      action_c_coordination_.startPressure(arm, now.toSec(), measured_pressure_[arm],
                          action_c_coordination_rebend_target_[arm]);
                      action_c_front_pressure_[arm] = action_c_coordination_rebend_target_[arm];
                      inchworm_pressure_[arm] = action_c_front_pressure_[arm];
                    }
                }
              if (action_c_propulsion_enabled_ && action_c_pressure_pull_enabled_ && !action_c_reference_pulse_mode_ &&
                  action_c_propulsion_.response[arm].early(pulse_elapsed, action_c_pulse_duration_sec_,
                      extension, action_c_extension_target_rad_, action_c_filtered_bend_rate_rad_s_[arm]))
                {
                  action_c_front_pressure_[arm] = action_c_coordination_rebend_target_[arm];
                  inchworm_pressure_[arm] = action_c_front_pressure_[arm];
                }
              if (action_c_propulsion_enabled_ && action_c_pressure_pull_enabled_ && !action_c_reference_pulse_mode_)
                {
                  const double bend_floor = std::max(hugmy::actionCPulseBendFloor(
                      action_c_pulse_start_angle_rad_[arm], action_c_min_safe_angle_rad_,
                      action_c_hard_min_bend_rad_, action_c_extension_target_rad_, action_c_start_extension_margin_rad_),
                      action_c_pulse_start_angle_rad_[arm]-action_c_max_extension_rad_);
                  if (!action_c_propulsion_.flexing[arm] && hugmy::actionCFlexionHandoff(
                      pulse_elapsed, extension, action_c_extension_target_rad_,
                      action_c_filtered_bend_rate_rad_s_[arm], action_c_front_angle_rad_[arm]-bend_floor,
                      action_c_extension_rate_limit_rad_s_, action_c_flexion_lookahead_sec_))
                    {
                      const double residual=std::min(action_c_pressure_pull_max_n_,
                          action_c_pressure_pull_ratio_*action_c_thrust_n_+action_c_propulsion_.extra_n);
                      action_c_propulsion_.startFlexion(arm, std::min(previous_thrust,inchworm_thrust_[arm]), residual);
                      ROS_INFO("Action C arm%zu SOFT FLEXION: extension %.1f deg, rate %.1f deg/s; residual <= %.2f N, pressure %.1f kPa; other arm keeps its phase",
                          arm, extension*180./M_PI, action_c_filtered_bend_rate_rad_s_[arm]*180./M_PI,
                          action_c_propulsion_.flex_ceiling[arm], action_c_coordination_rebend_target_[arm]);
                    }
                  if (action_c_propulsion_.flexing[arm])
                    {
                      // Latched phase: releasing a roll trim cannot re-boost
                      // an arm which has already started powered re-flexion.
                      inchworm_thrust_[arm]=std::min(std::min(previous_thrust,inchworm_thrust_[arm]),
                          action_c_propulsion_.flex_ceiling[arm]);
                      action_c_front_pressure_[arm]=action_c_coordination_rebend_target_[arm];
                      inchworm_pressure_[arm]=action_c_front_pressure_[arm];
                    }
                }
              action_c_pulse_peak_thrust_n_[arm] = std::max(
                  action_c_pulse_peak_thrust_n_[arm], inchworm_thrust_[arm]);
            }
          if (action_c_coordination_enabled_)
            ROS_INFO_THROTTLE(0.1,
                "Action C SYNC: requested assist [%.3f %.3f %.3f %.3f] N, output [%.3f %.3f %.3f %.3f] N, common limit %.2f N, rotor hard limit %.2f N",
                coordinated_assist[0], coordinated_assist[1], coordinated_assist[2], coordinated_assist[3],
                inchworm_thrust_[0], inchworm_thrust_[1], inchworm_thrust_[2], inchworm_thrust_[3],
                action_c_optimizer_safe_thrust_max_n_, std::min(action_c_max_thrust_n_, action_c_thrust_safety_.hard_max_n));
          if (action_c_individual_extension_enabled_)
            ROS_INFO_THROTTLE(0.1,
                "Action C TRACK: shared extension %.1f deg, measured [%.1f %.1f %.1f %.1f] deg, ceilings [%.2f %.2f %.2f %.2f] N",
                shared_extension_reference * 180.0 / M_PI,
                (action_c_pulse_start_angle_rad_[0] - action_c_front_angle_rad_[0]) * 180.0 / M_PI,
                (action_c_pulse_start_angle_rad_[1] - action_c_front_angle_rad_[1]) * 180.0 / M_PI,
                (action_c_pulse_start_angle_rad_[2] - action_c_front_angle_rad_[2]) * 180.0 / M_PI,
                (action_c_pulse_start_angle_rad_[3] - action_c_front_angle_rad_[3]) * 180.0 / M_PI,
                action_c_pulse_arm_ceiling_n_[0], action_c_pulse_arm_ceiling_n_[1],
                action_c_pulse_arm_ceiling_n_[2], action_c_pulse_arm_ceiling_n_[3]);
          if (angle_count > 0)
            {
              action_c_pulse_angle_rad_ =
                  angle_sum / static_cast<double>(angle_count);

              if (action_c_online_pressure_adaptation_enabled_)
                {
              // While thrust is ON, independently compensate both front-arm
              // angles.  A thrust pulse tends to straighten an arm; a bend
              // angle below 90 deg therefore produces a positive pressure
              // correction.  This replaces the old pair-average/model-angle
              // feedback which could drive both front pressures toward zero.
              const double dt = std::max(0.0, std::min(
                  0.1, (now - action_c_pressure_feedback_time_).toSec()));
              const double pressure_limit = std::min(
                  maximum_pressure_kpa_, action_c_max_pressure_kpa_);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (inchworm_front_[arm])
                  {
                    // Use the same relative target as the pulse-end logic.
                    // An absolute 90-deg target made a 137-deg starting arm
                    // deliberately exhaust until it had straightened by
                    // roughly 47 deg, even though this gait requests only an
                    // 8-deg extension.
                    const double desired_bend =
                        action_c_pulse_start_angle_rad_[arm] -
                        action_c_extension_target_rad_;
                    const double angle_error = desired_bend -
                        action_c_front_angle_rad_[arm];
                    // Estimate the local sign from *actual* pressure and the
                    // directly measured bend.  This makes the controller
                    // robust to a reversed Neuron mounting/sign without an
                    // arm-specific hand-tuned offset.  Ignore tiny changes,
                    // which are dominated by pressure and accelerometer noise.
                    if (action_c_pressure_gradient_initialized_)
                      {
                        const double delta_pressure = measured_pressure_[arm] -
                            action_c_previous_pressure_kpa_[arm];
                        const double delta_angle =
                            action_c_front_angle_rad_[arm] -
                            action_c_previous_bend_angle_rad_[arm];
                        if (std::abs(delta_pressure) >=
                                action_c_gradient_min_pressure_delta_kpa_ &&
                            std::abs(delta_angle) >=
                                action_c_gradient_min_angle_delta_rad_)
                          {
                            const double measured_sensitivity = delta_angle /
                                delta_pressure;
                            action_c_pressure_bend_sign_[arm] =
                                measured_sensitivity >= 0.0 ? 1.0 : -1.0;
                            // Limit one dynamic sample so a transient body
                            // acceleration cannot overwrite the local model.
                            const double bounded_sensitivity = std::max(-0.20,
                                std::min(0.20, measured_sensitivity));
                            action_c_dtheta_dpressure_[arm] =
                                (1.0 - action_c_model_update_alpha_) *
                                action_c_dtheta_dpressure_[arm] +
                                action_c_model_update_alpha_ *
                                bounded_sensitivity;
                          }
                      }
                    if (dt > 0.0 && std::abs(angle_error) >
                        action_c_pulse_angle_deadband_rad_)
                      {
                        const double requested_delta =
                            action_c_pressure_bend_sign_[arm] *
                            action_c_pulse_pressure_gain_kpa_rad_ * angle_error;
                        const double max_delta =
                            action_c_pulse_pressure_rate_kpa_s_ * dt;
                        const double pressure_delta = std::max(-max_delta,
                            std::min(max_delta, requested_delta));
                        action_c_front_pressure_[arm] = std::max(0.0, std::min(
                            pressure_limit,
                            action_c_front_pressure_[arm] + pressure_delta));
                        // Preserve the independent correction for a retry at
                        // the same thrust level, but never let it set a
                        // non-front arm's pressure.
                        action_c_pressure_bias_kpa_[arm] = std::max(
                            -action_c_max_pressure_kpa_, std::min(
                                action_c_max_pressure_kpa_,
                                action_c_pressure_bias_kpa_[arm] + pressure_delta));
                      }
                    action_c_previous_pressure_kpa_[arm] = measured_pressure_[arm];
                    action_c_previous_bend_angle_rad_[arm] =
                        action_c_front_angle_rad_[arm];
                    inchworm_pressure_[arm] = action_c_front_pressure_[arm];
                  }
              action_c_pressure_feedback_time_ = now;
                }
            }

          observeActionCPropulsion(now);
          const double pulse_on_duration = action_c_reference_pulse_mode_ &&
              action_c_pulse_search_enabled_ && !action_c_reference_single_pulse_
              ? hugmy::actionCSearchPulseDuration(action_c_pulse_duration_sec_,
                    action_c_thrust_n_, action_c_thrust_ramp_n_s_)
              : action_c_pulse_duration_sec_;
          const bool pulse_timeout =
              (now - action_c_pulse_start_).toSec() >=
                  pulse_on_duration;
          if (action_c_reference_pulse_mode_ || action_c_pressure_pull_enabled_)
            {
              std_msgs::Float32MultiArray diagnostic;
              diagnostic.data.resize(ARM_COUNT);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                diagnostic.data[arm] = inchworm_front_[arm]
                    ? action_c_front_angle_rad_[arm] * 180.0 / M_PI
                    : std::numeric_limits<float>::quiet_NaN();
              action_c_pulse_bend_pub_.publish(diagnostic);
            }
          const double pulse_travel = travel -
              action_c_pulse_start_travel_m_;
          const double pulse_tilt_change = std::hypot(
              wrapAngle(roll_ - action_c_pulse_start_roll_),
              wrapAngle(pitch_ - action_c_pulse_start_pitch_));
          const bool pressure_pull_mode = action_c_pressure_pull_enabled_;
          // Small ON recoil is diagnostic, not an early cut at the same
          // amplitude on every attempt. Score it AFTER the full pull/regrip.
          if (pressure_pull_mode && pulse_travel <= -action_c_max_reverse_travel_m_)
            ROS_WARN_THROTTLE(0.2, "Action C transient ON retreat %.1f mm; evaluating after pressure pull/regrip (hard stop %.1f mm)",
                -1000.0*pulse_travel, 1000.0*action_c_abort_reverse_travel_m_);
          if (pulse_travel <= -(pressure_pull_mode ? action_c_abort_reverse_travel_m_
                                                  : action_c_max_reverse_travel_m_))
            guard_reasons |= hugmy::ACTION_C_REVERSE_TRAVEL;
          if (pressure_pull_mode && !actionCRearReleased(now))
            {
              stopInchwormToGrasp(now, "Action C lost rear release during pulse");
              break;
            }
          if (pulse_tilt_change >= action_c_max_body_tilt_change_rad_)
            guard_reasons |= hugmy::ACTION_C_BODY_TILT;
          extension_guard = guard_reasons != 0;
          const bool flexion_pair_ready = action_c_propulsion_enabled_ && pressure_pull_mode &&
              action_c_propulsion_.allFlexing(inchworm_front_);
          if (((!action_c_reference_pulse_mode_ || pressure_pull_mode) && both_extended) ||
              flexion_pair_ready ||
              extension_guard || pulse_timeout)
            {
              action_c_pulse_had_extension_ = both_extended;
              action_c_pulse_overextended_ = extension_guard;
              action_c_optimizer_trial_had_extension_guard_ =
                  action_c_optimizer_trial_had_extension_guard_ ||
                  extension_guard;
              action_c_optimizer_trial_guard_reasons_ |= guard_reasons;
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (inchworm_front_[arm])
                  {
                    const double extension = action_c_pulse_start_angle_rad_[arm] -
                        action_c_front_angle_rad_[arm];
                    action_c_pulse_terminal_extension_rad_[arm] = extension;
                    // Only target overshoot reduces the next nominal thrust.
                    // Normal pair mismatch is handled by lagging-arm assist.
                    if (action_c_individual_extension_enabled_ && action_c_adaptive_burst_enabled_ &&
                        extension > action_c_extension_target_rad_ + action_c_extension_sync_deadband_rad_)
                      {
                        const double next_ceiling = std::max(0.0,
                            action_c_pulse_arm_nominal_n_[arm] - action_c_arm_learning_step_n_);
                        const double next_offset = next_ceiling - action_c_thrust_n_;
                        if (next_offset < action_c_arm_thrust_offset_n_[arm] - 1.0e-6)
                          {
                            action_c_arm_thrust_offset_n_[arm] = next_offset;
                            action_c_arm_calibration_changed_ = true;
                            ROS_INFO("Action C arm%zu target overshoot: next nominal %.2f N; measured extension %.1f deg",
                                     arm, next_ceiling, extension * 180.0 / M_PI);
                          }
                      }
                  }
              if (extension_guard)
                ROS_WARN("Action C guard reasons: over_extension=%d low_bend=%d extension_rate=%d pair_mismatch=%d reverse_travel=%d body_tilt=%d angle_unreliable=%d; pulse stopped",
                         (guard_reasons & hugmy::ACTION_C_OVER_EXTENSION) != 0,
                         (guard_reasons & hugmy::ACTION_C_LOW_BEND) != 0,
                         (guard_reasons & hugmy::ACTION_C_EXTENSION_RATE) != 0,
                         (guard_reasons & hugmy::ACTION_C_PAIR_MISMATCH) != 0,
                         (guard_reasons & hugmy::ACTION_C_REVERSE_TRAVEL) != 0,
                         (guard_reasons & hugmy::ACTION_C_BODY_TILT) != 0,
                         (guard_reasons & hugmy::ACTION_C_ANGLE_UNRELIABLE) != 0);
              // Store the terminal operating point before pressure is boosted
              // for the recoil/re-bend phase.
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (inchworm_front_[arm])
                  {
                    if (action_c_online_pressure_adaptation_enabled_ &&
                        action_c_last_pulse_valid_)
                      {
                        const double thrust_delta = action_c_thrust_n_ -
                            action_c_last_pulse_thrust_n_;
                        if (std::abs(thrust_delta) >= 0.25)
                          {
                            const double total_angle_delta =
                                action_c_front_angle_rad_[arm] -
                                action_c_last_pulse_bend_angle_rad_[arm];
                            const double pressure_angle_delta =
                                action_c_dtheta_dpressure_[arm] *
                                (measured_pressure_[arm] -
                                 action_c_last_pulse_pressure_kpa_[arm]);
                            const double measured_dtheta_dthrust =
                                (total_angle_delta - pressure_angle_delta) /
                                thrust_delta;
                            const double bounded_dtheta_dthrust = std::max(
                                -1.0, std::min(1.0,
                                    measured_dtheta_dthrust));
                            action_c_dtheta_dthrust_[arm] =
                                (1.0 - action_c_model_update_alpha_) *
                                action_c_dtheta_dthrust_[arm] +
                                action_c_model_update_alpha_ *
                                bounded_dtheta_dthrust;
                          }
                      }
                    action_c_last_pulse_pressure_kpa_[arm] =
                        measured_pressure_[arm];
                    action_c_last_pulse_bend_angle_rad_[arm] =
                        action_c_front_angle_rad_[arm];
                    action_c_front_pressure_[arm] = (action_c_coordination_enabled_ || action_c_propulsion_enabled_)
                        ? action_c_coordination_rebend_target_[arm] : std::min(
                        std::min(maximum_pressure_kpa_,
                                 action_c_max_pressure_kpa_),
                        std::max(base_pressure_kpa_,
                            std::max(action_c_front_pressure_[arm],
                                     measured_pressure_[arm]) +
                                action_c_rebend_pressure_boost_kpa_));
                    if (action_c_coordination_enabled_)
                      action_c_coordination_.startPressure(arm, now.toSec(), measured_pressure_[arm],
                          action_c_front_pressure_[arm]);
                  }
              action_c_last_pulse_thrust_n_ = action_c_thrust_n_;
              action_c_last_pulse_valid_ =
                  action_c_online_pressure_adaptation_enabled_;
              action_c_pressure_pull_active_ = pressure_pull_mode && !extension_guard;
              const double residual = action_c_pressure_pull_active_
                  ? std::min(action_c_pressure_pull_max_n_, action_c_pressure_pull_ratio_ * action_c_thrust_n_ +
                      (action_c_propulsion_enabled_ && !action_c_reference_pulse_mode_ ? action_c_propulsion_.extra_n : 0.0)) : 0.0;
              action_c_command_thrust_n_ = extension_guard ? 0.0 : approach(
                  action_c_command_thrust_n_, residual,
                  action_c_thrust_down_rate_n_s_);
              // The residual ceiling applies on the transition tick too;
              // do not leave one full-pulse sample in the flexion phase.
              if (action_c_pressure_pull_active_)
                action_c_command_thrust_n_ = std::min(action_c_command_thrust_n_, residual);
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                {
                  if (action_c_reference_pulse_mode_ && inchworm_front_[arm])
                    action_c_front_pressure_[arm] = base_pressure_kpa_;
                  inchworm_thrust_[arm] = inchworm_front_[arm]
                      ? std::min(inchworm_thrust_[arm],
                                 action_c_command_thrust_n_) : 0.0;
                  action_c_pressure_pull_ceiling_n_[arm] = std::min(residual, inchworm_thrust_[arm]);
                  if (action_c_propulsion_enabled_ && action_c_pressure_pull_active_ && inchworm_front_[arm])
                    {
                      action_c_propulsion_.startFlexion(arm, inchworm_thrust_[arm], residual);
                      action_c_pressure_pull_ceiling_n_[arm] = std::min(action_c_pressure_pull_ceiling_n_[arm],
                          action_c_propulsion_.flex_ceiling[arm]);
                    }
                  // Front pressure supplies flexion; rear stays released
                  // until the bounded powered pull is finished.
                  inchworm_pressure_[arm] = inchworm_front_[arm]
                      ? action_c_front_pressure_[arm]
                      : (action_c_pressure_pull_active_ ? 0.0 : base_pressure_kpa_);
                }
              publishCorrectionThrust(inchworm_thrust_);
              observeActionCPropulsion(now);
              // Lower the support slightly for re-placement/regrip. Keep the
              // optimized ON pressure unchanged and restore it next PREPARE.
              action_c_bottom_post_pulse_ = true;
              action_c_bottom_timing_.filling = false;
              action_c_bottom_timing_.peak_at = -1.; // early cutoff: no nominal-peak sample
              publishBottomTarget(actionCBottomTarget());
              ROS_INFO("Action C bottom after pulse: %.1f -> %.1f kPa until next PREPARE",
                       action_c_bottom_hold_kpa_, actionCBottomTarget());
              action_c_recovery_ready_start_ = ros::Time(0);
              ROS_WARN("Action C pulse OFF (%s): command %.2f/%.2f N, bend [%.1f %.1f %.1f %.1f] deg; waiting for recoil/re-bend",
                       extension_guard ? "extension guard" :
                           (both_extended ? "both arms extended" : "duration"),
                       action_c_command_thrust_n_, action_c_thrust_n_,
                       action_c_front_angle_rad_[0] * 180.0 / M_PI,
                       action_c_front_angle_rad_[1] * 180.0 / M_PI,
                       action_c_front_angle_rad_[2] * 180.0 / M_PI,
                       action_c_front_angle_rad_[3] * 180.0 / M_PI);
              enterState(ACTION_C_EVALUATE, now);
            }
        }
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.1,
          "Action C PULL %s: common command %.1f N, thrust [%.2f %.2f %.2f %.2f] N, angle reduction [%.2f %.2f %.2f %.2f] N, extension mismatch %.1f/%.1f deg, front [%d %d %d %d], roll reduction selector %.2f N (+: +Y, -: -Y), roll %.1f deg (target %.1f), rear command %.1f kPa, front pressure [%.1f %.1f %.1f %.1f] kPa, bend [%.1f %.1f %.1f %.1f] deg (mean %.1f/%.1f), P->bend sign [%.0f %.0f %.0f %.0f], mocap x %.4f->%.4f, signed travel %.2f/%.2f mm, pulse %d",
          action_c_direction_sign_x_ > 0.0 ? "+x" : "-x",
          action_c_command_thrust_n_,
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3],
          angle_thrust_reduction_n[0], angle_thrust_reduction_n[1],
          angle_thrust_reduction_n[2], angle_thrust_reduction_n[3],
          front_extension_mismatch_rad * 180.0 / M_PI,
          action_c_max_extension_mismatch_rad_ * 180.0 / M_PI,
          inchworm_front_[0], inchworm_front_[1],
          inchworm_front_[2], inchworm_front_[3],
          pulse_roll_trim_n,
          roll_ * 180.0 / M_PI,
          action_c_roll_target_rad_ * 180.0 / M_PI, rear_target,
          action_c_front_pressure_[0], action_c_front_pressure_[1],
          action_c_front_pressure_[2], action_c_front_pressure_[3],
          action_c_front_angle_rad_[0] * 180.0 / M_PI,
          action_c_front_angle_rad_[1] * 180.0 / M_PI,
          action_c_front_angle_rad_[2] * 180.0 / M_PI,
          action_c_front_angle_rad_[3] * 180.0 / M_PI,
          action_c_pulse_angle_rad_ * 180.0 / M_PI,
          action_c_target_angle_rad_ * 180.0 / M_PI,
          action_c_pressure_bend_sign_[0], action_c_pressure_bend_sign_[1],
          action_c_pressure_bend_sign_[2], action_c_pressure_bend_sign_[3],
          inchworm_start_position_[0], mocap_position_[0],
          1000.0 * travel, 1000.0 * inchworm_target_distance_m_,
          action_c_pulse_started_);
      }
      break;

    case ACTION_C_EVALUATE:
      {
      if (action_c_pressure_pull_active_)
        {
          updateActionCPressurePull(now);
          break;
        }
      publishBottomDirection(-1.0);
      publishBottomTarget(actionCBottomTarget());
      action_c_command_thrust_n_ = approach(
          action_c_command_thrust_n_, 0.0,
          action_c_thrust_down_rate_n_s_);
      bool rebend_ready = true;
      double worst_angle_error = 0.0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          inchworm_pressure_[arm] = inchworm_front_[arm]
              ? action_c_front_pressure_[arm] : base_pressure_kpa_;
          inchworm_thrust_[arm] = inchworm_front_[arm]
              ? std::min(inchworm_thrust_[arm],
                         action_c_command_thrust_n_) : 0.0;
          if (inchworm_front_[arm])
            {
              action_c_front_angle_rad_[arm] = measuredArmBendAngle(arm);
              const double rebound = action_c_front_angle_rad_[arm] -
                  action_c_pulse_min_angle_rad_[arm];
              rebend_ready = rebend_ready &&
                  (rebound >= action_c_rebend_target_rad_ ||
                   action_c_front_angle_rad_[arm] >=
                       action_c_target_angle_rad_ -
                           action_c_angle_tolerance_rad_);
              worst_angle_error = std::max(worst_angle_error,
                  std::abs(action_c_target_angle_rad_ -
                           action_c_front_angle_rad_[arm]));
            }
        }
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      const bool bottom_ready = std::isfinite(bottom_pressure_kpa_) &&
          std::abs(bottom_pressure_kpa_ - actionCBottomTarget()) <=
              pressure_tolerance_kpa_;
      const bool thrust_off = action_c_command_thrust_n_ <= 0.01;
      const bool recovery_condition = bottom_ready && thrust_off &&
          (action_c_reference_pulse_mode_
              ? (elapsed >= action_c_reference_off_sec_ && pressureReached(inchworm_pressure_))
              : rebend_ready);
      if (recovery_condition)
        {
          if (action_c_recovery_ready_start_.isZero())
            action_c_recovery_ready_start_ = now;
        }
      else
        action_c_recovery_ready_start_ = ros::Time(0);

      const bool recovery_complete =
          !action_c_recovery_ready_start_.isZero() &&
          (now - action_c_recovery_ready_start_).toSec() >=
              action_c_recovery_settle_sec_;
      // Reference OFF must wait for pneumatic regrip, not inherit the short
      // legacy angle-rebound timeout. Thrust is already zero while waiting.
      const double recovery_wait_limit = action_c_reference_pulse_mode_
          ? std::max(inchworm_anchor_timeout_sec_,
                     action_c_reference_off_sec_ + action_c_recovery_settle_sec_)
          : action_c_recovery_timeout_sec_;
      const bool recovery_timeout = elapsed >= recovery_wait_limit;
      const double pulse_travel = travel - action_c_pulse_start_travel_m_;
      ROS_INFO_THROTTLE(0.1,
          "Action C RECOIL: thrust %.2f N, bottom %.1f/%.1f kPa hold, rear command %.1f kPa (no pressure wait), rebend %d, bend [%.1f %.1f %.1f %.1f] deg, pulse travel %.2f/%.2f mm",
          action_c_command_thrust_n_, bottom_pressure_kpa_,
          actionCBottomTarget(), base_pressure_kpa_,
          rebend_ready,
          action_c_front_angle_rad_[0] * 180.0 / M_PI,
          action_c_front_angle_rad_[1] * 180.0 / M_PI,
          action_c_front_angle_rad_[2] * 180.0 / M_PI,
          action_c_front_angle_rad_[3] * 180.0 / M_PI,
          1000.0 * pulse_travel, 1000.0 * inchworm_target_distance_m_);
      if (!recovery_complete && !recovery_timeout) break;
      action_c_pulse_recoil_complete_ = recovery_complete;

      const bool controlled_success = recovery_complete &&
          action_c_pulse_had_extension_ &&
          !action_c_pulse_overextended_ &&
          pulse_travel >= inchworm_target_distance_m_;
      if (action_c_reference_pulse_mode_)
        {
          // Extension/rebound remain diagnostics, not a condition that forces
          // amplitude learning or repeats a failed fixed pulse indefinitely.
          action_c_success_ = controlled_success;
          enterState(ACTION_C_REGRIP, now);
          break;
        }
      if (action_c_adaptive_burst_enabled_ && !controlled_success)
        {
          // T_pulse is an optimizer variable and must stay fixed throughout
          // one trial. Record unsuccessful extension/rebound/travel as a poor
          // outcome instead of changing thrust inside the candidate.
          action_c_success_ = false;
          ROS_WARN("Action C optimizer pulse failed at fixed candidate [T %.2f N, bottom %.1f kPa, burst %d]: extension=%d guard=%d recovery=%d travel=%.2f mm",
                   action_c_thrust_n_, action_c_bottom_hold_kpa_,
                   action_c_optimizer_burst_pulses_,
                   action_c_pulse_had_extension_,
                   action_c_pulse_overextended_, recovery_complete,
                   1000.0 * pulse_travel);
          enterState(ACTION_C_REGRIP, now);
        }
      else if (controlled_success)
        {
          action_c_success_ = true;
          ROS_INFO("ACTION C SUCCESS: controlled extension and rebound retained %.2f/%.2f mm at %.1f N",
                   1000.0 * pulse_travel,
                   1000.0 * inchworm_target_distance_m_,
                   action_c_thrust_n_);
          enterState(ACTION_C_REGRIP, now);
        }
      else if (action_c_pulse_overextended_)
        {
          if (action_c_thrust_n_ - action_c_thrust_step_n_ >=
              action_c_min_thrust_n_ + 1.0e-6)
            {
              action_c_thrust_n_ = std::max(action_c_min_thrust_n_,
                  action_c_thrust_n_ - action_c_thrust_step_n_);
              action_c_angle_retry_ = 0;
              ROS_WARN("Action C extension guard fired; reducing next pulse to %.1f N",
                       action_c_thrust_n_);
              enterState(ACTION_C_PREPARE, now);
            }
          else
            {
              action_c_success_ = false;
              ROS_ERROR("ACTION C FAILED safely: extension guard fired at minimum %.1f N",
                        action_c_min_thrust_n_);
              enterState(ACTION_C_REGRIP, now);
            }
        }
      else if (!action_c_pulse_had_extension_)
        {
          if (action_c_thrust_n_ + action_c_thrust_step_n_ <=
              action_c_max_thrust_n_ + 1.0e-6)
            {
              action_c_thrust_n_ = std::min(action_c_max_thrust_n_,
                  action_c_thrust_n_ + action_c_thrust_step_n_);
              ROS_WARN("Action C pulse did not extend both arms; advancing gently to %.1f N",
                       action_c_thrust_n_);
              enterState(ACTION_C_PREPARE, now);
            }
          else
            {
              action_c_success_ = false;
              ROS_ERROR("ACTION C FAILED: both-arm extension was not obtained through %.1f N",
                        action_c_max_thrust_n_);
              enterState(ACTION_C_REGRIP, now);
            }
        }
      else if (!recovery_complete &&
               action_c_angle_retry_ < action_c_max_angle_retries_)
        {
          // Extension occurred but the pneumatic recoil was insufficient.
          // Learn an independent pressure correction and repeat at the same
          // thrust instead of making the dangerous response of increasing it.
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                const double angle_error = action_c_target_angle_rad_ -
                    action_c_front_angle_rad_[arm];
                const double correction = action_c_pressure_bend_sign_[arm] *
                    action_c_pressure_gain_kpa_rad_ * angle_error;
                action_c_pressure_bias_kpa_[arm] = std::max(
                    -action_c_max_pressure_kpa_, std::min(
                        action_c_max_pressure_kpa_,
                        action_c_pressure_bias_kpa_[arm] + correction));
              }
          ++action_c_angle_retry_;
          ROS_WARN("Action C rebound timeout; pressure retry %d/%d at the same %.1f N",
                   action_c_angle_retry_, action_c_max_angle_retries_,
                   action_c_thrust_n_);
          enterState(ACTION_C_PREPARE, now);
        }
      else if (action_c_thrust_n_ + action_c_thrust_step_n_ <=
               action_c_max_thrust_n_ + 1.0e-6 &&
               worst_angle_error <
                   action_c_max_error_before_thrust_increase_rad_)
        {
          action_c_thrust_n_ = std::min(action_c_max_thrust_n_,
              action_c_thrust_n_ + action_c_thrust_step_n_);
          action_c_angle_retry_ = 0;
          ROS_WARN("Action C controlled cycle had no travel; advancing to %.1f N",
                   action_c_thrust_n_);
          enterState(ACTION_C_PREPARE, now);
        }
      else
        {
          action_c_success_ = false;
          ROS_ERROR("ACTION C FAILED: no retained travel after controlled extension/rebound; restoring grasp");
          enterState(ACTION_C_REGRIP, now);
        }
      }
      break;

    case ACTION_C_REGRIP:
      inchworm_pressure_.fill(base_pressure_kpa_);
      inchworm_thrust_.fill(0.0);
      publishBottomTarget(actionCBottomTarget());
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) ||
          elapsed >= inchworm_anchor_timeout_sec_)
        {
          if (action_c_reference_pulse_mode_ &&
              (action_c_reference_single_pulse_ || action_c_pulse_overextended_ ||
               !action_c_pulse_recoil_complete_ || !pressureReached(inchworm_pressure_)))
            {
              ROS_WARN("Action C REFERENCE done: retained x %.2f mm, extension=%d, guard=%d, OFF/regrip=%d; stopping without automatic retry",
                       1000.0*(travel-action_c_pulse_start_travel_m_),
                       action_c_pulse_had_extension_, action_c_pulse_overextended_,
                       action_c_pulse_recoil_complete_ && pressureReached(inchworm_pressure_));
              stopInchwormToGrasp(now, "reference pulse complete or guarded; inspect diagnostics before next command");
              break;
            }
          const double pulse_retained_travel = action_c_pulse_started_
              ? travel - action_c_pulse_start_travel_m_ : 0.0;
          const double retained_threshold =
              action_c_regrip_min_retained_ratio_ *
              inchworm_target_distance_m_;
          if (action_c_success_ &&
              pulse_retained_travel < retained_threshold)
            {
              ROS_WARN("ACTION C retained-motion check failed: %.2f mm < %.2f mm after REGRIP",
                       1000.0 * pulse_retained_travel,
                       1000.0 * retained_threshold);
              action_c_success_ = false;
            }
          ROS_INFO("Action C regrip complete: %s, retained travel %.2f mm (required %.2f mm)",
                   action_c_success_ ? "SUCCESS" : "FAILED",
                   1000.0 * pulse_retained_travel,
                   1000.0 * retained_threshold);
          if (action_c_adaptive_burst_enabled_)
            {
              if (!action_c_optimizer_cycle_accounted_)
                {
                  const bool valid_progress = action_c_pulse_started_ &&
                      action_c_pulse_recoil_complete_ &&
                      !action_c_pulse_overextended_ &&
                      pressureReached(inchworm_pressure_);
                  action_c_propulsion_.regrip(valid_progress);
                  if (action_c_coordination_enabled_)
                    action_c_coordination_.observePulse(inchworm_front_,
                        action_c_pulse_terminal_extension_rad_, action_c_extension_target_rad_,
                        valid_progress && action_c_success_);
                  action_c_optimizer_retained_progress_m_ +=
                      hugmy::actionCMeasuredProgress(pulse_retained_travel,
                                                    valid_progress);
                  if (valid_progress) ++action_c_optimizer_valid_pulses_;
                  bool tracking_ready = valid_progress &&
                      action_c_cycle_peak_tilt_rad_ < action_c_posture_recovery_trigger_rad_ &&
                      action_c_cycle_peak_rate_rad_s_ < action_c_posture_rate_trigger_rad_s_;
                  double minimum_final_extension = std::numeric_limits<double>::infinity();
                  double maximum_final_extension = -std::numeric_limits<double>::infinity();
                  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                    if (inchworm_front_[arm])
                      {
                        const double extension = action_c_pulse_terminal_extension_rad_[arm];
                        tracking_ready = tracking_ready && std::abs(extension - action_c_extension_target_rad_) <=
                            action_c_extension_sync_deadband_rad_;
                        minimum_final_extension = std::min(minimum_final_extension, extension);
                        maximum_final_extension = std::max(maximum_final_extension, extension);
                      }
                  tracking_ready = tracking_ready && std::isfinite(minimum_final_extension) &&
                      maximum_final_extension - minimum_final_extension <= action_c_extension_sync_deadband_rad_;
                  if (tracking_ready) ++action_c_optimizer_tracking_pulses_;
                  if (action_c_individual_extension_enabled_)
                    {
                      // Increase only ONE saturated, lagging arm after a safe
                      // complete cycle. Do not reward an opposite angle response.
                      size_t lagging = ARM_COUNT;
                      double least_extension = action_c_extension_target_rad_;
                      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                        if (inchworm_front_[arm] && hugmy::actionCCanRaiseArm(
                            valid_progress, action_c_optimizer_trial_had_extension_guard_,
                            action_c_cycle_peak_tilt_rad_ < action_c_posture_recovery_trigger_rad_ &&
                            action_c_cycle_peak_rate_rad_s_ < action_c_posture_rate_trigger_rad_s_,
                            action_c_optimizer_peak_extension_mismatch_rad_,
                            action_c_thrust_reprobe_max_mismatch_rad_,
                            action_c_pulse_terminal_extension_rad_[arm],
                            action_c_pulse_start_angle_rad_[arm] - action_c_pulse_min_angle_rad_[arm],
                            action_c_extension_target_rad_, action_c_extension_sync_deadband_rad_,
                            action_c_pulse_peak_thrust_n_[arm], action_c_pulse_arm_ceiling_n_[arm]) &&
                            action_c_pulse_terminal_extension_rad_[arm] < least_extension)
                          {
                            lagging = arm;
                            least_extension = action_c_pulse_terminal_extension_rad_[arm];
                          }
                      if (lagging < ARM_COUNT)
                        {
                          const double next_ceiling = std::min(
                              std::min(action_c_max_thrust_n_, action_c_thrust_safety_.hard_max_n),
                              action_c_pulse_arm_nominal_n_[lagging] + action_c_arm_learning_step_n_);
                          const double next_offset = next_ceiling - action_c_thrust_n_;
                          if (next_offset > action_c_arm_thrust_offset_n_[lagging] + 1.0e-6)
                            {
                              action_c_arm_thrust_offset_n_[lagging] = next_offset;
                              action_c_arm_calibration_changed_ = true;
                              ROS_INFO("Action C arm%zu safely lagging at ceiling: next nominal %.2f N (extension %.1f deg)",
                                       lagging, next_ceiling, least_extension * 180.0 / M_PI);
                            }
                        }
                    }
                  ROS_INFO("Action C optimizer measurement: valid=%d, retained %.2f mm (sub-goal motion is scored; unsafe/incomplete positive motion is excluded)",
                           valid_progress, 1000.0 * pulse_retained_travel);
                  if (action_c_success_ && valid_progress)
                    {
                      ++action_c_optimizer_successful_pulses_;
                    }
                  else
                    {
                      // Keep the missed-stroke penalty independent of the
                      // continuous, validated displacement measurement.
                      ++action_c_optimizer_failed_pulses_;
                    }
                  action_c_optimizer_cycle_accounted_ = true;
                }
              ++action_c_optimizer_completed_pulses_;
              ++action_c_burst_pulse_count_;
            }
          if (action_c_success_)
            {
              const double end_tilt =
                  std::abs(wrapAngle(roll_ - target_roll_));
              const double observed_growth = std::max(0.0,
                  end_tilt - action_c_cycle_start_tilt_rad_);
              action_c_estimated_tilt_growth_rad_ =
                  (1.0 - action_c_tilt_growth_alpha_) *
                      action_c_estimated_tilt_growth_rad_ +
                  action_c_tilt_growth_alpha_ * observed_growth;
              action_c_last_retained_progress_m_ = pulse_retained_travel;

              const double predicted_next_tilt = end_tilt + std::max(
                  0.0, action_c_estimated_tilt_growth_rad_);
              const double signed_goal_remaining =
                  action_c_direction_sign_x_ *
                      (action_c_absolute_target_x_m_ - mocap_position_[0]);
              const bool target_reached = signed_goal_remaining <=
                  inchworm_min_cycle_progress_m_;
              const bool predicted_posture_limit =
                  predicted_next_tilt >=
                      action_c_posture_recovery_trigger_rad_;
              ROS_INFO("Action C optimizer sample %d/%d: retained %.2f mm, roll start/end/peak %.1f/%.1f/%.1f deg, peak roll rate %.2f rad/s, predicted next %.1f deg, fixed candidate [T %.2f N, bottom %.1f kPa, burst %d]",
                       action_c_optimizer_completed_pulses_,
                       action_c_optimizer_burst_pulses_,
                       1000.0 * pulse_retained_travel,
                       action_c_cycle_start_tilt_rad_ * 180.0 / M_PI,
                       end_tilt * 180.0 / M_PI,
                       action_c_cycle_peak_tilt_rad_ * 180.0 / M_PI,
                       action_c_cycle_peak_rate_rad_s_,
                       predicted_next_tilt * 180.0 / M_PI,
                       action_c_thrust_n_, action_c_bottom_hold_kpa_,
                       action_c_optimizer_burst_pulses_);
              if (action_c_adaptive_burst_enabled_ && !target_reached &&
                  predicted_posture_limit)
                {
                  // Keep this prediction as an optimizer diagnostic only.
                  // Recovery is defined by the actual Spinal roll crossing
                  // the configured 8-deg threshold; using predicted growth
                  // here caused handoffs while measured roll was still safe.
                  ROS_WARN("Action C predicts %.1f deg on the next pulse, but measured roll is %.1f deg; continuing until the actual %.1f-deg recovery threshold",
                           predicted_next_tilt * 180.0 / M_PI,
                           end_tilt * 180.0 / M_PI,
                           action_c_posture_recovery_trigger_rad_ *
                               180.0 / M_PI);
                }
              enterState(SETTLE, now);
            }
          else if (action_c_adaptive_burst_enabled_)
            {
              // A failed/reversed stride is still a valid low-score sample.
              // SETTLE closes this candidate early and lets the optimizer
              // choose a different u instead of heuristically increasing it.
              enterState(SETTLE, now);
            }
          else
            {
              inchworm_active_ = false;
              enterState(GRASP, now);
            }
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
      if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_)
        publishBottomTarget(actionCBottomTarget());
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !action_c_boundary_ready_)
        ROS_INFO_THROTTLE(1.0,
            "Action C boundary assessment: bottom %.1f/%.1f kPa held, regrasp=%d, roll %.2f deg, rate %.3f rad/s; waiting before next pulse",
            bottom_pressure_kpa_, actionCBottomTarget(),
            pressureReached(inchworm_pressure_),
            wrapAngle(roll_ - target_roll_) * 180.0 / M_PI,
            body_angular_velocity_[0]);
      // Do not advance into the next pulse while the boundary assessment is
      // pending; otherwise rapid state transitions can starve confirmation.
      if (inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
          !action_c_boundary_ready_ && elapsed >= action_c_prepare_timeout_sec_)
        {
          stopInchwormToGrasp(now, "Action C boundary settling timeout");
          break;
        }
      if (elapsed >= inchworm_settle_duration_sec_ &&
          (!(inchworm_action_c_enabled_ && !inchworm_action_d_enabled_) ||
           action_c_boundary_ready_))
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
          const bool action_c_optimizer =
              inchworm_action_c_enabled_ && !inchworm_action_d_enabled_ &&
              action_c_adaptive_burst_enabled_;
          const bool candidate_complete = action_c_optimizer &&
              (action_c_optimizer_completed_pulses_ >= action_c_optimizer_burst_pulses_ ||
               action_c_arm_calibration_changed_);
          if (action_c_optimizer && action_c_optimizer_trial_active_ &&
              (candidate_complete || no_progress || target_reached ||
               cycle_limit))
            finishActionCOptimizerTrial(false,
                target_reached ? "goal reached" :
                (cycle_limit ? "cycle limit" :
                 (no_progress ? "no retained progress" :
                  "configured burst complete")));

          if (!target_reached && !cycle_limit &&
              (action_c_optimizer || !no_progress))
            {
              // A normal positive-progress cycle continues the same candidate
              // until N_burst samples have completed. Candidate completion or
              // early no-progress termination starts the newly selected one.
              if (action_c_optimizer &&
                  (candidate_complete || no_progress))
                beginActionCOptimizerTrial(now);
              startInchwormCycle(now);
            }
          else
            {
              if (no_progress && !target_reached)
                ROS_WARN("Inchworm cycle retained only %.2f mm%s",
                         1000.0 * cycle_travel,
                         action_c_optimizer
                             ? "; candidate scored and optimizer advanced"
                             : "; repetition stopped");
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
  return result;
}

double AttitudePressureController::recoveryRotorLever(size_t arm, double bend) const
{
      const Vector3 position = rotorPositionFromAngle(arm, bend);
      // Same local-X cant and rotor signs as hugmy2.urdf.xacro. Neuron bend
      // is measured BELOW the rotor, so add cant here, not to the sensor angle.
      const double direction = (arm == 0 || arm == 2) ? 1.0 : -1.0;
      const Vector3 unit_force = rotateLocalToBody(rotateAboutLocalY(
          {{0.0, direction * std::sin(recovery_rotor_cant_rad_),
            std::cos(recovery_rotor_cant_rad_)}}, std::max(0.0, std::min(2.966, bend))), arm_yaw_[arm]);
      Vector3 lever;
      for (size_t axis = 0; axis < 3; ++axis) lever[axis] = position[axis] - contact_position_[axis];
      return cross(lever, unit_force)[0] * correction_axis_[0];
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
