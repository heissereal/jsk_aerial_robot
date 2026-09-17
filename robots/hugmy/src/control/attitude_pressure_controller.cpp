#include <hugmy/control/attitude_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <tf/transform_datatypes.h>

namespace
{
constexpr double GRAVITY = 9.80665;

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
  pnh_.param("correction_trigger_angle_rad", correction_trigger_angle_, 0.12);
  pnh_.param("correction_stop_angle_rad", correction_stop_angle_, 0.04);
  pnh_.param("correction_max_angular_rate_rad_s", correction_max_angular_rate_, 0.8);
  pnh_.param("correction_thrust_duration_sec", correction_thrust_duration_, 0.20);
  pnh_.param("prepare_timeout_sec", prepare_timeout_, 30.0);
  pnh_.param("prepare_stable_duration_sec", prepare_stable_duration_, 0.30);
  pnh_.param("recover_timeout_sec", recover_timeout_, 5.0);
  pnh_.param("cooldown_duration_sec", cooldown_duration_, 1.0);
  pnh_.param("pressure_tolerance_kpa", pressure_tolerance_kpa_, 1.0);
  pnh_.param("arm_angle_tolerance_rad", arm_angle_tolerance_, 0.20);
  pnh_.param("arm_gyro_tolerance_rad_s", arm_gyro_tolerance_, 0.20);
  pnh_.param("correction_planning_thrust_n", correction_planning_thrust_n_, 2.0);
  pnh_.param("correction_min_thrust_n", correction_min_thrust_n_, 2.0);
  pnh_.param("correction_max_thrust_n", correction_max_thrust_n_, 3.0);
  pnh_.param("correction_hold_min_thrust_n", correction_hold_min_thrust_n_, 0.8);
  pnh_.param("correction_motion_start_rate_rad_s", correction_motion_start_rate_, 0.05);
  pnh_.param("correction_motion_stop_rate_rad_s", correction_motion_stop_rate_, 0.02);
  pnh_.param("correction_slowdown_angle_rad", correction_slowdown_angle_, 0.10);
  pnh_.param("correction_thrust_down_rate_n_s", correction_thrust_down_rate_, 1.5);
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
  pnh_.param("inchworm_reach_thrust_n", inchworm_reach_thrust_n_, 2.0);
  pnh_.param("inchworm_reach_max_thrust_n", inchworm_reach_max_thrust_n_, 6.0);
  pnh_.param("inchworm_reach_angle_kp_n_rad", inchworm_reach_angle_kp_n_rad_, 45.0);
  pnh_.param("inchworm_reach_min_pressure_kpa", inchworm_reach_min_pressure_kpa_, 10.0);
  pnh_.param("inchworm_reach_pressure_ramp_kpa_s", inchworm_reach_pressure_ramp_kpa_s_, 5.0);
  pnh_.param("inchworm_min_usable_reach_angle_rad", inchworm_min_usable_reach_angle_rad_, 0.0349066);
  pnh_.param("inchworm_rear_lift_thrust_n", inchworm_rear_lift_thrust_n_, 3.0);
  pnh_.param("inchworm_rear_lift_max_thrust_n", inchworm_rear_lift_max_thrust_n_, 7.0);
  pnh_.param("inchworm_thrust_ramp_n_s", inchworm_thrust_ramp_n_s_, 1.0);
  pnh_.param("inchworm_default_distance_m", inchworm_default_distance_m_, 0.002);
  pnh_.param("inchworm_stroke_distance_m", inchworm_stroke_distance_m_, 0.002);
  pnh_.param("inchworm_min_cycle_progress_m", inchworm_min_cycle_progress_m_, 0.0002);
  pnh_.param("inchworm_max_cycles", inchworm_max_cycles_, 10);
  pnh_.param("inchworm_reach_angle_rad", inchworm_reach_angle_rad_, 0.0872665);
  pnh_.param("inchworm_target_bend_angle_rad", inchworm_target_bend_angle_rad_, 0.0872665);
  pnh_.param("inchworm_target_straightening_angle_rad",
             inchworm_target_straightening_angle_rad_, -1.0);
  pnh_.param("inchworm_body_tilt_target_rad", inchworm_body_tilt_target_rad_, 0.0174533);
  pnh_.param("inchworm_angular_rate_fault_delay_sec",
             inchworm_angular_rate_fault_delay_sec_, 0.20);
  pnh_.param("inchworm_mocap_timeout_sec", inchworm_mocap_timeout_sec_, 0.20);
  pnh_.param("use_mocap_attitude_for_inchworm",
             use_mocap_attitude_for_inchworm_, false);
  pnh_.param("inchworm_release_timeout_sec", inchworm_release_timeout_sec_, 10.0);
  pnh_.param("inchworm_reach_duration_sec", inchworm_reach_duration_sec_, 10.0);
  pnh_.param("inchworm_anchor_timeout_sec", inchworm_anchor_timeout_sec_, 8.0);
  pnh_.param("inchworm_pull_timeout_sec", inchworm_pull_timeout_sec_, 5.0);
  pnh_.param("inchworm_settle_duration_sec", inchworm_settle_duration_sec_, 0.5);
  pnh_.param("inchworm_motion_start_speed_m_s", inchworm_motion_start_speed_m_s_, 0.001);
  pnh_.param("inchworm_motion_stop_speed_m_s", inchworm_motion_stop_speed_m_s_, 0.0002);
  pnh_.param("inchworm_motion_stop_timeout_sec", inchworm_motion_stop_timeout_sec_, 0.3);
  pnh_.param("inchworm_progress_epsilon_m", inchworm_progress_epsilon_m_, 0.00025);
  pnh_.param("inchworm_progress_regression_m", inchworm_progress_regression_m_, 0.00020);
  pnh_.param("inchworm_action_b_enabled", inchworm_action_b_enabled_, false);
  pnh_.param("inchworm_action_c_enabled", inchworm_action_c_enabled_, false);
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
  pnh_.param("action_c_min_thrust_n", action_c_min_thrust_n_, 5.0);
  pnh_.param("action_c_max_thrust_n", action_c_max_thrust_n_, 8.0);
  pnh_.param("action_c_thrust_step_n", action_c_thrust_step_n_, 0.5);
  pnh_.param("action_c_target_angle_rad", action_c_target_angle_rad_, M_PI / 2.0);
  pnh_.param("action_c_angle_tolerance_rad", action_c_angle_tolerance_rad_, 15.0 * M_PI / 180.0);
  pnh_.param("action_c_pressure_gain_kpa_rad", action_c_pressure_gain_kpa_rad_, 8.0);
  pnh_.param("action_c_rear_pressure_kpa", action_c_rear_pressure_kpa_, 10.0);
  pnh_.param("action_c_pulse_duration_sec", action_c_pulse_duration_sec_, 0.30);
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
  prepare_max_straighten_angle_ = std::max(
      prepare_min_straighten_angle_, prepare_max_straighten_angle_);
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
          neuron_stamp_[arm] = now;
          break;
        }
}//imu callback stampいるのか？

void AttitudePressureController::pressureCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != ARM_COUNT) return;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm) if (!std::isfinite(msg->data[arm])) return;
  std::copy(msg->data.begin(), msg->data.end(), measured_pressure_.begin());
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
  if (std::isfinite(msg->data)) bottom_pressure_kpa_ = msg->data;
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
  if (!mocap_received_)
    {
      ROS_ERROR("Cannot start inchworm motion before receiving mocap/pose");
      return;
    }
  startInchwormStep({{msg->vector.x / norm, msg->vector.y / norm}},
                    msg->vector.z, ros::Time::now());
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
  grasp_baseline_valid_ = false;
  inchworm_active_ = false;
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

  if (state_machine_enabled_ && enabled_)
    {
      PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
      const bool body_imu_fresh = imu_received_ && (now - imu_stamp_).toSec() <= imu_timeout_sec_;
      bool pressure_valid = true;
      for (const double pressure : measured_pressure_) pressure_valid = pressure_valid && std::isfinite(pressure) && pressure < pressure_safety_limit_kpa_;
      if (rocking_gait_enabled_)
        pressure_valid = pressure_valid && std::isfinite(bottom_pressure_kpa_) &&
            bottom_pressure_kpa_ < pressure_safety_limit_kpa_;
      bool neurons_fresh = true;
      for (const auto& stamp : neuron_stamp_) neurons_fresh = neurons_fresh && !stamp.isZero() && (now - stamp).toSec() <= neuron_timeout_sec_;

      std_msgs::UInt8 state_msg;
      state_msg.data = state_;
      state_pub_.publish(state_msg);

      if (!body_imu_fresh || !pressure_valid || !neurons_fresh)
        {
          publishCorrectionThrust(zero_thrust);
          if (!inchworm_active_ && state_ != GRASP && state_ != RECOVER)
            {
              ROS_ERROR("Perching correction cancelled by stale body IMU, Neuron IMU, or pressure");
              enterState(RECOVER, now);
            }
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
                  "mocap=%d age=%.3f/%.3f s, angular_rate=%.3f/%.3f rad/s",
                  state_, body_imu_fresh, body_imu_age, imu_timeout_sec_,
                  pressure_valid, neurons_fresh, oldest_neuron_age,
                  neuron_timeout_sec_, mocap_fresh, mocap_age,
                  inchworm_mocap_timeout_sec_, angular_rate,
                  correction_max_angular_rate_);
              stopInchwormToGrasp(now,
                  "stale data, invalid pressure, mocap timeout, or angular-rate limit");
            }
          else
            {
              updateInchworm(now, dt, pressure_valid, neurons_fresh);
              return;
            }
        }

      const double attitude_error = std::hypot(roll_error, pitch_error);
      const double angular_rate = std::hypot(body_angular_velocity_[0], body_angular_velocity_[1]);
      switch (state_)
        {
        case GRASP:
          publishCorrectionThrust(zero_thrust);
          if (rocking_gait_enabled_)
            publishBottomTarget(rocking_bottom_low_kpa_);
          publishPressureTarget(PressureArray{{base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_, base_pressure_kpa_}}, maximum_step);
          {
          bool grasp_pressure_ready = pressure_valid;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            grasp_pressure_ready = grasp_pressure_ready &&
                std::abs(measured_pressure_[arm] - base_pressure_kpa_) <=
                pressure_tolerance_kpa_;
          if (!grasp_pressure_ready)
            ROS_INFO_THROTTLE(1.0,
                "GRASP waiting for all arm pressures to reach %.1f +/- %.1f kPa",
                base_pressure_kpa_, pressure_tolerance_kpa_);
          if (target_received_ && body_imu_fresh && pressure_valid && neurons_fresh &&
              grasp_pressure_ready && attitude_error >= correction_trigger_angle_ &&
              (last_correction_end_.isZero() ||
               (now - last_correction_end_).toSec() >= cooldown_duration_))
            {
              const Vector2 desired_moment = desiredThrustMoment();
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
            }
          }
          break;

        case PREPARE:
          {
          publishCorrectionThrust(fixed_arm_test_enabled_ ? zero_thrust
                                                          : prepare_shape_thrust_);
          publishPressureTarget(prepare_pressure_, maximum_step);
          const double correction_omega =
              body_angular_velocity_[0] * correction_axis_[0] +
              body_angular_velocity_[1] * correction_axis_[1];
          const double angle_scale = std::max(0.0, std::min(1.0,
              (attitude_error - correction_stop_angle_) /
              std::max(1e-6, correction_slowdown_angle_ -
                                correction_stop_angle_)));
          const double rate_scale = std::max(0.0, std::min(1.0,
              (correction_max_angular_rate_ - correction_omega) /
              std::max(1e-6, correction_max_angular_rate_ -
                                correction_motion_start_rate_)));
          const double feedback_scale = std::min(angle_scale, rate_scale);
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
                  if (target_error > prepare_angle_tolerance_)
                    {
                      // Body motion is detected from the Spinal IMU, not from
                      // Neuron arm motion. Hold the pressure and reduce thrust
                      // only as attitude error or correction angular rate says
                      // that the target is approaching.
                      if (body_motion_detected_)
                        {
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
                      else if (prepare_pressure_[arm] > minimum_prepare_pressure_kpa_ + 1e-6)
                        {
                          prepare_pressure_[arm] = std::max(minimum_prepare_pressure_kpa_,
                              prepare_pressure_[arm] - prepare_pressure_ramp_rate_ * dt);
                          prepare_shape_thrust_[arm] = std::max(0.0,
                              prepare_shape_thrust_[arm] - prepare_thrust_ramp_rate_ * dt);
                        }
                      else if (measured_pressure_[arm] <=
                               minimum_prepare_pressure_kpa_ + pressure_tolerance_kpa_)
                        prepare_shape_thrust_[arm] = std::min(prepare_max_thrust_n_,
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
          publishArmShapeDebug(relative_angle);

          if (!body_motion_detected_ &&
              correction_omega >= correction_motion_start_rate_)
            {
              body_motion_detected_ = true;
              motion_hold_thrust_ = prepare_shape_thrust_;
              body_motion_last_seen_ = now;
              ROS_INFO("Spinal detected correction-direction body motion at %.3f rad/s; holding thrust",
                       correction_omega);
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
                  body_motion_detected_ = false;
                  ROS_INFO("Correction-direction body motion stopped; resuming minimum-thrust search");
                }
            }

          // PREPARE thrust may already have corrected the body attitude.
          // Stop increasing it and begin regripping as soon as the attitude
          // enters the stop band; RECOVER tapers the thrust instead of
          // dropping it in one cycle.
          if (!fixed_arm_test_enabled_ &&
              attitude_error <= correction_stop_angle_)
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
              for (size_t arm = 0; arm < ARM_COUNT; ++arm)
                if (prepare_arm_active_[arm])
                  output_limited = output_limited &&
                      prepare_shape_thrust_[arm] >= prepare_max_thrust_n_ - 0.01;
              const bool shape_limited = output_limited &&
                  (now - last_angle_progress_).toSec() >= prepare_progress_timeout_;
              const bool timed_out = (now - state_start_).toSec() >= prepare_timeout_;
              geometry_ready = target_angle_reached || shape_limited || timed_out;
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
                  (attitude_error - correction_stop_angle_) /
                  std::max(1e-6, correction_slowdown_angle_ -
                                    correction_stop_angle_)));
              for (double& thrust : correction_thrust_)
                if (thrust > 0.0)
                  thrust = std::max(correction_hold_min_thrust_n_,
                      thrust - correction_thrust_down_rate_ *
                                   (1.0 - angle_scale) * dt);
            }
          publishCorrectionThrust(correction_thrust_);
          if (attitude_error <= correction_stop_angle_ ||
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
  const bool use_roll_pairs = roll_error >= pitch_error;
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
          const double pitch_residual = desired[1] - moment[1];
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
  // Select only a pair whose positive thrust produces the requested moment
  // sign. Taking abs() of a negative allocation would reverse the moment.
  using ArmPair = std::array<size_t, 2>;
  const bool pitch_dominant =
      std::abs(wrapAngle(target_pitch_ - pitch_)) >=
      std::abs(wrapAngle(target_roll_ - roll_));
  const std::array<ArmPair, 2> candidates = pitch_dominant
      ? std::array<ArmPair, 2>{{ArmPair{{1, 2}}, ArmPair{{0, 3}}}}
      : std::array<ArmPair, 2>{{ArmPair{{0, 1}}, ArmPair{{2, 3}}}};
  PressureArray result{{0.0, 0.0, 0.0, 0.0}};
  double best_cost = std::numeric_limits<double>::infinity();
  ArmPair best_pair{{0, 0}};
  double best_thrust = 0.0;
  for (const ArmPair& pair : candidates)
    {
      const Vector2 pair_column{{
          columns[pair[0]][0] + columns[pair[1]][0],
          columns[pair[0]][1] + columns[pair[1]][1]}};
      const double denominator = pair_column[0] * pair_column[0] +
                                 pair_column[1] * pair_column[1] + 1.0e-8;
      const double allocated_thrust =
          (pair_column[0] * desired[0] + pair_column[1] * desired[1]) /
          denominator;
      if (allocated_thrust <= 0.0) continue;

      // A mathematically small command is too close to ESC idle to start the
      // real rotor, so retain the measured minimum useful thrust.
      const double common_thrust = std::min(correction_max_thrust_n_,
          std::max(correction_min_thrust_n_, allocated_thrust));
      const double residual_roll =
          desired[0] - pair_column[0] * common_thrust;
      const double residual_pitch =
          desired[1] - pair_column[1] * common_thrust;
      const double cost = residual_roll * residual_roll +
                          residual_pitch * residual_pitch;
      if (cost < best_cost)
        {
          best_cost = cost;
          best_pair = pair;
          best_thrust = common_thrust;
        }
    }

  if (best_thrust <= 0.0)
    {
      ROS_ERROR_THROTTLE(1.0,
          "No rotor pair can produce the requested correction moment with positive thrust");
      return result;
    }
  result[best_pair[0]] = result[best_pair[1]] = best_thrust;
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
  // In MuJoCo use the exact sum of the five articulated joint coordinates.
  // This also avoids interpreting accelerometer sign conventions as a gait
  // trajectory. Hardware has no grip_state extension and retains the Neuron
  // IMU fallback below.
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
    const PressureArray& requested, double maximum_step)
{
  std_msgs::Float32MultiArray target;
  target.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bounded = std::max(minimum_prepare_pressure_kpa_,
          std::min(maximum_pressure_kpa_, requested[arm]));
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
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double bounded_thrust = std::max(-pwm_test_max_reverse_thrust_n_,
          std::min(pwm_test_max_thrust_n_, thrust[arm]));
      if (std::abs(bounded_thrust) <= 0.01) continue;
      const double pwm = signedThrustToPwm(bounded_thrust);
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
    "ROCK_REAR_LAND"
  };
  state_ = state;
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
// FRONT_REACH simultaneously unloads and thrusts one leading arm, then
// FRONT_ANCHOR -> SUPPORT_RESTORE is repeated for the second leading arm,
// followed by REAR_UNLOAD -> PULL -> REAR_ANCHOR.
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
}

void AttitudePressureController::startInchwormCycle(const ros::Time& now)
{
  const double overall_travel =
      (mocap_position_[0] - inchworm_overall_start_position_[0]) *
          inchworm_direction_world_[0] +
      (mocap_position_[1] - inchworm_overall_start_position_[1]) *
          inchworm_direction_world_[1];
  const double remaining = std::max(0.0,
      inchworm_total_target_distance_m_ - overall_travel);
  inchworm_start_position_ = mocap_position_;
  inchworm_target_distance_m_ = std::min(inchworm_stroke_distance_m_, remaining);
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
  else if (inchworm_action_c_enabled_)
    {
      action_c_thrust_n_ = action_c_min_thrust_n_;
      action_c_pressure_bias_kpa_.fill(0.0);
      action_c_pressure_bend_sign_.fill(1.0);
      action_c_dtheta_dpressure_.fill(0.02);
      action_c_dtheta_dthrust_.fill(0.0);
      action_c_angle_retry_ = 0;
      action_c_rear_zero_retry_ = false;
      action_c_pulse_started_ = false;
      action_c_success_ = false;
      action_c_pulse_angle_rad_ = 0.0;
      action_c_front_angle_rad_.fill(0.0);
      action_c_pressure_gradient_initialized_ = false;
      action_c_last_pulse_valid_ = false;
      action_c_last_pulse_thrust_n_ = 0.0;
      action_c_front_pressure_.fill(base_pressure_kpa_);
      enterState(ACTION_C_PREPARE, now);
      ROS_WARN("Action C started: direct Spinal--Neuron bend feedback; 90-deg front pull pulses from %.1f to %.1f N in %.1f N steps",
               action_c_min_thrust_n_, action_c_max_thrust_n_,
               action_c_thrust_step_n_);
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
    beginInchwormArmReach(now);
  ROS_INFO("Inchworm cycle %d/%d started: stroke target %.2f mm, remaining %.2f mm; swing order arm%zu then arm%zu",
           inchworm_cycle_count_, inchworm_max_cycles_,
           1000.0 * inchworm_target_distance_m_, 1000.0 * remaining,
           inchworm_front_order_[0], inchworm_front_order_[1]);
}

void AttitudePressureController::beginInchwormArmReach(
    const ros::Time& now)
{
  // Unload only the swinging arm. Its pressure target and rotor thrust are
  // applied together in FRONT_REACH; the other three arms keep GRASP pressure.
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    inchworm_pressure_[arm] = arm == inchworm_active_arm_
        ? std::max(minimum_prepare_pressure_kpa_, inchworm_swing_pressure_kpa_)
        : base_pressure_kpa_;
  grasp_imu_angle_[inchworm_active_arm_] =
      measuredArmAngle(inchworm_active_arm_);
  inchworm_swing_start_roll_ = roll_;
  inchworm_swing_start_pitch_ = pitch_;
  inchworm_angular_rate_fault_start_ = ros::Time(0);
  enterState(FRONT_REACH, now);
  ROS_INFO("Arm%zu simultaneous unload/reach started: pressure %.1f kPa, bend target %.1f deg, body response %.1f deg",
           inchworm_active_arm_, inchworm_pressure_[inchworm_active_arm_],
           inchworm_target_bend_angle_rad_ * 180.0 / M_PI,
           inchworm_body_tilt_target_rad_ * 180.0 / M_PI);
}

void AttitudePressureController::stopInchwormToGrasp(
    const ros::Time& now, const char* reason)
{
  const PressureArray zero_thrust{{0.0, 0.0, 0.0, 0.0}};
  const PressureArray grasp_target{{base_pressure_kpa_, base_pressure_kpa_,
                                    base_pressure_kpa_, base_pressure_kpa_}};
  inchworm_active_ = false;
  inchworm_thrust_ = zero_thrust;
  prepare_shape_thrust_ = zero_thrust;
  publishCorrectionThrust(zero_thrust);
  publishBottomTarget(rocking_bottom_low_kpa_);
  publishPressureTarget(grasp_target,
                        std::numeric_limits<double>::infinity());
  last_correction_end_ = now;
  enterState(GRASP, now);
  ROS_WARN("Inchworm stopped and returned directly to GRASP: %s", reason);
}

bool AttitudePressureController::pressureReached(
    const PressureArray& target) const
{
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (!std::isfinite(measured_pressure_[arm]) ||
        std::abs(measured_pressure_[arm] - target[arm]) > pressure_tolerance_kpa_)
      return false;
  return true;
}

double AttitudePressureController::inchwormTravel() const
{
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
  const double elapsed = (now - state_start_).toSec();
  const double pressure_step = pressure_rate_limit_kpa_s_ * dt;
  const double travel = inchwormTravel();
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
    case FRONT_RELEASE:
      inchworm_thrust_.fill(0.0);
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_pressure_[arm] = arm == inchworm_active_arm_
            ? std::max(minimum_prepare_pressure_kpa_,
                       inchworm_swing_pressure_kpa_)
            : base_pressure_kpa_;
      publishCorrectionThrust(inchworm_thrust_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      if (pressureReached(inchworm_pressure_))
        {
          // Keep the GRASP Neuron reference: pressure-driven extension is a
          // real part of the motion toward the absolute 5-deg bend target.
          inchworm_swing_start_roll_ = roll_;
          inchworm_swing_start_pitch_ = pitch_;
          inchworm_angular_rate_fault_start_ = ros::Time(0);
          ROS_INFO("Arm%zu released at %.2f kPa; targeting %.1f deg absolute bend and %.1f deg body response",
                   inchworm_active_arm_,
                   measured_pressure_[inchworm_active_arm_],
                   inchworm_target_bend_angle_rad_ * 180.0 / M_PI,
                   inchworm_body_tilt_target_rad_ * 180.0 / M_PI);
          enterState(FRONT_REACH, now);
        }
      else if (elapsed >= inchworm_release_timeout_sec_)
        {
          ROS_ERROR("FRONT_RELEASE timed out");
          stopInchwormToGrasp(now, "FRONT_RELEASE timeout");
        }
      break;

    case FRONT_REACH:
      {
      // Continue pressure reduction and thrust ramp concurrently. Reasserting
      // all four targets prevents the three support arms from being unloaded.
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_pressure_[arm] = arm == inchworm_active_arm_
            ? std::max(minimum_prepare_pressure_kpa_,
                       inchworm_swing_pressure_kpa_)
            : base_pressure_kpa_;
      PressureArray relative_angle{{0.0, 0.0, 0.0, 0.0}};
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        relative_angle[arm] = measuredArmAngle(arm) - grasp_imu_angle_[arm];
      // measuredArmAngle() is the Neuron attitude relative to the Spinal body
      // attitude.  Therefore zero is a straight arm and the magnitude is the
      // absolute bend angle; do not anchor this target to the pressure model
      // or to the bend at the beginning of GRASP.
      const double estimated_bend = std::abs(
          wrapAngle(measuredArmAngle(inchworm_active_arm_)));
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
      const bool body_response_ready = body_tilt >= inchworm_body_tilt_target_rad_;
      // Even after the arm reaches its absolute bend target, continue the
      // slow thrust search until Spinal observes a small body response. This
      // guarantees a mechanically effective thrust instead of accepting an
      // angle change caused by pressure release alone.
      const double adaptive_thrust = (!bend_ready || !body_response_ready)
          ? inchworm_reach_max_thrust_n_
          : inchworm_reach_thrust_n_;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_thrust_[arm] = approach(inchworm_thrust_[arm],
            arm == inchworm_active_arm_ ? adaptive_thrust : 0.0,
            inchworm_thrust_ramp_n_s_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2, "Inchworm FRONT_REACH arm%zu bend %.1f/%.1f deg, body tilt %.2f/%.2f deg, thrust %.2f N",
                        inchworm_active_arm_,
                        estimated_bend * 180.0 / M_PI,
                        effective_target_bend * 180.0 / M_PI,
                        body_tilt * 180.0 / M_PI,
                        inchworm_body_tilt_target_rad_ * 180.0 / M_PI,
                        inchworm_thrust_[inchworm_active_arm_]);
      const bool front_pressure_following =
          measured_pressure_[inchworm_active_arm_] <=
              inchworm_pressure_[inchworm_active_arm_] + pressure_tolerance_kpa_;
      ROS_INFO_THROTTLE(0.5,
          "FRONT_REACH pressure target [%.1f, %.1f, %.1f, %.1f], measured [%.1f, %.1f, %.1f, %.1f], thrust [%.2f, %.2f, %.2f, %.2f]",
          inchworm_pressure_[0], inchworm_pressure_[1],
          inchworm_pressure_[2], inchworm_pressure_[3],
          measured_pressure_[0], measured_pressure_[1],
          measured_pressure_[2], measured_pressure_[3],
          inchworm_thrust_[0], inchworm_thrust_[1],
          inchworm_thrust_[2], inchworm_thrust_[3]);
      if (bend_ready && body_response_ready && front_pressure_following)
        {
          ROS_INFO("FRONT_REACH achieved on arm%zu: bend %.1f deg, body tilt %.1f deg; advancing to FRONT_ANCHOR",
                   inchworm_active_arm_, estimated_bend * 180.0 / M_PI,
                   body_tilt * 180.0 / M_PI);
          enterState(FRONT_ANCHOR, now);
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
          ROS_ERROR("FRONT_REACH timed out: bend %.1f/%.1f deg, body tilt %.2f/%.2f deg; restoring grasp",
                    estimated_bend * 180.0 / M_PI,
                    effective_target_bend * 180.0 / M_PI,
                    body_tilt * 180.0 / M_PI,
                    inchworm_body_tilt_target_rad_ * 180.0 / M_PI);
          stopInchwormToGrasp(now, "FRONT_REACH timeout");
        }
      }
      break;

    case FRONT_ANCHOR:
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_pressure_[arm] = arm == inchworm_active_arm_
            ? base_pressure_kpa_ : inchworm_transfer_pressure_kpa_;
      for (double& thrust : inchworm_thrust_)
        thrust = approach(thrust, 0.0, inchworm_thrust_ramp_n_s_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) &&
          inchworm_thrust_[inchworm_active_arm_] <= 0.01)
        {
          ROS_INFO("Front arm%zu anchored at %.1f kPa; other arms transferred to %.1f kPa",
                   inchworm_active_arm_, base_pressure_kpa_,
                   inchworm_transfer_pressure_kpa_);
          enterState(SUPPORT_RESTORE, now);
        }
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          ROS_ERROR("FRONT_ANCHOR timed out");
          stopInchwormToGrasp(now, "FRONT_ANCHOR timeout");
        }
      break;

    case SUPPORT_RESTORE:
      inchworm_pressure_.fill(base_pressure_kpa_);
      inchworm_thrust_.fill(0.0);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_))
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
            enterState(REAR_UNLOAD, now);
        }
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          ROS_ERROR("SUPPORT_RESTORE timed out");
          stopInchwormToGrasp(now, "SUPPORT_RESTORE timeout");
        }
      break;

    case REAR_UNLOAD:
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        inchworm_pressure_[arm] = inchworm_front_[arm]
            ? base_pressure_kpa_
            : std::max(minimum_prepare_pressure_kpa_,
                       base_pressure_kpa_ - inchworm_rear_unload_kpa_);
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
      double rear_thrust = 0.0;
      size_t rear_count = 0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (!inchworm_front_[arm])
          {
            rear_thrust += inchworm_thrust_[arm];
            ++rear_count;
          }
      rear_thrust /= std::max<size_t>(1, rear_count);
      const bool meaningful_progress =
          travel >= inchworm_best_pull_travel_m_ + inchworm_progress_epsilon_m_;
      if (meaningful_progress &&
          rear_thrust >= inchworm_rear_lift_thrust_n_ - 0.05)
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
              // The newly placed front anchors use the normal 30 kPa grasp
              // pressure; locomotion never requests a stronger contact than
              // the established human-contact value.
              inchworm_pressure_[arm] = base_pressure_kpa_;
              inchworm_thrust_[arm] = approach(inchworm_thrust_[arm], 0.0,
                                                inchworm_thrust_ramp_n_s_);
            }
          else
            {
              // Low pressure lets the trailing arm slide/extend while its
              // rotor pulls the body toward the two leading anchors.  This
              // parameter used to be loaded but was never applied.
              inchworm_pressure_[arm] = std::max(
                  minimum_prepare_pressure_kpa_, inchworm_pull_pressure_kpa_);
              // Search upward only until forward motion is observed. Hold the
              // minimum effective value while moving; resume the ramp if the
              // body stalls before reaching the requested displacement.
              const double target = inchworm_motion_detected_
                  ? inchworm_thrust_[arm]
                  : std::min(inchworm_rear_lift_max_thrust_n_,
                      std::max(inchworm_rear_lift_thrust_n_,
                               inchworm_thrust_[arm] +
                                   inchworm_thrust_ramp_n_s_ * dt));
              inchworm_thrust_[arm] = approach(inchworm_thrust_[arm], target,
                                                inchworm_thrust_ramp_n_s_);
            }
        }
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.2, "Inchworm PULL travel %.2f/%.2f mm, speed %.2f mm/s, motion %d",
                        1000.0 * inchwormTravel(),
                        1000.0 * inchworm_target_distance_m_,
                        1000.0 * inchworm_travel_speed_m_s_,
                        inchworm_motion_detected_);
      if (inchwormTravel() >= inchworm_target_distance_m_ ||
          elapsed >= inchworm_pull_timeout_sec_)
        enterState(REAR_ANCHOR, now);
      }
      break;

    case REAR_ANCHOR:
      inchworm_pressure_.fill(base_pressure_kpa_);
      for (double& thrust : inchworm_thrust_)
        thrust = approach(thrust, 0.0, inchworm_thrust_ramp_n_s_);
      publishPressureTarget(inchworm_pressure_, pressure_step);
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) &&
          std::all_of(inchworm_thrust_.begin(), inchworm_thrust_.end(),
                      [](double value) { return value <= 0.01; }))
        enterState(SETTLE, now);
      else if (elapsed >= inchworm_anchor_timeout_sec_)
        {
          inchworm_thrust_.fill(0.0);
          enterState(SETTLE, now);
        }
      break;

    case ACTION_C_PREPARE:
      {
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
          double selected_pressure = best_pressure +
              action_c_pressure_bias_kpa_[arm];
          // Once one pulse has been observed, use the local measured model
          // to prepare the next operating point.  This compensates the
          // expected arm-angle change caused by a new thrust level before
          // that thrust is applied.  The table remains the safe initial
          // guess for the first pulse.
          if (action_c_last_pulse_valid_ &&
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
          inchworm_pressure_[arm] = action_c_front_pressure_[arm];
        }
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.5,
          "Action C PREPARE: thrust %.1f N, local model %d, learned front bias [%.1f %.1f %.1f %.1f] kPa, rear next %.1f kPa, front pressure [%.1f, %.1f, %.1f, %.1f]",
          action_c_thrust_n_, action_c_last_pulse_valid_ ? 1 : 0,
          action_c_pressure_bias_kpa_[0],
          action_c_pressure_bias_kpa_[1], action_c_pressure_bias_kpa_[2],
          action_c_pressure_bias_kpa_[3],
          action_c_rear_zero_retry_ ? 0.0 : action_c_rear_pressure_kpa_,
          action_c_front_pressure_[0], action_c_front_pressure_[1],
          action_c_front_pressure_[2], action_c_front_pressure_[3]);
      if (pressureReached(inchworm_pressure_))
        {
          action_c_pulse_started_ = false;
          action_c_pulse_angle_rad_ = 0.0;
          enterState(ACTION_C_HORIZONTAL_PULL, now);
        }
      }
      break;

    case ACTION_C_HORIZONTAL_PULL:
      {
      bool rear_pressure_ready = true;
      const double rear_target = action_c_rear_zero_retry_
          ? 0.0 : action_c_rear_pressure_kpa_;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          if (inchworm_front_[arm])
            inchworm_pressure_[arm] = action_c_front_pressure_[arm];
          else
            {
              inchworm_pressure_[arm] = rear_target;
              rear_pressure_ready = rear_pressure_ready &&
                  std::abs(measured_pressure_[arm] - rear_target) <=
                      pressure_tolerance_kpa_;
            }
          inchworm_thrust_[arm] = 0.0;
        }
      if (rear_pressure_ready && !action_c_pulse_started_)
        {
          action_c_pulse_started_ = true;
          action_c_pulse_start_ = now;
          action_c_pressure_feedback_time_ = now;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                action_c_previous_pressure_kpa_[arm] = measured_pressure_[arm];
                action_c_previous_bend_angle_rad_[arm] =
                    measuredArmBendAngle(arm);
              }
          action_c_pressure_gradient_initialized_ = true;
          ready_start_ = ros::Time(0);
          ROS_INFO("Action C pulse ON: %.1f N, rear %.1f kPa, angle target %.1f +/- %.1f deg",
                   action_c_thrust_n_, rear_target,
                   action_c_target_angle_rad_ * 180.0 / M_PI,
                   action_c_angle_tolerance_rad_ * 180.0 / M_PI);
        }
      if (action_c_pulse_started_)
        {
          double angle_sum = 0.0;
          size_t angle_count = 0;
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                inchworm_thrust_[arm] = action_c_thrust_n_;
                // Do not fuse the pressure model into this feedback angle.
                // The model is useful for the initial pressure guess, but
                // under thrust/contact it can have a large bias.  Control
                // each arm from its directly measured Spinal--Neuron bend.
                action_c_front_angle_rad_[arm] = measuredArmBendAngle(arm);
                angle_sum += action_c_front_angle_rad_[arm];
                ++angle_count;
              }
          if (angle_count > 0)
            {
              action_c_pulse_angle_rad_ =
                  angle_sum / static_cast<double>(angle_count);

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
                    const double angle_error = action_c_target_angle_rad_ -
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
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      ROS_INFO_THROTTLE(0.1,
          "Action C PULL: thrust %.1f N, rear %.1f kPa, front pressure [%.1f %.1f %.1f %.1f] kPa, bend [%.1f %.1f %.1f %.1f] deg (mean %.1f/%.1f), P->bend sign [%.0f %.0f %.0f %.0f], mocap start [%.4f %.4f], now [%.4f %.4f], direction [%.2f %.2f], projected travel %.2f/%.2f mm, pulse %d",
          action_c_thrust_n_, rear_target,
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
          inchworm_start_position_[0], inchworm_start_position_[1],
          mocap_position_[0], mocap_position_[1],
          inchworm_direction_world_[0], inchworm_direction_world_[1],
          1000.0 * travel, 1000.0 * inchworm_target_distance_m_,
          action_c_pulse_started_);
      // Motion accumulated while waiting for the rear pressure must not be
      // reported as a pull success.  A C success is valid only during ON.
      if (action_c_pulse_started_ && travel >= inchworm_target_distance_m_)
        {
          if (ready_start_.isZero()) ready_start_ = now;
          if ((now - ready_start_).toSec() >= action_c_success_confirm_sec_)
            {
              action_c_success_ = true;
              inchworm_thrust_.fill(0.0);
              publishCorrectionThrust(inchworm_thrust_);
              ROS_INFO("ACTION C SUCCESS: mocap displacement held for %.2f s, moved %.2f/%.2f mm at %.1f N, angle %.1f deg, rear %.1f kPa",
                       action_c_success_confirm_sec_,
                       1000.0 * travel,
                       1000.0 * inchworm_target_distance_m_,
                       action_c_thrust_n_,
                       action_c_pulse_angle_rad_ * 180.0 / M_PI,
                       rear_target);
              enterState(ACTION_C_REGRIP, now);
            }
        }
      else
        ready_start_ = ros::Time(0);
      if (state_ == ACTION_C_HORIZONTAL_PULL &&
          action_c_pulse_started_ &&
          ready_start_.isZero() &&
               (now - action_c_pulse_start_).toSec() >=
                   action_c_pulse_duration_sec_)
        {
          // Store the terminal operating point of this pulse.  On the next
          // thrust level, subtract the pressure-induced angle change from the
          // total change to identify dtheta/dT, then pre-compensate it in
          // ACTION_C_PREPARE.
          for (size_t arm = 0; arm < ARM_COUNT; ++arm)
            if (inchworm_front_[arm])
              {
                if (action_c_last_pulse_valid_)
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
                        const double bounded_dtheta_dthrust = std::max(-1.0,
                            std::min(1.0, measured_dtheta_dthrust));
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
              }
          action_c_last_pulse_thrust_n_ = action_c_thrust_n_;
          action_c_last_pulse_valid_ = true;
          inchworm_thrust_.fill(0.0);
          publishCorrectionThrust(inchworm_thrust_);
          enterState(ACTION_C_EVALUATE, now);
        }
      }
      break;

    case ACTION_C_EVALUATE:
      {
      inchworm_thrust_.fill(0.0);
      bool rear_grasped = true;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          inchworm_pressure_[arm] = inchworm_front_[arm]
              ? action_c_front_pressure_[arm] : base_pressure_kpa_;
          if (!inchworm_front_[arm])
            rear_grasped = rear_grasped &&
                std::abs(measured_pressure_[arm] - base_pressure_kpa_) <=
                    pressure_tolerance_kpa_;
        }
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      if (!rear_grasped) break;

      bool angle_ready = true;
      double worst_angle_error = 0.0;
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        if (inchworm_front_[arm])
          {
            const double angle_error = action_c_target_angle_rad_ -
                action_c_front_angle_rad_[arm];
            worst_angle_error = std::max(worst_angle_error,
                std::abs(angle_error));
            angle_ready = angle_ready && std::abs(angle_error) <=
                action_c_angle_tolerance_rad_;
          }
      if (!angle_ready && action_c_angle_retry_ < action_c_max_angle_retries_)
        {
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
          ROS_WARN("Action C angle retry %d/%d: bend [%.1f %.1f %.1f %.1f] deg, worst error %.1f deg, front bias [%.1f %.1f %.1f %.1f] kPa",
                   action_c_angle_retry_, action_c_max_angle_retries_,
                   action_c_front_angle_rad_[0] * 180.0 / M_PI,
                   action_c_front_angle_rad_[1] * 180.0 / M_PI,
                   action_c_front_angle_rad_[2] * 180.0 / M_PI,
                   action_c_front_angle_rad_[3] * 180.0 / M_PI,
                   worst_angle_error * 180.0 / M_PI,
                   action_c_pressure_bias_kpa_[0], action_c_pressure_bias_kpa_[1],
                   action_c_pressure_bias_kpa_[2], action_c_pressure_bias_kpa_[3]);
          enterState(ACTION_C_PREPARE, now);
        }
      else if (!angle_ready)
        {
          // Two pressure-learning retries were used without entering the
          // 90 +/- 15 deg band. Preserve the learned bias and let the next
          // thrust level use a newly model-inverted base pressure.
          if (worst_angle_error >=
              action_c_max_error_before_thrust_increase_rad_)
            {
              action_c_success_ = false;
              ROS_ERROR("ACTION C FAILED: worst bend error %.1f deg exceeds %.1f-deg thrust-increase guard; regripping",
                        worst_angle_error * 180.0 / M_PI,
                        action_c_max_error_before_thrust_increase_rad_ * 180.0 / M_PI);
              enterState(ACTION_C_REGRIP, now);
            }
          else if (action_c_thrust_n_ + action_c_thrust_step_n_ <=
                  action_c_max_thrust_n_ + 1.0e-6)
            {
              action_c_thrust_n_ = std::min(
                  action_c_max_thrust_n_,
                  action_c_thrust_n_ + action_c_thrust_step_n_);
              action_c_angle_retry_ = 0;
              action_c_rear_zero_retry_ = false;
              ROS_WARN("Action C angle remained outside tolerance; advancing to %.1f N",
                       action_c_thrust_n_);
              enterState(ACTION_C_PREPARE, now);
            }
          else
            {
              action_c_success_ = false;
              ROS_ERROR("ACTION C FAILED: angle %.1f deg remained outside tolerance through %.1f N",
                        action_c_pulse_angle_rad_ * 180.0 / M_PI,
                        action_c_max_thrust_n_);
              enterState(ACTION_C_REGRIP, now);
            }
        }
      else if (!action_c_rear_zero_retry_)
        {
          action_c_rear_zero_retry_ = true;
          ROS_WARN("Action C did not move at %.1f N; retrying with rear pressure 0 kPa (angle %.1f deg, ready %d)",
                   action_c_thrust_n_,
                   action_c_pulse_angle_rad_ * 180.0 / M_PI, angle_ready);
          enterState(ACTION_C_PREPARE, now);
        }
      else if (action_c_thrust_n_ + action_c_thrust_step_n_ <=
                   action_c_max_thrust_n_ + 1.0e-6)
        {
          action_c_thrust_n_ = std::min(
              action_c_max_thrust_n_,
              action_c_thrust_n_ + action_c_thrust_step_n_);
          action_c_angle_retry_ = 0;
          action_c_rear_zero_retry_ = false;
          ROS_WARN("Action C advancing to %.1f N; carrying independent front pressure biases",
                   action_c_thrust_n_);
          enterState(ACTION_C_PREPARE, now);
        }
      else
        {
          action_c_success_ = false;
          ROS_ERROR("ACTION C FAILED: no target motion through %.1f N; restoring grasp",
                    action_c_max_thrust_n_);
          enterState(ACTION_C_REGRIP, now);
        }
      }
      break;

    case ACTION_C_REGRIP:
      inchworm_pressure_.fill(base_pressure_kpa_);
      inchworm_thrust_.fill(0.0);
      publishPressureTarget(inchworm_pressure_,
                            std::numeric_limits<double>::infinity());
      publishCorrectionThrust(inchworm_thrust_);
      if (pressureReached(inchworm_pressure_) ||
          elapsed >= inchworm_anchor_timeout_sec_)
        {
          const double retained_threshold =
              action_c_regrip_min_retained_ratio_ *
              inchworm_target_distance_m_;
          if (action_c_success_ && travel < retained_threshold)
            {
              ROS_WARN("ACTION C retained-motion check failed: %.2f mm < %.2f mm after REGRIP",
                       1000.0 * travel, 1000.0 * retained_threshold);
              action_c_success_ = false;
            }
          ROS_INFO("Action C regrip complete: %s, retained travel %.2f mm (required %.2f mm)",
                   action_c_success_ ? "SUCCESS" : "FAILED",
                   1000.0 * travel, 1000.0 * retained_threshold);
          inchworm_active_ = false;
          enterState(GRASP, now);
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
              (mocap_position_[0] - inchworm_overall_start_position_[0]) *
                  inchworm_direction_world_[0] +
              (mocap_position_[1] - inchworm_overall_start_position_[1]) *
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
          if (!target_reached && !no_progress && !cycle_limit)
            startInchwormCycle(now);
          else
            {
              if (no_progress && !target_reached)
                ROS_WARN("Inchworm repetition stopped because the last cycle advanced only %.2f mm",
                         1000.0 * cycle_travel);
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
