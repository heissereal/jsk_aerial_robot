#ifndef HUGMY_ATTITUDE_PRESSURE_CONTROLLER_H
#define HUGMY_ATTITUDE_PRESSURE_CONTROLLER_H

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <spinal/Imu.h>
#include <spinal/NeuronImuStates.h>
#include <spinal/PerchingThrustCommand.h>
#include <spinal/PwmTest.h>
#include <spinal/FlightConfigCmd.h>
#include <spinal/Thrust.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <hugmy/control/hybrid_locomotion_search.h>
#include <hugmy/control/hybrid_measurement.h>
#include <hugmy/control/hybrid_single_pass.h>
#include <hugmy/control/arm_bend_limits.h>
#include <hugmy/control/posture_recovery_gate.h>
#include <hugmy/control/pressure_ready_gate.h>
#include <hugmy/control/rear_release_gate.h>
#include <hugmy/control/roll_trend_gate.h>
#include <hugmy/control/continuous_roll_recovery.h>
#include <hugmy/control/recovery_predictive_control.h>
#include <hugmy/control/pulse_bend_observer.h>

class AttitudePressureController
{
public:
  static constexpr size_t ARM_COUNT = 4;
  using Vector3 = std::array<double, 3>;
  using Vector2 = std::array<double, 2>;
  using PressureArray = std::array<double, ARM_COUNT>;

  AttitudePressureController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void imuCallback(const spinal::Imu::ConstPtr& msg);
  void rootImuCallback(const spinal::Imu::ConstPtr& msg);
  void updateImu(const spinal::Imu::ConstPtr& msg, bool root_topics);
  void thrustCallback(const spinal::Thrust::ConstPtr& msg);
  void rootThrustCallback(const spinal::Thrust::ConstPtr& msg);
  void neuronImuCallback(const spinal::NeuronImuStates::ConstPtr& msg);
  void rootNeuronImuCallback(const spinal::NeuronImuStates::ConstPtr& msg);
  void updateNeuronImu(const spinal::NeuronImuStates::ConstPtr& msg);
  void pressureCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void gripStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void bottomPressureCallback(const std_msgs::Float32::ConstPtr& msg);
  void targetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void inchwormCommandCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void rootMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void updateMocap(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void handMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void rootHandMocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void updateHandMocap(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void update(const ros::TimerEvent& event);

  Vector3 rotorPosition(size_t arm, double pressure_kpa) const;
  Vector3 rotorDirection(size_t arm, double pressure_kpa) const;
  double bendAngle(size_t arm, double pressure_kpa) const;
  std::array<double, 3> jointAngles(size_t arm, double pressure_kpa) const;
  Vector2 thrustMoment(const PressureArray& pressure) const;
  PressureArray allocatePressure(const Vector2& desired_moment) const;
  PressureArray allocateCorrectionThrust(const Vector2& desired_moment,
                                         const PressureArray& pressure) const;
  PressureArray allocateCorrectionThrustFromAngles(
      const Vector2& desired_moment, const PressureArray& bend_angle) const;
  Vector2 desiredThrustMoment() const;
  bool correctionRollOnly() const;
  double measuredArmAngle(size_t arm) const;
  double measuredArmBendAngle(size_t arm) const;
  Vector3 rotorPositionFromAngle(size_t arm, double bend_angle) const;
  Vector3 rotorDirectionFromAngle(size_t arm, double bend_angle) const;
  bool armGeometryReady(const PressureArray& target, const ros::Time& now) const;
  void publishPressureTarget(const PressureArray& target, double maximum_step,
                             double pressure_limit_kpa = -1.0);
  void publishBottomTarget(double pressure_kpa);
  void publishBottomDirection(double direction_scale = 1.0);
  void publishCorrectionThrust(const PressureArray& thrust);
  void publishPwmTest(const PressureArray& thrust);
  double signedThrustToPwm(double thrust_n) const;
  double pwmModelForce(double pwm, bool reverse) const;
  void publishArmShapeDebug(const PressureArray& current_relative_angle) const;

  // Shared perching state-machine support.
  void enterState(uint8_t state, const ros::Time& now);

  // Inchworm locomotion lifecycle. Action C may temporarily release ownership
  // to the shared GRASP -> PREPARE -> THRUST_CORRECTION -> RECOVER sequence,
  // then continue toward the saved mocap goal.
  void startInchwormStep(const Vector2& direction, double distance,
                         const ros::Time& now);
  void startInchwormCycle(const ros::Time& now);
  void beginInchwormArmReach(const ros::Time& now);
  void stopInchwormToGrasp(const ros::Time& now, const char* reason);
  void requestActionCPostureRecovery(const ros::Time& now,
                                     const char* reason);
  bool resumeActionCAfterRecovery(const ros::Time& now);
  bool actionCRearReleased(const ros::Time& now);
  bool hybridLocomotionActive() const;
  void startHybridLocomotion(const ros::Time& now);
  void beginHybridRegrip(const ros::Time& now, bool prediction_limited);
  void beginHybridTerminalSettle(const ros::Time& now, const char* reason);
  void updateHybridLocomotion(const ros::Time& now, double dt);
  void finishHybridLocomotion(const ros::Time& now, bool completed, bool safe);
  void pauseHybridLocomotion(const ros::Time& now);
  bool hybridResumeAllowed(const ros::Time& now);
  void finishHybridRecovery(const ros::Time& now);
  void observeHybridThrust(const ros::Time& now, const PressureArray& thrust);
  hugmy::hybrid::Force hybridRotorForce(size_t arm, double bend, double roll_error, double pitch) const;
  void updateInchworm(const ros::Time& now, double dt, bool pressure_valid,
                      bool neurons_fresh);
  bool pressureReached(const PressureArray& target) const;
  bool actionCPressureReady(const PressureArray& target, const ros::Time& now);
  Vector3 locomotionPosition() const;
  bool locomotionMocapReady() const;
  bool locomotionMocapFresh(const ros::Time& now) const;
  double inchwormTravel() const;

  static Vector3 cross(const Vector3& lhs, const Vector3& rhs);
  static Vector3 rotateLocalToBody(const Vector3& value, double yaw);
  static Vector3 rotateAboutLocalY(const Vector3& value, double angle);
  static double wrapAngle(double angle);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber imu_sub_, root_imu_sub_, thrust_sub_, root_thrust_sub_;
  ros::Subscriber target_sub_, neuron_imu_sub_, root_neuron_imu_sub_, pressure_sub_;
  ros::Subscriber grip_state_sub_;
  ros::Subscriber bottom_pressure_sub_;
  ros::Subscriber inchworm_command_sub_, mocap_sub_, root_mocap_sub_;
  ros::Subscriber hand_mocap_sub_, root_hand_mocap_sub_;
  ros::Publisher pressure_target_pub_, rpy_pub_, attitude_error_pub_;
  ros::Publisher bottom_target_pub_, bottom_direction_pub_;
  ros::Publisher arm_relative_angle_pub_, arm_target_relative_angle_pub_;
  ros::Publisher arm_shape_error_pub_, arm_shape_active_pub_;
  ros::Publisher arm_bend_angle_deg_pub_;
  ros::Publisher hand_relative_position_pub_;
  ros::Publisher desired_moment_pub_, achieved_moment_pub_;
  ros::Publisher correction_thrust_pub_, root_correction_thrust_pub_;
  ros::Publisher pwm_test_pub_, root_pwm_test_pub_;
  ros::Publisher flight_config_pub_, root_flight_config_pub_, state_pub_;
  ros::ServiceServer enable_server_;
  ros::ServiceClient pressure_enable_client_;
  ros::Timer timer_;

  std::array<Vector3, ARM_COUNT> arm_base_position_{{
    {{ 0.03812,  0.03812, -0.039}}, {{-0.03812,  0.03812, -0.039}},
    {{-0.03812, -0.03812, -0.039}}, {{ 0.03812, -0.03812, -0.039}}
  }};
  std::array<double, ARM_COUNT> arm_yaw_{{
    -2.3561944902, -0.7853981634, 0.7853981634, 2.3561944902
  }};
  PressureArray thrust_n_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray last_pressure_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray measured_pressure_{{NAN, NAN, NAN, NAN}};
  PressureArray prepare_pressure_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray correction_thrust_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray prepare_shape_thrust_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray grasp_pressure_{{30.0, 30.0, 30.0, 30.0}};
  PressureArray grasp_imu_angle_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray grasp_model_angle_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray prepare_target_angle_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray prepare_straighten_angle_by_arm_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, ARM_COUNT> prepare_arm_active_{{false, false, false, false}};
  std::array<Vector3, ARM_COUNT> neuron_acc_{};
  std::array<Vector3, ARM_COUNT> neuron_gyro_{};
  PressureArray action_b_neuron_angle_offset_{{0.0, 0.0, 0.0, 0.0}};
  std::array<ros::Time, ARM_COUNT> neuron_stamp_{};
  std::array<bool, ARM_COUNT> neuron_acc_initialized_{{false, false, false, false}};
  std::array<int, ARM_COUNT> neuron_slave_ids_{{1, 2, 3, 4}};
  hugmy::PressureThrustBendModel bend_model_;

  ros::Time imu_stamp_, thrust_stamp_;
  ros::Time state_start_, ready_start_, last_correction_end_, last_angle_progress_;
  ros::Time body_motion_last_seen_;
  ros::Time mocap_stamp_, hand_mocap_stamp_;
  ros::Time inchworm_rate_stamp_, inchworm_motion_last_seen_;
  ros::Time action_b_pulse_phase_start_;
  ros::Time action_b_shape_phase_start_;
  ros::Time action_b_limit_start_;
  ros::Time action_b_lift_settle_start_;
  Vector3 mocap_position_{{0.0, 0.0, 0.0}};
  Vector3 hand_mocap_position_{{0.0, 0.0, 0.0}};
  Vector3 inchworm_start_position_{{0.0, 0.0, 0.0}};
  Vector3 inchworm_overall_start_position_{{0.0, 0.0, 0.0}};
  Vector2 inchworm_direction_body_{{1.0, 0.0}};
  Vector2 inchworm_direction_world_{{1.0, 0.0}};
  std::array<bool, ARM_COUNT> inchworm_front_{{false, false, false, false}};
  std::array<bool, ARM_COUNT> grip_active_{{false, false, false, false}};
  PressureArray grip_axis_coordinate_m_{{NAN, NAN, NAN, NAN}};
  PressureArray grip_surface_gap_m_{{NAN, NAN, NAN, NAN}};
  PressureArray grip_joint_bend_rad_{{NAN, NAN, NAN, NAN}};
  PressureArray rocking_front_start_axis_m_{{NAN, NAN, NAN, NAN}};
  ros::Time grip_state_stamp_;
  bool grip_state_valid_ = false;
  std::array<size_t, 2> inchworm_front_order_{{0, 3}};
  size_t inchworm_front_sequence_ = 0;
  size_t inchworm_active_arm_ = 0;
  PressureArray inchworm_pressure_{{30.0, 30.0, 30.0, 30.0}};
  PressureArray inchworm_thrust_{{0.0, 0.0, 0.0, 0.0}};
  double action_a_place_start_bend_rad_ = 0.0;
  double action_a_place_start_thrust_n_ = 0.0;
  Vector3 body_angular_velocity_{{0.0, 0.0, 0.0}};
  Vector3 gravity_body_{{0.0, 0.0, -9.80665}};
  Vector3 contact_position_{{0.0, 0.0, -0.070}};
  Vector3 center_of_mass_{{-0.00348932, 0.00065510, 0.01200916}};
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  double roll_moment_kp_, pitch_moment_kp_, roll_moment_kd_, pitch_moment_kd_;
  double mass_kg_, base_pressure_kpa_, maximum_pressure_kpa_;
  double bend_model_thrust_offset_n_;
  double pressure_rate_limit_kpa_s_, imu_timeout_sec_, thrust_timeout_sec_;
  double control_rate_hz_, allocation_damping_, allocation_step_limit_kpa_;
  double allocation_pressure_epsilon_kpa_, allocation_bias_weight_;
  int allocation_iterations_;
  enum State : uint8_t {
    GRASP = 0,
    PREPARE = 1,
    THRUST_CORRECTION = 2,
    RECOVER = 3,
    FRONT_RELEASE = 4,
    FRONT_REACH = 5,
    FRONT_ANCHOR = 6,
    REAR_UNLOAD = 7,
    PULL = 8,
    REAR_ANCHOR = 9,
    SETTLE = 10,
    SUPPORT_RESTORE = 11,
    ACTION_B_FRONT_REACH = 12,
    ACTION_B_REGRASP = 13,
    ACTION_B_FRONT_REGRIP = 14,
    ACTION_C_PREPARE = 15,
    ACTION_C_HORIZONTAL_PULL = 16,
    // State id 17 is reserved for old bag compatibility.
    ACTION_C_REGRIP = 18,
    ROCK_FRONT_UNLOAD = 19,
    ROCK_REAR_INFLATE = 20,
    ROCK_FRONT_LAND = 21,
    ROCK_REAR_UNLOAD = 22,
    ROCK_REAR_RECOVER = 23,
    ROCK_REAR_LAND = 24,
    ACTION_A_FRONT_PLACE = 25,
    ACTION_D_PREPARE = 26,
    ACTION_D_RAMP = 27,
    ACTION_D_REGRIP = 28
  };
  uint8_t state_ = GRASP;
  bool correction_roll_only_;
  // The real vehicle uses the empirically verified fixed Spinal motor pairs.
  // MuJoCo's applied bidirectional-thrust axis has the opposite roll sign.
  bool correction_pair_reverse_spinal_mapping_ = false;
  bool action_c_continuous_recovery_enabled_ = true;
  double action_c_recovery_no_progress_timeout_sec_ = 30.0;
  bool continuous_recovery_blocked_ = false;
  bool continuous_recovery_started_ = false;
  bool continuous_recovery_waiting_for_settle_ = false;
  hugmy::ContinuousRollRecovery continuous_roll_recovery_;
  hugmy::RollRecoveryConfig roll_recovery_config_;
  hugmy::RecoveryPredictiveConfig recovery_predictive_config_;
  hugmy::RecoveryPredictiveControl recovery_predictive_;
  std::array<hugmy::RollRecoveryResponse, 2> roll_recovery_response_;
  double recovery_rotor_cant_rad_ = 6.5 * M_PI / 180.0;
  double recovery_release_pressure_kpa_ = 0.0;
  double recovery_pressure_down_rate_ = 0.5;
  hugmy::RollRecoveryModel measuredRecoveryModel() const;
  double recoveryRotorLever(size_t arm, double bend) const;
  void selectContinuousRecoveryPair(double roll_error);
  double correction_trigger_angle_, correction_stop_angle_;
  double correction_trigger_confirm_sec_;
  hugmy::PostureRecoveryGate correction_entry_gate_, action_c_recovery_gate_;
  hugmy::PostureRecoveryGate action_c_resume_gate_;
  hugmy::PostureRecoveryGate action_c_boundary_settle_gate_;
  // Optional motion-free window for acquiring PPG after a gait or posture
  // recovery.  Separate gates prevent the gait endpoint and recovery handoff
  // from sharing elapsed time.
  hugmy::PostureRecoveryGate hybrid_ppg_settle_gate_;
  hugmy::PostureRecoveryGate action_c_ppg_settle_gate_;
  double ppg_settle_sec_ = 0.0;
  double action_c_posture_settle_sec_ = 0.5;
  double correction_max_angular_rate_, correction_thrust_duration_;
  double prepare_timeout_, prepare_stable_duration_, recover_timeout_, cooldown_duration_;
  double pressure_tolerance_kpa_, arm_angle_tolerance_, arm_gyro_tolerance_;
  double correction_planning_thrust_n_, correction_min_thrust_n_, correction_max_thrust_n_;
  double correction_hold_min_thrust_n_, correction_motion_start_rate_;
  double correction_motion_detection_min_thrust_n_;
  double correction_motion_confirm_sec_;
  double correction_motion_min_error_reduction_;
  double correction_motion_stop_rate_, correction_slowdown_angle_;
  double correction_prediction_horizon_sec_;
  double correction_stall_timeout_sec_;
  double correction_stall_abort_timeout_sec_;
  double correction_stall_thrust_ramp_n_s_;
  double correction_progress_epsilon_;
  double correction_thrust_down_rate_, correction_motion_stop_timeout_;
  double prepare_shape_thrust_n_;
  double prepare_angle_per_moment_, prepare_min_straighten_angle_;
  double prepare_max_straighten_angle_;
  double prepare_max_rotor_tilt_;
  double prepare_thrust_ramp_rate_, prepare_max_thrust_n_, pwm_test_max_thrust_n_;
  double pwm_test_max_reverse_thrust_n_, pwm_test_min_pwm_;
  double pwm_test_neutral_pwm_, pwm_test_max_pwm_;
  double pwm_test_voltage_, pwm_test_reference_voltage_;
  bool pwm_test_positive_thrust_below_neutral_ = true;
  std::array<double, 3> pwm_test_forward_polynomial_{{65.0663776343,
                                                       -14.931187117,
                                                       0.083500613}};
  std::array<double, 3> pwm_test_reverse_polynomial_{{-15.9407056717,
                                                       5.310953554,
                                                       -0.043050962}};
  double prepare_pressure_ramp_rate_, minimum_prepare_pressure_kpa_;
  double prepare_angle_tolerance_, prepare_progress_epsilon_, prepare_progress_timeout_;
  double best_prepare_angle_ = std::numeric_limits<double>::infinity();
  Vector2 correction_axis_{{1.0, 0.0}};
  Vector2 action_b_hold_reference_moment_{{0.0, 0.0}};
  double action_b_hold_reference_vertical_force_n_ = 0.0;
  Vector3 action_b_rear_contact_position_{{0.0, 0.0, -0.070}};
  double action_b_lift_tilt_rad_ = 0.0;
  PressureArray motion_hold_thrust_{{0.0, 0.0, 0.0, 0.0}};
  bool body_motion_detected_ = false;
  bool prepare_pair_pressure_ready_latched_ = false;
  ros::Time correction_motion_candidate_start_;
  double correction_motion_candidate_error_ = 0.0;
  ros::Time correction_last_progress_;
  double correction_best_error_ = std::numeric_limits<double>::infinity();
  double neuron_timeout_sec_;
  double neuron_acc_lpf_tau_sec_;
  double pressure_safety_limit_kpa_;
  double pressure_safety_confirm_sec_;
  ros::Time pressure_over_limit_start_;
  double inchworm_front_release_kpa_, inchworm_rear_unload_kpa_;
  double inchworm_swing_pressure_kpa_;
  double inchworm_transfer_pressure_kpa_;
  double inchworm_pull_pressure_kpa_, inchworm_front_pull_pressure_kpa_;
  double inchworm_reach_thrust_n_;
  double inchworm_rear_lift_thrust_n_, inchworm_thrust_ramp_n_s_;
  double inchworm_rear_reverse_thrust_n_;
  double inchworm_default_distance_m_, inchworm_mocap_timeout_sec_;
  double inchworm_stroke_distance_m_, inchworm_min_cycle_progress_m_;
  double inchworm_reach_angle_rad_;
  double inchworm_target_bend_angle_rad_;
  double inchworm_target_straightening_angle_rad_;
  double inchworm_angular_rate_fault_delay_sec_;
  double inchworm_swing_start_roll_ = 0.0, inchworm_swing_start_pitch_ = 0.0;
  ros::Time inchworm_angular_rate_fault_start_;
  double inchworm_reach_max_thrust_n_, inchworm_reach_angle_kp_n_rad_;
  double inchworm_reach_min_pressure_kpa_, inchworm_reach_pressure_ramp_kpa_s_;
  double inchworm_min_usable_reach_angle_rad_;
  double inchworm_rear_lift_max_thrust_n_;
  double inchworm_motion_start_speed_m_s_, inchworm_motion_stop_speed_m_s_;
  double inchworm_motion_stop_timeout_sec_;
  double inchworm_progress_epsilon_m_, inchworm_progress_regression_m_;
  double inchworm_release_timeout_sec_, inchworm_reach_duration_sec_;
  double inchworm_anchor_timeout_sec_, inchworm_pull_timeout_sec_;
  double inchworm_settle_duration_sec_;
  double action_a_place_bend_target_rad_;
  double action_a_place_hold_sec_, action_a_place_thrust_down_rate_n_s_;
  double action_a_rear_thrust_down_rate_n_s_;
  double action_a_place_timeout_sec_, action_a_place_contact_gap_m_;
  double action_a_place_min_bend_change_rad_;
  double action_a_tilt_soft_limit_rad_, action_a_tilt_hard_limit_rad_;
  double action_a_tilt_prediction_horizon_sec_;
  double action_a_tilt_thrust_down_rate_n_s_;
  double action_a_tilt_limited_bend_target_rad_;
  bool action_a_yaw_assist_enabled_;
  double action_a_yaw_target_rad_, action_a_yaw_tolerance_rad_;
  double action_a_yaw_max_reverse_thrust_n_;
  double action_a_yaw_thrust_ramp_n_s_, action_a_yaw_thrust_down_rate_n_s_;
  double action_a_yaw_kp_n_rad_, action_a_yaw_max_rate_rad_s_;
  double action_a_yaw_wrong_way_limit_rad_;
  bool action_a_diagonal_assist_enabled_;
  double action_a_diagonal_assist_thrust_n_;
  double action_a_diagonal_assist_thrust_ramp_n_s_;
  double action_a_diagonal_assist_thrust_down_rate_n_s_;
  double action_a_arm0_rebend_pressure_kpa_;
  double action_a_yaw_start_rad_ = 0.0;
  double action_a_yaw_signed_target_rad_ = 0.0;
  bool action_a_yaw_assist_active_ = false;
  double action_a_reach_thrust_ceiling_n_ = 0.0;
  double action_a_previous_body_tilt_rad_ = 0.0;
  double action_a_best_reach_bend_rad_ = M_PI;
  double action_a_bottom_low_kpa_, action_a_bottom_high_kpa_;
  double bottom_pressure_kpa_ = NAN;
  double rocking_bottom_low_kpa_, rocking_bottom_high_kpa_;
  double rocking_front_release_kpa_, rocking_rear_release_kpa_;
  double rocking_front_windup_pressure_kpa_;
  double rocking_front_extend_thrust_n_, rocking_rear_reverse_thrust_n_;
  double rocking_front_windup_target_rad_, rocking_front_extend_target_rad_;
  double rocking_front_place_bend_target_rad_;
  double rocking_front_place_pressure_kpa_, rocking_front_min_advance_m_;
  double rocking_front_angle_tolerance_rad_;
  double rocking_front_tilt_target_rad_, rocking_recovery_tilt_target_rad_;
  double rocking_phase_timeout_sec_, rocking_thrust_ramp_n_s_;
  double rocking_front_windup_hold_sec_, rocking_front_extend_hold_sec_;
  double rocking_front_lower_rate_n_s_;
  double rocking_land_settle_sec_, rocking_max_tilt_rad_;
  double rocking_phase_start_roll_ = 0.0, rocking_phase_start_pitch_ = 0.0;
  double rocking_rear_recover_start_travel_m_ = 0.0;
  enum RockingSwingPhase : uint8_t {
    ROCK_SWING_RELEASE = 0,
    ROCK_SWING_EXTEND = 1,
    ROCK_SWING_PLACE = 2,
    ROCK_SWING_REGRIP = 3
  };
  uint8_t rocking_swing_phase_ = ROCK_SWING_RELEASE;
  ros::Time rocking_swing_phase_start_, rocking_swing_hold_start_;
  double action_b_front_duration_sec_, action_b_thrust_ramp_n_s_;
  double action_b_front_thrust_ramp_up_n_s_;
  double action_b_shape_target_thrust_n_;
  double action_b_prediction_horizon_sec_;
  double action_b_front_transition_tilt_rad_;
  double action_b_min_lift_tilt_rad_;
  double action_b_hold_tilt_tolerance_rad_, action_b_front_regrip_timeout_sec_;
  double action_b_hold_thrust_up_rate_n_s_, action_b_hold_thrust_down_rate_n_s_;
  double action_b_hold_stable_duration_sec_, action_b_pulse_amplitude_n_;
  double action_b_max_thrust_n_, action_b_pre_pulse_max_thrust_n_;
  double action_b_pulse_on_duration_sec_, action_b_pulse_off_duration_sec_;
  double action_b_shape_max_pressure_kpa_, action_b_shape_pressure_rate_kpa_s_;
  double action_b_shape_search_timeout_sec_;
  double action_b_shape_settle_timeout_sec_, action_b_hold_tilt_kp_n_rad_;
  double action_b_hold_tilt_ki_n_rad_s_, action_b_hold_tilt_integral_n_ = 0.0;
  double action_b_pulse_angle_pressure_gain_kpa_rad_;
  double action_b_hold_thrust_n_ = 0.0;
  double action_b_current_front_angle_rad_ = 0.0;
  double action_b_initial_hold_thrust_n_ = 0.0;
  double action_b_shape_pressure_kpa_ = 0.0;
  double action_b_shape_start_angle_rad_ = 0.0;
  double action_b_last_bend_angle_rad_ = 0.0;
  double action_b_bend_rate_rad_s_ = 0.0;
  ros::Time action_b_last_bend_time_;
  double action_b_best_pressure_kpa_ = 0.0, action_b_best_thrust_n_ = 0.0;
  double action_b_best_horizontal_force_n_ = -1.0;
  PressureArray action_b_pulse_pressure_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_b_pre_pulse_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_b_shape_start_neuron_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_b_shape_start_rotor_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  int action_b_shape_phase_ = 0;
  int action_b_pulse_count_ = 0, action_b_pulse_repetitions_ = 3;
  bool action_b_pulse_on_ = false;
  bool action_b_pressure_limited_by_feedback_ = false;
  bool action_b_lift_target_adapted_ = false;
  double action_c_max_thrust_n_;
  double action_c_extension_target_rad_, action_c_max_extension_rad_;
  double action_c_max_extension_mismatch_rad_;
  double action_c_min_safe_angle_rad_, action_c_extension_rate_limit_rad_s_;
  double action_c_hard_min_bend_rad_;
  double action_c_start_extension_margin_rad_;
  double action_c_bottom_deflation_tolerance_kpa_ = 5.0;
  hugmy::RollTrendGate action_c_roll_trend_;
  double action_c_posture_hard_roll_rad_ = 15.0 * M_PI / 180.0;
  hugmy::PressureReadyGate action_c_pressure_ready_gate_;
  double action_c_pressure_hold_tolerance_kpa_ = 5.0;
  double action_c_pressure_dropout_sec_ = 0.10;
  double action_c_pressure_max_deviation_kpa_ = 8.0;
  hugmy::hybrid::Config hybrid_config_;
  hugmy::hybrid::SinglePassConfig hybrid_single_config_;
  hugmy::hybrid::SinglePass hybrid_single_;
  std::array<hugmy::hybrid::BendCoordinateRate,2> hybrid_sim_rate_;
  bool hybrid_hold_bottom_=false;
  hugmy::hybrid::Planner hybrid_planner_;
  hugmy::hybrid::Search hybrid_search_;
  hugmy::hybrid::Result hybrid_result_;
  hugmy::hybrid::PositionWindow hybrid_position_window_, hybrid_settled_window_;
  hugmy::hybrid::PositionWindow hybrid_endpoint_mocap_window_;
  hugmy::hybrid::PositionWindow hybrid_endpoint_pitch_window_;
  hugmy::hybrid::PositionWindow hybrid_bottom_window_;
  hugmy::hybrid::BodyRateFilter hybrid_body_rate_filter_;
  hugmy::hybrid::BottomAssist hybrid_bottom_assist_;
  hugmy::hybrid::PressureGuard hybrid_pressure_guard_;
  hugmy::hybrid::ProbeBudget hybrid_probe_budget_;
  Vector3 hybrid_contact_position_{{0.,0.,-.07}};
  PressureArray hybrid_regrip_thrust_{},hybrid_regrip_pressure_{};
  PressureArray hybrid_grasp_pressure_{};
  bool hybrid_retain_grasp_pressure_=false,hybrid_hold_pressure_=false;
  double hybrid_regrip_bottom_=0.;
  double hybrid_regrip_x_=0.,hybrid_settling_loss_=0.;
  hugmy::PressureReadyGate hybrid_pressure_ready_gate_;
  bool hybrid_start_mean_valid_ = false, hybrid_end_mean_valid_ = false;
  bool hybrid_preinflate_measured_ = false;
  bool hybrid_terminal_settle_ = false;
  bool hybrid_terminal_retry_ = false;
  std::string hybrid_terminal_reason_;
  bool hybrid_bottom_changed_ = false;
  double hybrid_end_mean_x_ = 0.;
  hugmy::hybrid::Pair hybrid_start_bend_{};
  ros::Publisher hybrid_diagnostic_pub_;
  double hybrid_task_start_ = -1., hybrid_trial_start_ = -1., hybrid_run_start_ = -1.;
  double hybrid_ready_since_ = -1., hybrid_run_x_ = 0., hybrid_trial_x_ = 0.;
  double hybrid_last_x_ = 0., hybrid_last_stamp_ = -1., hybrid_speed_ = 0.;
  double hybrid_pitch_start_ = 0., hybrid_recovery_start_ = -1., hybrid_pre_recovery_dx_ = 0.;
  double hybrid_recovery_x_ = 0.;
  // Confirm a roll-rate violation before handing control to posture recovery.
  // Absolute roll remains an immediate guard; this timer rejects a one-sample
  // impact/IMU spike without weakening the configured angular-rate limit.
  double hybrid_roll_rate_confirm_sec_ = 0.10;
  double hybrid_roll_rate_limit_since_ = -1.;
  double hybrid_effort_stamp_ = -1.;
  PressureArray hybrid_sent_thrust_{};
  bool hybrid_trial_active_ = false;
  bool hybrid_candidate_selected_ = false;
  bool hybrid_prediction_limited_ = false;
  int hybrid_phase_ = 0;
  double action_c_rear_release_max_kpa_ = 8.0;
  double action_c_rear_release_hold_kpa_ = 10.0;
  double action_c_rear_release_hard_kpa_ = 12.0;
  double action_c_rear_release_grace_sec_ = 0.10;
  hugmy::RearReleaseGate action_c_rear_release_gate_;
  double action_c_abort_reverse_travel_m_ = 0.03;
  ros::Time measured_pressure_stamp_;
  ros::Time measured_bottom_pressure_stamp_;
  PressureArray neuron_acc_norm_{};
  double action_c_pre_roll_rate_tolerance_rad_s_;
  double action_c_prepare_timeout_sec_;
  double action_c_posture_recovery_trigger_rad_;
  double action_c_posture_resume_rad_;
  double action_c_posture_rate_trigger_rad_s_;
  double action_c_emergency_stop_roll_rad_ = M_PI / 2.0;
  int action_c_max_posture_recoveries_;
  // Goal and resume state shared by predictive locomotion and roll recovery.
  double action_c_absolute_target_x_m_ = 0.0;
  double action_c_direction_sign_x_ = 1.0;
  Vector2 action_c_resume_direction_{{1.0, 0.0}};
  int action_c_posture_recovery_count_ = 0;
  bool action_c_resume_pending_ = false;
  bool action_c_preserve_adaptation_on_start_ = false;
  double action_d_target_angle_rad_;
  double action_d_soft_min_angle_rad_, action_d_soft_max_angle_rad_;
  double action_d_hard_min_angle_rad_, action_d_hard_max_angle_rad_;
  double action_d_max_angle_rate_rad_s_;
  double action_d_immediate_angle_rate_rad_s_;
  double action_d_angle_rate_confirm_sec_;
  double action_d_rear_extension_margin_rad_;
  double action_d_rear_rate_filter_alpha_;
  double action_d_prediction_horizon_sec_;
  double action_d_pressure_kp_kpa_rad_, action_d_pressure_rate_kpa_s_;
  double action_d_max_pressure_kpa_, action_d_bottom_pressure_kpa_;
  double action_d_front_pressure_per_thrust_kpa_n_;
  double action_d_front_pressure_lead_n_;
  double action_d_rear_pressure_drop_per_thrust_kpa_n_;
  double action_d_rear_min_pressure_kpa_;
  double action_d_front_thrust_ramp_n_s_, action_d_rear_thrust_ramp_n_s_;
  double action_d_thrust_step_n_;
  double action_d_pulse_on_sec_, action_d_pulse_off_sec_;
  double action_d_thrust_down_rate_n_s_;
  double action_d_max_front_thrust_n_, action_d_max_rear_thrust_n_;
  double action_d_prepare_tolerance_rad_, action_d_prepare_stable_sec_;
  double action_d_prepare_timeout_sec_;
  double action_d_ramp_timeout_sec_, action_d_max_tilt_rad_;
  double action_d_tilt_soft_limit_rad_, action_d_tilt_resume_rad_;
  double action_d_max_body_rate_rad_s_;
  double action_d_posture_thrust_down_rate_n_s_;
  double action_d_motion_min_front_thrust_n_;
  double action_d_motion_min_travel_m_, action_d_motion_confirm_sec_;
  PressureArray action_d_pressure_{{30.0, 30.0, 30.0, 30.0}};
  PressureArray action_d_initial_pressure_{{30.0, 30.0, 30.0, 30.0}};
  PressureArray action_d_previous_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_d_ramp_reference_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_d_filtered_extension_rate_rad_s_{{0.0, 0.0, 0.0, 0.0}};
  PressureArray action_d_pressure_bend_sign_{{1.0, 1.0, 1.0, 1.0}};
  PressureArray action_d_probe_pressure_kpa_{{30.0, 30.0, 30.0, 30.0}};
  PressureArray action_d_probe_angle_rad_{{0.0, 0.0, 0.0, 0.0}};
  std::array<ros::Time, ARM_COUNT> action_d_previous_neuron_stamp_{};
  double action_d_front_thrust_n_ = 0.0, action_d_rear_thrust_n_ = 0.0;
  double action_d_thrust_step_target_n_ = 0.0;
  double action_d_pulse_start_travel_m_ = 0.0;
  double action_d_locked_front_thrust_n_ = 0.0;
  double action_d_locked_rear_thrust_n_ = 0.0;
  double action_d_start_roll_ = 0.0, action_d_start_pitch_ = 0.0;
  ros::Time action_d_previous_angle_time_;
  ros::Time action_d_pulse_phase_start_;
  ros::Time action_d_motion_candidate_start_;
  std::array<ros::Time, ARM_COUNT> action_d_rapid_extension_start_{};
  bool action_d_limit_locked_ = false;
  bool action_d_posture_limited_ = false;
  uint8_t action_d_pulse_phase_ = 0;
  bool mocap_topic_route_initialized_ = false;
  bool use_root_mocap_topic_ = false;
  bool hand_mocap_topic_route_initialized_ = false;
  bool use_root_hand_mocap_topic_ = false;
  bool hand_relative_mocap_enabled_ = false;
  bool hand_target_selects_direction_ = false;
  double hand_progress_direction_sign_x_ = 1.0;
  bool use_mocap_attitude_for_inchworm_ = false;
  double fixed_test_pressure_kpa_, fixed_test_thrust_n_;
  int fixed_test_arm_index_;
  bool state_machine_enabled_;
  bool fixed_arm_test_enabled_, use_pwm_test_for_correction_;
  bool inchworm_action_b_enabled_ = false;
  bool inchworm_action_c_enabled_ = false;
  bool inchworm_action_d_enabled_ = false;
  bool rocking_gait_enabled_ = false;
  bool action_a_bottom_enabled_ = true;
  bool require_arm_geometry_for_prepare_;
  bool use_root_spinal_topics_ = false;
  bool spinal_topic_route_initialized_ = false;
  bool gravity_compensation_, enabled_ = false;
  bool grasp_baseline_valid_ = false;
  bool mocap_received_ = false, hand_mocap_received_ = false;
  bool inchworm_active_ = false;
  bool inchworm_motion_detected_ = false;
  double inchworm_target_distance_m_ = 0.002;
  double inchworm_total_target_distance_m_ = 0.002;
  double inchworm_last_travel_m_ = 0.0, inchworm_travel_speed_m_s_ = 0.0;
  double inchworm_best_pull_travel_m_ = 0.0;
  int inchworm_cycle_count_ = 0, inchworm_max_cycles_ = 10;
  bool imu_received_ = false, thrust_received_ = false, target_received_ = false;
};

#endif
