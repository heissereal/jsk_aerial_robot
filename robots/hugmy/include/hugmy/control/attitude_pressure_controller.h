#ifndef HUGMY_ATTITUDE_PRESSURE_CONTROLLER_H
#define HUGMY_ATTITUDE_PRESSURE_CONTROLLER_H

#include <array>
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <spinal/Imu.h>
#include <spinal/NeuronImuStates.h>
#include <spinal/PerchingThrustCommand.h>
#include <spinal/PwmTest.h>
#include <spinal/FlightConfigCmd.h>
#include <spinal/Thrust.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <hugmy/model/pressure_thrust_bend_model.h>

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
  void targetCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
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
  double measuredArmAngle(size_t arm) const;
  Vector3 rotorPositionFromAngle(size_t arm, double bend_angle) const;
  Vector3 rotorDirectionFromAngle(size_t arm, double bend_angle) const;
  bool armGeometryReady(const PressureArray& target, const ros::Time& now) const;
  void publishPressureTarget(const PressureArray& target, double maximum_step);
  void publishCorrectionThrust(const PressureArray& thrust);
  void publishPwmTest(const PressureArray& thrust);
  void publishArmShapeDebug(const PressureArray& current_relative_angle) const;
  void enterState(uint8_t state, const ros::Time& now);

  static Vector3 cross(const Vector3& lhs, const Vector3& rhs);
  static Vector3 rotateLocalToBody(const Vector3& value, double yaw);
  static Vector3 rotateAboutLocalY(const Vector3& value, double angle);
  static double wrapAngle(double angle);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber imu_sub_, root_imu_sub_, thrust_sub_, root_thrust_sub_;
  ros::Subscriber target_sub_, neuron_imu_sub_, root_neuron_imu_sub_, pressure_sub_;
  ros::Publisher pressure_target_pub_, rpy_pub_, attitude_error_pub_;
  ros::Publisher arm_relative_angle_pub_, arm_target_relative_angle_pub_;
  ros::Publisher arm_shape_error_pub_, arm_shape_active_pub_;
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
  std::array<ros::Time, ARM_COUNT> neuron_stamp_{};
  std::array<int, ARM_COUNT> neuron_slave_ids_{{1, 2, 3, 4}};
  hugmy::PressureThrustBendModel bend_model_;

  ros::Time imu_stamp_, thrust_stamp_;
  ros::Time state_start_, ready_start_, last_correction_end_, last_angle_progress_;
  ros::Time body_motion_last_seen_;
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
    RECOVER = 3
  };
  uint8_t state_ = GRASP;
  double correction_trigger_angle_, correction_stop_angle_;
  double correction_max_angular_rate_, correction_thrust_duration_;
  double prepare_timeout_, prepare_stable_duration_, recover_timeout_, cooldown_duration_;
  double pressure_tolerance_kpa_, arm_angle_tolerance_, arm_gyro_tolerance_;
  double correction_planning_thrust_n_, correction_min_thrust_n_, correction_max_thrust_n_;
  double correction_hold_min_thrust_n_, correction_motion_start_rate_;
  double correction_motion_stop_rate_, correction_slowdown_angle_;
  double correction_thrust_down_rate_, correction_motion_stop_timeout_;
  double prepare_shape_thrust_n_;
  double prepare_angle_per_moment_, prepare_min_straighten_angle_;
  double prepare_max_rotor_tilt_;
  double prepare_thrust_ramp_rate_, prepare_max_thrust_n_, pwm_test_max_thrust_n_;
  double prepare_pressure_ramp_rate_, minimum_prepare_pressure_kpa_;
  double prepare_angle_tolerance_, prepare_progress_epsilon_, prepare_progress_timeout_;
  double best_prepare_angle_ = std::numeric_limits<double>::infinity();
  Vector2 correction_axis_{{1.0, 0.0}};
  PressureArray motion_hold_thrust_{{0.0, 0.0, 0.0, 0.0}};
  bool body_motion_detected_ = false;
  double neuron_timeout_sec_;
  double pressure_safety_limit_kpa_;
  double fixed_test_pressure_kpa_, fixed_test_thrust_n_;
  int fixed_test_arm_index_;
  bool state_machine_enabled_;
  bool fixed_arm_test_enabled_, use_pwm_test_for_correction_;
  bool require_arm_geometry_for_prepare_;
  bool use_root_spinal_topics_ = false;
  bool spinal_topic_route_initialized_ = false;
  bool gravity_compensation_, enabled_ = false;
  bool grasp_baseline_valid_ = false;
  bool imu_received_ = false, thrust_received_ = false, target_received_ = false;
};

#endif
