#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <spinal/NeuronAdcStates.h>
#include <spinal/NeuronImuStates.h>
#include <spinal/PneumaticCommand.h>
#include <spinal/Thrust.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <hugmy/model/pressure_thrust_bend_model.h>

namespace hugmy
{
class HugmyPneumaticHWSim : public mujoco_ros_control::AerialRobotHWSim
{
public:
  static constexpr size_t ARM_COUNT = 4;
  static constexpr size_t JOINT_COUNT = 5;
  static constexpr size_t BAG_JOINT_COUNT = 3;

  bool init(const std::string& robot_namespace, ros::NodeHandle model_nh,
            mjModel* mujoco_model, mjData* mujoco_data) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  using BendTrend = PressureThrustBendModel::Trend;

  void commandCallback(const spinal::PneumaticCommand::ConstPtr& msg);
  void thrustCallback(const spinal::Thrust::ConstPtr& msg);
  void bottomDirectionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  bool resetCallback(std_srvs::Trigger::Request& request,
                     std_srvs::Trigger::Response& response);
  bool stepCallback(std_srvs::Trigger::Request& request,
                    std_srvs::Trigger::Response& response);
  void processReset(const ros::Time& time);
  void reloadEpisodeParameters();
  void updatePressure(const spinal::PneumaticCommand& command, bool fresh, double dt);
  void applyBagSprings(const std::array<double, ARM_COUNT>& thrust);
  void applyBottomInflatable();
  void applyGripForces(const ros::Time& time);
  void publishGripMarkers(
      const ros::Time& time,
      const std::array<std::array<double, 3>, ARM_COUNT>& points,
      const std::array<std::array<double, 3>, ARM_COUNT>& forces,
      const std::array<double, ARM_COUNT>& slip_ratios,
      const std::array<double, ARM_COUNT>& normal_forces,
      const std::array<double, ARM_COUNT>& tangential_forces,
      const std::array<bool, ARM_COUNT>& active);
  void publishSensors(const ros::Time& time);
  void publishActuatorVisualization(
      const ros::Time& time,
      const std::array<double, ARM_COUNT>& thrust);
  static double clamp01(double value);
  static BendTrend updateTrend(double value, double previous,
                               BendTrend current, double epsilon);

  std::array<std::array<int, JOINT_COUNT>, ARM_COUNT> joint_ids_{};
  std::array<std::array<int, JOINT_COUNT>, ARM_COUNT> actuator_ids_{};
  std::array<int, ARM_COUNT> rotor_site_ids_{};
  std::array<int, ARM_COUNT> acc_sensor_ids_{};
  std::array<int, ARM_COUNT> gyro_sensor_ids_{};
  std::array<int, ARM_COUNT> grip_body_ids_{};
  int grip_target_body_id_ = -1;
  int grip_target_geom_id_ = -1;
  int main_body_id_ = -1;
  int bottom_inflatable_geom_id_ = -1;

  ros::Subscriber command_sub_, thrust_sub_, bottom_direction_sub_;
  ros::Publisher adc_pub_, neuron_imu_pub_, grip_marker_pub_, grip_state_pub_;
  ros::Publisher bottom_pressure_pub_, bottom_state_pub_;
  ros::Publisher actuator_state_pub_, actuator_marker_pub_;
  ros::Publisher reset_count_pub_, step_count_pub_;
  ros::ServiceServer reset_server_, step_server_;
  std::mutex command_mutex_;
  spinal::PneumaticCommand command_;
  ros::Time command_stamp_, last_sensor_time_, last_pressure_time_;
  ros::Time last_grip_marker_time_;
  ros::Time last_actuator_marker_time_;
  bool command_received_ = false;
  std::atomic<bool> reset_requested_{false};
  uint32_t reset_count_ = 0;
  uint32_t step_count_ = 0;
  bool gym_step_mode_ = false;
  double gym_step_duration_s_ = 0.1;
  double gym_physics_timestep_s_ = 0.0;
  size_t gym_step_iterations_ = 1;
  size_t permitted_iterations_ = 0;
  std::atomic<bool> gym_client_connected_{false};
  std::mutex step_mutex_;
  std::condition_variable step_condition_;
  std::string pneumatic_namespace_;
  std::vector<mjtNum> initial_qpos_;
  std::vector<mjtNum> initial_qvel_;

  std::array<double, ARM_COUNT> pressure_kpa_{};
  double bottom_pressure_kpa_ = 0.0;
  std::array<double, 2> bottom_direction_body_{{1.0, 0.0}};
  std::array<double, ARM_COUNT> thrust_n_{};
  std::array<double, ARM_COUNT> previous_model_pressure_{};
  std::array<double, ARM_COUNT> previous_model_thrust_{};
  std::array<BendTrend, ARM_COUNT> pressure_trend_{{
      BendTrend::NEUTRAL, BendTrend::NEUTRAL, BendTrend::NEUTRAL, BendTrend::NEUTRAL}};
  std::array<BendTrend, ARM_COUNT> thrust_trend_{{
      BendTrend::NEUTRAL, BendTrend::NEUTRAL, BendTrend::NEUTRAL, BendTrend::NEUTRAL}};
  PressureThrustBendModel bend_model_;

  double pressure_rate_hz_ = 100.0;
  double sensor_rate_hz_ = 100.0;
  double supply_rate_kpa_s_ = 25.0;
  double exhaust_rate_kpa_s_ = 18.0;
  double bottom_supply_rate_kpa_s_ = 100.0;
  double bottom_exhaust_rate_kpa_s_ = 80.0;
  double maximum_pressure_kpa_ = 60.0;
  double initial_pressure_kpa_ = 0.0;
  double leak_rate_per_s_ = 0.0;
  double bag_stiffness_nm_rad_ = 1.0;
  double structural_stiffness_nm_rad_ = 0.01;
  double bag_damping_nms_rad_ = 0.005;
  double thrust_offset_n_ = 1.8;
  double thrust_scale_ = 1.0;
  double command_timeout_s_ = 0.25;
  double pressure_noise_kpa_ = 0.05;
  double imu_acc_noise_ms2_ = 0.017;
  double imu_gyro_noise_rads_ = 0.0002;
  bool spawn_human_ = false;
  double human_arm_center_z_ = 0.05;
  double human_arm_half_length_m_ = 0.25;
  double human_arm_radius_m_ = 0.05;
  double human_arm_roll_rad_ = 0.0;
  double human_arm_pitch_rad_ = 0.0;
  double human_arm_yaw_rad_ = 0.0;
  double grip_range_m_ = 0.035;
  double grip_stiffness_n_m_ = 180.0;
  double grip_damping_ns_m_ = 2.0;
  double grip_force_per_kpa_n_ = 0.08;
  double grip_min_pressure_kpa_ = 1.5;
  double grip_friction_coefficient_ = 1.2;
  double grip_tangent_damping_ns_m_ = 4.0;
  double grip_marker_scale_m_n_ = 0.025;
  double grip_marker_rate_hz_ = 30.0;
  std::array<double, ARM_COUNT> grip_axis_coordinate_m_{};
  std::array<double, ARM_COUNT> grip_azimuth_rad_{};
  std::array<double, ARM_COUNT> grip_surface_gap_m_{};
  std::vector<double> previous_grip_qfrc_;
  std::vector<double> previous_bottom_qfrc_;
  double bottom_force_per_kpa_n_ = 0.12;
  double bottom_lever_arm_m_ = 0.0;
  double bottom_point_z_m_ = -0.050;
  double bottom_contact_range_m_ = 0.080;
  double bottom_visual_base_height_m_ = 0.003;
  double bottom_visual_height_per_kpa_m_ = 0.00025;
  bool bottom_virtual_tilt_enabled_ = false;
  double bottom_tilt_reference_pressure_kpa_ = 50.0;
  double bottom_tilt_at_reference_rad_ = 0.2617993877991494;
  double bottom_tilt_stiffness_nm_rad_ = 50.0;
  double bottom_tilt_damping_nms_rad_ = 3.0;
  double bottom_tilt_max_torque_nm_ = 5.0;
  double bottom_tilt_reference_reset_pressure_kpa_ = 0.5;
  std::array<double, 9> bottom_reference_rotation_{{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0}};
  bool bottom_reference_initialized_ = false;
  double bottom_tilt_target_rad_ = 0.0;
  double bottom_tilt_measured_rad_ = 0.0;
  double bottom_virtual_torque_nm_ = 0.0;

  std::default_random_engine random_engine_{1};
  std::normal_distribution<double> normal_{0.0, 1.0};
};
}  // namespace hugmy
