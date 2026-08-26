#ifndef HUGMY_INDEPENDENT_ARM_PRESSURE_CONTROLLER_H
#define HUGMY_INDEPENDENT_ARM_PRESSURE_CONTROLLER_H

#include <array>
#include <ros/ros.h>
#include <spinal/NeuronAdcStates.h>
#include <spinal/PneumaticCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>

class IndependentArmPressureController
{
public:
  static constexpr size_t ARM_COUNT = 4;
  IndependentArmPressureController(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void adcCallback(const spinal::NeuronAdcStates::ConstPtr& msg);
  void rootAdcCallback(const spinal::NeuronAdcStates::ConstPtr& msg);
  void updatePressure(const spinal::NeuronAdcStates::ConstPtr& msg, bool root_topics);
  void targetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void update(const ros::TimerEvent& event);
  void publishDisabledCommand();
  void publishCommand(const spinal::PneumaticCommand& command);
  void resetIntegrators();

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber adc_sub_, root_adc_sub_, target_sub_;
  ros::Publisher command_pub_, root_command_pub_, pressure_pub_, error_pub_;
  ros::ServiceServer enable_server_;
  ros::Timer timer_;

  std::array<int, ARM_COUNT> slave_ids_{{1, 2, 3, 4}};
  std::array<double, ARM_COUNT> pressure_;
  std::array<double, ARM_COUNT> last_valid_pressure_;
  std::array<double, ARM_COUNT> target_;
  std::array<double, ARM_COUNT> integral_;
  std::array<bool, ARM_COUNT> pressurizing_{};
  std::array<ros::WallTime, ARM_COUNT> invalid_since_;
  std::array<bool, ARM_COUNT> sensor_fault_active_{};
  double kp_, ki_, deadband_kpa_, minimum_duty_, maximum_duty_;
  double exhaust_kp_, exhaust_maximum_duty_, integral_limit_;
  double gain_schedule_min_scale_, gain_schedule_reference_kpa_;
  double pump_on_duty_, pressurize_start_error_kpa_, pressurize_stop_error_kpa_;
  double control_rate_hz_, maximum_target_kpa_, pressure_limit_kpa_;
  double sensor_fault_delay_s_;
  bool enable_gain_scheduling_;
  bool enabled_ = false;
  bool target_received_ = false;
  bool use_root_spinal_topics_ = false;
};

#endif
