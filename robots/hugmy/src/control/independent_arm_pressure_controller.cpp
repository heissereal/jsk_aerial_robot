#include <hugmy/control/independent_arm_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <vector>

IndependentArmPressureController::IndependentArmPressureController(ros::NodeHandle& nh,
                                                                   ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  pnh_.param("kp", kp_, 0.08);
  pnh_.param("ki", ki_, 0.0);
  pnh_.param("deadband_kpa", deadband_kpa_, 0.7);
  pnh_.param("minimum_duty", minimum_duty_, 0.20); //pump duty below this value is not effective
  pnh_.param("maximum_duty", maximum_duty_, 0.80);
  pnh_.param("exhaust_maximum_duty", exhaust_maximum_duty_, 1.0);
  pnh_.param("integral_limit", integral_limit_, 20.0);
  pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
  pnh_.param("maximum_target_kpa", maximum_target_kpa_, 55.0);
  pnh_.param("pressure_limit_kpa", pressure_limit_kpa_, 60.0);

  std::vector<int> slave_ids;
  if (pnh_.getParam("arm_slave_ids", slave_ids) && slave_ids.size() == ARM_COUNT)
    std::copy(slave_ids.begin(), slave_ids.end(), slave_ids_.begin());

  pressure_.fill(NAN); //not 0 for initialization
  target_.fill(0.0);
  integral_.fill(0.0);
  adc_sub_ = nh_.subscribe("neuron/adc_states", 1, &IndependentArmPressureController::adcCallback, this);
  // bridge.launch started outside a robot namespace publishes this root topic.
  // The callback selects the matching command route automatically.
  if (nh_.resolveName("neuron/adc_states") != "/neuron/adc_states")
    root_adc_sub_ = nh_.subscribe("/neuron/adc_states", 1,
                                  &IndependentArmPressureController::rootAdcCallback, this);
  target_sub_ = pnh_.subscribe("target_pressure", 1, &IndependentArmPressureController::targetCallback, this);
  command_pub_ = nh_.advertise<spinal::PneumaticCommand>("pneumatic/command", 1);
  if (nh_.resolveName("pneumatic/command") != "/pneumatic/command")
    root_command_pub_ = nh_.advertise<spinal::PneumaticCommand>("/pneumatic/command", 1);
  pressure_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("pressure", 1);
  error_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("error", 1);
  enable_server_ = pnh_.advertiseService("enable", &IndependentArmPressureController::enableCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_hz_)), &IndependentArmPressureController::update, this);
}

void IndependentArmPressureController::adcCallback(const spinal::NeuronAdcStates::ConstPtr& msg)
{
  updatePressure(msg, false);
}

void IndependentArmPressureController::rootAdcCallback(
    const spinal::NeuronAdcStates::ConstPtr& msg)
{
  updatePressure(msg, true);
}

void IndependentArmPressureController::updatePressure(
    const spinal::NeuronAdcStates::ConstPtr& msg, bool root_topics)
{
  use_root_spinal_topics_ = root_topics;
  for (const auto& adc : msg->adcs)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      if (adc.slave_id == slave_ids_[arm])
        {
          pressure_[arm] = adc.pressure;
          break;
        }
} //adcここでも切るか見る必要があるのか？

void IndependentArmPressureController::targetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != ARM_COUNT)
    {
      ROS_ERROR("target_pressure must contain exactly four values");
      return;
    }
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (!std::isfinite(msg->data[arm]) || msg->data[arm] < 0.0 ||
        msg->data[arm] > maximum_target_kpa_)
      {
        ROS_ERROR("Invalid target pressure for arm %zu: %.3f kPa (allowed 0..%.1f)",
                  arm, msg->data[arm], maximum_target_kpa_);
        return;
      }
  std::copy(msg->data.begin(), msg->data.end(), target_.begin());
  target_received_ = true;
}
//警告だけならいらなくないか？

bool IndependentArmPressureController::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data && !target_received_)
    {
      // Enabling before the first target message must not start inflation.
      // Hold the currently measured pressures until the owner's target arrives.
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        target_[arm] = std::isfinite(pressure_[arm]) ? pressure_[arm] : 0.0;
      target_received_ = true;
    }
  enabled_ = req.data;
  resetIntegrators();
  if (!enabled_) publishDisabledCommand();
  res.success = true;
  res.message = enabled_ ? "four-arm pressure control enabled" : "pressure control disabled";
  return true;
}

void IndependentArmPressureController::update(const ros::TimerEvent& event)
{
  spinal::PneumaticCommand command; //これできるのかな？
  command.enable = enabled_;
  command.pump_pwm = 0.0f;
  const double dt = std::max(0.0, std::min(0.1, (event.current_real - event.last_real).toSec()));
  bool sensors_healthy = true;

  std_msgs::Float32MultiArray pressure_msg, error_msg;
  pressure_msg.data.resize(ARM_COUNT);
  error_msg.data.resize(ARM_COUNT);

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      command.supply_pwm[arm] = 0.0f;
      command.exhaust_pwm[arm] = 0.0f;
      pressure_msg.data[arm] = pressure_[arm];
      error_msg.data[arm] = target_[arm] - pressure_[arm];

      if (!std::isfinite(pressure_[arm]) || pressure_[arm] < -5.0 ||
          pressure_[arm] >= pressure_limit_kpa_)
        {
          sensors_healthy = false;
          continue;
        }

      const double error = target_[arm] - pressure_[arm];
      if (!enabled_ || std::abs(error) <= deadband_kpa_)
        {
          integral_[arm] = 0.0;
          continue;
        }

      integral_[arm] = std::max(-integral_limit_, std::min(integral_limit_, integral_[arm] + error * dt));
      const double effort = kp_ * std::abs(error) + ki_ * std::abs(integral_[arm]);
      if (error > 0.0)
        {
          command.supply_pwm[arm] = std::max(minimum_duty_, std::min(maximum_duty_, effort));
          command.pump_pwm = std::max(command.pump_pwm, command.supply_pwm[arm]);
        }
      else
        command.exhaust_pwm[arm] = std::min(exhaust_maximum_duty_, effort);
    }

  if (!enabled_ || !sensors_healthy)
    {
      command.enable = false;
      command.pump_pwm = 0.0f;
      resetIntegrators();
      if (enabled_ && !sensors_healthy)
        {
          enabled_ = false;
          ROS_ERROR("Pressure control latched off by missing, invalid, or over-limit sensor data");
        }
    }

  publishCommand(command);
  pressure_pub_.publish(pressure_msg);
  error_pub_.publish(error_msg);
}

void IndependentArmPressureController::publishDisabledCommand()
{
  spinal::PneumaticCommand command;
  command.enable = false;
  command.pump_pwm = 0.0f;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    command.supply_pwm[arm] = command.exhaust_pwm[arm] = 0.0f;
  publishCommand(command);
}

void IndependentArmPressureController::publishCommand(
    const spinal::PneumaticCommand& command)
{
  if (use_root_spinal_topics_ && root_command_pub_)
    root_command_pub_.publish(command);
  else
    command_pub_.publish(command);
}

void IndependentArmPressureController::resetIntegrators()
{
  integral_.fill(0.0);
}
