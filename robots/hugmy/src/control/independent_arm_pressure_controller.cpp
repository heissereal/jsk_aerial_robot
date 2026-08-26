#include <hugmy/control/independent_arm_pressure_controller.h>

#include <algorithm>
#include <cmath>
#include <vector>

IndependentArmPressureController::IndependentArmPressureController(ros::NodeHandle& nh,
                                                                   ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  pnh_.param("kp", kp_, 0.38);
  pnh_.param("ki", ki_, 0.03);
  pnh_.param("deadband_kpa", deadband_kpa_, 2.0);
  pnh_.param("minimum_duty", minimum_duty_, 0.20);
  pnh_.param("maximum_duty", maximum_duty_, 0.80);
  pnh_.param("exhaust_kp", exhaust_kp_, 0.05);
  pnh_.param("exhaust_maximum_duty", exhaust_maximum_duty_, 1.0);
  pnh_.param("enable_gain_scheduling", enable_gain_scheduling_, true);
  pnh_.param("gain_schedule_min_scale", gain_schedule_min_scale_, 0.10);
  pnh_.param("gain_schedule_reference_kpa", gain_schedule_reference_kpa_, 60.0);
  pnh_.param("pump_on_duty", pump_on_duty_, 0.80);
  pnh_.param("pressurize_start_error_kpa", pressurize_start_error_kpa_, 1.0);
  pnh_.param("pressurize_stop_error_kpa", pressurize_stop_error_kpa_, 0.0);
  pnh_.param("integral_limit", integral_limit_, 20.0);
  pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
  pnh_.param("maximum_target_kpa", maximum_target_kpa_, 55.0);
  pnh_.param("pressure_limit_kpa", pressure_limit_kpa_, 60.0);
  pnh_.param("sensor_fault_delay_s", sensor_fault_delay_s_, 0.5);

  std::vector<int> slave_ids;
  if (pnh_.getParam("arm_slave_ids", slave_ids) && slave_ids.size() == ARM_COUNT)
    std::copy(slave_ids.begin(), slave_ids.end(), slave_ids_.begin());

  pressure_.fill(NAN);
  last_valid_pressure_.fill(NAN);
  target_.fill(0.0);
  integral_.fill(0.0);
  adc_sub_ = nh_.subscribe("neuron/adc_states", 1, &IndependentArmPressureController::adcCallback, this);
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
}

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
bool IndependentArmPressureController::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data && !target_received_)
    {
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
  spinal::PneumaticCommand command;
  command.enable = enabled_;
  command.pump_pwm = 0.0f;
  const double dt = std::max(0.0, std::min(0.1, (event.current_real - event.last_real).toSec()));
  bool sensors_healthy = true;
  bool any_arm_pressurizing = false;
  const ros::WallTime wall_now = ros::WallTime::now();

  std_msgs::Float32MultiArray pressure_msg, error_msg;
  pressure_msg.data.resize(ARM_COUNT);
  error_msg.data.resize(ARM_COUNT);

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      command.supply_pwm[arm] = 0.0f;
      command.exhaust_pwm[arm] = 0.0f;
      const double raw_pressure = pressure_[arm];
      const bool sample_valid = std::isfinite(raw_pressure) &&
                                raw_pressure >= -5.0 &&
                                raw_pressure < pressure_limit_kpa_;
      if (sample_valid)
        {
          last_valid_pressure_[arm] = raw_pressure;
          invalid_since_[arm] = ros::WallTime();
          if (sensor_fault_active_[arm])
            ROS_WARN("Arm %zu pressure sensor recovered; pressure control resumes automatically", arm + 1);
          sensor_fault_active_[arm] = false;
        }
      else if (invalid_since_[arm].isZero())
        invalid_since_[arm] = wall_now;

      const bool invalid_too_long = !sample_valid &&
          (wall_now - invalid_since_[arm]).toSec() >= sensor_fault_delay_s_;
      if (invalid_too_long)
        {
          sensors_healthy = false;
          if (!sensor_fault_active_[arm])
            ROS_ERROR("Arm %zu pressure sensor invalid for %.3f s (raw: %.3f kPa); control pauses until recovery",
                      arm + 1, sensor_fault_delay_s_, raw_pressure);
          sensor_fault_active_[arm] = true;
        }

      const double control_pressure = sample_valid ? raw_pressure : last_valid_pressure_[arm];
      pressure_msg.data[arm] = raw_pressure;
      error_msg.data[arm] = target_[arm] - control_pressure;
      if (!std::isfinite(control_pressure))
        {
          sensors_healthy = false;
          continue;
        }

      const double error = target_[arm] - control_pressure;
      if (!enabled_)
        {
          integral_[arm] = 0.0;
          pressurizing_[arm] = false;
          continue;
        }

      if (pressurizing_[arm])
        {
          if (error <= pressurize_stop_error_kpa_)
            {
              pressurizing_[arm] = false;
              integral_[arm] = 0.0;
            }
        }
      else if (error >= pressurize_start_error_kpa_)
        pressurizing_[arm] = true;

      if (pressurizing_[arm])
        {
          const double pressure_ratio = std::max(
              0.0, std::min(1.0, control_pressure /
                                      std::max(1.0, gain_schedule_reference_kpa_)));
          const double gain_scale = enable_gain_scheduling_
              ? gain_schedule_min_scale_ +
                    (1.0 - gain_schedule_min_scale_) * pressure_ratio
              : 1.0;
          const double scheduled_kp = kp_ * gain_scale;
          const double scheduled_ki = ki_ * gain_scale;

          // anti-windup
          const double candidate_integral = std::max(
              0.0, std::min(integral_limit_, integral_[arm] + error * dt));
          const double candidate_effort =
              scheduled_kp * error + scheduled_ki * candidate_integral;
          if (candidate_effort < maximum_duty_)
            integral_[arm] = candidate_integral;

          const double effort = scheduled_kp * error + scheduled_ki * integral_[arm];
          command.supply_pwm[arm] = std::max(minimum_duty_, std::min(maximum_duty_, effort));
          any_arm_pressurizing = true;
        }
      else if (error < -deadband_kpa_)
        {
          integral_[arm] = 0.0;
          command.exhaust_pwm[arm] =
              std::min(exhaust_maximum_duty_, exhaust_kp_ * std::abs(error));
        }
    }

  if (any_arm_pressurizing)
    command.pump_pwm = pump_on_duty_;

  if (!enabled_ || !sensors_healthy)
    {
      command.enable = false;
      command.pump_pwm = 0.0f;
      resetIntegrators();
      pressurizing_.fill(false);
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
