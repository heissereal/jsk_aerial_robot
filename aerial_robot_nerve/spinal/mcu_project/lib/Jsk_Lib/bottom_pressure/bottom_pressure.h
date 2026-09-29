#ifndef __BOTTOM_PRESSURE_H
#define __BOTTOM_PRESSURE_H

#include <cmath>

#include <config.h>
#include <ros.h>
#include <spinal/Pressure.h>

#define BOTTOM_PRESSURE_CHECK_INTERVAL 20 // ms
#define BOTTOM_PRESSURE_ROS_PUB_INTERVAL 20 // ms

/**
 * Pressure transducer connected to the flight-controller ADC.
 *
 * Battery voltage is supplied by ESC telemetry on the STM32H7_v2 target, so
 * the former battery-sense ADC input is dedicated to the bottom inflatable.
 */
class BottomPressure
{
public:
  BottomPressure(): pressure_pub_("pressure/bottom", &pressure_msg_)
  {
  }

  void init(ADC_HandleTypeDef* hadc, ros::NodeHandle* nh)
  {
    hadc_ = hadc;
    nh_ = nh;
    raw_ = 0;
    valid_ = false;
    ros_pub_last_time_ = HAL_GetTick();
    nh_->advertise(pressure_pub_);
    HAL_ADC_Start(hadc_);
  }

  void update()
  {
    if (HAL_ADC_PollForConversion(hadc_, 10) == HAL_OK)
      {
        raw_ = static_cast<uint16_t>(HAL_ADC_GetValue(hadc_));
        valid_ = true;
      }
    else
      {
        valid_ = false;
      }

    HAL_ADC_Start(hadc_);

    const uint32_t now = HAL_GetTick();
    if (now - ros_pub_last_time_ < BOTTOM_PRESSURE_ROS_PUB_INTERVAL) return;
    ros_pub_last_time_ = now;

    pressure_msg_.stamp = nh_->now();
    pressure_msg_.raw = raw_;
    pressure_msg_.valid = valid_;
    if (valid_)
      {
        pressure_msg_.adc_voltage = adcVoltage(raw_);
        pressure_msg_.sensor_voltage = sensorVoltage(raw_);
        pressure_msg_.pressure = pressureFromRaw(raw_);
      }
    else
      {
        pressure_msg_.adc_voltage = NAN;
        pressure_msg_.sensor_voltage = NAN;
        pressure_msg_.pressure = NAN;
      }
    pressure_pub_.publish(&pressure_msg_);
  }

private:
  static constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
  static constexpr float ADC_MAX_VALUE = 4095.0f;
  static constexpr float DIVIDER_INPUT_RESISTANCE = 10.0f;
  static constexpr float DIVIDER_GROUND_RESISTANCE = 15.0f;
  static constexpr float SENSOR_OFFSET_VOLTAGE = 0.2243f;
  static constexpr float SENSOR_FULL_SCALE_SPAN = 3.75f;
  static constexpr float SENSOR_FULL_SCALE_PRESSURE = 103.421f;

  static float adcVoltage(uint16_t raw)
  {
    return raw * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
  }

  static float sensorVoltage(uint16_t raw)
  {
    return adcVoltage(raw) *
      (DIVIDER_INPUT_RESISTANCE + DIVIDER_GROUND_RESISTANCE) /
      DIVIDER_GROUND_RESISTANCE;
  }

  static float pressureFromRaw(uint16_t raw)
  {
    return (sensorVoltage(raw) - SENSOR_OFFSET_VOLTAGE) /
      SENSOR_FULL_SCALE_SPAN * SENSOR_FULL_SCALE_PRESSURE;
  }

  ADC_HandleTypeDef* hadc_ = nullptr;
  ros::NodeHandle* nh_ = nullptr;
  uint16_t raw_ = 0;
  bool valid_ = false;
  uint32_t ros_pub_last_time_ = 0;
  spinal::Pressure pressure_msg_;
  ros::Publisher pressure_pub_;
};

#endif
