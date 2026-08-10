/*
 * motor.cpp
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer : Bakui Chou
 */

#include "motor.h"
#include "CAN/can_device_manager.h"

void Motor::init(TIM_HandleTypeDef* htim)
{
  pwm_htim_ = htim;
  HAL_TIM_PWM_Start(pwm_htim_,TIM_CHANNEL_1);

#ifdef STM32G4
  HAL_TIM_PWM_Start(pwm_htim_,TIM_CHANNEL_2);
#endif
}

void Motor::sendData()
{
  return;
}

void Motor::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  auto getPwmFromData = [=](int index)->uint16_t {
                          switch (index)
                            {
                            case 1:
                              return ((data[1] << 8) & 0x300) | (data[0] & 0xFF);
                            case 2:
                              return ((data[2] << 6) & 0x3C0) | ((data[1] >> 2) & 0x3F);
                            case 3:
                              return ((data[3] << 4) & 0x3F0) | ((data[2] >> 4) & 0x0F);
                            case 4:
                              return ((data[5] << 8) & 0x300) | (data[4] & 0xFF);
                            case 5:
                              return ((data[6] << 6) & 0x3C0) | ((data[5] >> 2) & 0x3F);
                            case 6:
                              return ((data[7] << 4) & 0x3F0) | ((data[6] >> 4) & 0x0F);
                            default:
                              return 0;
                            }
                        };

  if (message_id == CAN::MESSAGEID_RECEIVE_PWM_0_5 && 1 <= m_slave_id && m_slave_id <= 6) {
#ifndef STM32G4
    setSyncPwm(getPwmFromData(m_slave_id));
#endif
  } else if (message_id == CAN::MESSAGEID_RECEIVE_PWM_6_11 && 7 <= m_slave_id && m_slave_id <= 12) {
#ifndef STM32G4
    setSyncPwm(getPwmFromData(m_slave_id - 6));
#endif
  } else if (message_id == CAN::MESSAGEID_RECEIVE_VALVE_PWM_CH1 && 1 <= m_slave_id && m_slave_id <= 6) {
    setValvePwm(0, getPwmFromData(m_slave_id));
  } else if (message_id == CAN::MESSAGEID_RECEIVE_VALVE_PWM_CH2 && 1 <= m_slave_id && m_slave_id <= 6) {
    setValvePwm(1, getPwmFromData(m_slave_id));
  }
}

void Motor::setValvePwm(uint8_t channel, uint16_t pwm)
{
  if (channel >= 2) return;
  valve_pwm_[channel] = std::min<uint16_t>(pwm, 1000);
  applyValvePwm();
}

void Motor::applyValvePwm()
{
#ifdef STM32G4
  uint16_t ch1 = valve_pwm_[0];
  uint16_t ch2 = valve_pwm_[1];
  if (ch1 > 0 && ch2 > 0) ch1 = ch2 = 0;
  pwm_htim_->Instance->CCR1 = static_cast<uint32_t>(ch1) * pwm_htim_->Init.Period / 1000;
  pwm_htim_->Instance->CCR2 = static_cast<uint32_t>(ch2) * pwm_htim_->Init.Period / 1000;
#endif
}

void Motor::setSyncPwm(uint16_t pwm)
{
  uint32_t raw_val = (pwm + 1000.0f) / 2000.0f * pwm_htim_->Init.Period;

  pwm_htim_->Instance->CCR1 = raw_val;

#ifdef STM32G4
  // CH2 is reserved for the second independently controlled valve.
#endif

  return;
}

void Motor::update()
{
  if (CANDeviceManager::isTimeout()) {
    valve_pwm_[0] = valve_pwm_[1] = 0;
    applyValvePwm();
#ifndef STM32G4
    setSyncPwm(0); //force stop all outputs
#endif
  }
}
