#include "adc.h"

void NeuronADC::init(ADC_HandleTypeDef* hadc)
{
  hadc_ = hadc;
  value_ = 0;
  last_send_time_ = 0;

  HAL_ADCEx_Calibration_Start(hadc_, ADC_SINGLE_ENDED);
}

void NeuronADC::sendData()
{
  if (hadc_ == nullptr) return;

  const uint32_t now = HAL_GetTick();
  if (now - last_send_time_ < SEND_INTERVAL_MS) return;

  if (HAL_ADC_Start(hadc_) != HAL_OK) return;
  if (HAL_ADC_PollForConversion(hadc_, 1) != HAL_OK)
    {
      HAL_ADC_Stop(hadc_);
      return;
    }

  value_ = static_cast<uint16_t>(HAL_ADC_GetValue(hadc_));
  HAL_ADC_Stop(hadc_);

  sendMessage(CAN::MESSAGEID_SEND_ADC, m_slave_id, sizeof(value_), reinterpret_cast<uint8_t*>(&value_), 1);
  last_send_time_ = now;
}

void NeuronADC::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  return;
}
