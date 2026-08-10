#include "can_adc.h"

#include <string.h>

void CANADC::receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                                 uint32_t DLC, uint8_t* data)
{
  if (message_id != CAN::MESSAGEID_SEND_ADC || DLC < sizeof(raw_)) return;
  memcpy(&raw_, data, sizeof(raw_));
  last_receive_time_ = HAL_GetTick();
  received_ = true;
}
