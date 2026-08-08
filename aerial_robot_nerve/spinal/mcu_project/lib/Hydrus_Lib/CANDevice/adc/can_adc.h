#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_

#include "CAN/can_device.h"

class CANADC : public CANDevice
{
public:
  CANADC() : raw_(0) {}
  explicit CANADC(uint8_t slave_id)
    : CANDevice(CAN::DEVICEID_ADC, slave_id), raw_(0) {}

  void sendData() override {}
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                           uint32_t DLC, uint8_t* data) override;

  uint16_t getRaw() const { return raw_; }
  float getVoltage() const { return raw_ * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE; }

private:
  static constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
  static constexpr float ADC_MAX_VALUE = 4095.0f;

  uint16_t raw_;
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_ */
