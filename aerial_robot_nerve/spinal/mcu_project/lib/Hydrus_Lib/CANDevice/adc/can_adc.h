#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_

#include "CAN/can_device.h"

class CANADC : public CANDevice
{
public:
  CANADC() : raw_(0), last_receive_time_(0), received_(false) {}
  explicit CANADC(uint8_t slave_id)
    : CANDevice(CAN::DEVICEID_ADC, slave_id), raw_(0), last_receive_time_(0), received_(false) {}

  void sendData() override {}
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                           uint32_t DLC, uint8_t* data) override;

  uint16_t getRaw() const { return raw_; }
  float getVoltage() const { return raw_ * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE; }
  float getSensorVoltage() const
  {
    return getVoltage() * (ADC_DIVIDER_INPUT_RESISTANCE + ADC_DIVIDER_GROUND_RESISTANCE) /
      ADC_DIVIDER_GROUND_RESISTANCE;
  }
  float getPressure() const
  {
    return (getSensorVoltage() - SENSOR_OFFSET_VOLTAGE) /
      SENSOR_FULL_SCALE_SPAN * SENSOR_FULL_SCALE_PRESSURE;
  }
  bool isFresh(uint32_t now, uint32_t timeout_ms) const
  {
    return received_ && (now - last_receive_time_ <= timeout_ms);
  }

private:
  static constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
  static constexpr float ADC_MAX_VALUE = 4095.0f;
  static constexpr float ADC_DIVIDER_INPUT_RESISTANCE = 10.0f;
  static constexpr float ADC_DIVIDER_GROUND_RESISTANCE = 15.0f;
  static constexpr float SENSOR_OFFSET_VOLTAGE = 0.2243f;
  static constexpr float SENSOR_FULL_SCALE_SPAN = 3.75f;
  static constexpr float SENSOR_FULL_SCALE_PRESSURE = 103.421f;

  uint16_t raw_;
  uint32_t last_receive_time_;
  bool received_;
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_ADC_CAN_ADC_H_ */
