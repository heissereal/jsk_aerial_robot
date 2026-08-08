#ifndef APPLICATION_ADC_ADC_H_
#define APPLICATION_ADC_ADC_H_

#include "CAN/can_device.h"

class NeuronADC : public CANDevice
{
public:
  NeuronADC() : hadc_(nullptr), value_(0), last_send_time_(0) {}
  NeuronADC(uint8_t slave_id) : CANDevice(CAN::DEVICEID_ADC, slave_id), hadc_(nullptr), value_(0), last_send_time_(0) {}
  ~NeuronADC() {}

  void init(ADC_HandleTypeDef* hadc);
  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

private:
  ADC_HandleTypeDef* hadc_;
  static constexpr uint32_t SEND_INTERVAL_MS = 20; // 50Hz
  uint16_t value_;
  uint32_t last_send_time_;
};

#endif /* APPLICATION_ADC_ADC_H_ */
