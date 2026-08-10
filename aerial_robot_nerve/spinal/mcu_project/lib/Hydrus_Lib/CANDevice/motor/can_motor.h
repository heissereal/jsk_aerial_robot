/*
 * can_motor.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_

#include "CAN/can_device.h"
#include <algorithm>
#include <functional>

class CANMotor : public CANDevice
{
private:
	uint16_t m_pwm;
	uint16_t valve_pwm_[2];
public:
	CANMotor() : m_pwm(0), valve_pwm_{0, 0} {}
	CANMotor(uint8_t slave_id) : CANDevice(CAN::DEVICEID_MOTOR, slave_id), m_pwm(0), valve_pwm_{0, 0} {}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	void setPwm(uint16_t pwm){m_pwm = pwm;}
	uint16_t getPwm()const {return m_pwm;}
	void setValvePwm(uint8_t channel, uint16_t pwm)
	{
		if (channel < 2) valve_pwm_[channel] = std::min<uint16_t>(pwm, 1000);
	}
	uint16_t getValvePwm(uint8_t channel) const { return channel < 2 ? valve_pwm_[channel] : 0; }
};

class CANMotorSendDevice : public CANDevice
{
private:
	std::vector<std::reference_wrapper<CANMotor>> can_motor_;
public:
	CANMotorSendDevice():CANDevice(CAN::DEVICEID_MOTOR, CAN::BROADCAST_ID) {}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	void addMotor(CANMotor& motor) {can_motor_.push_back(motor);}
};



#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_ */
