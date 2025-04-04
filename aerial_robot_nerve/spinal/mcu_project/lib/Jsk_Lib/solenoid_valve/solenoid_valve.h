/*
******************************************************************************
* File Name          : solenoid_valve.h
* Description        : solenoid valve interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __SOLENOID_VALVE_H
#define __SOLENOID_VALVE_H

#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <spinal/ValveControlCmd.h>

#define NUM_VALVES 2
#define SV_UPDATE_INTERVAL 10


#define SC_NONE 0x40
#define SC_SW_ADC 0x41
#define SC_PS  0x42
#define SC_IMU 0x44
#define SC_MAGENC 0x48
#define SC_ALL (SC_SW_ADC | SC_PS | SC_IMU | SC_MAGENC)

 // GPIO SC
#define SC_GPIO_NONE 0x90
#define SC_GPIO0 0x91
#define SC_GPIO1 0x92
#define SC_GPIO2 0x94
#define SC_GPIO3 0x98
#define SC_GPIO_ALL (SC_GPIO0 | SC_GPIO1 | SC_GPIO2 | SC_GPIO3)

class SolenoidValve
{
private:
  UART_HandleTypeDef* port_;
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ValveControlCmd, SolenoidValve> solenoid_valve_control_sub_;
  ros::Publisher valve_state_pub_;
  std_msgs::UInt8 state_msg_;
  bool valve_states_[NUM_VALVES];
  // uint8_t current_status_;
  uint8_t valves_command_ = 0x90;
  uint8_t pre_command_ = 0x90;
  //int ICS_FALSE = -1;

//protected:
//  virtual bool synchronize(uint8_t* txBuffer,
//                               size_t txLength,
//                               uint8_t* rxBuffer,
//                               size_t rxLength){
//	  return false;
//  }

public:
  ~SolenoidValve(){}
  SolenoidValve()
  	  :solenoid_valve_control_sub_("solenoid_valve_cmd", &SolenoidValve::valveControlCallback, this),
	   valve_state_pub_("solenoid_valve_state", &state_msg_)
  {
    for(int i = 0; i < NUM_VALVES; i++)
      {
        valve_states_[i]= false;
      }
  }

  void init(UART_HandleTypeDef* port, ros::NodeHandle* nh)
  {
    port_ = port;
    nh_ = nh;
    nh_ -> subscribe(solenoid_valve_control_sub_);
    nh_ -> advertise(valve_state_pub_);
    // current_status_ = readStates();
  }

  void update()
  {
    for(int i = 0; i < NUM_VALVES; i++)
      {
        if(valve_states_[i])
          {
            valves_command_ |= (1 << i);
          }
      }
    //char log_buf[50];
    //snprintf(log_buf, sizeof(log_buf), "valves_command_: 0x%02X", valves_command_);
    //nh_->logwarn(log_buf);

    //if (valves_command_ != pre_command_)
      //{
    //pre_command_ = valves_command_;
    valves_command_ = 0x44; //SC_GPIO_NONE
    setStates(valves_command_);
    nh_->logwarn("fiiiiiiiiiiiiiinish:");
        //valves_command_ = 0x90; //SC_GPIO_NONE
      //}
  // }
    // if (current_status_ != 0xFF)
    //   {
    //     if (valves_command_ != current_status_)
    //       {
    //         setStates(valves_command_);
    //         current_status_ = valves_command_;
    //       };
    //   }
    // else
    //   {
    //     nh_->logwarn("SolenoidValve: Communication error in readStates()");
    //   }

    //state_msg_.data = valves_command_;
    //valve_state_pub_.publish(&state_msg_);
}
  void setStates(uint8_t valves_command_)
  {
    int tx_size = 2; //recv_length = 18;
    uint8_t recv_length = getLen(valves_command_);
    //uint8_t recv_length = 18;
    uint8_t tx_buff[tx_size], rx_buff[recv_length];
    uint8_t ret;
    //for(int i = 0; i < rx_size; i++)
       //   {
        //        rx_buff[i] = 0;
          //}

    tx_buff[0] = 0xA0 + 0x13; // ICS id=19
    tx_buff[1] = valves_command_;
    nh_->logwarn("setStates(): Sending command...");
    char log_buf[64];
    //snprintf(log_buf, sizeof(log_buf), "TX: [0x%02X, 0x%02X]", tx_buff[0], tx_buff[1]);
    //nh_->logwarn(log_buf);
    HAL_HalfDuplex_EnableTransmitter(port_);
    ret = HAL_UART_Transmit(port_, tx_buff, tx_size, 10);
    if(ret == HAL_OK)
    {
    	HAL_HalfDuplex_EnableReceiver(port_);
    	while (HAL_UART_GetState(port_) != HAL_UART_STATE_READY) {
    	      nh_->logwarn("UART RX busy, waiting...");
    	      HAL_Delay(1);
    	}
    	ret = HAL_UART_Receive(port_, rx_buff, recv_length, 100);
    	snprintf(log_buf, sizeof(log_buf), "RX: [0x%02X, 0x%02X]", rx_buff[0], rx_buff[1]);
    	nh_->logwarn("Received response:");
    	nh_->logwarn(log_buf);
    }
    else
      {
        nh_->logwarn("UART Receive failed.");
      }
  }

  //bool fig;
      //flg = synchronize(txCmd, sizeof txCmd, rx_buff, sizeof rx_buff);
      //if (flg == false)
      //{
      	//return ICS_FALSE;
      //}
      	//return 0;
    	//}

  uint16_t getLen(uint8_t sub_command_raw) {
    // SENSOR sub_command
    const int sub_command_NONE = 0x40;
    const int sub_command_SW_ADC = 0x41;
    const int sub_command_PS = 0x42;
    const int sub_command_IMU = 0x44;
    const int sub_command_MAGENC = 0x48;
    const int sub_command_ALL = (sub_command_SW_ADC | sub_command_PS | sub_command_IMU | sub_command_MAGENC);
    const int sub_command_SW_ADC_R = (sub_command_SW_ADC & 0x0F);
    const int sub_command_PS_R = (sub_command_PS & 0x0F);
    const int sub_command_IMU_R = (sub_command_IMU & 0x0F);
    const int sub_command_MAGENC_R = (sub_command_MAGENC & 0x0F);
    const int ADC_CHANNEL_NUM = 5;
    const int FORCE_CHANNEL_NUM = 4;
    const int PS_CHANNEL_NUM = 4;
    const int GYRO_CHANNEL_NUM = 3;
    const int ACC_CHANNEL_NUM = 3;
    const int MAGENC_CHANNEL_NUM = 1;
    const int ADC_LEN = ADC_CHANNEL_NUM * 2;
    const int PS_LEN = PS_CHANNEL_NUM * 3;
    const int GYRO_LEN = GYRO_CHANNEL_NUM * 3;
    const int ACC_LEN = ACC_CHANNEL_NUM * 3;
    const int IMU_LEN = GYRO_LEN + ACC_LEN;
    const int MAGENC_LEN = MAGENC_CHANNEL_NUM * 2;
    const int MAX_BUFFER_LENGTH = 2 + ADC_LEN + PS_LEN + IMU_LEN + MAGENC_LEN;
    uint16_t length = 2;
    uint8_t sub_command = sub_command_raw & 0x0F;
    if ((sub_command & sub_command_SW_ADC_R) == sub_command_SW_ADC_R) {
      length += ADC_LEN;
    }
    if ((sub_command & sub_command_PS_R) == sub_command_PS_R) {
      length += PS_LEN;
    }
    if ((sub_command & sub_command_IMU_R) == sub_command_IMU_R) {
      length += IMU_LEN;
    }
    if ((sub_command & sub_command_MAGENC_R) == sub_command_MAGENC_R) {
      length += MAGENC_LEN;
    }
    return MAX_BUFFER_LENGTH;
  }

//  int setGPIO(uint8_t valves_command_) {
//	  uint8_t recv_length = getLen(valves_command_);
//	  uint8_t txCmd[2], rxCmd[2];
//	  const int ADC_CHANNEL_NUM = 5;
//	  uint16_t adc_buff[ADC_CHANNEL_NUM] = {};
//	  uint8_t SC_send = valves_command_; // SC_SW_ADC, SC_PS, SC_IMU, SC_MAGENC;
//	  unsigned int reData;
//	  bool flg;
//	  txCmd[0] = 0xA0 + 19;    // CMD
//	  txCmd[1] = SC_send;
//	  flg = synchronize(txCmd, sizeof txCmd, rxCmd, sizeof rxCmd);
//	  if (flg == false) {
//		  return ICS_FALSE;
//	  }
//	  	  return 0;
//  }

//  int getPressure(uint8_t id, float* pressure_out) {
//    const uint8_t sub_command_NONE = 0x40;
//    const uint8_t sub_command_SW_ADC = 0x41;
//    const uint8_t sub_command_PS = 0x42;
//    const uint8_t sub_command_IMU = 0x44;
//    const uint8_t sub_command_MAGENC = 0x48;
//    // const uint8_t sub_command_ALL = sub_command_SW_ADC | sub_command_PS | sub_command_IMU | sub_command_MAGENC;
//    const uint8_t sub_command_ALL = sub_command_SW_ADC;
//
//    uint8_t tx_cmd[2] = {static_cast<uint8_t>(0xA0 + id), sub_command_ALL};
//    uint8_t recv_length = getLen(sub_command_ALL);
//    uint8_t rx_buff[recv_length];
//    //if (!synchronize(tx_cmd, sizeof(tx_cmd), rx_buff, recv_length)) {
//      //return ICS_FALSE;
//    //}
//    const int index = 2 + 2 * 3;
//    float adc_raw = ((0x1F & rx_buff[index]) << 7) | rx_buff[index + 1];
//    const float V_OFFSET = 1.65f;
//    const float GAIN = 7.4f;
//    const float V_REF = 3.3f;
//    const float ADC_MAX = 4095.0f;
//    const float SCALE_FACTOR = 5.0f / 4.14f;
//    const float PRESSURE_OFFSET = 10.739f;
//    const float PRESSURE_SCALE = 3.1395f;
//    float v_amplified = V_REF * adc_raw / ADC_MAX;
//    float v_diff = (v_amplified - V_OFFSET) / GAIN;
//    float pressure = (v_diff * 1000.0f * SCALE_FACTOR - PRESSURE_OFFSET) / PRESSURE_SCALE;
//    *pressure_out = pressure;
//    return 0;
//  }

  // uint8_t readStates()
  // {
  //   int tx_size = 2, rx_size = 2;
  //   uint8_t tx_buff[tx_size], rx_buff[rx_size];
  //   uint8_t ret;
  //   tx_buff[0] = 0xA0 + 19;
  //   tx_buff[1] = 0x05; //status read

  //   HAL_HalfDuplex_EnableTransmitter(port_);
  //   ret = HAL_UART_Transmit(port_, tx_buff, tx_size, 1);
  //   if(ret == HAL_OK)
  //     {
  //       HAL_HalfDuplex_EnableReceiver(port_);
  //       ret = HAL_UART_Receive(port_, rx_buff, rx_size, 1);
  //     }

  //   if(19 == (rx_buff[0] & 0x1f))
  //     {
  //       return rx_buff[1] & 0x03;
  //     } else
  //     {
  //       return 0xFF;
  //     }
  // }

  void valveControlCallback(const spinal::ValveControlCmd &cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length && i < NUM_VALVES; i++)
      {
        int idx = cmd_msg.index[i];
        bool state = cmd_msg.states[i];
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Index: %d, State: %s", idx, state ? "ON" : "OFF");
        nh_->logwarn(buffer);
        if (idx >= 0 && idx < NUM_VALVES)
          {
            valve_states_[idx] = cmd_msg.states[i];
          }
      }
    // if(cmd_msg.index_length > 0)
    //   {
    //     valve_states_[0] = cmd_msg.states[0];
    //   }
    // for(int i = 0; i < NUM_VALVES; i++)
    //   {
    //     if(valve_states_[i])
    //       {
    //         valves_command_ |= (1 << i);
    //       }
    //   }
    //     setStates(valves_command_);
  }
};

#endif
