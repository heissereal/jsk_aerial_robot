/**
******************************************************************************
* File Name          : spine.cpp
* Description        : can-based internal comm network, spine side interface
 ------------------------------------------------------------------*/

#include "spine.h"

namespace Spine
{
  /* components */
  /* CAUTIONS: be careful about the order of the var definition and func definition */
  namespace
  {
    std::vector<Neuron> neuron_;
    CANMotorSendDevice can_motor_send_device_;
    std::vector<std::reference_wrapper<Servo>> servo_;
    std::vector<std::reference_wrapper<Servo>> servo_with_send_flag_;
    CANInitializer can_initializer_(neuron_);
    std::vector<float> imu_weight_;
    FlightControl* controller_;

    uint8_t slave_num_ = 0;
    uint8_t servo_num_ = 0;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    constexpr uint8_t ARM_PRESSURE_COUNT = 4;
    constexpr uint32_t ARM_PRESSURE_CONTROL_INTERVAL_MS = 10;
    constexpr uint32_t ARM_PRESSURE_ADC_TIMEOUT_MS = 100;
    constexpr float ARM_PRESSURE_LIMIT_KPA = 60.0f;
    constexpr float ARM_PRESSURE_MIN_VALID_KPA = -5.0f;
    constexpr uint32_t ARM_PRESSURE_VALUE_FAULT_DELAY_MS = 100;
    constexpr float ARM_PRESSURE_SUPPLY_SENSOR_MIN_KPA = 1.0f;
    // Pneumatic pressure can take several seconds to start rising from an
    // exhausted bag.  Two seconds falsely latched the disconnected-sensor
    // protection during normal 0 -> 10 kPa identification steps.
    constexpr uint32_t ARM_PRESSURE_SUPPLY_SENSOR_TIMEOUT_MS = 10000;
    constexpr uint32_t ARM_PRESSURE_COMMAND_TIMEOUT_MS = 200;
    constexpr uint8_t ARM_PRESSURE_PUMP_PWM_PORT_1 = 4;
    constexpr uint8_t ARM_PRESSURE_PUMP_PWM_PORT_2 = 6;
    uint8_t arm_pressure_slave_ids_[ARM_PRESSURE_COUNT] = {1, 2, 3, 4};
    float arm_pressures_[ARM_PRESSURE_COUNT] = {NAN, NAN, NAN, NAN};
    float arm_pressure_supply_duties_[ARM_PRESSURE_COUNT] = {};
    float arm_pressure_exhaust_duties_[ARM_PRESSURE_COUNT] = {};
    uint32_t arm_pressure_low_supply_start_ms_[ARM_PRESSURE_COUNT] = {};
    bool arm_pressure_supply_sensor_fault_[ARM_PRESSURE_COUNT] = {};
    uint32_t arm_pressure_value_fault_start_ms_[ARM_PRESSURE_COUNT] = {};
    float arm_pressure_pump_duty_ = 0.0f;
    bool arm_pressure_command_enabled_ = false;
    bool arm_pressure_command_received_ = false;
    uint32_t arm_pressure_last_command_ms_ = 0;
    uint32_t arm_pressure_last_update_ms_ = 0;

    Neuron* findNeuron(uint8_t slave_id)
    {
      for (auto& neuron : neuron_)
        if (neuron.getSlaveId() == slave_id) return &neuron;
      return nullptr;
    }

    void setArmPressurePumpDuty(float duty)
    {
      arm_pressure_pump_duty_ = std::max(0.0f, std::min(0.9f, duty));
      controller_->setAuxiliaryPwm(ARM_PRESSURE_PUMP_PWM_PORT_1, arm_pressure_pump_duty_);
      controller_->setAuxiliaryPwm(ARM_PRESSURE_PUMP_PWM_PORT_2, arm_pressure_pump_duty_);
    }

    void updateArmPressureControl(uint32_t now)
    {
      if (now - arm_pressure_last_update_ms_ < ARM_PRESSURE_CONTROL_INTERVAL_MS) return;
      arm_pressure_last_update_ms_ = now;
      if (!arm_pressure_command_received_ || !arm_pressure_command_enabled_)
        return;
      if (now - arm_pressure_last_command_ms_ > ARM_PRESSURE_COMMAND_TIMEOUT_MS)
        {
          setArmPressurePumpDuty(0.0f);
          arm_pressure_command_enabled_ = false;
          arm_pressure_command_received_ = false;
          return;
        }

      bool all_sensors_fresh = true;
      bool pressure_fault = false;
      for (uint8_t arm = 0; arm < ARM_PRESSURE_COUNT; ++arm)
        {
          Neuron* neuron = findNeuron(arm_pressure_slave_ids_[arm]);
          if (neuron == nullptr || !neuron->can_adc_.isFresh(now, ARM_PRESSURE_ADC_TIMEOUT_MS))
            {
              arm_pressures_[arm] = NAN;
              all_sensors_fresh = false;
              continue;
            }
          arm_pressures_[arm] = neuron->can_adc_.getPressure();
          const bool pressure_value_invalid =
              !std::isfinite(arm_pressures_[arm]) ||
              arm_pressures_[arm] < ARM_PRESSURE_MIN_VALID_KPA ||
              arm_pressures_[arm] >= ARM_PRESSURE_LIMIT_KPA;
          if (pressure_value_invalid)
            {
              if (arm_pressure_value_fault_start_ms_[arm] == 0)
                arm_pressure_value_fault_start_ms_[arm] = now;
              else if (now - arm_pressure_value_fault_start_ms_[arm] >=
                       ARM_PRESSURE_VALUE_FAULT_DELAY_MS)
                pressure_fault = true;
            }
          else
            arm_pressure_value_fault_start_ms_[arm] = 0;

          const bool supplying = arm_pressure_supply_duties_[arm] > 0.0f &&
                                 arm_pressure_pump_duty_ > 0.0f;
          if (arm_pressure_supply_sensor_fault_[arm] && !supplying)
            {
              arm_pressure_supply_sensor_fault_[arm] = false;
              arm_pressure_low_supply_start_ms_[arm] = 0;
            }
          if (!arm_pressure_supply_sensor_fault_[arm] && supplying &&
              arm_pressures_[arm] >= 0.0f &&
              arm_pressures_[arm] <= ARM_PRESSURE_SUPPLY_SENSOR_MIN_KPA)
            {
              if (arm_pressure_low_supply_start_ms_[arm] == 0)
                arm_pressure_low_supply_start_ms_[arm] = now;
              else if (now - arm_pressure_low_supply_start_ms_[arm] >=
                       ARM_PRESSURE_SUPPLY_SENSOR_TIMEOUT_MS)
                {
                  arm_pressure_supply_sensor_fault_[arm] = true;
                  neuron->can_motor_.setValvePwm(0, 0);
                }
            }
          else if (!arm_pressure_supply_sensor_fault_[arm])
            arm_pressure_low_supply_start_ms_[arm] = 0;
        }

      // On ADC/CAN failure only the shared pump is stopped; valve states are retained.
      if (!all_sensors_fresh || pressure_fault)
        {
          setArmPressurePumpDuty(0.0f);
          return;
        }

      bool any_healthy_supply = false;
      for (uint8_t arm = 0; arm < ARM_PRESSURE_COUNT; ++arm)
        {
          Neuron* neuron = findNeuron(arm_pressure_slave_ids_[arm]);
          if (neuron == nullptr) continue;
          const float supply_duty = arm_pressure_supply_sensor_fault_[arm]
                                      ? 0.0f : arm_pressure_supply_duties_[arm];
          neuron->can_motor_.setValvePwm(0, static_cast<uint16_t>(supply_duty * 1000.0f));
          neuron->can_motor_.setValvePwm(1, static_cast<uint16_t>(arm_pressure_exhaust_duties_[arm] * 1000.0f));
          any_healthy_supply = any_healthy_supply || supply_duty > 0.0f;
        }
      setArmPressurePumpDuty(any_healthy_supply ? arm_pressure_pump_duty_ : 0.0f);
    }

    /* sensor fusion */
    StateEstimate* estimator_;

    /* ros */
    constexpr uint8_t SERVO_PUB_INTERVAL = 20; //[ms]
    constexpr uint32_t SERVO_TORQUE_PUB_INTERVAL = 1000; //[ms]
    constexpr uint32_t NEURON_IMU_PUB_INTERVAL = 20; //[ms]
    spinal::NeuronImuStates neuron_imu_state_msg_;
    ros::Publisher neuron_imu_state_pub_("neuron/imu_states", &neuron_imu_state_msg_);
    constexpr uint32_t NEURON_ADC_PUB_INTERVAL = 20; //[ms]
    spinal::ServoStates servo_state_msg_;
    spinal::ServoTorqueStates servo_torque_state_msg_;
    spinal::NeuronAdcStates neuron_adc_state_msg_;
    ros::Publisher servo_state_pub_("servo/states", &servo_state_msg_);
    // merge torque_states to states
    ros::Publisher servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_);
    ros::Publisher neuron_adc_state_pub_("neuron/adc_states", &neuron_adc_state_msg_);

    // rename following subscriber.
    // taget_states -> target_position
    // torque_enable -> control_enable
    ros::Subscriber<spinal::ServoControlCmd> servo_position_sub_("servo/target_states", servoPositionCallback);
    ros::Subscriber<spinal::ServoControlCmd> servo_current_sub_("servo/target_current", servoCurrentCallback);
    ros::Subscriber<spinal::ServoTorqueCmd> servo_torque_ctrl_sub_("servo/torque_enable", servoTorqueControlCallback);
    ros::Subscriber<spinal::PneumaticCommand> pneumatic_command_sub_("pneumatic/command", pneumaticCommandCallback);

    ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response> board_info_srv_("get_board_info", boardInfoCallback);
    ros::ServiceServer<spinal::SetBoardConfig::Request, spinal::SetBoardConfig::Response> board_config_srv_("set_board_config", boardConfigCallback);

    spinal::GetBoardInfo::Response board_info_res_;

    ros::NodeHandle* nh_;
    uint32_t servo_last_pub_time_ = 0;
    uint32_t servo_torque_last_pub_time_ = 0;
    uint32_t neuron_imu_last_pub_time_ = 0;
    uint32_t neuron_adc_last_pub_time_ = 0;
    unsigned int can_idle_count_ = 0;
    bool servo_control_flag_ = true;

    uint32_t can_tx_idle_start_time_ = 0; // for pause CAN TX -> TODO: change to another task for spinal process
    uint32_t CAN_TX_PAUSE_TIME = 2000; // 2000 ms for 1Khz task rate. TODO: change to another task for spinal process
    unsigned int send_board_index = 0; // incremental board id assignment for CAN TX

    uint32_t last_connected_time_ =0;
  }

  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
  {
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i];
      board.imu_send_data_flag = neuron.can_imu_.getSendDataFlag() ? 1 : 0;
      board.dynamixel_ttl_rs485_mixed = neuron.can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
      board.servo_pulley_skip_thresh = neuron.can_servo_.getPulleySkipThresh();
      board.slave_id = neuron.getSlaveId();

      for (unsigned int j = 0; j < board.servos_length; j++) {
        Servo& s = neuron.can_servo_.servo_.at(j);
        board.servos[j].id = s.getId();
        board.servos[j].p_gain = s.getPGain();
        board.servos[j].i_gain = s.getIGain();
        board.servos[j].d_gain = s.getDGain();
        board.servos[j].profile_velocity = s.getProfileVelocity();
        board.servos[j].current_limit = s.getCurrentLimit();
        board.servos[j].send_data_flag = s.getSendDataFlag() ? 1 : 0;
        board.servos[j].external_encoder_flag = s.getExternalEncoderFlag() ? 1 : 0;
        board.servos[j].joint_resolution = s.getJointResolution();
        board.servos[j].servo_resolution = s.getServoResolution();
      }
    }
    res = board_info_res_;
  }

  void servoPositionCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    if (control_msg.index_length != control_msg.angles_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setGoalPosition(control_msg.angles[i]);
    }
  }

  void servoCurrentCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    if (control_msg.index_length != control_msg.angles_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setGoalCurrent(control_msg.angles[i]);
      // TODO: change angles -> commands
    }
  }

  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
  {
    if (control_msg.index_length != control_msg.torque_enable_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      servo_.at(control_msg.index[i]).get().setTorqueEnable((control_msg.torque_enable[i] != 0) ? true : false);

      /* update the target angle */
      if (servo_.at(control_msg.index[i]).get().getSendDataFlag()) {
        servo_.at(control_msg.index[i]).get().setGoalPosition(servo_.at(control_msg.index[i]).get().getPresentPosition());
      }
    }
  }

  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res)
  {
    /* Pause the spinal sending command for neuron to have enough time for flashmemory erase&write */
    can_tx_idle_start_time_ = HAL_GetTick();
    // TODO: change the return value to bool
    can_initializer_.configDevice(req);

    // TODO: please add string type message for consoling
    res.success = true;
  }

  bool init(CAN_GeranlHandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, FlightControl* controller, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);

    /* Estimation */
    estimator_ = estimator;

    /* Control */
    controller_ = controller;

    HAL_Delay(5000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::CAN_START();
    can_initializer_.initDevices();

    slave_num_ = neuron_.size();
    if(slave_num_ == 0) return false;

    //add CAN devices to CANDeviceManager
    for (unsigned int i = 0; i < neuron_.size(); i++) {
      CANDeviceManager::addDevice(neuron_.at(i).can_motor_);
      can_motor_send_device_.addMotor(neuron_.at(i).can_motor_);
      CANDeviceManager::addDevice(neuron_.at(i).can_imu_);
      CANDeviceManager::addDevice(neuron_.at(i).can_servo_);
      CANDeviceManager::addDevice(neuron_.at(i).can_adc_);
      for (unsigned int j = 0; j < neuron_.at(i).can_servo_.servo_.size(); j++) {
        neuron_.at(i).can_servo_.servo_.at(j).setIndex(servo_.size());
        servo_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        if (neuron_.at(i).can_servo_.servo_.at(j).getSendDataFlag()) {
          servo_with_send_flag_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        }
      }
    }
    servo_num_ = servo_.size();

    /* ros */
    nh_ = nh;

    if (servo_num_ > 0)
      {
        nh_->advertise(servo_state_pub_);
        nh_->advertise(servo_torque_state_pub_);
        nh_->subscribe(servo_position_sub_);
        nh_->subscribe(servo_current_sub_);
        nh_->subscribe(servo_torque_ctrl_sub_);
      }

    nh_->advertise(neuron_imu_state_pub_);

    nh_->advertiseService(board_info_srv_);
    nh_->advertiseService(board_config_srv_);
    nh_->advertise(neuron_adc_state_pub_);
    nh_->subscribe(pneumatic_command_sub_);

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_num_ - servo_with_send_flag_.size();

    /* TODO: not good case processing */
    if(gimbal_servo_num == 0)
      {
        uav_model_ = spinal::UavInfo::HYDRUS;
      }
    if(gimbal_servo_num  == slave_num_)
      {
        uav_model_ = spinal::UavInfo::HYDRUS_XI;
      }
    if(gimbal_servo_num  == 2 * slave_num_)
      {
        uav_model_ = spinal::UavInfo::DRAGON;
      }

    /* update controller */
    controller_->setUavModel(uav_model_);
    controller_->setMotorNumber(slave_num_);

    servo_state_msg_.servos_length = servo_with_send_flag_.size();
    servo_state_msg_.servos = new spinal::ServoState[servo_with_send_flag_.size()];
    servo_torque_state_msg_.torque_enable_length = servo_num_;
    servo_torque_state_msg_.torque_enable = new uint8_t[servo_num_];
    neuron_imu_state_msg_.imus_length = slave_num_;
    neuron_imu_state_msg_.imus = new spinal::NeuronImu[slave_num_];
    neuron_adc_state_msg_.adcs_length = slave_num_;
    neuron_adc_state_msg_.adcs = new spinal::NeuronAdc[slave_num_];

    /* other component */
    imu_weight_.resize(slave_num_ + 1);

    /* set IMU weights */
    // no fusion
    imu_weight_[0] = 1.0;
    for (uint i = 1; i < imu_weight_.size(); i++) imu_weight_[i] = 0.0;

    for (int i = 0; i < slave_num_; i++) {
      HAL_Delay(100);
      neuron_.at(i).can_imu_.init();

      IMU_ROS_CMD::addImu(&(neuron_.at(i).can_imu_));
    }

    //set response for get_board_info
    board_info_res_.boards_length = slave_num_;
    board_info_res_.boards = new spinal::BoardInfo[slave_num_];
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i];
      board.servos_length = neuron.can_servo_.servo_.size();
      board.servos = new spinal::ServoInfo[board.servos_length];
    }

    return true;
  }

  void send()
  {
    if (slave_num_ == 0) return;

    if(HAL_GetTick() < can_tx_idle_start_time_ + CAN_TX_PAUSE_TIME) return;

    if(HAL_GetTick() % 2 == 0) {
      // 500Hz
      can_motor_send_device_.sendData();
    }
    else {
      if (slave_num_ != 0) {
        // 500Hz
        neuron_.at(send_board_index).can_servo_.sendData();
        send_board_index++;
        if (send_board_index == slave_num_) send_board_index = 0;
      }
    }

    can_initializer_.sendData(); // if necessary
  }

  void update(void)
  {
    if (slave_num_ == 0) return;

    /* update the motor PWM command */
    for(int i = 0; i < slave_num_; i++) {
      float pwm_rate = controller_->getTargetPwm(i);
      uint16_t pwm_bit = pwm_rate * 2000 - 1000;
      neuron_.at(i).can_motor_.setPwm(pwm_bit);
    }

    /* uodate IMU */
    for (int i = 0; i < slave_num_; i++)
      neuron_.at(i).can_imu_.update();

    updateArmPressureControl(HAL_GetTick());

    /* ros publish */
    servoPublish();
    neuronImuPublish();
    neuronAdcPublish();

    CANDeviceManager::tick(1);

    uint32_t now_time = HAL_GetTick();
    if(CANDeviceManager::connected()) last_connected_time_ = now_time;

    if(now_time - last_connected_time_ > 1000 /* ms */)
      {
        if(nh_->connected()) nh_->logerror("CAN disconnected!!");
        last_connected_time_ = now_time;
      }
  }

  void useRTOS(osMailQId* handle)
  {
    CANDeviceManager::useRTOS(handle);
  }

  void setMotorPwm(uint16_t pwm, uint8_t motor)
  {
    if(slave_num_ == 0) {
      return;
    }
    neuron_.at(motor).can_motor_.setPwm(pwm);
  }

  bool connected()
  {
    if (slave_num_ > 0) return true;

    return false;
  }

  uint8_t getSlaveNum()
  {
    return slave_num_;
  }

  int8_t getUavModel()
  {
    return uav_model_;
  }

  void setServoControlFlag(bool flag)
  {
    servo_control_flag_ = flag;
  }

  void servoPublish()
  {
    if (servo_num_ == 0) return;

    uint32_t now_time = HAL_GetTick();
    if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
      {
        /* send servo */
        servo_state_msg_.stamp = nh_->now();
        for (unsigned int i = 0; i < servo_with_send_flag_.size(); i++)
          {
            spinal::ServoState servo;

            servo.index = servo_with_send_flag_.at(i).get().getIndex();
            servo.angle = servo_with_send_flag_.at(i).get().getPresentPosition();
            servo.temp = servo_with_send_flag_.at(i).get().getPresentTemperature();
            servo.load = servo_with_send_flag_.at(i).get().getPresentCurrent();
            servo.error = servo_with_send_flag_.at(i).get().getError();

            servo_state_msg_.servos[i] = servo;
          }

        servo_state_pub_.publish(&servo_state_msg_);
        servo_last_pub_time_ = now_time;
      }

    if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
      {
        for (unsigned int i = 0; i < servo_num_; i++)
          {
            servo_torque_state_msg_.torque_enable[i] = servo_.at(i).get().getTorqueEnable() ? 1 : 0;
          }
        servo_torque_state_pub_.publish(&servo_torque_state_msg_);
        servo_torque_last_pub_time_ = now_time;
      }
  }

  void neuronImuPublish()
  {
    uint32_t now_time = HAL_GetTick();
    if( now_time - neuron_imu_last_pub_time_ < NEURON_IMU_PUB_INTERVAL)
      {
        return;
      }
    
    neuron_imu_last_pub_time_ = now_time;
    neuron_imu_state_msg_.stamp = nh_->now();
    for (unsigned int i = 0; i < slave_num_; ++i)
    {
      CANIMU& imu = neuron_.at(i).can_imu_;
      Vector3f acc = imu.getAcc();
      Vector3f gyro = imu.getGyro();

      neuron_imu_state_msg_.imus[i].slave_id = neuron_.at(i).getSlaveId();
      for (int axis = 0; axis < 3; ++axis)
      {
        neuron_imu_state_msg_.imus[i].acc[axis] = acc[axis];
        neuron_imu_state_msg_.imus[i].gyro[axis] = gyro[axis];
      }
    }
    neuron_imu_state_pub_.publish(&neuron_imu_state_msg_);
  }
  void neuronAdcPublish()
  {
    const uint32_t now_time = HAL_GetTick();
    if (now_time - neuron_adc_last_pub_time_ < NEURON_ADC_PUB_INTERVAL) return;

    neuron_adc_last_pub_time_ = now_time;
    neuron_adc_state_msg_.stamp = nh_->now();

    constexpr uint32_t NEURON_ADC_TIMEOUT_MS = 100;

    for (unsigned int i = 0; i < slave_num_; i++)
      {
        neuron_adc_state_msg_.adcs[i].slave_id = neuron_.at(i).getSlaveId();
        if (!neuron_.at(i).can_adc_.isFresh(now_time, NEURON_ADC_TIMEOUT_MS))
          {
            neuron_adc_state_msg_.adcs[i].voltage = NAN;
            neuron_adc_state_msg_.adcs[i].pressure = NAN;
            continue;
          }
        // neuron_adc_state_msg_.adcs[i].raw = neuron_.at(i).can_adc_.getRaw();

        neuron_adc_state_msg_.adcs[i].voltage = neuron_.at(i).can_adc_.getSensorVoltage();
        neuron_adc_state_msg_.adcs[i].pressure = neuron_.at(i).can_adc_.getPressure();
      }
    neuron_adc_state_pub_.publish(&neuron_adc_state_msg_);
  }

  bool setNeuronValvePwm(uint8_t slave_id, uint8_t channel, float duty)
  {
    if (channel >= 2 || !std::isfinite(duty)) return false;
    const uint16_t pwm = static_cast<uint16_t>(std::max(0.0f, std::min(1.0f, duty)) * 1000.0f + 0.5f);
    for (auto& neuron : neuron_)
      if (neuron.getSlaveId() == slave_id)
        {
          neuron.can_motor_.setValvePwm(channel, pwm);
          return true;
        }
    return false;
  }

  void pneumaticCommandCallback(const spinal::PneumaticCommand& command)
  {
    const bool pneumatic_was_enabled = arm_pressure_command_enabled_;
    bool valid = std::isfinite(command.pump_pwm) && command.pump_pwm >= 0.0f && command.pump_pwm <= 0.9f;
    for (uint8_t arm = 0; arm < ARM_PRESSURE_COUNT; ++arm)
      valid = valid && std::isfinite(command.supply_pwm[arm]) && command.supply_pwm[arm] >= 0.0f && command.supply_pwm[arm] <= 1.0f &&
              std::isfinite(command.exhaust_pwm[arm]) && command.exhaust_pwm[arm] >= 0.0f && command.exhaust_pwm[arm] <= 1.0f &&
              !(command.supply_pwm[arm] > 0.0f && command.exhaust_pwm[arm] > 0.0f);
    if (!valid)
      {
        arm_pressure_command_enabled_ = false;
        setArmPressurePumpDuty(0.0f);
        return;
      }
    arm_pressure_command_enabled_ = command.enable;
    arm_pressure_command_received_ = true;
    arm_pressure_last_command_ms_ = HAL_GetTick();
    arm_pressure_pump_duty_ = command.enable ? command.pump_pwm : 0.0f;
    for (uint8_t arm = 0; arm < ARM_PRESSURE_COUNT; ++arm)
      {
        arm_pressure_supply_duties_[arm] = command.enable ? command.supply_pwm[arm] : 0.0f;
        arm_pressure_exhaust_duties_[arm] = command.enable ? command.exhaust_pwm[arm] : 0.0f;
      }
    if (!command.enable && pneumatic_was_enabled)
      {
        setArmPressurePumpDuty(0.0f);
        for (auto& neuron : neuron_)
        {
          neuron.can_motor_.setValvePwm(0, 0);
          neuron.can_motor_.setValvePwm(1, 0);
        }
        for (uint8_t arm = 0; arm < ARM_PRESSURE_COUNT; ++arm)
        {
          arm_pressure_low_supply_start_ms_[arm] = 0;
          arm_pressure_supply_sensor_fault_[arm] = false;
          arm_pressure_value_fault_start_ms_[arm] = 0;
        }
      }
  }

  float getArmPressure(uint8_t arm)
  {
    return arm < ARM_PRESSURE_COUNT ? arm_pressures_[arm] : NAN;
  }

  float getArmPressurePumpDuty()
  {
    return arm_pressure_pump_duty_;
  }
};
