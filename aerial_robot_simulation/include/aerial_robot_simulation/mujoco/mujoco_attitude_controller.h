#pragma once

#include <ros/ros.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <flight_control/flight_control.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <spinal/PwmTest.h>
#include <mutex>
#include <vector>

namespace flight_controllers
{
  class MujocoAttitudeController : public controller_interface::Controller<hardware_interface::MujocoSpinalInterface>
  {
  public:
    MujocoAttitudeController();
    ~MujocoAttitudeController() {}

    bool init(hardware_interface::MujocoSpinalInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    void pwmTestCallback(const spinal::PwmTest::ConstPtr& msg);

    hardware_interface::MujocoSpinalInterface* spinal_interface_;
    boost::shared_ptr<FlightControl> controller_core_;
    int motor_num_;
    ros::Subscriber pwm_test_sub_;
    std::mutex pwm_test_mutex_;
    std::vector<double> pwm_test_values_;
    std::vector<bool> pwm_test_active_;
    bool pwm_test_mode_ = false;
    double minimum_pwm_ = 0.5;
    double neutral_pwm_ = 0.5;
    double maximum_pwm_ = 1.0;

  };
}
