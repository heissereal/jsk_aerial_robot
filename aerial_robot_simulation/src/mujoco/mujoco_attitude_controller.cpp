#include <aerial_robot_simulation/mujoco/mujoco_attitude_controller.h>

#include <algorithm>
#include <cmath>

namespace flight_controllers
{
  MujocoAttitudeController::MujocoAttitudeController():
    controller_core_(new FlightControl())
  {
  }

  bool MujocoAttitudeController::init(hardware_interface::MujocoSpinalInterface *robot, ros::NodeHandle &n)
  {
    spinal_interface_ = robot;
    motor_num_ = spinal_interface_->getMotorNum();

    int index = n.getNamespace().rfind('/');
    std::string robot_ns = n.getNamespace().substr(0, index);
    ros::NodeHandle n_robot = ros::NodeHandle(robot_ns);
    controller_core_->init(&n_robot, robot->getEstimatorPtr());
    n_robot.param("motor_info/min_pwm", minimum_pwm_, minimum_pwm_);
    n_robot.param("motor_info/neutral_pwm", neutral_pwm_, neutral_pwm_);
    n_robot.param("motor_info/max_pwm", maximum_pwm_, maximum_pwm_);
    pwm_test_values_.assign(motor_num_, neutral_pwm_);
    pwm_test_active_.assign(motor_num_, false);
    // The embedded callback is excluded by #ifndef SIMULATION. MuJoCo still
    // needs the same individual-motor PwmTest semantics for gait experiments.
    pwm_test_sub_ = n_robot.subscribe(
        "pwm_test", 1, &MujocoAttitudeController::pwmTestCallback, this);

    return true;

  }

  void MujocoAttitudeController::starting(const ros::Time& time)
  {
  }

  void MujocoAttitudeController::update(const ros::Time& time, const ros::Duration& period)
  {
    /* freeze the attitude estimator while touching the ground, since the bad contact simulation performance in gazebo */
    spinal_interface_->onGround(!controller_core_->getAttController().getIntegrateFlag());

    /* update the controller */
    controller_core_->update();

    for(int i = 0; i < motor_num_; i++)
      {
        spinal_interface_->setForce(i, controller_core_->getAttController().getForce(i));
        // A stopped bidirectional ESC uses its configured neutral, not the
        // legacy simulation value 0.5 (which is maximum reverse for Hugmy).
        const double normal_pwm = controller_core_->getAttController().getIntegrateFlag()
            ? controller_core_->getAttController().getTargetPwm(i)
            : neutral_pwm_;
        spinal_interface_->setPwm(i, normal_pwm);
      }

    std::lock_guard<std::mutex> lock(pwm_test_mutex_);
    spinal_interface_->setPwmTestMode(pwm_test_mode_);
    if (pwm_test_mode_)
      for (int i = 0; i < motor_num_; ++i)
        spinal_interface_->setPwm(
            i, pwm_test_active_[i] ? pwm_test_values_[i] : neutral_pwm_);
  }

  void MujocoAttitudeController::pwmTestCallback(
      const spinal::PwmTest::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(pwm_test_mutex_);
    if (msg->motor_index.empty() && msg->pwms.empty())
      {
        pwm_test_mode_ = false;
        std::fill(pwm_test_active_.begin(), pwm_test_active_.end(), false);
        return;
      }
    if (msg->motor_index.size() != msg->pwms.size())
      {
        ROS_ERROR_THROTTLE(1.0,
            "MuJoCo PwmTest index/PWM arrays have different lengths");
        return;
      }
    pwm_test_mode_ = true;
    std::fill(pwm_test_active_.begin(), pwm_test_active_.end(), false);
    for (size_t item = 0; item < msg->motor_index.size(); ++item)
      {
        const size_t motor = msg->motor_index[item];
        const double pwm = msg->pwms[item];
        if (motor >= static_cast<size_t>(motor_num_) || !std::isfinite(pwm))
          {
            ROS_WARN_THROTTLE(1.0,
                "MuJoCo PwmTest contains an invalid motor or PWM");
            continue;
          }
        pwm_test_active_[motor] = true;
        pwm_test_values_[motor] = std::max(
            minimum_pwm_, std::min(maximum_pwm_, pwm));
      }
  }
}


PLUGINLIB_EXPORT_CLASS(flight_controllers::MujocoAttitudeController, controller_interface::ControllerBase)
