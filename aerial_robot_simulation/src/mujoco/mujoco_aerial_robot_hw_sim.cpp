#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

#include <algorithm>
#include <cmath>

namespace mujoco_ros::control
{
bool AerialRobotHWSim::InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                               const std::string& robot_namespace, ros::NodeHandle model_nh,
                               const urdf::Model* const urdf_model,
                               std::vector<transmission_interface::TransmissionInfo> transmissions,
                               bool ignore_actuators)
{
  /* separate rotor transmissions and standard transmissions */
  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::vector<transmission_interface::TransmissionInfo> rotor_transmissions;

  int motor_num = 0;
  rotor_list_.resize(0);
  for (const auto& trans : transmissions)
  {
    bool is_rotor = false;
    for (const auto& hw_iface : trans.actuators_[0].hardware_interfaces_)
    {
      if (hw_iface == "RotorInterface" || hw_iface == "hardware_interface/RotorInterface")
      {
        is_rotor = true;
        break;
      }
    }
    if (is_rotor)
    {
      std::string actuator_name = trans.joints_[0].name_;
      rotor_list_.push_back(actuator_name);
      motor_num++;

      rotor_transmissions.push_back(trans);
    }
    else
      standard_transmissions.push_back(trans);
  }

  /* Initialize standard joint handlers */
  DefaultRobotHWSim::InitSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                             standard_transmissions);

  /* get entire mass */
  float mass = 0.0;
  for (int i = 0; i < m_ptr_->nbody; i++)
  {
    mass += m_ptr_->body_mass[i];
  }
  ROS_INFO_STREAM("[mujoco] robot mass is " << mass);

  ros::NodeHandle motor_nh(model_nh, "motor_info");
  ros::NodeHandle motor_sim_nh(motor_nh, "simulation");
  ros::NodeHandle reverse_motor_nh(motor_nh, "reverse");
  motor_sim_nh.param("use_pwm_model", use_pwm_motor_model_, false);
  motor_nh.param("min_pwm", motor_min_pwm_, motor_min_pwm_);
  motor_nh.param("neutral_pwm", motor_neutral_pwm_, motor_neutral_pwm_);
  motor_nh.param("max_pwm", motor_max_pwm_, motor_max_pwm_);
  motor_nh.param("positive_thrust_below_neutral",
                 positive_thrust_below_neutral_, false);
  motor_nh.param("m_f_rate", forward_m_f_rate_, forward_m_f_rate_);
  reverse_motor_nh.param("m_f_rate", reverse_m_f_rate_, forward_m_f_rate_);
  motor_nh.param("pwm_conversion_mode", pwm_conversion_mode_, pwm_conversion_mode_);
  motor_sim_nh.param("voltage", motor_sim_voltage_, motor_sim_voltage_);
  motor_sim_nh.param("neutral_deadband", motor_neutral_deadband_, motor_neutral_deadband_);

  int forward_reference_count = 0;
  int reverse_reference_count = 0;
  motor_nh.param("vel_ref_num", forward_reference_count, forward_reference_count);
  reverse_motor_nh.param("vel_ref_num", reverse_reference_count, forward_reference_count);
  forward_motor_curve_ = loadMotorCurve(motor_nh, forward_reference_count);
  reverse_motor_curve_ = loadMotorCurve(reverse_motor_nh, reverse_reference_count);
  if (forward_motor_curve_.empty() || reverse_motor_curve_.empty())
  {
    ROS_ERROR("mujoco: PWM-test support requires both forward and reverse motor curves");
    return false;
  }
  if (motor_sim_voltage_ <= 0.0)
    motor_sim_voltage_ = forward_motor_curve_.front().voltage;

  rotor_direction_signs_.reserve(rotor_list_.size());
  const double forward_m_f_sign = forward_m_f_rate_ < 0.0 ? -1.0 : 1.0;
  for (const auto& rotor_name : rotor_list_)
  {
    const int actuator_id = mj_name2id(m_ptr_, mjOBJ_ACTUATOR, rotor_name.c_str());
    if (actuator_id < 0)
    {
      ROS_ERROR("mujoco: rotor actuator '%s' does not exist", rotor_name.c_str());
      return false;
    }
    const double torque_gear = m_ptr_->actuator_gear[6 * actuator_id + 5];
    const double torque_gear_sign = torque_gear < 0.0 ? -1.0 : 1.0;
    rotor_direction_signs_.push_back(torque_gear_sign * forward_m_f_sign);
  }
  ROS_INFO("[mujoco] bidirectional PWM motor model: %.3f--%.3f, neutral %.3f, voltage %.1f V, positive thrust on %s side",
           motor_min_pwm_, motor_max_pwm_, motor_neutral_pwm_, motor_sim_voltage_,
           positive_thrust_below_neutral_ ? "low-PWM" : "high-PWM");

  /* Initialize spinal interface */
  spinal_interface_.init(model_nh, rotor_list_.size());
  registerInterface(&spinal_interface_);

  ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
  simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01);                               // [sec]
  simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0);                              // m
  simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0);                              // rad
  simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0);                              // rad
  simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0);  // 1/s

  simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01);     // [sec]
  simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001);  // m
  simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001);  // rad
  ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
  mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

  return true;
}

void AerialRobotHWSim::ReadSim(ros::Time time, ros::Duration period)
{
  /* get the pose of the fc site in mujoco, and set it as the ground truth value for spinal interface */
  int fc_id = mj_name2id(m_ptr_, mjtObj_::mjOBJ_SITE, "fc");
  mjtNum* site_xpos = d_ptr_->site_xpos;
  mjtNum* site_xmat = d_ptr_->site_xmat;
  tf::Matrix3x3 fc_rot_mat =
      tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                    site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                    site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tf::Quaternion fc_quat;
  fc_rot_mat.getRotation(fc_quat);

  /* get the imu sensor data in mujoco, and set it as the input for spinal interface */
  tf::Vector3 acc, gyro, mag;
  for (int i = 0; i < m_ptr_->nsensor; i++)
  {
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "acc")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        acc[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "gyro")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        gyro[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "mag")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        mag[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
  }

  spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
  spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

  spinal_interface_.stateEstimate();

  /* publish ground truth value */
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0];
  odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1];
  odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2];
  odom_msg.pose.pose.orientation.x = fc_quat.x();
  odom_msg.pose.pose.orientation.y = fc_quat.y();
  odom_msg.pose.pose.orientation.z = fc_quat.z();
  odom_msg.pose.pose.orientation.w = fc_quat.w();

  /* set ground truth for controller: use the value with noise */
  spinal_interface_.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(), gyro.x(), gyro.y(),
                                         gyro.z());

  if ((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);

    tf::Quaternion q_delta;
    q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_), gazebo::gaussianKernel(mocap_rot_noise_),
                   gazebo::gaussianKernel(mocap_rot_noise_));
    tf::Quaternion q_noise = fc_quat * q_delta;
    pose_msg.pose.orientation.x = q_noise.x();
    pose_msg.pose.orientation.y = q_noise.y();
    pose_msg.pose.orientation.z = q_noise.z();
    pose_msg.pose.orientation.w = q_noise.w();

    mocap_pub_.publish(pose_msg);
    last_mocap_time_ = time;
  }

  DefaultRobotHWSim::ReadSim(time, period);
}

void AerialRobotHWSim::WriteSim(ros::Time time, ros::Duration period)
{
  // normal joints control by default robot hw sim
  DefaultRobotHWSim::WriteSim(time, period);

  // set rotor force got from spinal interface to mujoco actuator
  for (int i = 0; i < spinal_interface_.getMotorNum(); i++)
  {
    int rotor_id = mj_name2id(m_ptr_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
    double rotor_force = spinal_interface_.getForce(i);
    if (use_pwm_motor_model_ || spinal_interface_.getPwmTestMode())
    {
      const double pwm = std::max(
          motor_min_pwm_, std::min(motor_max_pwm_, spinal_interface_.getPwm(i)));
      const bool low_pwm_branch = pwm < motor_neutral_pwm_;
      rotor_force = forceFromPwm(pwm, low_pwm_branch);
      const double m_f_rate = low_pwm_branch ? reverse_m_f_rate_
                                             : forward_m_f_rate_;
      m_ptr_->actuator_gear[6 * rotor_id + 5] =
          rotor_direction_signs_.at(i) * m_f_rate;
    }
    else
    {
      m_ptr_->actuator_gear[6 * rotor_id + 5] =
          rotor_direction_signs_.at(i) * forward_m_f_rate_;
    }
    if (std::isfinite(rotor_force))
      d_ptr_->ctrl[rotor_id] = rotor_force;
  }
}

std::vector<AerialRobotHWSim::MotorCurveReference>
AerialRobotHWSim::loadMotorCurve(
    const ros::NodeHandle& motor_nh, int reference_count) const
{
  std::vector<MotorCurveReference> result;
  result.reserve(std::max(0, reference_count));
  for (int i = 0; i < reference_count; ++i)
  {
    const ros::NodeHandle reference_nh(
        motor_nh, "ref" + std::to_string(i + 1));
    MotorCurveReference reference;
    if (!reference_nh.getParam("voltage", reference.voltage))
      continue;
    reference_nh.param("max_thrust", reference.max_thrust, 0.0);
    for (size_t coefficient = 0; coefficient < reference.polynomial.size(); ++coefficient)
      reference_nh.param(
          "polynominal" + std::to_string(coefficient),
          reference.polynomial[coefficient], 0.0);
    result.push_back(reference);
  }
  return result;
}

double AerialRobotHWSim::forceFromPwm(
    double pwm, bool low_pwm_branch) const
{
  if (std::abs(pwm - motor_neutral_pwm_) <= motor_neutral_deadband_)
    return 0.0;
  const auto& curve = low_pwm_branch ? reverse_motor_curve_
                                     : forward_motor_curve_;
  if (curve.empty())
  {
    ROS_ERROR_THROTTLE(1.0,
        "mujoco: no motor curve for the selected PWM-test direction");
    return 0.0;
  }
  const auto reference = std::min_element(
      curve.begin(), curve.end(), [this](const auto& lhs, const auto& rhs)
      {
        return std::abs(lhs.voltage - motor_sim_voltage_) <
               std::abs(rhs.voltage - motor_sim_voltage_);
      });
  const double pwm_percent = 100.0 * pwm;
  const double reference_force = reference->polynomial[0] +
      (reference->polynomial[1] * pwm_percent +
       reference->polynomial[2] * pwm_percent * pwm_percent) / 10.0;
  double scaled_force = reference_force;
  if (reference->voltage > 0.0 && motor_sim_voltage_ > 0.0)
  {
    const double voltage_ratio = motor_sim_voltage_ / reference->voltage;
    const double exponent = pwm_conversion_mode_ == 1 ? 1.5 : 2.0;
    scaled_force *= std::pow(voltage_ratio, exponent);
  }
  const bool positive_thrust = positive_thrust_below_neutral_
      ? low_pwm_branch : !low_pwm_branch;
  return positive_thrust ? std::abs(scaled_force)
                         : -std::abs(scaled_force);
}

}  // namespace mujoco_ros::control

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::AerialRobotHWSim, mujoco_ros::control::RobotHWSim)
