#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

#include <algorithm>
#include <cmath>

namespace mujoco_ros_control
{

  bool AerialRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    DefaultRobotHWSim::init(robot_namespace, model_nh, mujoco_model, mujoco_data);

    rotor_list_.resize(0);

    // get entire mass
    float mass = 0.0;
    for(int i = 0; i < mujoco_model_->nbody; i++)
      {
       mass += mujoco_model_->body_mass[i];
      }
    ROS_INFO_STREAM("[mujoco] robot mass is " << mass);


    // get rotor names from mujoco model
    int motor_num = 0;
    for(int i = 0; i < mujoco_model_->nu; i++)
      {
        std::string actuator_name = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, i);
        if(actuator_name.find("rotor") != std::string::npos)
          {
            rotor_list_.push_back(actuator_name);
            motor_num++;
          }
      }

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
    if(use_pwm_motor_model_ && (forward_motor_curve_.empty() || reverse_motor_curve_.empty()))
      {
        ROS_ERROR("mujoco: PWM motor model requires both forward and reverse curves");
        return false;
      }
    if(use_pwm_motor_model_ && motor_sim_voltage_ <= 0.0)
      motor_sim_voltage_ = forward_motor_curve_.front().voltage;

    rotor_direction_signs_.reserve(rotor_list_.size());
    const double forward_m_f_sign = forward_m_f_rate_ < 0.0 ? -1.0 : 1.0;
    for(const auto& rotor_name : rotor_list_)
      {
        const int actuator_id = mj_name2id(
          mujoco_model_, mjOBJ_ACTUATOR, rotor_name.c_str());
        const double torque_gear = mujoco_model_->actuator_gear[6 * actuator_id + 5];
        const double torque_gear_sign = torque_gear < 0.0 ? -1.0 : 1.0;
        // actuator torque gear = rotor direction * signed m_f_rate
        rotor_direction_signs_.push_back(torque_gear_sign * forward_m_f_sign);
      }
    if(use_pwm_motor_model_)
      ROS_INFO("[mujoco] bidirectional PWM motor model: %.3f--%.3f, neutral %.3f, voltage %.1f V, positive thrust on %s side",
               motor_min_pwm_, motor_max_pwm_, motor_neutral_pwm_, motor_sim_voltage_,
               positive_thrust_below_neutral_ ? "low-PWM" : "high-PWM");

    // init joints from rosparam
    XmlRpc::XmlRpcValue all_servos_params;
    model_nh.getParam("servo_controller", all_servos_params);
    std::string init_value_param_name = "init_value";
    for(auto servo_group_params: all_servos_params)
      {
        if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;
        for(auto servo_params : servo_group_params.second)
          {
            if(servo_params.first.find("controller") != string::npos)
              {
                std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
                double init_value = 0.0;

                // check simulation param exists
                if(!servo_group_params.second.hasMember("simulation") &&
                   !servo_params.second.hasMember("simulation"))
                  {
                    ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());
                    continue;
                  }

                // search init_value in servo params
                if(!servo_params.second.hasMember("simulation") ||
                   (servo_params.second.hasMember("simulation") && !servo_params.second["simulation"].hasMember(init_value_param_name)))
                  {
                    // search init_value in servo group params
                    if(!servo_group_params.second["simulation"].hasMember(init_value_param_name))
                      {
                        ROS_ERROR("can not find '%s' gazebo paramter for servo %s", init_value_param_name.c_str(),  string(servo_params.second["name"]).c_str());
                        return false;
                      }
                    // use param of servo group
                    init_value = static_cast<double>(servo_group_params.second["simulation"][init_value_param_name]);
                  }
                else
                  {
                    // use param of servo
                    init_value = static_cast<double>(servo_params.second["simulation"][init_value_param_name]);
                  }
                const int actuator_id = mj_name2id(
                  mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, servo_name.c_str());
                if(actuator_id < 0 || actuator_id >= static_cast<int>(control_input_.size()))
                  {
                    ROS_WARN("mujoco: servo actuator '%s' does not exist; skipping its initial value",
                             servo_name.c_str());
                    continue;
                  }
                control_input_.at(actuator_id) = init_value;
              }
          }
      }

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh, rotor_list_.size());
    registerInterface(&spinal_interface_);

    ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
    simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0); // m
    simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0); // m/s
    simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0); // rad
    simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0); // rad
    simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0); // m/s
    simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0); // 1/s

    simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01); // [sec]
    simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001); // m
    simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001); // rad
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

    return true;
  }

  void AerialRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, "fc");
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;
    tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                             site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                             site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
    tf::Quaternion fc_quat;
    fc_rot_mat.getRotation(fc_quat);

    // MuJoCo returns object velocity as [angular, linear].  Use world-frame
    // linear velocity for odometry; the gyro below provides body-frame angular
    // velocity, matching the Gazebo hardware interface convention.
    mjtNum fc_vel_world[6];
    mj_objectVelocity(mujoco_model_, mujoco_data_, mjOBJ_SITE, fc_id,
                      fc_vel_world, 0);

    tf::Vector3 acc, gyro, mag;
    for(int i = 0; i < mujoco_model_->nsensor; i++)
      {
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "acc")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                acc[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "gyro")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                gyro[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "mag")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                mag[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
      }

    spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
    spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

    spinal_interface_.stateEstimate();

    /* publish ground truth value */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(ground_truth_pos_noise_);
    odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(ground_truth_pos_noise_);
    odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(ground_truth_pos_noise_);

    tf::Quaternion ground_truth_delta;
    ground_truth_delta.setRPY(
      gazebo::addNoise(ground_truth_rot_curr_drift_[0], ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()),
      gazebo::addNoise(ground_truth_rot_curr_drift_[1], ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()),
      gazebo::addNoise(ground_truth_rot_curr_drift_[2], ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()));
    const tf::Quaternion ground_truth_quat = fc_quat * ground_truth_delta;
    odom_msg.pose.pose.orientation.x = ground_truth_quat.x();
    odom_msg.pose.pose.orientation.y = ground_truth_quat.y();
    odom_msg.pose.pose.orientation.z = ground_truth_quat.z();
    odom_msg.pose.pose.orientation.w = ground_truth_quat.w();

    odom_msg.twist.twist.linear.x = fc_vel_world[3] + gazebo::addNoise(ground_truth_vel_curr_drift_[0], ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    odom_msg.twist.twist.linear.y = fc_vel_world[4] + gazebo::addNoise(ground_truth_vel_curr_drift_[1], ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    odom_msg.twist.twist.linear.z = fc_vel_world[5] + gazebo::addNoise(ground_truth_vel_curr_drift_[2], ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    odom_msg.twist.twist.angular.x = gyro.x() + gazebo::addNoise(ground_truth_angular_curr_drift_[0], ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());
    odom_msg.twist.twist.angular.y = gyro.y() + gazebo::addNoise(ground_truth_angular_curr_drift_[1], ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());
    odom_msg.twist.twist.angular.z = gyro.z() + gazebo::addNoise(ground_truth_angular_curr_drift_[2], ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());

    /* set ground truth for controller: use the value with noise */
    spinal_interface_.setGroundTruthStates(
      ground_truth_quat.x(), ground_truth_quat.y(), ground_truth_quat.z(), ground_truth_quat.w(),
      odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z);

    if((time - last_ground_truth_time_).toSec() >= ground_truth_pub_rate_)
      {
        ground_truth_pub_.publish(odom_msg);
        last_ground_truth_time_ = time;
      }

    if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);


        tf::Quaternion q_delta;
        q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_));
        tf::Quaternion q_noise = fc_quat * q_delta;
        pose_msg.pose.orientation.x = q_noise.x();
        pose_msg.pose.orientation.y = q_noise.y();
        pose_msg.pose.orientation.z = q_noise.z();
        pose_msg.pose.orientation.w = q_noise.w();

        mocap_pub_.publish(pose_msg);
        last_mocap_time_ = time;
      }

    DefaultRobotHWSim::read(time, period);
  }

  void AerialRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        if(use_pwm_motor_model_)
          {
            const double pwm = std::max(
              motor_min_pwm_, std::min(motor_max_pwm_, spinal_interface_.getPwm(i)));
            const bool low_pwm_branch = pwm < motor_neutral_pwm_;
            rotor_force = forceFromPwm(pwm, low_pwm_branch);
            const double m_f_rate = low_pwm_branch ? reverse_m_f_rate_
                                                   : forward_m_f_rate_;
            mujoco_model_->actuator_gear[6 * rotor_id + 5] =
              rotor_direction_signs_.at(i) * m_f_rate;
          }
        control_input_.at(rotor_id) = rotor_force;
      }

      DefaultRobotHWSim::write(time, period);
  }

  std::vector<AerialRobotHWSim::MotorCurveReference>
  AerialRobotHWSim::loadMotorCurve(
    const ros::NodeHandle& motor_nh, int reference_count) const
  {
    std::vector<MotorCurveReference> result;
    result.reserve(std::max(0, reference_count));
    for(int i = 0; i < reference_count; ++i)
      {
        const ros::NodeHandle reference_nh(
          motor_nh, "ref" + std::to_string(i + 1));
        MotorCurveReference reference;
        if(!reference_nh.getParam("voltage", reference.voltage))
          continue;
        reference_nh.param("max_thrust", reference.max_thrust, 0.0);
        for(size_t coefficient = 0; coefficient < reference.polynomial.size(); ++coefficient)
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
    if(std::abs(pwm - motor_neutral_pwm_) <= motor_neutral_deadband_)
      return 0.0;
    // In the legacy MotorInfo schema the top-level curve is the high-PWM
    // branch and motor_info/reverse is the low-PWM branch.  Direction is a
    // separate property because some bidirectional ESCs reverse that polarity.
    const auto& curve = low_pwm_branch ? reverse_motor_curve_
                                       : forward_motor_curve_;
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
    if(reference->voltage > 0.0 && motor_sim_voltage_ > 0.0)
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

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros_control::RobotHWSim)
