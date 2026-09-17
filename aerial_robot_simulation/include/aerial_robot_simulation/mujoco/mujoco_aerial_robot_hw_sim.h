#pragma once

#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>
#include <nav_msgs/Odometry.h>
#include <array>

namespace mujoco_ros_control
{
  class AerialRobotHWSim : public mujoco_ros_control::DefaultRobotHWSim
  {
  public:
    AerialRobotHWSim() {};
    ~AerialRobotHWSim() {}

    bool init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              ) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

  protected:
    struct MotorCurveReference
    {
      double voltage = 0.0;
      double max_thrust = 0.0;
      std::array<double, 3> polynomial{{0.0, 0.0, 0.0}};
    };

    std::vector<MotorCurveReference> loadMotorCurve(
      const ros::NodeHandle& motor_nh, int reference_count) const;
    double forceFromPwm(double pwm, bool low_pwm_branch) const;

    hardware_interface::MujocoSpinalInterface spinal_interface_;

    std::vector<std::string> rotor_list_;
    std::vector<double> rotor_direction_signs_;
    std::vector<MotorCurveReference> forward_motor_curve_, reverse_motor_curve_;
    bool use_pwm_motor_model_ = false;
    double motor_min_pwm_ = 0.5;
    double motor_neutral_pwm_ = 0.5;
    double motor_max_pwm_ = 1.0;
    double motor_neutral_deadband_ = 1.0e-4;
    double motor_sim_voltage_ = 0.0;
    bool positive_thrust_below_neutral_ = false;
    double forward_m_f_rate_ = 0.0;
    double reverse_m_f_rate_ = 0.0;
    int pwm_conversion_mode_ = 0;
    ros::Publisher ground_truth_pub_;
    ros::Publisher mocap_pub_;
    double ground_truth_pub_rate_;
    double mocap_pub_rate_;
    double mocap_rot_noise_, mocap_pos_noise_;
    double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
    double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
    double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
    double ground_truth_rot_curr_drift_[3] = {};
    double ground_truth_vel_curr_drift_[3] = {};
    double ground_truth_angular_curr_drift_[3] = {};
    double joint_state_pub_rate_ = 0.02;

    ros::Time last_ground_truth_time_, last_mocap_time_;
  };
}
