#pragma once

#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <hugmy/model/pressure_thrust_bend_model.h>
#include <spinal/NeuronImuStates.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>

namespace aerial_robot_control
{

/**
 * @brief In-flight shape controller for the Hugmy Quad-to-Bi transition.
 *
 * The vehicle always has four rotors.  "Bi" denotes the optimized geometry
 * in which arms 2 and 4 are bent while arms 1 and 3 remain straight.
 * The parent controller provides flight control; its wrench allocation and
 * inertia compensation follow the updated articulated robot model while the
 * activation-time LQI gains are retained during the transition.
 */
class QuadBiController : public UnderActuatedLQIController
{
public:
  static constexpr size_t ARM_COUNT = 4;

  enum ShapeState : uint8_t
  {
    QUAD = 0,
    TO_BI = 1,
    BI = 2,
    TO_QUAD = 3,
    ERROR = 4,
  };

  QuadBiController();
  ~QuadBiController() override = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

protected:
  void controlCore() override;

private:
  using ArmArray = std::array<double, ARM_COUNT>;

  void loadQuadBiParams();
  bool setBiMode(std_srvs::SetBool::Request& req,
                 std_srvs::SetBool::Response& res);
  void neuronImuCallback(const spinal::NeuronImuStates::ConstPtr& msg);
  void gripStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  bool readBendAngles(ArmArray& bend_rad, ros::Time& newest_stamp) const;
  double measuredArmBendFromImu(size_t arm,
                                const std::array<double, 3>& acceleration) const;
  void updateRobotModelFromBend(const ArmArray& bend_rad);
  void updateShapeTransition(const ArmArray& measured_bend_rad,
                             bool feedback_fresh, double dt);
  void smoothTransitionThrust(const ArmArray& measured_bend_rad, double dt);
  void publishPressureTargets(const ArmArray& measured_bend_rad, double dt);
  double pressureForBend(double bend_rad, double thrust_n,
                         bool bending) const;
  bool flightConditionSafe() const;
  bool recoveryConditionUnsafe(std::string& reason) const;
  void startQuadRecoveryLocked(const ros::Time& now,
                               const std::string& reason);
  bool shapeReached(const ArmArray& measured_bend_rad) const;
  void publishStatus(const ArmArray& measured_bend_rad);
  bool enablePressureController(std::string& error);
  bool isBiArm(size_t arm) const;

  ros::Subscriber neuron_imu_sub_;
  ros::Subscriber root_neuron_imu_sub_;
  ros::Subscriber grip_state_sub_;
  ros::Publisher pressure_target_pub_;
  ros::Publisher state_pub_;
  ros::Publisher target_bend_pub_;
  ros::Publisher measured_bend_pub_;
  ros::Publisher target_pressure_pub_;
  ros::ServiceServer mode_service_;
  ros::ServiceClient pressure_enable_client_;

  mutable std::mutex feedback_mutex_;
  std::array<std::array<double, 3>, ARM_COUNT> neuron_acc_{};
  std::array<ros::Time, ARM_COUNT> neuron_stamp_{};
  std::array<bool, ARM_COUNT> neuron_acc_initialized_{{false, false, false, false}};
  ArmArray grip_bend_rad_{{NAN, NAN, NAN, NAN}};
  ros::Time grip_stamp_;

  ShapeState shape_state_ = QUAD;
  bool pressure_command_active_ = false;
  bool requested_bi_ = false;
  ArmArray commanded_bend_rad_{{0.0, 0.0, 0.0, 0.0}};
  ArmArray commanded_pressure_kpa_{{0.0, 0.0, 0.0, 0.0}};
  ArmArray smoothed_thrust_n_{{0.0, 0.0, 0.0, 0.0}};
  ArmArray quad_trim_thrust_n_{{0.0, 0.0, 0.0, 0.0}};
  ArmArray quad_trim_z_feedback_n_{{0.0, 0.0, 0.0, 0.0}};
  bool smoothed_thrust_initialized_ = false;
  bool quad_trim_thrust_valid_ = false;
  ros::Time last_transition_update_;
  ros::Time transition_start_;
  ros::Time shape_stable_start_;
  ros::Time unsafe_start_;

  std::array<int, ARM_COUNT> neuron_slave_ids_{{1, 2, 3, 4}};
  ArmArray neuron_bend_offset_rad_{{0.0, 0.0, 0.0, 0.0}};
  std::vector<int> bi_arm_ids_{2, 4};  // one-based arm IDs
  ArmArray arm_yaw_rad_{{-2.3561944902, -0.7853981634,
                          0.7853981634,  2.3561944902}};
  std::array<double, 3> joint_distribution_{{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};

  double bi_bend_rad_ = M_PI / 2.0;
  double transition_rate_rad_s_ = 15.0 * M_PI / 180.0;
  double bend_tolerance_rad_ = 8.0 * M_PI / 180.0;
  double command_tolerance_rad_ = 1.0 * M_PI / 180.0;
  double stable_duration_s_ = 0.5;
  double transition_timeout_s_ = 12.0;
  double feedback_timeout_s_ = 0.25;
  double unsafe_duration_s_ = 0.3;
  double max_tilt_rad_ = 15.0 * M_PI / 180.0;
  double max_angular_rate_rad_s_ = 0.5;
  bool auto_quad_recovery_ = true;
  double recovery_max_height_error_m_ = 0.35;
  double recovery_max_tilt_rad_ = 25.0 * M_PI / 180.0;
  double recovery_max_angular_rate_rad_s_ = 1.5;
  double recovery_trigger_duration_s_ = 0.20;
  double min_pressure_kpa_ = 0.0;
  double max_pressure_kpa_ = 50.0;
  double pressure_rate_kpa_s_ = 10.0;
  double transition_thrust_rate_n_s_ = 2.0;
  double bi_straight_arm_thrust_scale_ = 2.0;
  double bi_bent_arm_thrust_scale_ = 0.62;
  double bend_model_thrust_offset_n_ = 1.8;
  double neuron_acc_lpf_tau_s_ = 0.20;
  bool update_robot_model_from_feedback_ = false;
  bool require_hover_state_ = true;

  hugmy::PressureThrustBendModel bend_model_;
};

}  // namespace aerial_robot_control
