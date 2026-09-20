#include <hugmy/control/quad_bi_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{

namespace
{
double clampValue(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double approach(double value, double target, double maximum_step)
{
  return value + clampValue(target - value, -maximum_step, maximum_step);
}
}  // namespace

QuadBiController::QuadBiController() : UnderActuatedLQIController()
{
}

void QuadBiController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  UnderActuatedLQIController::initialize(
      nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  loadQuadBiParams();

  neuron_imu_sub_ = nh_.subscribe(
      "neuron/imu_states", 1, &QuadBiController::neuronImuCallback, this);
  if (nh_.resolveName("neuron/imu_states") != "/neuron/imu_states")
    root_neuron_imu_sub_ = nh_.subscribe(
        "/neuron/imu_states", 1, &QuadBiController::neuronImuCallback, this);
  grip_state_sub_ = nh_.subscribe(
      "pneumatic/grip_state", 1, &QuadBiController::gripStateCallback, this);

  pressure_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "independent_arm_pressure_controller/target_pressure", 1);
  state_pub_ = nh_.advertise<std_msgs::UInt8>(
      "controller/quad_bi/state", 1, true);
  target_bend_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "controller/quad_bi/target_bend_deg", 1);
  measured_bend_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "controller/quad_bi/measured_bend_deg", 1);
  target_pressure_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "controller/quad_bi/target_pressure_kpa", 1);
  mode_service_ = nh_.advertiseService(
      "controller/set_bi_mode", &QuadBiController::setBiMode, this);
  pressure_enable_client_ = nh_.serviceClient<std_srvs::SetBool>(
      "independent_arm_pressure_controller/enable");

  std_msgs::UInt8 initial_state;
  initial_state.data = shape_state_;
  state_pub_.publish(initial_state);
  ROS_INFO("Hugmy Quad/Bi controller ready: Bi bends arms 2 and 4 to %.1f deg",
           bi_bend_rad_ * 180.0 / M_PI);
}

void QuadBiController::loadQuadBiParams()
{
  ros::NodeHandle mode_nh(nh_, "controller/quad_bi");
  double value_deg = 0.0;
  mode_nh.param("bi_bend_angle_deg", value_deg, 90.0);
  bi_bend_rad_ = value_deg * M_PI / 180.0;
  mode_nh.param("transition_rate_deg_s", value_deg, 15.0);
  transition_rate_rad_s_ = value_deg * M_PI / 180.0;
  mode_nh.param("bend_tolerance_deg", value_deg, 8.0);
  bend_tolerance_rad_ = value_deg * M_PI / 180.0;
  mode_nh.param("command_tolerance_deg", value_deg, 1.0);
  command_tolerance_rad_ = value_deg * M_PI / 180.0;
  mode_nh.param("stable_duration_s", stable_duration_s_, 0.5);
  mode_nh.param("transition_timeout_s", transition_timeout_s_, 12.0);
  mode_nh.param("feedback_timeout_s", feedback_timeout_s_, 0.25);
  mode_nh.param("unsafe_duration_s", unsafe_duration_s_, 0.3);
  mode_nh.param("max_tilt_deg", value_deg, 15.0);
  max_tilt_rad_ = value_deg * M_PI / 180.0;
  mode_nh.param("max_angular_rate_rad_s", max_angular_rate_rad_s_, 0.5);
  mode_nh.param("auto_quad_recovery", auto_quad_recovery_, true);
  mode_nh.param("recovery_max_height_error_m",
                recovery_max_height_error_m_, 0.35);
  mode_nh.param("recovery_max_tilt_deg", value_deg, 25.0);
  recovery_max_tilt_rad_ = value_deg * M_PI / 180.0;
  mode_nh.param("recovery_max_angular_rate_rad_s",
                recovery_max_angular_rate_rad_s_, 1.5);
  mode_nh.param("recovery_trigger_duration_s",
                recovery_trigger_duration_s_, 0.20);
  mode_nh.param("min_pressure_kpa", min_pressure_kpa_, 0.0);
  mode_nh.param("max_pressure_kpa", max_pressure_kpa_, 50.0);
  mode_nh.param("pressure_rate_kpa_s", pressure_rate_kpa_s_, 10.0);
  mode_nh.param("transition_thrust_rate_n_s",
                transition_thrust_rate_n_s_, 2.0);
  mode_nh.param("bi_straight_arm_thrust_scale",
                bi_straight_arm_thrust_scale_, 2.0);
  mode_nh.param("bi_bent_arm_thrust_scale",
                bi_bent_arm_thrust_scale_, 0.62);
  mode_nh.param("bend_model_thrust_offset_n", bend_model_thrust_offset_n_, 1.8);
  mode_nh.param("neuron_acc_lpf_tau_s", neuron_acc_lpf_tau_s_, 0.20);
  mode_nh.param("update_robot_model_from_feedback",
                update_robot_model_from_feedback_, false);
  mode_nh.param("require_hover_state", require_hover_state_, true);

  std::vector<int> ids;
  if (mode_nh.getParam("bi_arm_ids", ids))
    {
      if (ids.size() != 2 || ids[0] < 1 || ids[0] > 4 ||
          ids[1] < 1 || ids[1] > 4 || ids[0] == ids[1])
        ROS_ERROR("controller/quad_bi/bi_arm_ids must contain two distinct IDs in 1..4; using [2, 4]");
      else
        bi_arm_ids_ = ids;
    }

  std::vector<int> neuron_ids;
  if (mode_nh.getParam("neuron_slave_ids", neuron_ids) &&
      neuron_ids.size() == ARM_COUNT)
    std::copy(neuron_ids.begin(), neuron_ids.end(), neuron_slave_ids_.begin());

  std::vector<double> bend_offsets_deg;
  if (mode_nh.getParam("neuron_bend_offset_deg", bend_offsets_deg) &&
      bend_offsets_deg.size() == ARM_COUNT)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      neuron_bend_offset_rad_[arm] = bend_offsets_deg[arm] * M_PI / 180.0;

  std::vector<double> yaw;
  if (mode_nh.getParam("arm_yaw_rad", yaw) && yaw.size() == ARM_COUNT)
    std::copy(yaw.begin(), yaw.end(), arm_yaw_rad_.begin());

  std::vector<double> distribution;
  if (mode_nh.getParam("joint_distribution", distribution) &&
      distribution.size() == joint_distribution_.size())
    {
      const double sum = distribution[0] + distribution[1] + distribution[2];
      if (sum > 1e-6 && std::all_of(distribution.begin(), distribution.end(),
                                   [](double value) { return value >= 0.0; }))
        for (size_t i = 0; i < joint_distribution_.size(); ++i)
          joint_distribution_[i] = distribution[i] / sum;
      else
        ROS_ERROR("controller/quad_bi/joint_distribution must be nonnegative with a positive sum; using equal distribution");
    }

  if (!(bi_bend_rad_ > 0.0 && bi_bend_rad_ <= M_PI) ||
      transition_rate_rad_s_ <= 0.0 || bend_tolerance_rad_ <= 0.0 ||
      stable_duration_s_ < 0.0 || transition_timeout_s_ <= 0.0 ||
      min_pressure_kpa_ < 0.0 || max_pressure_kpa_ <= min_pressure_kpa_ ||
      pressure_rate_kpa_s_ <= 0.0 || transition_thrust_rate_n_s_ <= 0.0 ||
      bi_straight_arm_thrust_scale_ <= 0.0 ||
      bi_bent_arm_thrust_scale_ < 0.0 ||
      recovery_max_height_error_m_ <= 0.0 ||
      recovery_max_tilt_rad_ <= 0.0 ||
      recovery_max_angular_rate_rad_s_ <= 0.0 ||
      recovery_trigger_duration_s_ < 0.0)
    throw std::runtime_error("Invalid controller/quad_bi parameters");
}

bool QuadBiController::enablePressureController(std::string& error)
{
  if (!pressure_enable_client_.exists())
    {
      error = "independent arm pressure controller service is unavailable";
      return false;
    }
  std_srvs::SetBool enable;
  enable.request.data = true;
  if (!pressure_enable_client_.call(enable) || !enable.response.success)
    {
      error = enable.response.message.empty()
          ? "failed to enable independent arm pressure controller"
          : enable.response.message;
      return false;
    }
  return true;
}

bool QuadBiController::setBiMode(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res)
{
  ArmArray measured;
  ros::Time feedback_stamp;
  const bool feedback_fresh = readBendAngles(measured, feedback_stamp) &&
      !feedback_stamp.isZero() &&
      (ros::Time::now() - feedback_stamp).toSec() <= feedback_timeout_s_;
  // Entering Bi needs a reliable pose and a quiet hover.  Returning to Quad
  // is a recovery action, so it must remain available while the aircraft is
  // tilted or arm feedback is temporarily missing.
  if (req.data && !feedback_fresh)
    {
      res.success = false;
      res.message = "fresh bend feedback is required before an in-flight shape transition";
      return true;
    }
  if (req.data && !flightConditionSafe())
    {
      res.success = false;
      res.message = "transition rejected: vehicle must be hovering with small tilt and angular rate";
      return true;
    }

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (shape_state_ == TO_QUAD && req.data)
      {
        res.success = false;
        res.message = "Bi request rejected while Quad recovery is active";
        return true;
      }
    if ((shape_state_ == TO_BI || shape_state_ == TO_QUAD) &&
        requested_bi_ == req.data)
      {
        res.success = true;
        res.message = req.data ? "Quad-to-Bi transition already active"
                               : "Bi-to-Quad transition already active";
        return true;
      }
    if ((req.data && shape_state_ == BI) || (!req.data && shape_state_ == QUAD))
      {
        res.success = true;
        res.message = req.data ? "already in Bi mode" : "already in Quad mode";
        return true;
      }
  }

  std::string enable_error;
  if (!enablePressureController(enable_error))
    {
      res.success = false;
      res.message = enable_error;
      return true;
    }

  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    requested_bi_ = req.data;
    shape_state_ = req.data ? TO_BI : TO_QUAD;
    pressure_command_active_ = true;
    if (feedback_fresh) commanded_bend_rad_ = measured;
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      commanded_pressure_kpa_[arm] = pressureForBend(
          commanded_bend_rad_[arm], arm < target_base_thrust_.size()
              ? target_base_thrust_[arm] : 0.0, req.data);
    transition_start_ = now;
    last_transition_update_ = now;
    shape_stable_start_ = ros::Time(0);
    unsafe_start_ = ros::Time(0);
  }

  // Preserve the altitude and attitude controller state.  Resetting all PIDs
  // here removes the hover integral for one cycle and makes every rotor lose
  // thrust at exactly the start of the shape transition.
  res.success = true;
  res.message = req.data ? "Quad-to-Bi transition accepted"
                         : "Bi-to-Quad transition accepted";
  ROS_WARN("%s", res.message.c_str());
  return true;
}

void QuadBiController::neuronImuCallback(
    const spinal::NeuronImuStates::ConstPtr& msg)
{
  const ros::Time now = ros::Time::now();
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (const auto& imu : msg->imus)
    for (size_t arm = 0; arm < ARM_COUNT; ++arm)
      if (imu.slave_id == neuron_slave_ids_[arm])
        {
          const double dt = neuron_stamp_[arm].isZero() ? 0.0
              : clampValue((now - neuron_stamp_[arm]).toSec(), 0.0, 0.1);
          const double alpha = neuron_acc_initialized_[arm]
              ? dt / std::max(1e-6, neuron_acc_lpf_tau_s_ + dt) : 1.0;
          for (size_t axis = 0; axis < 3; ++axis)
            neuron_acc_[arm][axis] +=
                alpha * (imu.acc[axis] - neuron_acc_[arm][axis]);
          neuron_acc_initialized_[arm] = true;
          neuron_stamp_[arm] = now;
          break;
        }
}

void QuadBiController::gripStateCallback(
    const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // MuJoCo publishes the exact articulated bend in [24:28].
  if (msg->data.size() < 7 * ARM_COUNT) return;
  ArmArray bend;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      bend[arm] = msg->data[6 * ARM_COUNT + arm];
      if (!std::isfinite(bend[arm])) return;
    }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  grip_bend_rad_ = bend;
  grip_stamp_ = ros::Time::now();
}

double QuadBiController::measuredArmBendFromImu(
    size_t arm, const std::array<double, 3>& acceleration) const
{
  const tf::Matrix3x3 orientation =
      estimator_->getOrientation(Frame::COG, estimate_mode_);
  const tf::Vector3 body_up_tf =
      orientation.inverse() * tf::Vector3(0.0, 0.0, 1.0);
  std::array<double, 3> body_up{{body_up_tf.x(), body_up_tf.y(), body_up_tf.z()}};
  const double body_norm = std::sqrt(body_up[0] * body_up[0] +
                                     body_up[1] * body_up[1] +
                                     body_up[2] * body_up[2]);
  const double measured_norm = std::sqrt(acceleration[0] * acceleration[0] +
                                         acceleration[1] * acceleration[1] +
                                         acceleration[2] * acceleration[2]);
  if (!std::isfinite(body_norm) || !std::isfinite(measured_norm) ||
      body_norm < 1e-6 || measured_norm < 1e-6)
    return std::numeric_limits<double>::quiet_NaN();
  for (double& value : body_up) value /= body_norm;
  std::array<double, 3> measured = acceleration;
  for (double& value : measured) value /= measured_norm;

  const double c_yaw = std::cos(arm_yaw_rad_[arm]);
  const double s_yaw = std::sin(arm_yaw_rad_[arm]);
  const std::array<double, 3> local_up{{
      c_yaw * body_up[0] + s_yaw * body_up[1],
      -s_yaw * body_up[0] + c_yaw * body_up[1], body_up[2]}};
  const double dot_xz = local_up[0] * measured[0] +
                        local_up[2] * measured[2];
  const double cross_xz = local_up[2] * measured[0] -
                          local_up[0] * measured[2];
  return clampValue(-std::atan2(cross_xz, dot_xz) -
                        neuron_bend_offset_rad_[arm],
                    0.0, M_PI);
}

bool QuadBiController::readBendAngles(ArmArray& bend_rad,
                                     ros::Time& newest_stamp) const
{
  std::array<std::array<double, 3>, ARM_COUNT> acceleration;
  std::array<ros::Time, ARM_COUNT> neuron_stamp;
  std::array<bool, ARM_COUNT> initialized;
  ArmArray grip_bend;
  ros::Time grip_stamp;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    acceleration = neuron_acc_;
    neuron_stamp = neuron_stamp_;
    initialized = neuron_acc_initialized_;
    grip_bend = grip_bend_rad_;
    grip_stamp = grip_stamp_;
  }

  const ros::Time now = ros::Time::now();
  if (!grip_stamp.isZero() &&
      (now - grip_stamp).toSec() <= feedback_timeout_s_)
    {
      bend_rad = grip_bend;
      newest_stamp = grip_stamp;
      return std::all_of(bend_rad.begin(), bend_rad.end(),
                         [](double value) { return std::isfinite(value); });
    }

  newest_stamp = now;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      if (!initialized[arm] || neuron_stamp[arm].isZero() ||
          (now - neuron_stamp[arm]).toSec() > feedback_timeout_s_)
        return false;
      newest_stamp = std::min(newest_stamp, neuron_stamp[arm]);
      bend_rad[arm] = measuredArmBendFromImu(arm, acceleration[arm]);
      if (!std::isfinite(bend_rad[arm])) return false;
    }
  return true;
}

void QuadBiController::updateRobotModelFromBend(const ArmArray& bend_rad)
{
  if (!update_robot_model_from_feedback_) return;
  KDL::JntArray joints = robot_model_->getJointPositions();
  const auto& index = robot_model_->getJointIndexMap();
  const std::array<double, 3> limits{{0.8727, 1.047, 1.047}};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    for (size_t joint = 0; joint < joint_distribution_.size(); ++joint)
      {
        const std::string name = "joint_" + std::to_string(arm + 1) +
                                 "_" + std::to_string(joint + 1);
        const auto it = index.find(name);
        if (it != index.end())
          joints(it->second) = clampValue(
              bend_rad[arm] * joint_distribution_[joint], 0.0, limits[joint]);
      }
  robot_model_->updateRobotModel(joints);
}

bool QuadBiController::isBiArm(size_t arm) const
{
  const int one_based_id = static_cast<int>(arm + 1);
  return std::find(bi_arm_ids_.begin(), bi_arm_ids_.end(), one_based_id) !=
         bi_arm_ids_.end();
}

bool QuadBiController::flightConditionSafe() const
{
  if (require_hover_state_ &&
      navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE)
    return false;
  return std::abs(rpy_.x()) <= max_tilt_rad_ &&
         std::abs(rpy_.y()) <= max_tilt_rad_ &&
         omega_.length() <= max_angular_rate_rad_s_ &&
         !navigator_->getForceLandingFlag();
}

bool QuadBiController::recoveryConditionUnsafe(std::string& reason) const
{
  const double current_height =
      estimator_->getPos(Frame::COG, estimate_mode_).z();
  const double target_height = navigator_->getTargetPos().z();
  const double height_error = target_height - current_height;
  if (!std::isfinite(current_height) || !std::isfinite(target_height) ||
      !std::isfinite(rpy_.x()) || !std::isfinite(rpy_.y()) ||
      !std::isfinite(omega_.length()))
    {
      reason = "non-finite flight state";
      return true;
    }
  if (height_error > recovery_max_height_error_m_)
    {
      reason = "height error " + std::to_string(height_error) + " m";
      return true;
    }
  if (std::abs(rpy_.x()) > recovery_max_tilt_rad_ ||
      std::abs(rpy_.y()) > recovery_max_tilt_rad_)
    {
      reason = "roll/pitch exceeded recovery limit";
      return true;
    }
  if (omega_.length() > recovery_max_angular_rate_rad_s_)
    {
      reason = "angular rate exceeded recovery limit";
      return true;
    }
  return false;
}

void QuadBiController::startQuadRecoveryLocked(
    const ros::Time& now, const std::string& reason)
{
  requested_bi_ = false;
  shape_state_ = TO_QUAD;
  pressure_command_active_ = true;
  transition_start_ = now;
  last_transition_update_ = now;
  shape_stable_start_ = ros::Time(0);
  unsafe_start_ = ros::Time(0);
  ROS_ERROR("Bi safety recovery: returning to Quad (%s)", reason.c_str());
}

bool QuadBiController::shapeReached(const ArmArray& measured_bend_rad) const
{
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double target = requested_bi_ && isBiArm(arm) ? bi_bend_rad_ : 0.0;
      if (std::abs(commanded_bend_rad_[arm] - target) > command_tolerance_rad_ ||
          std::abs(measured_bend_rad[arm] - target) > bend_tolerance_rad_)
        return false;
    }
  return true;
}

void QuadBiController::updateShapeTransition(
    const ArmArray& measured_bend_rad, bool feedback_fresh, double dt)
{
  const ros::Time now = ros::Time::now();
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (shape_state_ == QUAD || shape_state_ == ERROR) return;

  std::string recovery_reason;
  bool unsafe = false;
  if (shape_state_ == TO_BI)
    {
      if (!feedback_fresh)
        {
          unsafe = true;
          recovery_reason = "bend feedback became stale";
        }
      else if (!flightConditionSafe())
        {
          unsafe = true;
          recovery_reason = "transition flight condition became unsafe";
        }
    }
  if ((shape_state_ == TO_BI || shape_state_ == BI) &&
      auto_quad_recovery_)
    {
      std::string severe_reason;
      if (recoveryConditionUnsafe(severe_reason))
        {
          unsafe = true;
          recovery_reason = severe_reason;
        }
    }

  if (shape_state_ == TO_BI || shape_state_ == BI)
    {
      if (unsafe)
        {
          if (unsafe_start_.isZero()) unsafe_start_ = now;
          const double trigger_duration = shape_state_ == BI
              ? recovery_trigger_duration_s_ : unsafe_duration_s_;
          if ((now - unsafe_start_).toSec() >= trigger_duration)
            startQuadRecoveryLocked(now, recovery_reason);
          return;
        }
      unsafe_start_ = ros::Time(0);
      if (shape_state_ == BI) return;
    }

  // TO_QUAD intentionally keeps running despite tilt or stale bend feedback:
  // exhausting the bending arms is itself the recovery action.
  const double maximum_step = transition_rate_rad_s_ * dt;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double target = requested_bi_ && isBiArm(arm) ? bi_bend_rad_ : 0.0;
      commanded_bend_rad_[arm] = approach(
          commanded_bend_rad_[arm], target, maximum_step);
    }

  if (feedback_fresh && shapeReached(measured_bend_rad))
    {
      if (shape_stable_start_.isZero()) shape_stable_start_ = now;
      if ((now - shape_stable_start_).toSec() >= stable_duration_s_)
        {
          shape_state_ = requested_bi_ ? BI : QUAD;
          ROS_WARN("Hugmy shape transition complete: %s",
                   requested_bi_ ? "BI" : "QUAD");
          return;
        }
    }
  else
    shape_stable_start_ = ros::Time(0);

  if (!transition_start_.isZero() &&
      (now - transition_start_).toSec() > transition_timeout_s_)
    {
      if (shape_state_ == TO_BI)
        startQuadRecoveryLocked(now, "Quad-to-Bi transition timed out");
      else
        {
          shape_state_ = ERROR;
          ROS_ERROR("Bi-to-Quad recovery timed out after %.1f s",
                    transition_timeout_s_);
        }
    }
}

void QuadBiController::smoothTransitionThrust(
    const ArmArray& measured_bend_rad, double dt)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (target_base_thrust_.size() < ARM_COUNT) return;

  auto zFeedback = [&](size_t arm) {
    if (arm >= pid_msg_.z.p_term.size() ||
        arm >= pid_msg_.z.i_term.size() ||
        arm >= pid_msg_.z.d_term.size())
      return 0.0;
    return static_cast<double>(pid_msg_.z.p_term[arm]) +
           static_cast<double>(pid_msg_.z.i_term[arm]) +
           static_cast<double>(pid_msg_.z.d_term[arm]);
  };

  if (!smoothed_thrust_initialized_)
    {
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        smoothed_thrust_n_[arm] = target_base_thrust_[arm];
      smoothed_thrust_initialized_ = true;
    }

  const bool transitioning = shape_state_ == TO_BI || shape_state_ == TO_QUAD;
  if (!transitioning)
    {
      // Follow normal Quad/Bi flight control without adding bandwidth limits.
      // This also remembers the exact hover command used at the next request.
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          smoothed_thrust_n_[arm] = target_base_thrust_[arm];
          if (shape_state_ == QUAD)
            {
              quad_trim_thrust_n_[arm] = target_base_thrust_[arm];
              quad_trim_z_feedback_n_[arm] = zFeedback(arm);
            }
        }
      if (shape_state_ == QUAD) quad_trim_thrust_valid_ = true;
      return;
    }

  if (!quad_trim_thrust_valid_)
    {
      for (size_t arm = 0; arm < ARM_COUNT; ++arm)
        {
          quad_trim_thrust_n_[arm] = target_base_thrust_[arm];
          quad_trim_z_feedback_n_[arm] = zFeedback(arm);
        }
      quad_trim_thrust_valid_ = true;
    }

  double progress = 0.0;
  size_t bi_arm_count = 0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (isBiArm(arm))
      {
        progress += commanded_bend_rad_[arm] / bi_bend_rad_;
        ++bi_arm_count;
      }
  progress = bi_arm_count == 0 ? 0.0 :
      clampValue(progress / static_cast<double>(bi_arm_count), 0.0, 1.0);

  double measured_progress = 0.0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (isBiArm(arm))
      measured_progress += measured_bend_rad[arm] / bi_bend_rad_;
  measured_progress = bi_arm_count == 0 ? 0.0 :
      clampValue(measured_progress / static_cast<double>(bi_arm_count), 0.0, 1.0);

  const double maximum_step = transition_thrust_rate_n_s_ * dt;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      // Proactively unload the bending pair so rotor thrust no longer holds
      // those compliant arms straight. The endpoint scales come from the
      // robust fixed-tilt hover solution (about 7.0 N on arms 1/3 and 2.1 N
      // on arms 2/4, starting from about 3.5 N per arm). Preserve changes in
      // the altitude PID relative to the captured Quad trim.
      double scale = 1.0 + progress *
          (bi_straight_arm_thrust_scale_ - 1.0);
      if (isBiArm(arm))
        {
          if (shape_state_ == TO_BI)
            {
              // First remove the follower force that holds a compliant arm
              // straight. Restore the optimized Bi thrust only as feedback
              // confirms that the arm has actually bent.
              scale = 1.0 - progress +
                  bi_bent_arm_thrust_scale_ * measured_progress;
            }
          else
            {
              // During Bi-to-Quad, follow the physical extension so the bent
              // rotor is not loaded abruptly before its arm is straight.
              scale = 1.0 + measured_progress *
                  (bi_bent_arm_thrust_scale_ - 1.0);
            }
        }
      const double desired = std::max(0.0,
          quad_trim_thrust_n_[arm] * scale +
          zFeedback(arm) - quad_trim_z_feedback_n_[arm]);
      smoothed_thrust_n_[arm] = approach(
          smoothed_thrust_n_[arm], desired, maximum_step);
      target_base_thrust_[arm] = static_cast<float>(smoothed_thrust_n_[arm]);
      if (arm < pid_msg_.z.total.size())
        pid_msg_.z.total[arm] = smoothed_thrust_n_[arm];
    }
}

double QuadBiController::pressureForBend(double bend_rad, double thrust_n,
                                         bool bending) const
{
  using Trend = hugmy::PressureThrustBendModel::Trend;
  const Trend pressure_trend = bending ? Trend::INCREASING : Trend::DECREASING;
  const double table_thrust = std::max(0.0, thrust_n) + bend_model_thrust_offset_n_;
  const double target_deg = clampValue(bend_rad * 180.0 / M_PI, 0.0, 180.0);
  auto angle_at = [&](double pressure) {
    return bend_model_.angleDeg(
        pressure, table_thrust, pressure_trend, Trend::NEUTRAL);
  };
  if (target_deg <= angle_at(min_pressure_kpa_)) return min_pressure_kpa_;
  if (target_deg >= angle_at(max_pressure_kpa_)) return max_pressure_kpa_;
  double lower = min_pressure_kpa_;
  double upper = max_pressure_kpa_;
  for (int iteration = 0; iteration < 24; ++iteration)
    {
      const double middle = 0.5 * (lower + upper);
      if (angle_at(middle) < target_deg) lower = middle;
      else upper = middle;
    }
  return 0.5 * (lower + upper);
}

void QuadBiController::publishPressureTargets(
    const ArmArray& measured_bend_rad, double dt)
{
  (void)measured_bend_rad;
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!pressure_command_active_) return;
  const double maximum_step = pressure_rate_kpa_s_ * dt;
  std_msgs::Float32MultiArray command;
  command.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double desired = pressureForBend(
          commanded_bend_rad_[arm],
          arm < target_base_thrust_.size() ? target_base_thrust_[arm] : 0.0,
          requested_bi_);
      commanded_pressure_kpa_[arm] = approach(
          commanded_pressure_kpa_[arm], desired, maximum_step);
      command.data[arm] = static_cast<float>(commanded_pressure_kpa_[arm]);
    }
  pressure_target_pub_.publish(command);
}

void QuadBiController::publishStatus(const ArmArray& measured_bend_rad)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  std_msgs::UInt8 state;
  state.data = shape_state_;
  state_pub_.publish(state);

  std_msgs::Float32MultiArray target_bend, measured_bend, target_pressure;
  target_bend.data.resize(ARM_COUNT);
  measured_bend.data.resize(ARM_COUNT);
  target_pressure.data.resize(ARM_COUNT);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      target_bend.data[arm] = commanded_bend_rad_[arm] * 180.0 / M_PI;
      measured_bend.data[arm] = measured_bend_rad[arm] * 180.0 / M_PI;
      target_pressure.data[arm] = commanded_pressure_kpa_[arm];
    }
  target_bend_pub_.publish(target_bend);
  measured_bend_pub_.publish(measured_bend);
  target_pressure_pub_.publish(target_pressure);
}

void QuadBiController::controlCore()
{
  ArmArray measured_bend{{0.0, 0.0, 0.0, 0.0}};
  ros::Time feedback_stamp;
  const bool feedback_valid = readBendAngles(measured_bend, feedback_stamp);
  const bool feedback_fresh = feedback_valid && !feedback_stamp.isZero() &&
      (ros::Time::now() - feedback_stamp).toSec() <= feedback_timeout_s_;
  const ros::Time now = ros::Time::now();
  bool shape_control_active = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    shape_control_active = shape_state_ != QUAD;
  }
  // Keep the nominal straight model through arming/takeoff so the parent's
  // activation-time CARE solve sees the known Quad geometry.  Once a hover
  // transition has been accepted, allocation and inertia follow feedback.
  if (feedback_fresh && shape_control_active)
    updateRobotModelFromBend(measured_bend);

  UnderActuatedLQIController::controlCore();

  double dt = ctrl_loop_du_;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!last_transition_update_.isZero())
      dt = clampValue((now - last_transition_update_).toSec(), 0.0, 0.1);
    last_transition_update_ = now;
  }
  smoothTransitionThrust(measured_bend, dt);
  updateShapeTransition(measured_bend, feedback_fresh, dt);
  publishPressureTargets(measured_bend, dt);
  publishStatus(measured_bend);
}

}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::QuadBiController,
                       aerial_robot_control::ControlBase);
