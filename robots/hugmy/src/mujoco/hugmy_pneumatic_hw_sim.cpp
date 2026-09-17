#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include <pluginlib/class_list_macros.h>

#include <hugmy/mujoco/hugmy_pneumatic_hw_sim.h>

namespace
{
struct CylinderProjection
{
  std::array<double, 3> closest{};
  std::array<double, 3> point_to_surface{};
  double distance = 0.0;
  double azimuth = 0.0;
};

// Project a point expressed in the fixed forearm frame onto a finite cylinder
// whose longitudinal axis is local x.  Contacts used by this experiment are
// on the curved surface; clamping x also gives a stable rim projection if a
// candidate reaches beyond the dummy forearm ends.
CylinderProjection projectToCylinder(const std::array<double, 3>& point,
                                     double half_length, double radius)
{
  CylinderProjection result;
  result.closest[0] = std::max(-half_length, std::min(half_length, point[0]));
  const double radial_norm = std::hypot(point[1], point[2]);
  if (radial_norm > 1.0e-9)
    {
      result.closest[1] = radius * point[1] / radial_norm;
      result.closest[2] = radius * point[2] / radial_norm;
    }
  else
    {
      result.closest[1] = 0.0;
      result.closest[2] = radius;
    }
  for (size_t axis = 0; axis < 3; ++axis)
    result.point_to_surface[axis] = result.closest[axis] - point[axis];
  result.distance = std::sqrt(
      result.point_to_surface[0] * result.point_to_surface[0] +
      result.point_to_surface[1] * result.point_to_surface[1] +
      result.point_to_surface[2] * result.point_to_surface[2]);
  result.azimuth = std::atan2(result.closest[2], result.closest[1]);
  return result;
}
}  // namespace

namespace hugmy
{
bool HugmyPneumaticHWSim::init(const std::string& robot_namespace,
                               ros::NodeHandle model_nh,
                               mjModel* mujoco_model,
                               mjData* mujoco_data)
{
  if (!mujoco_ros_control::AerialRobotHWSim::init(
          robot_namespace, model_nh, mujoco_model, mujoco_data))
    return false;

  ros::NodeHandle pneumatic_nh(model_nh, "simulation/pneumatic");
  pneumatic_namespace_ = pneumatic_nh.getNamespace();
  pneumatic_nh.param("pressure_rate", pressure_rate_hz_, pressure_rate_hz_);
  pneumatic_nh.param("sensor_rate", sensor_rate_hz_, sensor_rate_hz_);
  pneumatic_nh.param("supply_rate", supply_rate_kpa_s_, supply_rate_kpa_s_);
  pneumatic_nh.param("exhaust_rate", exhaust_rate_kpa_s_, exhaust_rate_kpa_s_);
  pneumatic_nh.param("bottom_supply_rate", bottom_supply_rate_kpa_s_,
                     bottom_supply_rate_kpa_s_);
  pneumatic_nh.param("bottom_exhaust_rate", bottom_exhaust_rate_kpa_s_,
                     bottom_exhaust_rate_kpa_s_);
  pneumatic_nh.param("maximum_pressure", maximum_pressure_kpa_, maximum_pressure_kpa_);
  pneumatic_nh.param("initial_pressure", initial_pressure_kpa_, initial_pressure_kpa_);
  pneumatic_nh.param("leak_rate", leak_rate_per_s_, leak_rate_per_s_);
  pneumatic_nh.param("bag_stiffness", bag_stiffness_nm_rad_, bag_stiffness_nm_rad_);
  pneumatic_nh.param("structural_stiffness", structural_stiffness_nm_rad_,
                     structural_stiffness_nm_rad_);
  pneumatic_nh.param("bag_damping", bag_damping_nms_rad_, bag_damping_nms_rad_);
  pneumatic_nh.param("thrust_offset", thrust_offset_n_, thrust_offset_n_);
  pneumatic_nh.param("thrust_scale", thrust_scale_, thrust_scale_);
  pneumatic_nh.param("command_timeout", command_timeout_s_, command_timeout_s_);
  pneumatic_nh.param("pressure_noise", pressure_noise_kpa_, pressure_noise_kpa_);
  pneumatic_nh.param("imu_acceleration_noise", imu_acc_noise_ms2_, imu_acc_noise_ms2_);
  pneumatic_nh.param("imu_gyro_noise", imu_gyro_noise_rads_, imu_gyro_noise_rads_);
  pneumatic_nh.param("spawn_human", spawn_human_, spawn_human_);
  pneumatic_nh.param("human_arm_center_z", human_arm_center_z_, human_arm_center_z_);
  pneumatic_nh.param("human_arm_half_length", human_arm_half_length_m_,
                     human_arm_half_length_m_);
  pneumatic_nh.param("human_arm_radius", human_arm_radius_m_,
                     human_arm_radius_m_);
  pneumatic_nh.param("human_arm_roll", human_arm_roll_rad_,
                     human_arm_roll_rad_);
  pneumatic_nh.param("human_arm_pitch", human_arm_pitch_rad_,
                     human_arm_pitch_rad_);
  pneumatic_nh.param("human_arm_yaw", human_arm_yaw_rad_,
                     human_arm_yaw_rad_);
  pneumatic_nh.param("grip_range", grip_range_m_, grip_range_m_);
  pneumatic_nh.param("grip_stiffness", grip_stiffness_n_m_, grip_stiffness_n_m_);
  pneumatic_nh.param("grip_damping", grip_damping_ns_m_, grip_damping_ns_m_);
  pneumatic_nh.param("grip_force_per_pressure", grip_force_per_kpa_n_,
                     grip_force_per_kpa_n_);
  pneumatic_nh.param("grip_min_pressure", grip_min_pressure_kpa_,
                     grip_min_pressure_kpa_);
  pneumatic_nh.param("grip_friction_coefficient", grip_friction_coefficient_,
                     grip_friction_coefficient_);
  pneumatic_nh.param("grip_tangential_damping", grip_tangent_damping_ns_m_,
                     grip_tangent_damping_ns_m_);
  pneumatic_nh.param("grip_marker_scale", grip_marker_scale_m_n_, grip_marker_scale_m_n_);
  pneumatic_nh.param("grip_marker_rate", grip_marker_rate_hz_, grip_marker_rate_hz_);
  pneumatic_nh.param("bottom_force_per_pressure", bottom_force_per_kpa_n_,
                     bottom_force_per_kpa_n_);
  pneumatic_nh.param("bottom_lever_arm", bottom_lever_arm_m_, bottom_lever_arm_m_);
  pneumatic_nh.param("bottom_point_z", bottom_point_z_m_, bottom_point_z_m_);
  pneumatic_nh.param("bottom_contact_range", bottom_contact_range_m_,
                     bottom_contact_range_m_);
  pneumatic_nh.param("bottom_visual_base_height", bottom_visual_base_height_m_,
                     bottom_visual_base_height_m_);
  pneumatic_nh.param("bottom_visual_height_per_pressure",
                     bottom_visual_height_per_kpa_m_,
                     bottom_visual_height_per_kpa_m_);
  pneumatic_nh.param("bottom_virtual_tilt_enabled", bottom_virtual_tilt_enabled_,
                     bottom_virtual_tilt_enabled_);
  pneumatic_nh.param("bottom_tilt_reference_pressure",
                     bottom_tilt_reference_pressure_kpa_,
                     bottom_tilt_reference_pressure_kpa_);
  pneumatic_nh.param("bottom_tilt_at_reference", bottom_tilt_at_reference_rad_,
                     bottom_tilt_at_reference_rad_);
  pneumatic_nh.param("bottom_tilt_stiffness", bottom_tilt_stiffness_nm_rad_,
                     bottom_tilt_stiffness_nm_rad_);
  pneumatic_nh.param("bottom_tilt_damping", bottom_tilt_damping_nms_rad_,
                     bottom_tilt_damping_nms_rad_);
  pneumatic_nh.param("bottom_tilt_max_torque", bottom_tilt_max_torque_nm_,
                     bottom_tilt_max_torque_nm_);
  pneumatic_nh.param("bottom_tilt_reference_reset_pressure",
                     bottom_tilt_reference_reset_pressure_kpa_,
                     bottom_tilt_reference_reset_pressure_kpa_);
  pneumatic_nh.param("gym_step_mode", gym_step_mode_, gym_step_mode_);
  pneumatic_nh.param("gym_step_duration", gym_step_duration_s_,
                     gym_step_duration_s_);
  pneumatic_nh.param("gym_physics_timestep", gym_physics_timestep_s_,
                     gym_physics_timestep_s_);
  if (gym_step_mode_ && gym_physics_timestep_s_ > 0.0)
    {
      // The arm-stop solref time constant is 4 ms. Keep at least two physics
      // samples across it; a larger timestep noticeably changes contact and
      // joint-limit dynamics even though it is faster.
      const double safe_timestep = std::min(0.002, gym_physics_timestep_s_);
      if (gym_physics_timestep_s_ > safe_timestep + 1.0e-12)
        ROS_WARN("Hugmy Gym physics timestep %.6f s capped at %.6f s for arm-stop stability",
                 gym_physics_timestep_s_, safe_timestep);
      if (safe_timestep > mujoco_model_->opt.timestep)
        {
          ROS_INFO("Hugmy Gym physics timestep: %.6f -> %.6f s",
                   mujoco_model_->opt.timestep, safe_timestep);
          mujoco_model_->opt.timestep = safe_timestep;
        }
    }
  // read() observes state immediately before mj_step2 advances time. One
  // extra iteration guarantees the last observation reaches the policy time.
  gym_step_iterations_ = static_cast<size_t>(std::ceil(
      std::max(mujoco_model_->opt.timestep, gym_step_duration_s_) /
      mujoco_model_->opt.timestep)) + 1;

  pressure_kpa_.fill(initial_pressure_kpa_);
  bottom_pressure_kpa_ = initial_pressure_kpa_;
  previous_model_pressure_.fill(initial_pressure_kpa_);
  thrust_n_.fill(0.0);
  previous_model_thrust_.fill(0.0);

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const std::string id = std::to_string(arm + 1);
      rotor_site_ids_[arm] = mj_name2id(
          mujoco_model_, mjOBJ_SITE, ("rotor" + id).c_str());
      acc_sensor_ids_[arm] = mj_name2id(
          mujoco_model_, mjOBJ_SENSOR, ("neuron_acc_" + id).c_str());
      gyro_sensor_ids_[arm] = mj_name2id(
          mujoco_model_, mjOBJ_SENSOR, ("neuron_gyro_" + id).c_str());
      grip_body_ids_[arm] = mj_name2id(
          mujoco_model_, mjOBJ_BODY, ("link_" + id + "_end").c_str());
      if (rotor_site_ids_[arm] < 0 || acc_sensor_ids_[arm] < 0 ||
          gyro_sensor_ids_[arm] < 0 || grip_body_ids_[arm] < 0)
        {
          ROS_ERROR("Hugmy MuJoCo pneumatic model is missing arm %s sites/sensors", id.c_str());
          return false;
        }

      for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
        {
          const std::string joint_name =
              "joint_" + id + "_" + std::to_string(joint + 1);
          joint_ids_[arm][joint] = mj_name2id(
              mujoco_model_, mjOBJ_JOINT, joint_name.c_str());
          actuator_ids_[arm][joint] = mj_name2id(
              mujoco_model_, mjOBJ_ACTUATOR, joint_name.c_str());
          if (joint_ids_[arm][joint] < 0 || actuator_ids_[arm][joint] < 0)
            {
              ROS_ERROR("Hugmy MuJoCo pneumatic model is missing %s", joint_name.c_str());
              return false;
            }
        }
    }

  grip_target_body_id_ = mj_name2id(mujoco_model_, mjOBJ_BODY, "hugmy_human_arm");
  grip_target_geom_id_ = mj_name2id(
      mujoco_model_, mjOBJ_GEOM, "hugmy_human_arm_forearm");
  if (grip_target_body_id_ < 0 || grip_target_geom_id_ < 0)
    {
      ROS_ERROR("Hugmy MuJoCo model is missing the human-arm grip target");
      return false;
    }
  main_body_id_ = mj_name2id(mujoco_model_, mjOBJ_BODY, "main_body");
  bottom_inflatable_geom_id_ = mj_name2id(
      mujoco_model_, mjOBJ_GEOM, "hugmy_bottom_inflatable");
  if (main_body_id_ < 0)
    {
      ROS_ERROR("Hugmy MuJoCo model is missing main_body");
      return false;
    }
  const double target_z = spawn_human_ ? human_arm_center_z_ : -100.0;
  mujoco_model_->body_pos[3 * grip_target_body_id_ + 0] = 0.0;
  mujoco_model_->body_pos[3 * grip_target_body_id_ + 1] = 0.0;
  mujoco_model_->body_pos[3 * grip_target_body_id_ + 2] = target_z;
  if (mujoco_model_->geom_type[grip_target_geom_id_] != mjGEOM_CYLINDER)
    {
      ROS_ERROR("Hugmy MuJoCo grip target must be a cylinder");
      return false;
    }
  // MuJoCo stores cylinder radius and half-length in size[0] and size[1].
  mujoco_model_->geom_size[3 * grip_target_geom_id_ + 0] =
      std::max(0.005, human_arm_radius_m_);
  mujoco_model_->geom_size[3 * grip_target_geom_id_ + 1] =
      std::max(0.005, human_arm_half_length_m_);
  reloadEpisodeParameters();
  mujoco_model_->geom_rgba[4 * grip_target_geom_id_ + 3] = spawn_human_ ? 1.0 : 0.0;
  mj_forward(mujoco_model_, mujoco_data_);
  initial_qpos_.assign(mujoco_data_->qpos,
                       mujoco_data_->qpos + mujoco_model_->nq);
  initial_qvel_.assign(mujoco_data_->qvel,
                       mujoco_data_->qvel + mujoco_model_->nv);
  const mjtNum* initial_main_body_rotation =
      mujoco_data_->xmat + 9 * main_body_id_;
  std::copy(initial_main_body_rotation, initial_main_body_rotation + 9,
            bottom_reference_rotation_.begin());
  bottom_reference_initialized_ = true;
  previous_grip_qfrc_.assign(mujoco_model_->nv, 0.0);
  previous_bottom_qfrc_.assign(mujoco_model_->nv, 0.0);

  command_sub_ = model_nh.subscribe(
      "pneumatic/command", 1, &HugmyPneumaticHWSim::commandCallback, this);
  thrust_sub_ = model_nh.subscribe(
      "target_thrust", 1, &HugmyPneumaticHWSim::thrustCallback, this);
  bottom_direction_sub_ = model_nh.subscribe(
      "pneumatic/bottom_force_direction", 1,
      &HugmyPneumaticHWSim::bottomDirectionCallback, this);
  adc_pub_ = model_nh.advertise<spinal::NeuronAdcStates>("neuron/adc_states", 1);
  neuron_imu_pub_ = model_nh.advertise<spinal::NeuronImuStates>("neuron/imu_states", 1);
  grip_marker_pub_ = model_nh.advertise<visualization_msgs::MarkerArray>(
      "pneumatic/grip_force_markers", 1);
  grip_state_pub_ = model_nh.advertise<std_msgs::Float32MultiArray>(
      "pneumatic/grip_state", 1);
  bottom_pressure_pub_ = model_nh.advertise<std_msgs::Float32>(
      "pneumatic/bottom_pressure", 1);
  bottom_state_pub_ = model_nh.advertise<std_msgs::Float32MultiArray>(
      "pneumatic/bottom_state", 1);
  actuator_state_pub_ = model_nh.advertise<std_msgs::Float32MultiArray>(
      "pneumatic/actuator_state", 1);
  actuator_marker_pub_ = model_nh.advertise<visualization_msgs::MarkerArray>(
      "pneumatic/actuator_markers", 1);
  ros::NodeHandle pneumatic_io_nh(model_nh, "pneumatic");
  reset_count_pub_ = pneumatic_io_nh.advertise<std_msgs::UInt32>(
      "reset_count", 1, true);
  step_count_pub_ = pneumatic_io_nh.advertise<std_msgs::UInt32>(
      "step_count", 1, true);
  reset_server_ = pneumatic_io_nh.advertiseService(
      "reset_simulation", &HugmyPneumaticHWSim::resetCallback, this);
  step_server_ = pneumatic_io_nh.advertiseService(
      "step_simulation", &HugmyPneumaticHWSim::stepCallback, this);
  std_msgs::UInt32 reset_count;
  reset_count.data = reset_count_;
  reset_count_pub_.publish(reset_count);
  std_msgs::UInt32 step_count;
  step_count.data = step_count_;
  step_count_pub_.publish(step_count);

  if (gym_step_mode_)
    ROS_INFO("Hugmy Gym stepping enabled: %zu iterations per %.3f s action",
             gym_step_iterations_, gym_step_duration_s_);
  ROS_INFO("Loaded Hugmy MuJoCo pneumatic simulation with four articulated arms");
  return true;
}

void HugmyPneumaticHWSim::commandCallback(
    const spinal::PneumaticCommand::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_ = *msg;
  command_stamp_ = ros::Time::now();
  command_received_ = true;
}

void HugmyPneumaticHWSim::thrustCallback(const spinal::Thrust::ConstPtr& msg)
{
  if (msg->thrust.size() != ARM_COUNT) return;
  std::lock_guard<std::mutex> lock(command_mutex_);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    if (std::isfinite(msg->thrust[arm]))
      thrust_n_[arm] = std::max(0.0, static_cast<double>(msg->thrust[arm]));
}

void HugmyPneumaticHWSim::bottomDirectionCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  const double norm = std::hypot(msg->vector.x, msg->vector.y);
  if (!std::isfinite(norm) || norm < 1.0e-6) return;
  std::lock_guard<std::mutex> lock(command_mutex_);
  bottom_direction_body_ = {{msg->vector.x / norm, msg->vector.y / norm}};
}

bool HugmyPneumaticHWSim::resetCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
{
  // The callback runs in ROS's spinner thread while MuJoCo owns mjData in the
  // simulation thread. Queue the reset and apply it at the next read() point.
  reset_requested_.store(true);
  step_condition_.notify_all();
  response.success = true;
  response.message = "MuJoCo reset queued";
  return true;
}

bool HugmyPneumaticHWSim::stepCallback(
    std_srvs::Trigger::Request&,
    std_srvs::Trigger::Response& response)
{
  if (!gym_step_mode_)
    {
      response.success = false;
      response.message = "simulation/pneumatic/gym_step_mode is disabled";
      return true;
    }
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    permitted_iterations_ += gym_step_iterations_;
  }
  gym_client_connected_.store(true);
  step_condition_.notify_all();
  response.success = true;
  response.message = "one policy interval permitted";
  return true;
}

void HugmyPneumaticHWSim::reloadEpisodeParameters()
{
  ros::param::param(pneumatic_namespace_ + "/human_arm_radius",
                    human_arm_radius_m_, human_arm_radius_m_);
  ros::param::param(pneumatic_namespace_ + "/human_arm_roll",
                    human_arm_roll_rad_, human_arm_roll_rad_);
  ros::param::param(pneumatic_namespace_ + "/human_arm_pitch",
                    human_arm_pitch_rad_, human_arm_pitch_rad_);
  ros::param::param(pneumatic_namespace_ + "/human_arm_yaw",
                    human_arm_yaw_rad_, human_arm_yaw_rad_);
  ros::param::param(pneumatic_namespace_ + "/grip_friction_coefficient",
                    grip_friction_coefficient_, grip_friction_coefficient_);
  ros::param::param(pneumatic_namespace_ + "/grip_force_per_pressure",
                    grip_force_per_kpa_n_, grip_force_per_kpa_n_);
  ros::param::param(pneumatic_namespace_ + "/supply_rate",
                    supply_rate_kpa_s_, supply_rate_kpa_s_);
  ros::param::param(pneumatic_namespace_ + "/exhaust_rate",
                    exhaust_rate_kpa_s_, exhaust_rate_kpa_s_);
  ros::param::param(pneumatic_namespace_ + "/bottom_supply_rate",
                    bottom_supply_rate_kpa_s_, bottom_supply_rate_kpa_s_);
  ros::param::param(pneumatic_namespace_ + "/bottom_exhaust_rate",
                    bottom_exhaust_rate_kpa_s_, bottom_exhaust_rate_kpa_s_);
  ros::param::param(pneumatic_namespace_ + "/pressure_noise",
                    pressure_noise_kpa_, pressure_noise_kpa_);
  ros::param::param(pneumatic_namespace_ + "/imu_acceleration_noise",
                    imu_acc_noise_ms2_, imu_acc_noise_ms2_);
  ros::param::param(pneumatic_namespace_ + "/imu_gyro_noise",
                    imu_gyro_noise_rads_, imu_gyro_noise_rads_);
  ros::param::param(pneumatic_namespace_ + "/thrust_scale",
                    thrust_scale_, thrust_scale_);
  human_arm_radius_m_ = std::max(0.005, human_arm_radius_m_);
  grip_friction_coefficient_ = std::max(0.0, grip_friction_coefficient_);
  grip_force_per_kpa_n_ = std::max(0.0, grip_force_per_kpa_n_);
  thrust_scale_ = std::max(0.0, thrust_scale_);
  mujoco_model_->geom_size[3 * grip_target_geom_id_ + 0] = human_arm_radius_m_;
  const double cr = std::cos(0.5 * human_arm_roll_rad_);
  const double sr = std::sin(0.5 * human_arm_roll_rad_);
  const double cp = std::cos(0.5 * human_arm_pitch_rad_);
  const double sp = std::sin(0.5 * human_arm_pitch_rad_);
  const double cy = std::cos(0.5 * human_arm_yaw_rad_);
  const double sy = std::sin(0.5 * human_arm_yaw_rad_);
  mjtNum* target_quaternion =
      mujoco_model_->body_quat + 4 * grip_target_body_id_;
  target_quaternion[0] = cr * cp * cy + sr * sp * sy;
  target_quaternion[1] = sr * cp * cy - cr * sp * sy;
  target_quaternion[2] = cr * sp * cy + sr * cp * sy;
  target_quaternion[3] = cr * cp * sy - sr * sp * cy;
}

void HugmyPneumaticHWSim::processReset(const ros::Time& time)
{
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    permitted_iterations_ = 0;
  }
  reloadEpisodeParameters();
  std::copy(initial_qpos_.begin(), initial_qpos_.end(), mujoco_data_->qpos);
  std::copy(initial_qvel_.begin(), initial_qvel_.end(), mujoco_data_->qvel);
  std::fill(mujoco_data_->qacc,
            mujoco_data_->qacc + mujoco_model_->nv, 0.0);
  std::fill(mujoco_data_->qacc_warmstart,
            mujoco_data_->qacc_warmstart + mujoco_model_->nv, 0.0);
  std::fill(mujoco_data_->qfrc_applied,
            mujoco_data_->qfrc_applied + mujoco_model_->nv, 0.0);
  if (mujoco_model_->na > 0)
    std::fill(mujoco_data_->act,
              mujoco_data_->act + mujoco_model_->na, 0.0);
  if (mujoco_model_->nu > 0)
    std::fill(mujoco_data_->ctrl,
              mujoco_data_->ctrl + mujoco_model_->nu, 0.0);
  control_input_.assign(control_input_.size(), 0.0);
  pressure_kpa_.fill(initial_pressure_kpa_);
  bottom_pressure_kpa_ = initial_pressure_kpa_;
  thrust_n_.fill(0.0);
  previous_model_pressure_.fill(initial_pressure_kpa_);
  previous_model_thrust_.fill(0.0);
  pressure_trend_.fill(BendTrend::NEUTRAL);
  thrust_trend_.fill(BendTrend::NEUTRAL);
  previous_grip_qfrc_.assign(mujoco_model_->nv, 0.0);
  previous_bottom_qfrc_.assign(mujoco_model_->nv, 0.0);
  bottom_tilt_target_rad_ = 0.0;
  bottom_tilt_measured_rad_ = 0.0;
  bottom_virtual_torque_nm_ = 0.0;
  bottom_reference_initialized_ = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = spinal::PneumaticCommand();
    command_received_ = false;
    bottom_direction_body_ = {{1.0, 0.0}};
  }
  last_pressure_time_ = time;
  last_sensor_time_ = ros::Time(0);
  last_grip_marker_time_ = ros::Time(0);
  mj_forward(mujoco_model_, mujoco_data_);
  const mjtNum* rotation = mujoco_data_->xmat + 9 * main_body_id_;
  std::copy(rotation, rotation + 9, bottom_reference_rotation_.begin());
  bottom_reference_initialized_ = true;
  std_msgs::UInt32 reset_count;
  reset_count.data = ++reset_count_;
  reset_count_pub_.publish(reset_count);
  ROS_INFO("Hugmy MuJoCo state reset for Gym episode %u", reset_count_);
}

void HugmyPneumaticHWSim::read(const ros::Time& time,
                               const ros::Duration& period)
{
  bool policy_interval_complete = false;
  while (true)
    {
      if (reset_requested_.exchange(false)) processReset(time);
      if (!gym_step_mode_) break;
      std::unique_lock<std::mutex> lock(step_mutex_);
      if (permitted_iterations_ > 0)
        {
          --permitted_iterations_;
          policy_interval_complete = permitted_iterations_ == 0;
          break;
        }
      const bool signaled = step_condition_.wait_for(
          lock, std::chrono::milliseconds(20), [this]() {
        return permitted_iterations_ > 0 || reset_requested_.load();
      });
      if (!ros::ok()) return;
      if (!signaled && !gym_client_connected_.load())
        {
          // Before the first Gym client arrives, allow one slow bootstrap
          // iteration. controller_manager's switch service waits for update(),
          // and otherwise its single callback thread would deadlock with the
          // step service that is meant to open this gate. After the first Gym
          // step, time is strictly gated and this path is permanently disabled.
          break;
        }
      // Loop so a reset received while the gate is closed is applied before
      // waiting for the next step permission.
    }
  mujoco_ros_control::AerialRobotHWSim::read(time, period);
  if ((time - last_sensor_time_).toSec() >= 1.0 / std::max(1.0, sensor_rate_hz_))
    {
      publishSensors(time);
      last_sensor_time_ = time;
    }
  if (policy_interval_complete)
    {
      std_msgs::UInt32 step_count;
      step_count.data = ++step_count_;
      step_count_pub_.publish(step_count);
    }
}

void HugmyPneumaticHWSim::write(const ros::Time& time,
                                const ros::Duration& period)
{
  spinal::PneumaticCommand command;
  std::array<double, ARM_COUNT> thrust;
  bool command_fresh = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = command_;
    thrust = thrust_n_;
    const double age = (time - command_stamp_).toSec();
    command_fresh = command_received_ && age >= 0.0 && age <= command_timeout_s_;
  }

  // Use the force actually requested from each rotor, including PwmTest.
  // The gait drives PwmTest directly, whereas target_thrust only carries the
  // normal flight-controller request. Without this bridge the pressure spring
  // did not compensate the follower-force moment and a positive 6 N command
  // bent the arm farther, opposite to the measured pressure/thrust table.
  if (use_pwm_motor_model_)
    for (size_t arm = 0; arm < ARM_COUNT &&
                         arm < static_cast<size_t>(spinal_interface_.getMotorNum());
         ++arm)
      {
        const double pwm = std::max(motor_min_pwm_, std::min(
            motor_max_pwm_, spinal_interface_.getPwm(arm)));
        const bool low_pwm_branch = pwm < motor_neutral_pwm_;
        thrust[arm] = thrust_scale_ * forceFromPwm(pwm, low_pwm_branch);
      }

  const double pressure_period = 1.0 / std::max(1.0, pressure_rate_hz_);
  const double elapsed = (time - last_pressure_time_).toSec();
  if (elapsed >= pressure_period)
    {
      updatePressure(command, command_fresh, std::min(0.05, elapsed));
      last_pressure_time_ = time;
    }
  applyBagSprings(thrust);
  applyBottomInflatable();
  applyGripForces(time);
  if ((time - last_actuator_marker_time_).toSec() >=
      1.0 / std::max(1.0, grip_marker_rate_hz_))
    {
      publishActuatorVisualization(time, thrust);
      last_actuator_marker_time_ = time;
    }
  mujoco_ros_control::AerialRobotHWSim::write(time, period);
}

void HugmyPneumaticHWSim::publishActuatorVisualization(
    const ros::Time& time,
    const std::array<double, ARM_COUNT>& thrust)
{
  visualization_msgs::MarkerArray markers;
  std_msgs::Float32MultiArray state;
  // [P1..P4 kPa, T1..T4 N, bottom pressure kPa, direction x, direction y]
  state.data.resize(2 * ARM_COUNT + 3, 0.0f);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      state.data[arm] = static_cast<float>(pressure_kpa_[arm]);
      state.data[ARM_COUNT + arm] = static_cast<float>(thrust[arm]);
      const mjtNum* position =
          mujoco_data_->site_xpos + 3 * rotor_site_ids_[arm];
      const mjtNum* rotation =
          mujoco_data_->site_xmat + 9 * rotor_site_ids_[arm];

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = time;
      arrow.ns = "hugmy_rotor_thrust";
      arrow.id = static_cast<int>(arm);
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.scale.x = 0.008;
      arrow.scale.y = 0.014;
      arrow.scale.z = 0.018;
      arrow.color.a = 0.95;
      if (thrust[arm] >= 0.0) arrow.color.g = 1.0;
      else { arrow.color.r = 1.0; arrow.color.b = 0.3; }
      geometry_msgs::Point start, end;
      start.x = position[0]; start.y = position[1]; start.z = position[2];
      const double scale = 0.012 * thrust[arm];
      end.x = start.x + scale * rotation[2];
      end.y = start.y + scale * rotation[5];
      end.z = start.z + scale * rotation[8];
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.lifetime = ros::Duration(0.2);
      markers.markers.push_back(arrow);

      visualization_msgs::Marker text;
      text.header = arrow.header;
      text.ns = "hugmy_pressure_thrust_text";
      text.id = static_cast<int>(arm);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position = start;
      text.pose.position.z += 0.045;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.022;
      text.color.r = 0.9; text.color.g = 0.95; text.color.b = 1.0;
      text.color.a = 1.0;
      char label[80];
      std::snprintf(label, sizeof(label), "A%zu  P %.1f kPa  T %+.2f N",
                    arm + 1, pressure_kpa_[arm], thrust[arm]);
      text.text = label;
      text.lifetime = ros::Duration(0.2);
      markers.markers.push_back(text);
    }

  std::array<double, 2> bottom_direction;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    bottom_direction = bottom_direction_body_;
  }
  state.data[2 * ARM_COUNT] = static_cast<float>(bottom_pressure_kpa_);
  state.data[2 * ARM_COUNT + 1] = static_cast<float>(bottom_direction[0]);
  state.data[2 * ARM_COUNT + 2] = static_cast<float>(bottom_direction[1]);
  actuator_state_pub_.publish(state);

  visualization_msgs::Marker bottom_text;
  bottom_text.header.frame_id = "world";
  bottom_text.header.stamp = time;
  bottom_text.ns = "hugmy_bottom_state_text";
  bottom_text.id = 0;
  bottom_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  bottom_text.action = visualization_msgs::Marker::ADD;
  const mjtNum* body_position = mujoco_data_->xpos + 3 * main_body_id_;
  bottom_text.pose.position.x = body_position[0];
  bottom_text.pose.position.y = body_position[1];
  bottom_text.pose.position.z = body_position[2] + 0.11;
  bottom_text.pose.orientation.w = 1.0;
  bottom_text.scale.z = 0.025;
  bottom_text.color.r = 0.3; bottom_text.color.g = 0.8;
  bottom_text.color.b = 1.0; bottom_text.color.a = 1.0;
  char bottom_label[120];
  std::snprintf(bottom_label, sizeof(bottom_label),
      "bottom %.1f kPa  dir(%+.1f,%+.1f)  tilt %.1f deg",
      bottom_pressure_kpa_, bottom_direction[0], bottom_direction[1],
      bottom_tilt_measured_rad_ * 180.0 / M_PI);
  bottom_text.text = bottom_label;
  bottom_text.lifetime = ros::Duration(0.2);
  markers.markers.push_back(bottom_text);
  actuator_marker_pub_.publish(markers);
}

void HugmyPneumaticHWSim::updatePressure(
    const spinal::PneumaticCommand& command, bool fresh, double dt)
{
  const double pump = fresh && command.enable ? clamp01(command.pump_pwm) : 0.0;
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double supply = fresh && command.enable
          ? clamp01(command.supply_pwm[arm]) : 0.0;
      const double exhaust = fresh && command.enable
          ? clamp01(command.exhaust_pwm[arm]) : 0.0;
      const double head_fraction = std::max(
          0.0, (maximum_pressure_kpa_ - pressure_kpa_[arm]) / maximum_pressure_kpa_);
      const double pressure_dot =
          supply_rate_kpa_s_ * pump * supply * head_fraction -
          exhaust_rate_kpa_s_ * exhaust - leak_rate_per_s_ * pressure_kpa_[arm];
      pressure_kpa_[arm] = std::max(
          0.0, std::min(maximum_pressure_kpa_, pressure_kpa_[arm] + pressure_dot * dt));
    }
  const double bottom_supply = fresh && command.enable
      ? clamp01(command.bottom_supply_pwm) : 0.0;
  const double bottom_exhaust = fresh && command.enable
      ? clamp01(command.bottom_exhaust_pwm) : 0.0;
  const double bottom_head_fraction = std::max(
      0.0, (maximum_pressure_kpa_ - bottom_pressure_kpa_) /
               maximum_pressure_kpa_);
  const double bottom_pressure_dot =
      bottom_supply_rate_kpa_s_ * pump * bottom_supply * bottom_head_fraction -
      bottom_exhaust_rate_kpa_s_ * bottom_exhaust -
      leak_rate_per_s_ * bottom_pressure_kpa_;
  bottom_pressure_kpa_ = std::max(0.0, std::min(
      maximum_pressure_kpa_, bottom_pressure_kpa_ + bottom_pressure_dot * dt));
}

void HugmyPneumaticHWSim::applyBottomInflatable()
{
  for (int dof = 0; dof < mujoco_model_->nv; ++dof)
    {
      mujoco_data_->qfrc_applied[dof] -= previous_bottom_qfrc_[dof];
      previous_bottom_qfrc_[dof] = 0.0;
    }

  std::array<double, 2> direction;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    direction = bottom_direction_body_;
  }
  const double local_point[3] = {
      -bottom_lever_arm_m_ * direction[0],
      -bottom_lever_arm_m_ * direction[1], bottom_point_z_m_};
  if (bottom_inflatable_geom_id_ >= 0)
    {
      mujoco_model_->geom_pos[3 * bottom_inflatable_geom_id_ + 0] = local_point[0];
      mujoco_model_->geom_pos[3 * bottom_inflatable_geom_id_ + 1] = local_point[1];
      mujoco_model_->geom_size[3 * bottom_inflatable_geom_id_ + 2] =
          bottom_visual_base_height_m_ +
          bottom_visual_height_per_kpa_m_ * bottom_pressure_kpa_;
      mujoco_model_->geom_rgba[4 * bottom_inflatable_geom_id_ + 3] =
          std::min(0.85, 0.15 + bottom_pressure_kpa_ /
                                      std::max(1.0, maximum_pressure_kpa_));
    }
  bottom_tilt_target_rad_ = 0.0;
  bottom_tilt_measured_rad_ = 0.0;
  bottom_virtual_torque_nm_ = 0.0;
  if (!spawn_human_) return;

  const mjtNum* body_position = mujoco_data_->xpos + 3 * main_body_id_;
  const mjtNum* body_rotation = mujoco_data_->xmat + 9 * main_body_id_;
  if (!bottom_reference_initialized_ ||
      bottom_pressure_kpa_ <= bottom_tilt_reference_reset_pressure_kpa_)
    {
      std::copy(body_rotation, body_rotation + 9,
                bottom_reference_rotation_.begin());
      bottom_reference_initialized_ = true;
    }
  if (bottom_pressure_kpa_ <= 0.0) return;
  mjtNum world_point[3];
  for (size_t axis = 0; axis < 3; ++axis)
    world_point[axis] = body_position[axis] +
        body_rotation[3 * axis + 0] * local_point[0] +
        body_rotation[3 * axis + 1] * local_point[1] +
        body_rotation[3 * axis + 2] * local_point[2];

  const mjtNum* target_position = mujoco_data_->xpos + 3 * grip_target_body_id_;
  const mjtNum* target_rotation = mujoco_data_->xmat + 9 * grip_target_body_id_;
  const double delta_world[3] = {
      world_point[0] - target_position[0],
      world_point[1] - target_position[1],
      world_point[2] - target_position[2]};
  std::array<double, 3> target_local{};
  for (size_t axis = 0; axis < 3; ++axis)
    target_local[axis] = target_rotation[axis] * delta_world[0] +
                         target_rotation[3 + axis] * delta_world[1] +
                         target_rotation[6 + axis] * delta_world[2];
  const CylinderProjection bottom_projection = projectToCylinder(
      target_local, human_arm_half_length_m_, human_arm_radius_m_);
  const bool physical_contact =
      bottom_projection.distance <= bottom_contact_range_m_;
  // The bottom bladder pushes away from the curved surface.  Use the radial
  // outward direction even for small numerical penetration of the visual
  // cylinder, rather than reversing the force at its centerline.
  std::array<double, 3> normal_local{{0.0, target_local[1], target_local[2]}};
  double radial_norm = std::hypot(normal_local[1], normal_local[2]);
  if (radial_norm < 1.0e-9)
    {
      normal_local[1] = 0.0;
      normal_local[2] = 1.0;
      radial_norm = 1.0;
    }
  normal_local[1] /= radial_norm;
  normal_local[2] /= radial_norm;
  mjtNum normal_world[3] = {0.0, 0.0, 0.0};
  for (size_t axis = 0; axis < 3; ++axis)
    normal_world[axis] = target_rotation[3 * axis + 0] * normal_local[0] +
                         target_rotation[3 * axis + 1] * normal_local[1] +
                         target_rotation[3 * axis + 2] * normal_local[2];

  const double magnitude = physical_contact
      ? bottom_force_per_kpa_n_ * bottom_pressure_kpa_ : 0.0;
  const mjtNum force[3] = {
      magnitude * normal_world[0], magnitude * normal_world[1],
      magnitude * normal_world[2]};
  mjtNum torque[3] = {0.0, 0.0, 0.0};

  if (bottom_virtual_tilt_enabled_ && bottom_reference_initialized_ &&
      bottom_tilt_reference_pressure_kpa_ > 1.0e-6)
    {
      // The real bottom bladder is expected to produce approximately 15 deg
      // of body rocking at 50 kPa.  Contact/friction randomization made the
      // old eccentric-force-only model saturate at 2--3 deg, so add a virtual
      // compliant attitude actuator.  It does not translate the vehicle: it
      // only makes the pressure-to-rocking-angle relation reproducible while
      // the arm rotors remain the locomotion actuator.
      bottom_tilt_target_rad_ = bottom_tilt_at_reference_rad_ * clamp01(
          bottom_pressure_kpa_ / bottom_tilt_reference_pressure_kpa_);

      // Relative rotation expressed in the zero-pressure main-body frame.
      // xmat is row-major and maps body coordinates into world coordinates.
      double relative_rotation[9]{};
      for (size_t row = 0; row < 3; ++row)
        for (size_t column = 0; column < 3; ++column)
          for (size_t world_axis = 0; world_axis < 3; ++world_axis)
            relative_rotation[3 * row + column] +=
                bottom_reference_rotation_[3 * world_axis + row] *
                body_rotation[3 * world_axis + column];
      const double cosine_angle = std::max(-1.0, std::min(1.0,
          0.5 * (relative_rotation[0] + relative_rotation[4] +
                 relative_rotation[8] - 1.0)));
      const double rotation_angle = std::acos(cosine_angle);
      const double skew[3] = {
          relative_rotation[7] - relative_rotation[5],
          relative_rotation[2] - relative_rotation[6],
          relative_rotation[3] - relative_rotation[1]};
      double rotation_vector[3] = {
          0.5 * skew[0], 0.5 * skew[1], 0.5 * skew[2]};
      if (rotation_angle > 1.0e-6 &&
          std::abs(std::sin(rotation_angle)) > 1.0e-6)
        {
          const double scale = rotation_angle /
              (2.0 * std::sin(rotation_angle));
          for (size_t axis = 0; axis < 3; ++axis)
            rotation_vector[axis] = scale * skew[axis];
        }
      const double rocking_axis_body[3] = {
          -direction[1], direction[0], 0.0};
      bottom_tilt_measured_rad_ =
          rotation_vector[0] * rocking_axis_body[0] +
          rotation_vector[1] * rocking_axis_body[1];

      mjtNum rocking_axis_world[3] = {0.0, 0.0, 0.0};
      for (size_t world_axis = 0; world_axis < 3; ++world_axis)
        rocking_axis_world[world_axis] =
            bottom_reference_rotation_[3 * world_axis + 0] *
                rocking_axis_body[0] +
            bottom_reference_rotation_[3 * world_axis + 1] *
                rocking_axis_body[1];
      mjtNum body_velocity[6]{};
      mj_objectVelocity(mujoco_model_, mujoco_data_, mjOBJ_BODY,
                        main_body_id_, body_velocity, 0);
      const double rocking_rate =
          body_velocity[0] * rocking_axis_world[0] +
          body_velocity[1] * rocking_axis_world[1] +
          body_velocity[2] * rocking_axis_world[2];
      const double requested_torque =
          bottom_tilt_stiffness_nm_rad_ *
              (bottom_tilt_target_rad_ - bottom_tilt_measured_rad_) -
          bottom_tilt_damping_nms_rad_ * rocking_rate;
      bottom_virtual_torque_nm_ = std::max(-bottom_tilt_max_torque_nm_,
          std::min(bottom_tilt_max_torque_nm_, requested_torque));
      for (size_t axis = 0; axis < 3; ++axis)
        torque[axis] = bottom_virtual_torque_nm_ * rocking_axis_world[axis];
    }
  mj_applyFT(mujoco_model_, mujoco_data_, force, torque, world_point,
             main_body_id_, previous_bottom_qfrc_.data());
  for (int dof = 0; dof < mujoco_model_->nv; ++dof)
    mujoco_data_->qfrc_applied[dof] += previous_bottom_qfrc_[dof];
}

void HugmyPneumaticHWSim::applyBagSprings(
    const std::array<double, ARM_COUNT>& thrust)
{
  std::array<double, ARM_COUNT> target_bend_rad{};
  std::array<double, ARM_COUNT> measured_bend_rad{};
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      pressure_trend_[arm] = updateTrend(
          pressure_kpa_[arm], previous_model_pressure_[arm], pressure_trend_[arm], 1.0e-4);
      thrust_trend_[arm] = updateTrend(
          thrust[arm], previous_model_thrust_[arm], thrust_trend_[arm], 1.0e-4);
      const double total_angle = bend_model_.angleRad(
          pressure_kpa_[arm], thrust[arm] + thrust_offset_n_,
          pressure_trend_[arm], thrust_trend_[arm]);
      target_bend_rad[arm] = total_angle;
      previous_model_pressure_[arm] = pressure_kpa_[arm];
      previous_model_thrust_[arm] = thrust[arm];

      const double pressure_fraction = clamp01(pressure_kpa_[arm] / maximum_pressure_kpa_);
      const double inflatable_stiffness = bag_stiffness_nm_rad_ * pressure_fraction;
      const double total_stiffness = structural_stiffness_nm_rad_ + inflatable_stiffness;
      const double desired_joint_angle = total_angle / BAG_JOINT_COUNT;
      const int rotor_site = rotor_site_ids_[arm];
      const mjtNum* rotor_position = mujoco_data_->site_xpos + 3 * rotor_site;
      const mjtNum* rotor_rotation = mujoco_data_->site_xmat + 9 * rotor_site;
      const double thrust_force[3] = {
          rotor_rotation[2] * thrust[arm],
          rotor_rotation[5] * thrust[arm],
          rotor_rotation[8] * thrust[arm]};

      for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
        {
          const int joint_id = joint_ids_[arm][joint];
          const int qpos_address = mujoco_model_->jnt_qposadr[joint_id];
          const int dof_address = mujoco_model_->jnt_dofadr[joint_id];
          const bool has_bag = joint < BAG_JOINT_COUNT;
          double reference = 0.0;
          if (has_bag && inflatable_stiffness > 0.0 && total_stiffness > 0.0)
            {
              const mjtNum* anchor = mujoco_data_->xanchor + 3 * joint_id;
              const mjtNum* axis = mujoco_data_->xaxis + 3 * joint_id;
              const double lever[3] = {
                  rotor_position[0] - anchor[0], rotor_position[1] - anchor[1],
                  rotor_position[2] - anchor[2]};
              const double torque[3] = {
                  lever[1] * thrust_force[2] - lever[2] * thrust_force[1],
                  lever[2] * thrust_force[0] - lever[0] * thrust_force[2],
                  lever[0] * thrust_force[1] - lever[1] * thrust_force[0]};
              const double thrust_torque =
                  axis[0] * torque[0] + axis[1] * torque[1] + axis[2] * torque[2];
              reference = desired_joint_angle - thrust_torque / total_stiffness;
            }

          const double upper = joint == 0 ? 0.8727 : 1.047;
          const double spring_reference = has_bag
              ? std::max(0.0, std::min(upper, reference)) : 0.0;
          const double stiffness = has_bag ? total_stiffness : structural_stiffness_nm_rad_;
          // Use MuJoCo's passive spring and damping arrays.  They are handled
          // implicitly by the integrator and remain stable for the very light
          // cable-carrier links; applying the same terms as explicit actuator
          // effort makes their acceleration numerically stiff.
          mujoco_model_->jnt_stiffness[joint_id] = stiffness;
          mujoco_model_->qpos_spring[qpos_address] = spring_reference;
          mujoco_model_->dof_damping[dof_address] = bag_damping_nms_rad_;
          control_input_.at(actuator_ids_[arm][joint]) = 0.0;
          measured_bend_rad[arm] += mujoco_data_->qpos[qpos_address];
        }
    }
  ROS_DEBUG_THROTTLE(0.2,
      "Hugmy arm PWM model front(1,4): pressure [%.1f %.1f] kPa, force [%.2f %.2f] N, target bend [%.1f %.1f] deg, joint bend [%.1f %.1f] deg",
      pressure_kpa_[0], pressure_kpa_[3], thrust[0], thrust[3],
      target_bend_rad[0] * 180.0 / M_PI,
      target_bend_rad[3] * 180.0 / M_PI,
      measured_bend_rad[0] * 180.0 / M_PI,
      measured_bend_rad[3] * 180.0 / M_PI);
}

void HugmyPneumaticHWSim::publishSensors(const ros::Time& time)
{
  spinal::NeuronAdcStates adc_states;
  spinal::NeuronImuStates imu_states;
  adc_states.stamp = imu_states.stamp = time;
  adc_states.adcs.resize(ARM_COUNT);
  imu_states.imus.resize(ARM_COUNT);

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const double measured_pressure = std::max(
          0.0, pressure_kpa_[arm] + pressure_noise_kpa_ * normal_(random_engine_));
      auto& adc = adc_states.adcs[arm];
      adc.slave_id = static_cast<uint8_t>(arm + 1);
      adc.pressure = measured_pressure;
      adc.raw = static_cast<uint16_t>(std::max(
          0.0, std::min(4095.0, measured_pressure / maximum_pressure_kpa_ * 4095.0)));
      adc.voltage = 3.3 * static_cast<double>(adc.raw) / 4095.0;

      const int acc_address = mujoco_model_->sensor_adr[acc_sensor_ids_[arm]];
      const int gyro_address = mujoco_model_->sensor_adr[gyro_sensor_ids_[arm]];
      auto& imu = imu_states.imus[arm];
      imu.slave_id = static_cast<uint8_t>(arm + 1);
      for (size_t axis = 0; axis < 3; ++axis)
        {
          imu.acc[axis] = mujoco_data_->sensordata[acc_address + axis] +
                          imu_acc_noise_ms2_ * normal_(random_engine_);
          imu.gyro[axis] = mujoco_data_->sensordata[gyro_address + axis] +
                           imu_gyro_noise_rads_ * normal_(random_engine_);
        }
    }
  adc_pub_.publish(adc_states);
  neuron_imu_pub_.publish(imu_states);
  std_msgs::Float32 bottom_pressure;
  bottom_pressure.data = std::max(
      0.0, bottom_pressure_kpa_ +
               pressure_noise_kpa_ * normal_(random_engine_));
  bottom_pressure_pub_.publish(bottom_pressure);
  std::array<double, 2> bottom_direction;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    bottom_direction = bottom_direction_body_;
  }
  std_msgs::Float32MultiArray bottom_state;
  bottom_state.data = {
      static_cast<float>(bottom_pressure_kpa_),
      static_cast<float>(bottom_force_per_kpa_n_ * bottom_pressure_kpa_),
      static_cast<float>(-bottom_lever_arm_m_ * bottom_direction[0]),
      static_cast<float>(-bottom_lever_arm_m_ * bottom_direction[1]),
      static_cast<float>(bottom_point_z_m_),
      static_cast<float>(bottom_tilt_target_rad_),
      static_cast<float>(bottom_tilt_measured_rad_),
      static_cast<float>(bottom_virtual_torque_nm_)};
  bottom_state_pub_.publish(bottom_state);
}

void HugmyPneumaticHWSim::applyGripForces(const ros::Time& time)
{
  for (int dof = 0; dof < mujoco_model_->nv; ++dof)
    {
      mujoco_data_->qfrc_applied[dof] -= previous_grip_qfrc_[dof];
      previous_grip_qfrc_[dof] = 0.0;
    }
  if (!spawn_human_) return;

  const mjtNum* target_position = mujoco_data_->xpos + 3 * grip_target_body_id_;
  const mjtNum* target_rotation = mujoco_data_->xmat + 9 * grip_target_body_id_;
  std::array<std::array<double, 3>, ARM_COUNT> points{};
  std::array<std::array<double, 3>, ARM_COUNT> forces{};
  std::array<double, ARM_COUNT> slip_ratios{};
  std::array<double, ARM_COUNT> normal_forces{};
  std::array<double, ARM_COUNT> tangential_forces{};
  std::array<bool, ARM_COUNT> active{};
  grip_axis_coordinate_m_.fill(0.0);
  grip_azimuth_rad_.fill(0.0);
  grip_surface_gap_m_.fill(0.0);

  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      const int body = grip_body_ids_[arm];
      const mjtNum* grip_position = mujoco_data_->xpos + 3 * body;
      std::array<double, 3> local{};
      for (size_t axis = 0; axis < 3; ++axis)
        local[axis] = target_rotation[axis] * (grip_position[0] - target_position[0]) +
                      target_rotation[3 + axis] * (grip_position[1] - target_position[1]) +
                      target_rotation[6 + axis] * (grip_position[2] - target_position[2]);

      const CylinderProjection projection = projectToCylinder(
          local, human_arm_half_length_m_, human_arm_radius_m_);
      std::array<double, 3> closest = projection.closest;
      std::array<double, 3> delta_local = projection.point_to_surface;
      double delta_norm = projection.distance;
      grip_axis_coordinate_m_[arm] = projection.closest[0];
      grip_azimuth_rad_[arm] = projection.azimuth;
      grip_surface_gap_m_[arm] = projection.distance;
      if (delta_norm < 1.0e-9)
        {
          // At exact surface contact, the closest-point delta is zero.  A
          // gripping chamber pulls radially inward into the cylinder instead
          // of losing its force direction for that sample.
          delta_local = {{0.0, -closest[1], -closest[2]}};
          delta_norm = std::hypot(delta_local[1], delta_local[2]);
        }

      std::array<double, 3> contact_position{};
      std::array<double, 3> inward{};
      for (size_t axis = 0; axis < 3; ++axis)
        {
          contact_position[axis] = target_position[axis] +
              target_rotation[3 * axis + 0] * closest[0] +
              target_rotation[3 * axis + 1] * closest[1] +
              target_rotation[3 * axis + 2] * closest[2];
          inward[axis] = target_rotation[3 * axis + 0] * delta_local[0] +
                         target_rotation[3 * axis + 1] * delta_local[1] +
                         target_rotation[3 * axis + 2] * delta_local[2];
          points[arm][axis] = grip_position[axis];
        }
      const double dx = contact_position[0] - grip_position[0];
      const double dy = contact_position[1] - grip_position[1];
      const double dz = contact_position[2] - grip_position[2];
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double inward_norm = std::sqrt(inward[0] * inward[0] +
                                           inward[1] * inward[1] +
                                           inward[2] * inward[2]);
      if (distance > grip_range_m_ || inward_norm < 1e-9 ||
          pressure_kpa_[arm] < grip_min_pressure_kpa_)
        continue;
      for (double& value : inward) value /= inward_norm;

      mjtNum velocity[6];
      mj_objectVelocity(mujoco_model_, mujoco_data_, mjOBJ_BODY, body, velocity, 0);
      const double normal_speed = velocity[3] * inward[0] +
                                  velocity[4] * inward[1] +
                                  velocity[5] * inward[2];
      const double pressure_limit = grip_force_per_kpa_n_ * pressure_kpa_[arm];
      const double normal_force = std::max(0.0, std::min(
          pressure_limit,
          grip_stiffness_n_m_ * (grip_range_m_ - distance) -
              grip_damping_ns_m_ * normal_speed));
      if (normal_force <= 0.0) continue;

      std::array<double, 3> tangent{{
          velocity[3] - normal_speed * inward[0],
          velocity[4] - normal_speed * inward[1],
          velocity[5] - normal_speed * inward[2]}};
      const double tangent_speed = std::sqrt(tangent[0] * tangent[0] +
                                             tangent[1] * tangent[1] +
                                             tangent[2] * tangent[2]);
      const double requested_friction = grip_tangent_damping_ns_m_ * tangent_speed;
      const double friction_limit = grip_friction_coefficient_ * normal_force;
      const double friction = std::min(requested_friction, friction_limit);
      normal_forces[arm] = normal_force;
      tangential_forces[arm] = friction;
      for (size_t axis = 0; axis < 3; ++axis)
        forces[arm][axis] = normal_force * inward[axis] -
            (tangent_speed > 1e-9 ? friction * tangent[axis] / tangent_speed : 0.0);
      slip_ratios[arm] = friction_limit > 1e-9
          ? requested_friction / friction_limit : 0.0;
      active[arm] = true;
      const mjtNum force[3] = {forces[arm][0], forces[arm][1], forces[arm][2]};
      const mjtNum torque[3] = {0.0, 0.0, 0.0};
      mj_applyFT(mujoco_model_, mujoco_data_, force, torque, grip_position,
                 body, previous_grip_qfrc_.data());
    }
  for (int dof = 0; dof < mujoco_model_->nv; ++dof)
    mujoco_data_->qfrc_applied[dof] += previous_grip_qfrc_[dof];

  if ((time - last_grip_marker_time_).toSec() >=
      1.0 / std::max(1.0, grip_marker_rate_hz_))
    {
      publishGripMarkers(time, points, forces, slip_ratios, normal_forces,
                         tangential_forces, active);
      last_grip_marker_time_ = time;
    }
}

void HugmyPneumaticHWSim::publishGripMarkers(
    const ros::Time& time,
    const std::array<std::array<double, 3>, ARM_COUNT>& points,
    const std::array<std::array<double, 3>, ARM_COUNT>& forces,
    const std::array<double, ARM_COUNT>& slip_ratios,
    const std::array<double, ARM_COUNT>& normal_forces,
    const std::array<double, ARM_COUNT>& tangential_forces,
    const std::array<bool, ARM_COUNT>& active)
{
  visualization_msgs::MarkerArray array;
  // Layout: active[4], slip ratio[4], force magnitude [N][4], cylinder-axis
  // contact coordinate [m][4], contact azimuth [rad][4], and surface gap
  // [m][4], exact articulated joint-sum bend [rad][4], normal force [N][4],
  // and tangential force [N][4]. Keep the original first 28 values stable for
  // existing consumers.
  std_msgs::Float32MultiArray grip_state;
  grip_state.data.resize(9 * ARM_COUNT, 0.0f);
  for (size_t arm = 0; arm < ARM_COUNT; ++arm)
    {
      grip_state.data[arm] = active[arm] ? 1.0f : 0.0f;
      grip_state.data[ARM_COUNT + arm] = static_cast<float>(slip_ratios[arm]);
      grip_state.data[2 * ARM_COUNT + arm] = static_cast<float>(std::sqrt(
          forces[arm][0] * forces[arm][0] + forces[arm][1] * forces[arm][1] +
          forces[arm][2] * forces[arm][2]));
      grip_state.data[3 * ARM_COUNT + arm] =
          static_cast<float>(grip_axis_coordinate_m_[arm]);
      grip_state.data[4 * ARM_COUNT + arm] =
          static_cast<float>(grip_azimuth_rad_[arm]);
      grip_state.data[5 * ARM_COUNT + arm] =
          static_cast<float>(grip_surface_gap_m_[arm]);
      double joint_sum = 0.0;
      for (size_t joint = 0; joint < JOINT_COUNT; ++joint)
        {
          const int qpos_address = mujoco_model_->jnt_qposadr[
              joint_ids_[arm][joint]];
          joint_sum += mujoco_data_->qpos[qpos_address];
        }
      grip_state.data[6 * ARM_COUNT + arm] =
          static_cast<float>(joint_sum);
      grip_state.data[7 * ARM_COUNT + arm] =
          static_cast<float>(normal_forces[arm]);
      grip_state.data[8 * ARM_COUNT + arm] =
          static_cast<float>(tangential_forces[arm]);
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = time;
      marker.ns = "hugmy_grip_force";
      marker.id = static_cast<int>(arm);
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = active[arm]
          ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
      marker.scale.x = 0.008;
      marker.scale.y = 0.014;
      marker.scale.z = 0.018;
      marker.color.a = 0.95;
      if (slip_ratios[arm] >= 1.0) { marker.color.r = 1.0; marker.color.g = 0.1; }
      else if (slip_ratios[arm] >= 0.75) { marker.color.r = 1.0; marker.color.g = 0.75; }
      else { marker.color.g = 1.0; marker.color.b = 0.2; }
      geometry_msgs::Point start, end;
      start.x = points[arm][0]; start.y = points[arm][1]; start.z = points[arm][2];
      end.x = start.x + grip_marker_scale_m_n_ * forces[arm][0];
      end.y = start.y + grip_marker_scale_m_n_ * forces[arm][1];
      end.z = start.z + grip_marker_scale_m_n_ * forces[arm][2];
      marker.points.push_back(start);
      marker.points.push_back(end);
      marker.lifetime = ros::Duration(0.2);
      array.markers.push_back(marker);
    }
  grip_marker_pub_.publish(array);
  grip_state_pub_.publish(grip_state);
}

double HugmyPneumaticHWSim::clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

HugmyPneumaticHWSim::BendTrend HugmyPneumaticHWSim::updateTrend(
    double value, double previous, BendTrend current, double epsilon)
{
  if (value > previous + epsilon) return BendTrend::INCREASING;
  if (value < previous - epsilon) return BendTrend::DECREASING;
  return current;
}
}  // namespace hugmy

PLUGINLIB_EXPORT_CLASS(hugmy::HugmyPneumaticHWSim,
                       mujoco_ros_control::RobotHWSim)
