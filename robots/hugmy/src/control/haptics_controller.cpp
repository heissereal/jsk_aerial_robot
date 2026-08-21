#include <hugmy/control/haptics_controller.h>

// ボタンでリセットをかけれるようにする
// waypointのpubが来た時のタイミンリセット時に受け付けれるようにする

HapticsController::HapticsController(ros::NodeHandle& nh){
    ros::NodeHandle pnh("~");
    pnh.param("thrust_strength", thrust_strength_, 1.2);
    pnh.param("waypoint_reached_thresh", waypoint_reached_thresh_, 0.8);
    pnh.param("base_thrust", base_thrust_, 3.0);
    pnh.param("use_lidar", lidar_flag_, true);
    pnh.param("use_yaml", yaml_mode_, true);
    pnh.param("mode_switch", mode_switch_, 0);
    pnh.param("perceptual_compensation", perceptual_compensation_, true);
    pnh.param("perceptual_rho", perceptual_rho_, 1.5);
    pnh.param("perceptual_gain_max", perceptual_gain_max_, 1.5);

    if (perceptual_rho_ < 1.0) {
      ROS_WARN("~perceptual_rho must be >= 1.0; using 1.0 instead.");
      perceptual_rho_ = 1.0;
    }
    perceptual_gain_max_ = std::max(1.0, perceptual_gain_max_);
    ROS_INFO("Perceptual compensation: %s (rho=%.2f, gain_max=%.2f), behavior mode=%d",
             perceptual_compensation_ ? "ON" : "OFF",
             perceptual_rho_, perceptual_gain_max_, mode_switch_);

    pwm_haptic_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/haptic", 1);
    emotion_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/vad", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/target_marker", 1);
    alpha_pub_ = nh.advertise<spinal::PwmTest>("/motor_alpha", 1);
    thrust_pub_ = nh.advertise<spinal::Thrust>("/motor_haptics_thrust", 1);

    interaction_pub_ = nh.advertise<std_msgs::Int8>("/interaction/state", 1);

    std::string odom_topic = lidar_flag_ ? "/Odometry" : "/quadrotor/uav/cog/odom";
    odom_sub_ = nh.subscribe(odom_topic, 1, &HapticsController::odomCb, this);
    imu_sub_ = nh.subscribe("/imu", 1, &HapticsController::imuCb, this);

    wpt_sub_ = nh.subscribe("/waypoints", 1, &HapticsController::wptCb, this);

    target_x_ = 0.0;
    target_y_ = 0.0;
    pos_flag_ = true;
    output_ = 0.0;
    motor_pwms_.assign(4,0.5f);

    //yaml
    if (yaml_mode_){
      XmlRpc::XmlRpcValue wp_list;
      if (pnh.getParam("waypoints", wp_list)){
	if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray){
          ROS_ERROR("~waypoints must be an array");
	}else{
	  for (int i = 0; i < wp_list.size(); ++i){
	    if (wp_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || wp_list[i].size() != 2){
	      continue;
	    }
	    double x = static_cast<double>(wp_list[i][0]);
	    double y = static_cast<double>(wp_list[i][1]);
	    waypoints_.emplace_back(x, y);
          }
	}
      }else{
        ROS_WARN("No ~waypoints parameter, auto mode will have no targets");
      }
    }
    current_wp_idx_ = 0;
}

void HapticsController::publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms) {
    spinal::PwmTest msg;
    msg.motor_index = indices;
    msg.pwms = pwms;
    pwm_haptic_pub_.publish(msg);
    last_published_pwm_ = msg;
}

void HapticsController::imuCb(const spinal::Imu::ConstPtr& msg){
    imu_ = *msg;
}

void HapticsController::odomCb(const nav_msgs::Odometry::ConstPtr& msg){
    const geometry_msgs::Pose parent_pose = msg->pose.pose;
  
    double qx = parent_pose.orientation.x;
    double qy = parent_pose.orientation.y;
    double qz = parent_pose.orientation.z;
    double qw = parent_pose.orientation.w;

    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Vector3 offset(0.4, 0.2, -0.3);
    tf::Vector3 offset_world = m * offset;

    pose_.position.x = parent_pose.position.x + offset_world.x();
    pose_.position.y = parent_pose.position.y + offset_world.y();
    pose_.position.z = parent_pose.position.z + offset_world.z();

    pose_.orientation = parent_pose.orientation;

    euler_.x = roll;
    euler_.y = pitch;
    euler_.z = yaw;
}

void HapticsController::wptCb(const geometry_msgs::PoseArray::ConstPtr& msg){
  if (!msg->poses.empty()){
    get_wpt_flag_ = true; 
  }else{
    get_wpt_flag_ = false;
  }
  if(get_wpt_flag_){
    resetNavigationState();
    waypoints_.clear();
    for (int i = 0; i < msg->poses.size(); ++i){
      const geometry_msgs::Pose wpt_pose = msg->poses[i];
      double x = wpt_pose.position.x;
      double y = wpt_pose.position.y;
      waypoints_.emplace_back(x, y);
    }
  }
}

//thrust_strength_をわからなさに応じて変更できるようにする
double HapticsController::calThrustPower(double alpha) {
    thrust_ = std::min(8.0, base_thrust_ * haptics_thrust_gain_ * forward_gain_ * std::abs(alpha));
    // if (norm_mode_switch_ == 0){
    //   thrust_ = std::min(8.0, base_thrust_ * thrust_strength_ * forward_gain_ * std::abs(alpha));
    // }
    ROS_ERROR("base_thrust: %.2f, thrust_strength: %.2f, forward_gain: %.2f, haptics_gain: %.2f, total_thrust: %.2f",
              base_thrust_, thrust_strength_, forward_gain_, haptics_thrust_gain_, thrust_);
    double pwm = -0.000679 * thrust_ * thrust_ + 0.044878 * thrust_ + 0.5;
    return std::min(pwm, 0.76);
}

void HapticsController::controlManual() {
    ROS_INFO("Manual mode start");

    if (joy_.axes.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "Joy axes not received yet, skipping manual haptics.");
      return;
    }
    double x = -joy_.axes[0];
    double y = -joy_.axes[1];

    double deadzone = 0.05;
    x = (std::abs(x) > deadzone) ? x : 0.0;
    y = (std::abs(y) > deadzone) ? y : 0.0;

    Eigen::Vector2d target_vec(y,x);
    // ROS_ERROR("Target force: (%.2f, %.2f)", target_vec.y(), target_vec.x());
    double target_norm = target_vec.norm();

    motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
    double on_interval_default = 1.0; //0.5
    double off_interval_default = 0.8;
    outputPulse(motor_pwms_, on_interval_default, off_interval_default);
    // if (norm_mode_switch_ == 0){
    //   outputStrength(target_norm);
    //   motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
    //   double on_interval_default = 0.5;
    //   double off_interval_default = 0.5;
    //   outputPulse(motor_pwms_, on_interval_default, off_interval_default);
    // }else{
    //   motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
    //   if (norm_mode_switch_ == 1){
    //     outputPulseLengthPattern(target_norm, motor_pwms_);
    //   }else if(norm_mode_switch_ == 2){
    //     outputPulsePattern(target_norm, motor_pwms_);
    //   }
    // }
}

void HapticsController::controlAuto() {
    ROS_INFO("AUTO mode start");
    if (pos_flag_) {
      if (!waypoints_.empty()) {
	current_wp_idx_ = 0;
	target_x_ = waypoints_[0].x();
	target_y_ = waypoints_[0].y();
      } else {
	target_x_ = pose_.position.x + auto_target_x_;
        target_y_ = pose_.position.y + auto_target_y_;
      }
      
      last_pos_ = pose_.position;
      pos_flag_ = false;
      ROS_INFO("Current position: (%.2f, %.2f)", pose_.position.x, pose_.position.y);
      ROS_INFO("Target position : (%.2f, %.2f)", target_x_, target_y_);
    }
    
    Eigen::Vector2d cur_pos(pose_.position.x, pose_.position.y);
    Eigen::Vector2d tgt_pos(target_x_, target_y_);
    ROS_INFO("Current position: (%.2f, %.2f)", pose_.position.x, pose_.position.y);
    ROS_INFO("Target position : (%.2f, %.2f)", target_x_, target_y_);
    ROS_WARN_STREAM("base_thrust:"<< base_thrust_);
    ROS_WARN_STREAM("waypoint_reached_thresh_:"<< waypoint_reached_thresh_);
    Eigen::Vector2d target_vec = tgt_pos - cur_pos;
    double target_norm = target_vec.norm();
    ROS_INFO("Target vector: (%.2f, %.2f), norm: %.2f", target_vec.x(), target_vec.y(), target_norm);
    
    min_target_norm_ = std::min(min_target_norm_, target_norm);

    if (min_target_norm_ < waypoint_reached_thresh_) {
      ROS_INFO("Waypoint %d reached (%.2f, %.2f).", current_wp_idx_, target_x_, target_y_);
      if (!waypoints_.empty() && current_wp_idx_ + 1 < (int)waypoints_.size()) {
        current_wp_idx_++;
        const Eigen::Vector2d& wp = waypoints_[current_wp_idx_];

        target_x_ = wp.x();
        target_y_ = wp.y();

        first_haptics_done_ = false;
        haptics_finished_flag_ = false;
        finished_cnt_ = 0;
        min_target_norm_ = std::numeric_limits<double>::infinity();
        ROS_INFO("Switching to waypoint %d: (%.2f, %.2f)", current_wp_idx_, target_x_, target_y_);
      } else {
        ROS_ERROR("Final waypoint reached, stopping motors.");
        vibratePwms();
        finished_cnt_ += 1;
        if (finished_cnt_ > 500){
          haptics_finished_flag_ = true;
          publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
          interaction_state_ = 4;
          std_msgs::Int8 interaction_msg;
          interaction_msg.data = interaction_state_;
          interaction_pub_.publish(interaction_msg); 
        }
        return;
      }
      tgt_pos << target_x_, target_y_;
      target_vec = tgt_pos - cur_pos;
      target_norm = target_vec.norm();
    }
    
    if (!first_haptics_done_) {
      motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
      double on_interval_default = 0.8;
      double off_interval_default = 1.0;
      outputPulse(motor_pwms_, on_interval_default, off_interval_default);
      // if (norm_mode_switch_ == 0){
      //   outputStrength(target_norm);
      //   motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
      //   double on_interval_default = 0.5;
      //   double off_interval_default = 0.5;
      //   outputPulse(motor_pwms_, on_interval_default, off_interval_default);
      // }else{
      //   motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
      //   if (norm_mode_switch_ == 1){
      //     outputPulseLengthPattern(target_norm, motor_pwms_);
      //   }else if(norm_mode_switch_ == 2){
      //     outputPulsePattern(target_norm, motor_pwms_);
      //   }
      // }
      first_haptics_done_ = true;
    }

    //チェックはずっとやっていて，この状態がくるときは挙動を変えるようにする
    //もしtrue→true 出力しないまま
    // もしfalse→false　出力するまま
    // false→true　出力しなくなる
    // もしtrue→false　出力する
    //できればチェックの回数をもっと細かくして，最初の出力の挙動をもっとでかく
    // チェックはずっとしておく？　最初と，違うよーこっちだよーだけほしい
    // if ((ros::Time::now() - last_check_time_).toSec() > 5.0) {  // 5s
    //     isApproachingTarget(target_vec, target_norm);
    //     last_check_time_ = ros::Time::now();
    // }

    if (target_norm < waypoint_reached_thresh_ && haptics_finished_flag_ == false) {
        ROS_ERROR("target is close enough, stopping motors.");
        vibratePwms();
        finished_cnt_ += 1;
	emotion_msg_.data.resize(3);
	emotion_msg_.data[0] = 1.0;
	emotion_msg_.data[1] = 0.8;
	emotion_msg_.data[2] = 0.0;
	
	emotion_pub_.publish(emotion_msg_);
        if (finished_cnt_ > 300){
            haptics_finished_flag_ = true;
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        }
        return;
    }else{
        const bool wrong_dir_brake_enabled = (mode_switch_ == 2);
        const bool cooldown_enabled = (mode_switch_ != 0);
        const bool full_proposal_enabled = (mode_switch_ == 2);
        isApproachingTarget(target_vec, target_norm);
        if (wrong_dir_brake_enabled &&
            nav_state_ == NavState::WRONG_DIR &&
            (!was_wrong_dir_ || brake_phase_ != BrakePhase::DONE)) {
          ROS_WARN("Wrong direction. Warn + show correct direction with long pulse.");
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = 1.0;
          resetGuidancePattern();
          handleWrongDirection(target_vec, target_norm); //make human stop -> STUCK // TODO if human dont stop, what do next?
          // warnWrongDirectionPattern();
          // if (isArmRaised()) {
          //   outputStrength(target_norm);
          //   motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
          //   int on_interval = dotAwareOnInterval(dot_);
          //   outputPulse(motor_pwms_, on_interval);
          // }else{
          //   // 腕を下げている場合 → 振動パターン
          //   ROS_INFO("Arm is down, vibrating haptics.");
          //   vibratePwms();
          // }

        } else if (wrong_dir_brake_enabled &&
                   was_wrong_dir_ &&
                   brake_phase_ != BrakePhase::IDLE &&
                   brake_phase_ != BrakePhase::DONE) {
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = 1.0;
          handleWrongDirection(target_vec, target_norm);
        } else if (wrong_dir_brake_enabled &&
                   was_wrong_dir_ &&
                   brake_phase_ == BrakePhase::DONE) {
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = stuckAwareThrustGain();
          outputCorrectionAfterBrake(target_vec, target_norm);
        } else if (cooldown_enabled && tickHapticsCooldown()) {
          ROS_INFO("Haptics cooldown: only WRONG_DIR can interrupt.");
        } else if (guidancePatternRunning()) {
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = (full_proposal_enabled && nav_state_ == NavState::STUCK) ? stuckAwareThrustGain() : 1.0;
          if (wrong_dir_brake_enabled && was_wrong_dir_) {
            outputCorrectionAfterBrake(target_vec, target_norm);
          } else if (full_proposal_enabled && nav_state_ == NavState::STUCK) {
            outputStuckPattern(motor_pwms_);
          } else {
            outputProximityPattern(target_norm, motor_pwms_);
          }
        } else if (!wrong_dir_brake_enabled && nav_state_ == NavState::WRONG_DIR) {
          ROS_WARN("Wrong direction detected, but brake is disabled in this mode. Continuing normal guidance.");
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = 1.0;
          was_wrong_dir_ = false;
          brake_phase_ = BrakePhase::IDLE;
          motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
          outputProximityPattern(target_norm, motor_pwms_);
        } else if (nav_state_ == NavState::APPROACHING) {
          ROS_INFO("Approaching target. Outputting guidance.");
          total_thrust_c_ = base_total_thrust_c_;
          haptics_thrust_gain_ = 1.0;
          was_wrong_dir_ = false;
          brake_phase_ = BrakePhase::IDLE;
          motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
          outputProximityPattern(target_norm, motor_pwms_);
        } else { // STUCK
          ROS_WARN("Stuck. Increasing pulse length gradually.");
          if (full_proposal_enabled && !isArmRaised()) {
            total_thrust_c_ = base_total_thrust_c_;
            haptics_thrust_gain_ = 1.0;
            vibratePwms();
          }else{
            total_thrust_c_ = base_total_thrust_c_;
            haptics_thrust_gain_ = full_proposal_enabled ? stuckAwareThrustGain() : 1.0;
            if (wrong_dir_brake_enabled && was_wrong_dir_){
              outputCorrectionAfterBrake(target_vec, target_norm);
            }else if (full_proposal_enabled){
              motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
              outputStuckPattern(motor_pwms_);
            }else{
              // } else {
	      //   outputStrength(target_norm);
              motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
              outputProximityPattern(target_norm, motor_pwms_);
	      //   int on_interval = stuckAwareOnInterval();
	      //   outputPulse(motor_pwms_, on_interval);
	      // }
            }
          }
        }
        if ((ros::Time::now() - last_check_time_).toSec() > 1.0) {  // 5s
          publishEmotion(target_vec,target_norm);
          last_check_time_ = ros::Time::now();
        }
    }
}

double HapticsController::computeDirectionGain(const Eigen::Vector2d& d_body)
{
    if (!perceptual_compensation_ || d_body.squaredNorm() < 1e-12) return 1.0;

    // Paper Eq. (11)-(12): d_body.x() is the forearm-longitudinal component.
    const Eigen::Vector2d d = d_body.normalized();
    const double g_dir = std::hypot(perceptual_rho_ * d.x(), d.y());
    const double g = std::min(g_dir, perceptual_gain_max_);
    ROS_INFO_THROTTLE(1.0, "Perceptual intensity gain: %.2f", g);
    return g;
}


void HapticsController::publishEmotion(const Eigen::Vector2d& target_vec, double target_norm)
{
  Eigen::Vector2d last_vec = target_vec - Eigen::Vector2d(last_pos_.x, last_pos_.y);
  Eigen::Vector2d delta_vec = Eigen::Vector2d(pose_.position.x, pose_.position.y) - Eigen::Vector2d(last_pos_.x, last_pos_.y);


  const double eps = 1e-6;

  if (last_vec.norm() > target_norm) {
    a_ = 0.0;
    //     if (delta_vec.norm() < move_distance_threshold_) {
    //   a_ = 0.0;
    // } else {
    //   a_ = 0.5;
    // }

    if (last_vec.norm() > eps && delta_vec.norm() > eps) {
      v_ = last_vec.normalized().dot(delta_vec.normalized());
      v_ = std::max(0.0, std::min(1.0, v_));
    } else {
      v_ = 0.0;
    }
  } else {
    double normalized = std::min(1.0, std::max(0.0,(target_norm - waypoint_reached_thresh_) / 3.0));
    double min_d = -1.0;
    double max_d = 1.0;
    d_ = min_d + normalized * (max_d - min_d); 
    d_ = std::max(-1.0, std::min(1.0, d_));
    emotion_cnt_ = 0;
  }
  if (target_norm <= waypoint_reached_thresh_) {
      v_ = 0.8;
      a_ = 0.4;
      d_ = 0.0;
  }
  emotion_msg_.data.resize(3);
  emotion_msg_.data[0] = v_;
  emotion_msg_.data[1] = a_;
  emotion_msg_.data[2] = d_;

  emotion_pub_.publish(emotion_msg_);
  ROS_INFO("[HapticsController] emotion v=%.3f, a=%.3f, d=%.3f", v_, a_, d_);
}


Eigen::Vector4d HapticsController::computeAlphaFixedTotal(const Eigen::Vector2d& dir, double total_thrust_c)
{
    double cos_yaw = cos(euler_.z);
    double sin_yaw = sin(euler_.z);
    // ROS_INFO("cos_yaw: %.2f, sin_yaw: %.2f", cos_yaw, sin_yaw);
    Eigen::Vector4d alpha = Eigen::Vector4d::Zero();

    const double E = std::max(0.0, total_thrust_c);
    const double eps = 1e-9;

    double n = dir.norm();
    if (n < eps || E < eps) return alpha;
    Eigen::Vector2d d_world = dir / n;
    Eigen::Matrix<double,2,4> motor_base;
    // motor_base <<  1, -1, -1,  1,
    //      -1, -1,  1,  1;
    motor_base <<  -1, 1, 1, -1,
      1, 1,  -1,  -1;
    Eigen::Matrix2d R;
    R << cos_yaw, -sin_yaw,
         sin_yaw,  cos_yaw;
    Eigen::Matrix<double, 2, 4> M = R * motor_base;
    Eigen::Vector2d d_body = R.transpose() * d_world;

    // Paper Eq. (8): bias the physical direction toward the forearm axis.
    // The robot body x-axis is aligned with the forearm longitudinal axis.
    if (perceptual_compensation_) {
      Eigen::Vector2d compensated_body(perceptual_rho_ * d_body.x(), d_body.y());
      d_world = R * compensated_body.normalized();
    }

    int best_idx = -1;
    double best_cos = -1.0;
    forward_gain_ = computeDirectionGain(d_body);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d ui = M.col(i).normalized();
        double c = ui.dot(d_world);
        if (c > best_cos) { best_cos = c; best_idx = i; }
    }

    bool found_pair = false;
    double best_err = 1e9;
    int bi=-1, bj=-1;
    Eigen::Vector2d bsol(0,0);

    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            Eigen::Matrix2d P;
            P.col(0) = M.col(i);
            P.col(1) = M.col(j);
            double det = P.determinant();
            if (std::abs(det) < eps) continue;

            Eigen::Vector2d beta = P.inverse() * d_world;
            if (beta[0] < -1e-9 || beta[1] < -1e-9) continue;

            Eigen::Vector2d errv = P * beta - d_world;
            double err = errv.norm();
            if (err < best_err) {
                best_err = err; bi=i; bj=j; bsol = beta; found_pair = true;
            }
        }
    }

    if (found_pair) {
        double norm_b = std::sqrt(std::max(0.0, bsol.squaredNorm()));
        if (norm_b < eps) return alpha;
        alpha[bi] = (bsol[0] / norm_b) * E;
        alpha[bj] = (bsol[1] / norm_b) * E;
        for (int k=0;k<4;++k) if (alpha[k] < 0 && alpha[k] > -1e-9) alpha[k] = 0.0;
        return alpha;
    }

    Eigen::Matrix2d MMt = M * M.transpose();
    Eigen::Vector2d x = MMt.ldlt().solve(d_world);
    alpha = M.transpose() * x;

    for (int k=0;k<4;++k) if (alpha[k] < 0 && alpha[k] > -1e-9) alpha[k] = 0.0;

    double na = std::sqrt(std::max(0.0, alpha.squaredNorm()));
    if (na >= eps) alpha *= (E / na); else alpha.setZero();

    return alpha;
}


std::vector<float> HapticsController::computeMotorPwmFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c)
{
    Eigen::Vector4d alpha = computeAlphaFixedTotal(target_vec, total_thrust_c);
    spinal::Thrust thrust_msg;

    for (int i = 0; i < 4; ++i) if (alpha[i] < 0 && alpha[i] > -1e-6) alpha[i] = 0;

    for (int i = 0; i < 4; ++i) {
        motor_pwms_[i] = calThrustPower(alpha[i]);
        thrust_msg.thrust.push_back(static_cast<float>(thrust_));
    }

    // debug publish
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0,1,2,3};
    for (int i = 0; i < 4; ++i) alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    alpha_pub_.publish(alpha_msg);

    thrust_pub_.publish(thrust_msg);

    return motor_pwms_;
}

void HapticsController::vibratePwms(){
    if (vibrate_count_ > 5) {
        vibrate_toggle_ = !vibrate_toggle_;
        vibrate_count_ = 0;
    }
    if (vibrate_toggle_) {
        publishHapticsPwm({0,1,2,3}, {0.6, 0.6, 0.5, 0.5});
    } else {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.6, 0.6});
        //ros::Duration(0.5).sleep();
    }
    vibrate_count_ += 1;
    ROS_INFO("Vibration pattern completed.");
}
void HapticsController::outputPulsePattern(double target_norm, const std::vector<float>& motor_pwms){
  static const int rest_toggle_interval = 50; // method 3

    if (in_cooldown_) {
        double elapsed = (ros::Time::now() - cooldown_start_).toSec();
        if (elapsed < cooldown_duration_sec_) {
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
            ROS_INFO_THROTTLE(1.0, "In cooldown (%.2f / %.2f sec)", elapsed, cooldown_duration_sec_);
            return;
        } else {
            in_cooldown_ = false;
            pulse_count_ = 0;
            rest_count_ = 0;
            rest_toggle_ = false;
            ROS_INFO("Cooldown finished, restarting pulse pattern.");
        }
    }


    if (pulse_count_ == 0) {
        if (target_norm < 0.4) {
            pulse_target_ = 1 * 2;
        } else if (target_norm < 0.8) {
            pulse_target_ = 2 * 2;
	} else if (target_norm < 1.2) {
            pulse_target_ = 3 * 2;
        } else {
            pulse_target_ = 4 * 2;
        }
    }

    rest_count_ += 1;
    if (rest_count_ > rest_toggle_interval) {
        rest_toggle_ = !rest_toggle_;
        rest_count_ = 0;
        pulse_count_ += 1;
    }

    if (pulse_count_ < pulse_target_) {
        if (rest_toggle_) {
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        } else {
            publishHapticsPwm({0,1,2,3}, motor_pwms_);
        }
        ROS_INFO("Pulse pattern %d/%d, rest_toggle: %s",
                 pulse_count_ + 1, pulse_target_, rest_toggle_ ? "ON" : "OFF");
    } else {
        pulse_count_ = 0;
        rest_count_ = 0;
        rest_toggle_ = false;

        in_cooldown_ = true;
        cooldown_start_ = ros::Time::now();

        ROS_ERROR("Pulse pattern completed, entering cooldown (%.2f sec).",
                  cooldown_duration_sec_);
    }
}

void HapticsController::outputPulseLengthPattern(double target_norm, const std::vector<float>& motor_pwms)//Method 2
{
  static const double min_on_duration_sec = 0.10;
  static const double max_on_duration_sec = 1.10;
  static const double base_duration_sec = 0.50;

  double normalized = std::min(1.0, std::max(0.0, target_norm / 1.2));
  double on_duration_sec = min_on_duration_sec + normalized * (max_on_duration_sec - min_on_duration_sec);

  outputPulse(motor_pwms, on_duration_sec, base_duration_sec);
}

void HapticsController::outputStrength(double target_norm)//Method 1
{
  const double d_max = 0.6;
    double x = std::min(std::max(0.001, target_norm/d_max), 1.0);
    const double r_min = 2.5;
    const double r_max = 4.0;
    double rating = r_min + (r_max - r_min) * x;

    const double a = 2.5086;
    const double b = 0.9222;

    double F_des = std::exp((rating - b) / a);
    
    double raw_strength = F_des / base_thrust_;

    thrust_strength_ = std::min(1.2, std::max(0.7, raw_strength));
}

void HapticsController::outputPulse(const std::vector<float>& motor_pwms, double on_duration_sec, double off_duration_sec){

  if(tickLooseDown()) return;

    const ros::Time now = ros::Time::now();
    if (!pulse_phase_initialized_) {
      pulse_phase_initialized_ = true;
      rest_toggle_ = true;
      pulse_phase_start_sec_ = now.toSec();
    }

    const double elapsed = now.toSec() - pulse_phase_start_sec_;
    if (rest_toggle_) {
      publishHapticsPwm({0,1,2,3}, motor_pwms);
      if (elapsed >= on_duration_sec) {
        rest_toggle_ = false;
        pulse_phase_start_sec_ = now.toSec();
        pulse_count_ += 1;
        looseDownStart(motor_pwms);
      }
    } else {
      publishHapticsPwm({0,1,2,3}, {0.5,0.5,0.5,0.5});
      if (elapsed >= off_duration_sec) {
        rest_toggle_ = true;
        pulse_phase_start_sec_ = now.toSec();
      }
    }
    ROS_INFO("Pulse pattern: ON for %.2f sec, OFF for %.2f sec" ,on_duration_sec, off_duration_sec);
}

void HapticsController::outputProximityPattern(double target_norm, const std::vector<float>& motor_pwms)
{
  static const int pulses_per_cycle = 2;
  static const double on_duration_sec = 0.8; //mode_0 on duration
  // static const double d_max = 1.2;

  if (mode_switch_ != 0 && in_cooldown_) {
    double elapsed = (ros::Time::now() - cooldown_start_).toSec();
    if (elapsed < cooldown_duration_sec_) {
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
      return;
    }
    in_cooldown_ = false;
    resetGuidancePattern();
  }

  if (!guidance_pattern_active_) {
    guidance_pattern_ = GuidancePattern::APPROACH;
    guidance_pattern_active_ = true;
    guidance_off_duration_sec_ = mode_switch_ == 0 ? baseline_pause_sec_ : distanceToPauseDuration(target_norm);
    guidance_on_duration_sec_ = on_duration_sec;
    guidance_motor_pwms_ = motor_pwms;
  }

  // // direction -> pause length(continuous)
  // double x = std::min(1.0, std::max(0.0, target_norm / d_max));
  // int pause_interval = min_pause + (int)(x *(max_pause - min_pause));

  outputPulse(guidance_motor_pwms_, guidance_on_duration_sec_, guidance_off_duration_sec_);
  if (pulse_count_ >= pulses_per_cycle && rest_toggle_){
    resetGuidancePattern();
    if (mode_switch_ != 0) {
      in_cooldown_ = true;
      cooldown_start_ = ros::Time::now();
      cooldown_duration_sec_ = cooldown_short_sec_;
    }
  }
}

void HapticsController::outputStuckPattern(const std::vector<float>& motor_pwms)
{
  static const int pulses_per_cycle = 2;

  if (in_cooldown_) {
    double elapsed = (ros::Time::now() - cooldown_start_).toSec();
    if (elapsed < cooldown_duration_sec_) {
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
      return;
    }
    in_cooldown_ = false;
    resetGuidancePattern();
  }

  if (!guidance_pattern_active_) {
    guidance_pattern_ = GuidancePattern::STUCK;
    guidance_pattern_active_ = true;
    guidance_on_duration_sec_ = stuckAwareOnDuration();
    guidance_off_duration_sec_ = stuckAwareOffDuration();
    guidance_motor_pwms_ = motor_pwms;
  }

  outputPulse(guidance_motor_pwms_, guidance_on_duration_sec_, guidance_off_duration_sec_);
  if (pulse_count_ >= pulses_per_cycle && rest_toggle_) {
    resetGuidancePattern();
    in_cooldown_ = true;
    cooldown_start_ = ros::Time::now();
    cooldown_duration_sec_ = cooldown_short_sec_;
  }
}

// vibration providing warning
void HapticsController::warnWrongDirectionPattern()
{
    static int cnt = 0;
    cnt++;

    const int on1 = 10;
    const int off1 = 5;
    const int on2 = 10;
    const int off2 = 60;

    int phase = cnt % (on1 + off1 + on2 + off2);

    if (phase < on1) {
        publishHapticsPwm({0,1,2,3}, {0.6, 0.6, 0.5, 0.5});
    } else if (phase < on1 + off1) {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
    } else if (phase < on1 + off1 + on2) {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.62, 0.62});
    } else {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
    }
}

// provide brake haptics against the current motion direction
bool HapticsController::outputBrakePulse(const Eigen::Vector2d& target_vec)
{
  const ros::Time now = ros::Time::now();

  if (brake_phase_ == BrakePhase::IDLE) {
    const double eps = 1e-9;
    Eigen::Vector2d brake_dir = last_motion_vec_.norm() > eps ? -last_motion_vec_ : -target_vec;
    brake_motor_pwms_ = computeMotorPwmFixedTotal(brake_dir, total_thrust_c_);
    brake_long_motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
    brake_phase_ = BrakePhase::VIBRATION;
    brake_phase_start_time_ = now;
    brake_pulse_count_ = 0;
  }

  if (brake_phase_ == BrakePhase::VIBRATION) {
    const double elapsed = (now - brake_phase_start_time_).toSec();
    const double cycle_sec = brake_vibration_on_sec_ + brake_vibration_off_sec_;
    brake_pulse_count_ = std::min(brake_vibration_pulses_,
                                  static_cast<int>(elapsed / cycle_sec));

    if (brake_pulse_count_ >= brake_vibration_pulses_) {
      brake_phase_ = BrakePhase::PAUSE;
      brake_phase_start_time_ = now;
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
      return false;
    }

    const double phase_elapsed = elapsed - brake_pulse_count_ * cycle_sec;
    if (phase_elapsed < brake_vibration_on_sec_) {
      publishHapticsPwm({0,1,2,3}, brake_motor_pwms_);
    } else {
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
    }
    return false;
  }

  if (brake_phase_ == BrakePhase::PAUSE) {
    if ((now - brake_phase_start_time_).toSec() < brake_pause_sec_) {
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
      return false;
    }
    brake_phase_ = BrakePhase::LONG_PULSE;
    brake_phase_start_time_ = now;
  }

  if (brake_phase_ == BrakePhase::LONG_PULSE) {
    if ((now - brake_phase_start_time_).toSec() < brake_long_pulse_sec_) {
      publishHapticsPwm({0,1,2,3}, brake_long_motor_pwms_);
      return false;
    }
    brake_phase_ = BrakePhase::DONE;
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
  }

  if (brake_phase_ == BrakePhase::DONE) {
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
  }

  return brake_phase_ == BrakePhase::DONE;
}

void HapticsController::handleWrongDirection(const Eigen::Vector2d& target_vec, double target_norm)
{
  if (!isArmRaised()) {
    vibratePwms();
    return;
  }

  if (!was_wrong_dir_) {
    resetPulseTiming(true);
    in_cooldown_ = false;
    brake_phase_ = BrakePhase::IDLE;
    was_wrong_dir_ = true;
  }
  if (outputBrakePulse(target_vec)) {
    resetPulseTiming(false);
  }

}

void HapticsController::outputCorrectionAfterBrake(const Eigen::Vector2d& target_vec, double target_norm)
{
  static const int pulses_per_cycle = 3;

  if (in_cooldown_){
    double elapsed = (ros::Time::now() - cooldown_start_).toSec();
    if (elapsed < cooldown_duration_sec_) {
      publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
      ROS_INFO_THROTTLE(1.0, "In cooldown (%.2f / %.2f sec)", elapsed, cooldown_duration_sec_);
      return;
    }
    in_cooldown_ = false;
    resetGuidancePattern();
  }

  if (!guidance_pattern_active_) {
    motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
    guidance_pattern_active_ = true;
    guidance_on_duration_sec_ = dotAwareOnDuration(dot_);
    guidance_off_duration_sec_ = distanceToPauseDuration(target_norm);
    guidance_motor_pwms_ = motor_pwms_;
  }

  outputPulse(guidance_motor_pwms_, guidance_on_duration_sec_, guidance_off_duration_sec_);
  if (pulse_count_ >= pulses_per_cycle && rest_toggle_) {
    resetGuidancePattern();
    in_cooldown_ = true;
    cooldown_start_ = ros::Time::now();
    cooldown_duration_sec_ = stuckAwareCooldownDuration();
    was_wrong_dir_ = false;
    brake_phase_ = BrakePhase::IDLE;
    wrong_dir_start_time_ = ros::Time(0);
  }
}

double HapticsController::stuckAwareOnDuration()
{
    const double min_on_sec = 0.80;
    const double max_on_sec = 1.40;

    double t = std::min(1.0, std::max(0.0, stuck_time_sec_ / stuck_time_to_max_));
    return min_on_sec + t * (max_on_sec - min_on_sec);
}

double HapticsController::stuckAwareOffDuration()
{
  const double min_off_sec = 0.20;
  const double max_off_sec = 0.30;

  double t = std::min(1.0, std::max(0.0, stuck_time_sec_ / stuck_time_to_max_));
  return min_off_sec + t * (max_off_sec - min_off_sec);
}

double HapticsController::stuckAwareCooldownDuration()
{
  const double min_cooldown_sec = 2.00;
  const double max_cooldown_sec = 3.00;

  double t = std::min(1.0, std::max(0.0, stuck_time_sec_ / stuck_time_to_max_));
  return min_cooldown_sec + t * (max_cooldown_sec - min_cooldown_sec);
}

double HapticsController::stuckAwareThrustGain()
{
  double t = std::min(1.0, std::max(0.0, stuck_time_sec_ / stuck_time_to_max_));
  return 1.0 + t * (stuck_thrust_gain_max_ - 1.0);
}

double HapticsController::dotAwareOnDuration(double dot)
{
  const double min_on_duration_sec = 0.40;
  const double max_on_duration_sec = 1.40;
  double t = 1.0 - ((dot + 1.0) / 2.0);
  t = std::min(1.0, std::max(0.0, t));
  return min_on_duration_sec + t * (max_on_duration_sec - min_on_duration_sec);
}

double HapticsController::distanceToPauseDuration(double target_norm){
  if (target_norm < 1.0) return 0.2;
  else if (target_norm < 2.0) return 0.3;
  else if (target_norm < 3.0) return 0.5;
  else return 0.60;
}


//******* Human state ***************
void HapticsController::isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm)
{
    const ros::Time now = ros::Time::now();

    if (last_nav_check_time_.isZero()) {
        last_nav_check_time_ = now;
        last_nav_pos_ = pose_.position;
        stuck_time_sec_ = 0.0;
        nav_state_ = NavState::APPROACHING;
        return;
    }

    const double dt = (now - last_nav_check_time_).toSec();
    if (dt < check_dt_sec_) return;

    Eigen::Vector2d delta_vec(pose_.position.x - last_nav_pos_.x, pose_.position.y - last_nav_pos_.y);
    const double moved = delta_vec.norm();
    if (moved > 1e-9) {
        last_motion_vec_ = delta_vec;
    }

    const double eps = 1e-9;
    Eigen::Vector2d to_target = target_vec;  // tgt - cur
    if (to_target.norm() > eps && delta_vec.norm() > eps) {
        dot_ = to_target.normalized().dot(delta_vec.normalized()); // cos(angle)
        dot_ = std::max(-1.0, std::min(1.0, dot_));
    }

    Eigen::Vector2d last_cur(last_nav_pos_.x, last_nav_pos_.y);
    Eigen::Vector2d tgt(target_x_, target_y_);
    const double last_dist = (tgt - last_cur).norm();
    const double dist_delta = target_norm - last_dist;
    const bool progressed = dist_delta < -wrong_dir_distance_margin_;
    const bool moved_away = dist_delta > wrong_dir_distance_margin_;
    const bool near = target_norm < near_wrong_dir_distance_threshold_;
    NavState observed_state = nav_state_;


    if (moved < move_distance_threshold_) {
        stuck_time_sec_ += dt; //dont move
        observed_state = NavState::STUCK;
        approaching_target_flag_ = false;
    } else {
        //move
        stuck_time_sec_ = 0.0;

        if (progressed || dot_ > direction_threshold_) {
          if (near && dot_ < near_wrong_dir_dot_threshold_){
            observed_state = NavState::WRONG_DIR;
            approaching_target_flag_ = false;
          }else{
            observed_state = NavState::APPROACHING;
            approaching_target_flag_ = true;
          }
        } else if (moved_away && dot_ < wrong_dir_threshold_) {
            observed_state = NavState::WRONG_DIR;
            approaching_target_flag_ = false;
        } else if (near && dot_ < near_wrong_dir_dot_threshold_) {
            observed_state = NavState::WRONG_DIR;
            approaching_target_flag_ = false;
        } else {
            observed_state = NavState::STUCK;
            approaching_target_flag_ = false;
        }
    }

    if (observed_state == NavState::WRONG_DIR) {
        if (wrong_dir_start_time_.isZero()) {
            wrong_dir_start_time_ = now;
        }
        if ((now - wrong_dir_start_time_).toSec() >= wrong_dir_confirm_sec_) {
            nav_state_ = NavState::WRONG_DIR;
        }
    } else {
        wrong_dir_start_time_ = ros::Time(0);
        nav_state_ = observed_state;
    }

    last_nav_pos_ = pose_.position;
    last_nav_check_time_ = now;

    ROS_INFO("[NavCheck] state=%d observed=%d moved=%.3f dot=%.3f progressed=%d away=%d target_norm=%.3f last_dist=%.3f stuck=%.2f wrong_dir=%.2f/%.2f",
             (int)nav_state_, (int)observed_state, moved, dot_, progressed ? 1 : 0, moved_away ? 1 : 0, target_norm, last_dist, stuck_time_sec_,
             wrong_dir_start_time_.isZero() ? 0.0 : (now - wrong_dir_start_time_).toSec(), wrong_dir_confirm_sec_);
}


bool HapticsController::isArmRaised() {
    // angle thresh
    double roll  = imu_.angles[0];
    double pitch = imu_.angles[1];
    ROS_INFO("Checking if arm is raised with roll: %.2f, pitch: %.2f", roll,pitch);
    return (std::abs(roll) < roll_threshold_) && (std::abs(pitch) < pitch_threshold_);
}



// ********** others ************
void HapticsController::toggleSwitch() {
    rest_count_ += 1;
    if (rest_count_ > 50) {
        rest_toggle_ = !rest_toggle_;
        rest_count_ = 0;
    }
}

void HapticsController::resetPulseTiming(bool start_on) {
    rest_count_ = 0;
    pulse_phase_initialized_ = start_on;
    rest_toggle_ = start_on;
    pulse_phase_start_sec_ = ros::Time::now().toSec();
}

bool HapticsController::tickHapticsCooldown() {
    if (!in_cooldown_) return false;

    const double elapsed = (ros::Time::now() - cooldown_start_).toSec();
    if (elapsed < cooldown_duration_sec_) {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        ROS_INFO_THROTTLE(1.0, "In haptics cooldown (%.2f / %.2f sec)", elapsed, cooldown_duration_sec_);
        return true;
    }

    in_cooldown_ = false;
    resetGuidancePattern();
    return false;
}

bool HapticsController::guidancePatternRunning() const {
    return guidance_pattern_active_ || loose_down_ || pulse_count_ > 0;
}

void HapticsController::resetGuidancePattern() {
    guidance_pattern_active_ = false;
    guidance_on_duration_sec_ = 0.0;
    guidance_off_duration_sec_ = 0.0;
    guidance_motor_pwms_.assign(4, 0.5f);
    pulse_count_ = 0;
    rest_count_ = 0;
    rest_toggle_ = false;
    pulse_phase_initialized_ = false;
    pulse_phase_start_sec_ = 0.0;
    loose_down_ = false;
}

void HapticsController::stopAllMotors() {
    ROS_INFO("Stopping all motors.");
    resetPulseTiming(false);
    resetGuidancePattern();
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
}


void HapticsController::resetNavigationState() {
    ROS_WARN("Resetting haptics navigation state.");

    pos_flag_ = true;
    get_wpt_flag_ = false;

    first_haptics_done_ = false;
    haptics_finished_flag_ = false;
    approaching_target_flag_ = false;

    finished_cnt_ = 0;
    emotion_cnt_ = 0;
    pulse_count_ = 0;
    pulse_target_ = 0;
    rest_count_ = 0;
    vibrate_count_ = 0;

    rest_toggle_ = false;
    vibrate_toggle_ = false;
    in_cooldown_ = false;
    resetGuidancePattern();
    pulse_phase_start_sec_ = 0.0;
    was_wrong_dir_ = false;
    brake_phase_ = BrakePhase::IDLE;
    brake_phase_start_time_ = ros::Time(0);
    brake_motor_pwms_.assign(4, 0.5f);
    brake_long_motor_pwms_.assign(4, 0.5f);
    brake_pulse_count_ = 0;

    min_target_norm_ = std::numeric_limits<double>::infinity();
    stuck_time_sec_ = 0.0;
    dot_ = 0.0;
    forward_gain_ = 1.0;
    haptics_thrust_gain_ = 1.0;
    total_thrust_c_ = base_total_thrust_c_;
    last_motion_vec_.setZero();

    current_wp_idx_ = 0;
    target_x_ = 0.0;
    target_y_ = 0.0;

    nav_state_ = NavState::APPROACHING;

    last_check_time_ = ros::Time(0);
    last_nav_check_time_ = ros::Time(0);
    wrong_dir_start_time_ = ros::Time(0);

    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
}


void HapticsController::looseDownStart(const std::vector<float>& from_pwms)
{
  ramp_from_ = from_pwms;
  loose_down_step_ = 0;
  loose_down_ = true;
}


bool HapticsController::tickLooseDown(){
  if (!loose_down_) return false;
  loose_down_step_ += 1;
  double t = std::min(1.0, (double)loose_down_step_ / (double)all_loose_down_steps_);

  double e = 1.0 - (1.0 -t) *(1.0 - t);
  std::vector<float> pwm(4, 0.5);
  for (int i = 0; i < 4; ++i)
    pwm[i] = ramp_from_[i] + (0.5 - ramp_from_[i])*(float)e;
  publishHapticsPwm({0,1,2,3},pwm);
  if (loose_down_step_ >= all_loose_down_steps_){loose_down_ = false;}
  return true;
}
