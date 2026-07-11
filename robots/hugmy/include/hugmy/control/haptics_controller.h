#ifndef HAPTICS_CONTROLLER_H
#define HAPTICS_CONTROLLER_H

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <spinal/Thrust.h>
#include <spinal/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <XmlRpcValue.h>
#include <std_msgs/Int8.h>
// #include <tf/transform_listener.h>

class HapticsController{
public:
    HapticsController(ros::NodeHandle& nh);
    virtual ~HapticsController() = default;
    void controlManual();
    virtual void controlAuto();
    void stopAllMotors();
    void setJoy(const sensor_msgs::Joy& msg) { joy_ = msg; };
    void setNormModeSwitch(int norm_switch) { norm_mode_switch_ = norm_switch; };
    void setModeSwitch(int mode_switch) { mode_switch_ = mode_switch; };
    void setBaseThrust(double base_thrust) { base_thrust_ = base_thrust; };
    void setEmotionSwitch(bool emotion_switch) {emotion_switch_ = emotion_switch; };
    spinal::PwmTest getHapticsPwm() const {return last_published_pwm_; }
    bool getHapticsFinished() const { return haptics_finished_flag_; }
    bool pos_flag_;
    bool get_wpt_flag_;
    bool lidar_flag_;
    bool yaml_mode_;
    void vibratePwms();
    void resetNavigationState();
    Eigen::Vector4d computeAlphaFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c);
    std::vector<float> computeMotorPwmFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c);

    double auto_target_x_= 0.5;
    double auto_target_y_= 0.5;
    std::vector<Eigen::Vector2d> waypoints_;
    int current_wp_idx_ = 0;
    double waypoint_reached_thresh_ = 0.3;
    double thrust_strength_ = 1.0;
    double base_thrust_ = 4.0;
protected:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCb(const spinal::Imu::ConstPtr& msg);
    void wptCb(const geometry_msgs::PoseArray::ConstPtr& msg);
    void publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    double calThrustPower(double alpha);
    void outputPulsePattern(double target_force_norm, const std::vector<float>& motor_pwms);
    void isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm);
    bool isArmRaised();
    void toggleSwitch();
    double computeDirectionGain(const Eigen::Vector2d& d_body);

    void outputStrength(double target_norm);
    void outputPulse(const std::vector<float>& motor_pwms, double on_duration_sec, double off_duration_sec);
    void outputPulseLengthPattern(double target_norm, const std::vector<float>& motor_pwms);

    void publishEmotion(const Eigen::Vector2d& target_vec, double target_norm);

    // tf::TransformListener tfListener_;
  
    ros::Publisher pwm_haptic_pub_;
    ros::Publisher emotion_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher alpha_pub_;
    ros::Publisher thrust_pub_;
    ros::Publisher interaction_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_, wpt_sub_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 euler_;
    geometry_msgs::Point last_pos_;
    std_msgs::Float32MultiArray emotion_msg_;
    spinal::Imu imu_;
    double target_x_, target_y_;
    double output_;
    bool rest_toggle_ = false;
    int rest_count_ = 0;
    int pulse_count_ = 0;
    int pulse_target_ = 2;
    int norm_mode_switch_ = 0;
    int mode_switch_ = 0; // 0: simple baseline, 1: guidance without brake, 2: full proposal
    bool perceptual_compensation_ = true;
    double perceptual_rho_ = 1.5;
    double perceptual_gain_max_ = 1.5;
    std::vector<float> motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
    bool first_haptics_done_ = false;
    bool haptics_finished_flag_ = false;
    bool approaching_target_flag_ = true;
    ros::Time last_check_time_;
    // double move_distance_threshold_ = 0.1;
    // double direction_threshold_ = 0.8;
    double pitch_threshold_ = 0.8;
    double roll_threshold_ = 0.8;
    int vibrate_count_ = 0;
    bool vibrate_toggle_ = true;
    int finished_cnt_ = 0;
    spinal::PwmTest last_published_pwm_;
    double thrust_ = 0.0;

    double min_target_norm_ = std::numeric_limits<double>::infinity(); 

    bool in_cooldown_ = false;
    ros::Time cooldown_start_;
    double cooldown_duration_sec_ = 1.0; 
    double forward_gain_ = 1.0;
    double haptics_thrust_gain_ = 1.0;
  
    double base_total_thrust_c_ = 1.0;
    double total_thrust_c_ = 1.0;
    double stuck_thrust_gain_max_ = 1.2;
    bool emotion_switch_ = false;
    double v_ = 0.0;
    double a_ = 0.0;
    double d_ = 0.0;


    enum class NavState { APPROACHING, WRONG_DIR, STUCK };
    enum class BrakePhase { IDLE, VIBRATION, PAUSE, LONG_PULSE, DONE };
    enum class GuidancePattern { NONE, APPROACH, STUCK };

    NavState nav_state_ = NavState::APPROACHING;
  
    ros::Time last_nav_check_time_;
    geometry_msgs::Point last_nav_pos_;
    Eigen::Vector2d last_motion_vec_ = Eigen::Vector2d::Zero();

    double check_dt_sec_ = 0.2;          // チェック周期（細かくする）
    double stuck_time_sec_ = 0.0;        // 動いてない時間の蓄積
    double stuck_time_to_max_ = 3.0;     // これ以上で最大パルスに到達
    double wrong_dir_confirm_sec_ = 0.5;
    ros::Time wrong_dir_start_time_;

    // 判定しきい値
    double move_distance_threshold_ = 0.03; // 0.2秒で3cm未満なら「動いてない」等（要調整）
    double direction_threshold_ = 0.2;      // cos閾値（0.2なら約78度以内）
    double wrong_dir_threshold_ = -0.866;   // cos閾値（150-210度程度のほぼ逆向き）
    double near_wrong_dir_dot_threshold_ = -0.2; // 近距離で横/逆方向に逃げる判定
    double wrong_dir_distance_margin_ = 0.05;
    double near_wrong_dir_distance_threshold_ = 0.15;
    double dot_ = 0.0;

    double stuckAwareOnDuration();
    double stuckAwareOffDuration();
    double stuckAwareCooldownDuration();
    double stuckAwareThrustGain();
    double dotAwareOnDuration(double dot);
    void warnWrongDirectionPattern();
    int emotion_cnt_ = 0;

    int interaction_state_;

  void looseDownStart(const std::vector<float>& from_pwms);
  bool tickLooseDown();
  void outputProximityPattern(double target_norm, const std::vector<float>& motor_pwms);
  void outputStuckPattern(const std::vector<float>& motor_pwms);
  bool outputBrakePulse(const Eigen::Vector2d& target_vec);
  void handleWrongDirection(const Eigen::Vector2d& target_vec, double target_norm);
  void outputCorrectionAfterBrake(const Eigen::Vector2d& target_vec, double target_norm);
  double distanceToPauseDuration(double target_norm);
  void resetPulseTiming(bool start_on = false);
  bool tickHapticsCooldown();
  bool guidancePatternRunning() const;
  void resetGuidancePattern();

  std::vector<float> ramp_from_;
  bool loose_down_ = false;
  int loose_down_step_ = 0;
  int all_loose_down_steps_ = 30;
  int brake_ramp_ = 2;

  double pulse_phase_start_sec_ = 0.0;
  bool pulse_phase_initialized_ = false;

  double cooldown_short_sec_ = 0.3;
  double cooldown_long_sec_ = 1.0;
  double baseline_pause_sec_ = 0.5;

  bool was_wrong_dir_ = false;
  GuidancePattern guidance_pattern_ = GuidancePattern::NONE;
  bool guidance_pattern_active_ = false;
  double guidance_on_duration_sec_ = 0.0;
  double guidance_off_duration_sec_ = 0.0;
  std::vector<float> guidance_motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
  BrakePhase brake_phase_ = BrakePhase::IDLE;
  ros::Time brake_phase_start_time_;
  std::vector<float> brake_motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
  std::vector<float> brake_long_motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
  int brake_pulse_count_ = 0;
  double brake_vibration_on_sec_ = 0.15;
  double brake_vibration_off_sec_ = 0.06;
  int brake_vibration_pulses_ = 4;
  double brake_pause_sec_ = 0.30;
  double brake_long_pulse_sec_ = 1.5;
  
};

#endif
