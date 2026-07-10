//手順をはっきりさせる
#include <hugmy/control/haptics_controller.h>
#include <hugmy/control/haptics_visualizer.h>

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <std_msgs/Empty.h>
#include <map>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

class GuidanceManualController {
public:
  GuidanceManualController()
  : nh_(),
  // air_(nh_),
  hap_(nh_)
  
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &GuidanceManualController::joyCb, this);
  airstop_pub_ = nh_.advertise<std_msgs::Bool>("/air/stop", 1);
  reset_done_pub_ = nh_.advertise<std_msgs::Empty>("/guidance/reset_done", 1);

  pressure_cmd_bottom_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_bottom", 1);
  pressure_cmd_joint_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_joint", 1);
  
  ros::NodeHandle pnh("~");
  pnh.param("norm_control_switch", haptics_norm_mode_switch_, 0);
  pnh.param("mode_switch", mode_switch_, 2);
  pnh.param("waypoint_reached_thresh", waypoint_reached_thresh_, 0.8);
  pnh.param("base_thrust", base_thrust_, 4.0);
  pnh.param("emotion_on", emotion_switch_, false);
  pnh.param("use_lidar", lidar_flag_, true);
  hap_.setBaseThrust(base_thrust_);
  hap_.setEmotionSwitch(emotion_switch_);

  reset_done_pub_.publish(init_msg_);
  ROS_WARN("Published guidance init/reset topic.");
}


  // control_mode = -1 stop
  // 0 emergency stop
  // 1 auto
  // 2 manual (test)
  // 3 vibrate (test)
  // 4 stop only air

void spin(){
  ros::Rate rate(100);
  while (ros::ok()){
    // ROS_INFO_STREAM("control:" << control_mode_);
    hap_.setNormModeSwitch(haptics_norm_mode_switch_);
    hap_.setModeSwitch(mode_switch_);
    if (control_mode_ == 0 ){// || air_.getAirPressureJoint() >= 60 || air_.getAirPressureBottom() >= 50){
      stop_msg_.data = 1;
      airstop_pub_.publish(stop_msg_);
      hap_.stopAllMotors();
    }else if (control_mode_ == 4){
      stop_msg_.data = 1;
      airstop_pub_.publish(stop_msg_);
    } else {
      // air_.keepPerching();
      stop_msg_.data = 0;
      airstop_pub_.publish(stop_msg_);
      msg_bottom_P_.data = 20;
      msg_joint_P_.data = 30; //for test
      pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
      pressure_cmd_joint_pub_.publish(msg_joint_P_);

      if (vibrate_mode_) {
        ROS_INFO("Vibrate mode: output vibration pattern.");
        hap_.vibratePwms();
      } else {
        vibrate_mode_ = false;
	if(control_mode_ == 1){
	  hap_.controlAuto();
	}else if(control_mode_ == 3){
	  hap_.controlManual();
	// }else if(control_mode_ == 2) //not haptics
	}
      }
    }
    publishMergedPwm();
    hap_.updateRviz();
    ros::spinOnce();
    rate.sleep();
  }
}

private:
  ros::NodeHandle nh_;
  // AirPressureController air_;
  HapticsVisualizer hap_;
  ros::Publisher pwm_pub_, airstop_pub_;
  ros::Subscriber joy_sub_;
  sensor_msgs::Joy joy_;
  bool vibrate_mode_ = false;
  bool emotion_switch_ = false;
  bool prev_reset_button_ = false;
  bool lidar_flag_ = true;
  int control_mode_ = 2; // 0: STOP, 1: MANUAL, 2: AUTO
  int haptics_norm_mode_switch_ = 0;
  int mode_switch_ = 2;
  ros::Publisher pressure_cmd_bottom_pub_;
  ros::Publisher pressure_cmd_joint_pub_;
  ros::Publisher reset_done_pub_;
  std_msgs::Bool stop_msg_;
  double waypoint_reached_thresh_ = 0.3;
  double base_thrust_ = 3.5;
  std_msgs::Empty init_msg_;
  std_msgs::Int8 msg_joint_P_, msg_bottom_P_;
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_ = *msg;
    hap_.setJoy(joy_);
    bool reset_pressed = (joy_.buttons[6] == 1);
    if (joy_.buttons[1] == 1){
      control_mode_ = 0; //left-up
      hap_.stopAllMotors();
      ROS_INFO("Emergency stop");
    }else if (joy_.buttons[0] == 1) {
      vibrate_mode_ = true; //vib
      control_mode_ = 1;
    }else if(joy_.buttons[2] == 1) {
      vibrate_mode_ = false; //normal_auto
      control_mode_ = 1;
    }else if(joy_.buttons[3] == 1) {
      vibrate_mode_ = false; //only_air
      control_mode_ = 2;
    }else if(joy_.buttons[4] == 1) {
      vibrate_mode_ = false; //demo_manual_left_down
      control_mode_ = 3;
    }else if(joy_.buttons[5] == 1) {
      vibrate_mode_ = false; //exhaust_air, right_down
      control_mode_ = 4;
    }
    if (reset_pressed && !prev_reset_button_) {
       vibrate_mode_ = false;
       reset_done_pub_.publish(init_msg_);
       ROS_WARN("Published guidance init/reset topic.");
       hap_.resetNavigationState();
    }
    prev_reset_button_ = reset_pressed;
  }


  // pwmをパーチング時以外で使うと飛べないので注意
  // keep_perching の所以外は力覚提示とマージする必要はない
  void publishMergedPwm() {
    // spinal::PwmTest air_pwm_msg = air_.getAirPwm();
    spinal::PwmTest haptics_pwm_msg = hap_.getHapticsPwm();
    ROS_INFO("Manual haptics: merge");

    std::map<uint8_t, float> merged_pwm;
    for (size_t i = 0; i < haptics_pwm_msg.motor_index.size(); ++i) {
      merged_pwm[haptics_pwm_msg.motor_index[i]] = haptics_pwm_msg.pwms[i];
    }
    // for (size_t i = 0; i < air_pwm_msg.motor_index.size(); ++i) {
    //   merged_pwm[air_pwm_msg.motor_index[i]] = air_pwm_msg.pwms[i];
    // }
    spinal::PwmTest merged_msg;
    for (const auto& pair : merged_pwm) {
      merged_msg.motor_index.push_back(pair.first);
      merged_msg.pwms.push_back(pair.second);
    }
    pwm_pub_.publish(merged_msg);
  }

};
int main(int argc, char** argv) {
  ros::init(argc, argv, "guidance_manual_controller");
  GuidanceManualController ctrl;
  ctrl.spin();
  return 0;
}
