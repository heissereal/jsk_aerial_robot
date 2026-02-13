// #include <hugmy/control/air_pressure_controller.h>
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

#include <std_msgs/Float32MultiArray.h>

class IntegratedControllerAll {
public:
  IntegratedControllerAll()
  : nh_(),
  // air_(nh_),
  hap_(nh_)
  
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &IntegratedControllerAll::joyCb, this);
  vad_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/vad", 1);
  arming_on_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/start", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/takeoff", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/land", 1);
  interaction_state_sub_ = nh_.subscribe<std_msgs::Int8>("/interaction/state", 1, &IntegratedControllerAll::interactionStateCallback, this);
  kill_camera_pub_ = nh_.advertise<std_msgs::Bool>("/kill_realsense", 1);
  airstop_pub_ = nh_.advertise<std_msgs::Bool>("/air/stop", 1);
  model_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/arm/joint_estimate_enable", 1);
  interaction_pub_ = nh_.advertise<std_msgs::Int8>("/interaction/state", 1);
  flight_state_sub_ = nh_.subscribe<std_msgs::UInt8>("/quadrotor/flight_state", 1, &IntegratedControllerAll::flightStateCb, this);
  
  pressure_cmd_bottom_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_bottom", 1);
  pressure_cmd_joint_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_joint", 1);

  ros::NodeHandle pnh("~");
  pnh.param("norm_control_switch", haptics_norm_mode_switch_, 1);
  pnh.param("waypoint_reached_thresh", waypoint_reached_thresh_, 0.8);
  pnh.param("base_thrust", base_thrust_, 3.0);
  pnh.param("emotion_on", emotion_switch_, false);
  hap_.setBaseThrust(base_thrust_);
  hap_.setEmotionSwitch(emotion_switch_);
}

  // control_mode = -1 stop
  // 0 normal
  // 1 approach
  // 2 perching
  // 3 perhing finish
  // 4 deperch
void spin(){
  ros::Rate rate(100);
  while (ros::ok()){
    // perching_state_ = air_.getPerchingState();
    // perching_state_msg_.data = perching_state_;
    // perching_state_pub_.publish(perching_state_msg_);
    // ROS_INFO("Perch state: %d", perching_state_);

    ROS_INFO_STREAM("control:" << control_mode_);
    hap_.setNormModeSwitch(haptics_norm_mode_switch_);

    if (control_mode_ == -1){
      stop_msg_.data = 1;
      airstop_pub_.publish(stop_msg_);
      hap_.stopAllMotors();
      publishMergedPwm();
    }else{
       stop_msg_.data = 0;
       airstop_pub_.publish(stop_msg_);
      if (control_mode_ == 1){
	// 底面20kPa   
	msg_bottom_P_.data = 20;
	pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
	flight_state_flag_ = true;
	//   air_.initializePneumatics();
	// publishMergedPwm();
      }else if (control_mode_ == 2){
	//deformation中（何もしない）
      }else if (control_mode_ == 3){
	msg_bottom_P_.data = 20;
	msg_joint_P_.data = 50;
	pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
	pressure_cmd_joint_pub_.publish(msg_joint_P_);
	hap_.controlAuto();
	publishMergedPwm();
      }else{
	stop_msg_.data = 1;
	airstop_pub_.publish(stop_msg_);
      }
    }
    if (control_mode_ == 4){
      //deperching
      std_msgs::Bool kill_msg;
      // kill_msg.data = 1;
      // kill_camera_pub_.publish(kill_msg);
      deperching();
    }

    hap_.updateRviz();
    ros::spinOnce();
    rate.sleep();
  }
}

private:
  ros::NodeHandle nh_;
  // AirPressureController air_;
  HapticsVisualizer hap_;
  ros::Publisher pwm_pub_;
  ros::Publisher vad_pub_;
  ros::Publisher arming_on_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher airstop_pub_, model_pub_;
  ros::Publisher land_pub_;
  ros::Publisher kill_camera_pub_;
  ros::Publisher interaction_pub_;
  ros::Publisher pressure_cmd_bottom_pub_;
  ros::Publisher pressure_cmd_joint_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber interaction_state_sub_;
  ros::Subscriber flight_state_sub_;
  bool flight_state_flag_ = false; // true when data==5(arming)
  sensor_msgs::Joy joy_;
  
  std_msgs::Bool stop_msg_;
  std_msgs::Int8 msg_joint_P_, msg_bottom_P_;

  bool vibrate_mode_ = false;
  bool emotion_switch_ = false;
  bool first_flag_ = true;
  int control_mode_ = 0;
  int interaction_state_ = 0;
  int haptics_norm_mode_switch_ = 0;
  double waypoint_reached_thresh_ = 0.3;
  double base_thrust_ = 3.5;

  std_msgs::UInt8 flight_state_msg_{};
  std_msgs::Float32MultiArray vad_array_;
  
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_ = *msg;
    hap_.setJoy(joy_);
    if (joy_.buttons[4] == 1){
      control_mode_ = 1; //left1
    }else if (joy_.buttons[0] == 1) {
      control_mode_ = -1; //rectangle
      stop_msg_.data = 1;
      airstop_pub_.publish(stop_msg_);
      hap_.stopAllMotors();
      ROS_INFO("Emergency stop");
    // }else if(joy_.buttons[2] == 1) {
    //   control_mode_ = 0;
    }else if(joy_.buttons[6] == 1) {
      control_mode_ = 2; //left2
    }else if(joy_.buttons[7] == 1) {
      control_mode_ = 3; //right2
    }else if(joy_.buttons[5] == 1) {
      control_mode_ = 4; //right1
    }
  }

  void flightStateCb(const std_msgs::UInt8::ConstPtr& msg){
    flight_state_msg_ = *msg;
    flight_state_flag_ = true;// (flight_state_msg_.data == 5); //arming
  }

  void interactionStateCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    interaction_state_ = msg->data;
    control_mode_ = interaction_state_;
  }

  // pwmをパーチング時以外で使うと飛べないので注意
  // keep_perching の所以外は力覚提示とマージする必要はない
  void publishMergedPwm() {
    //spinal::PwmTest air_pwm_msg = air_.getAirPwm();
    spinal::PwmTest haptics_pwm_msg = hap_.getHapticsPwm();
    // ROS_INFO("Manual haptics: merge");

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

  void deperching(){
    ROS_WARN("==============deperching==================");
    //ros::Duration(2.0).sleep();

    std_msgs::Empty e;
    ROS_WARN("Arming");
    if (first_flag_){
     model_pub_.publish(e);
     first_flag_ = false;
    }
    arming_on_pub_.publish(e);
    ros::Duration(2.0).sleep();
    // msg_bottom_P_.data = 0;
    // msg_joint_P_.data = 0;
    // pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
    // pressure_cmd_joint_pub_.publish(msg_joint_P_);
    stop_msg_.data = 1;
    airstop_pub_.publish(stop_msg_);
    //face expression
    vad_array_.data = {-0.5f, -0.3f, -1.0f};
    vad_pub_.publish(vad_array_);

    if (flight_state_flag_) {
        ros::Duration(1.0).sleep();
	stop_msg_.data = 1;
	airstop_pub_.publish(stop_msg_);
        ROS_WARN("Takeoff");
        takeoff_pub_.publish(e);
        ros::Duration(3.0).sleep();
        //face expression
        vad_array_.data = {0.0f, 0.0f, 0.0f};
        vad_pub_.publish(vad_array_);
        ros::Duration(9.0).sleep();
        land_pub_.publish(e);
	interaction_state_ = 0;
	std_msgs::Int8 interaction_msg;
	interaction_msg.data = interaction_state_;
	interaction_pub_.publish(interaction_msg); 
    } 
    // air_.setPerchingState(0);
  }

};
int main(int argc, char** argv) {
  ros::init(argc, argv, "integrated_controller_all");
  IntegratedControllerAll ctrl;
  ctrl.spin();
  return 0;
}
