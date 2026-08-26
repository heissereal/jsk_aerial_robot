#include <ros/ros.h>
#include <array>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <std_msgs/Float32.h>
// #include <spinal/Thrust.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <hugmy/model/pressure_thrust_bend_model.h>

class JointModel{
public:
  JointModel() : nh_("~")
  {
    theta_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/quadrotor/debug/theta", 1);
    apply_to_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joint_states", 1);
    pressure_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/filterd_joint_cur_pressure", 1, &JointModel::pressureCb, this);
    //sim
    // pressure_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/sim_pressure", 1, &JointModel::pressureCb, this);
    thrust_sub_   = nh_.subscribe<spinal::FourAxisCommand>("/quadrotor/four_axes/command", 1, &JointModel::thrustCb, this);
    mode_sub_   = nh_.subscribe<std_msgs::Empty>("/quadrotor/arm/joint_estimate_enable", 1, &JointModel::modeCb, this);
    apply_to_joint_sim_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joints_ctrl", 1);

    // nh_.param("pressure", latest_pressure_, 0.0f); // kPa
    // nh_.param("thrust",   latest_thrust_,   f);  // N

    latest_thrust_.assign(4, 7.0f);
    theta_deg_.assign(4, 0.0f);
    theta_rad_.assign(4, 0.0f);

    theta_rad_filt_.assign(4, 0.0f);
    nh_.param("theta_lpf_tau", lpf_tau_sec_, 0.20);
    nh_.param("theta_max_slew", max_delta_rad_per_sec_, 2.0);
    last_update_ = ros::Time::now();


    // ROS_INFO_STREAM("ThrustPressureToTheta: pressure=" << latest_pressure_
    //                 << ", thrust=" << latest_thrust_.at(0)
    //                 << ", theta_out=" << theta_deg_.at(0) );
  }

  void spin(){
    ros::Rate rate(100);
    while(ros::ok()){

      ros::Time now = ros::Time::now();
      double dt = (now - last_update_).toSec();
      if (dt <= 0.0) dt =1e-3;
      last_update_ = now;
      //   nh_.getParam("pressure", latest_pressure_);
      //   nh_.getParam("thrust",   latest_thrust_);

      const double tau_p = std::max(1e-3, pressure_lpf_sec_);
      const double alpha_p = dt / (tau_p + dt);

      if (!pressure_filt_inited_) {
        latest_pressure_filt_ = latest_pressure_;
        pressure_filt_inited_ = true;
      } else {
        double prev_p = static_cast<double>(latest_pressure_filt_);
        double raw_p = static_cast<double>(latest_pressure_);
        double lpf_out_p = prev_p + alpha_p * (raw_p - prev_p);

        double max_step_p = max_pressure_per_sec_ * dt;
        double delta_p = lpf_out_p - prev_p;
        if (delta_p >  max_step_p) lpf_out_p = prev_p + max_step_p;
        if (delta_p < -max_step_p) lpf_out_p = prev_p - max_step_p;
        latest_pressure_filt_ = static_cast<float>(lpf_out_p);
      }

      double P = static_cast<double>(latest_pressure_filt_);

      auto [Pr, Tr] = model_.ranges();
      bool pressure_clamped = false;
      if (P < Pr.first)  { P = Pr.first;  pressure_clamped = true; }
      if (P > Pr.second) { P = Pr.second; pressure_clamped = true; }
      pressure_trend_ = updateTrend(P, previous_model_pressure_, pressure_trend_, 0.05);
      previous_model_pressure_ = P;

      // ROS_WARN_STREAM_THROTTLE(1.0, "latest_pressure_ :" << latest_pressure_);
      for(int i = 0; i < 4; ++i){
        double T = static_cast<double>(latest_thrust_.at(i));

        bool thrust_clamped = false;
        if (T < Tr.first)  { T = Tr.first;  thrust_clamped = true; }
        if (T > Tr.second) { T = Tr.second; thrust_clamped = true; }
        if (pressure_clamped || thrust_clamped){
          ROS_WARN_THROTTLE(1.0, "Input out of range. Clamped to P=[%.1f, %.1f], T=[%.1f, %.1f].",
                            Pr.first, Pr.second, Tr.first, Tr.second);
        }
        thrust_trend_[i] = updateTrend(T, previous_model_thrust_[i],
                                       thrust_trend_[i], 0.01);
        previous_model_thrust_[i] = T;
        const double theta_est_deg = model_.angleDeg(
            P, T, pressure_trend_, thrust_trend_[i]);
        theta_deg_[i] = static_cast<float>(theta_est_deg);
        theta_rad_[i] = static_cast<float>(theta_est_deg * (M_PI / 180.0));
      }

      const double tau = std::max(1e-3, lpf_tau_sec_);
      const double alpha = dt / (tau + dt);

      if (!filt_inited_) {
        for (int i = 0; i < 4; ++i) theta_rad_filt_[i] = theta_rad_[i];
        filt_inited_ = true;
      } else {
        for (int i = 0; i < 4; ++i) {
          double target = theta_rad_[i];
          double prev   = theta_rad_filt_[i];
          double lpf_out = prev + alpha * (target - prev);

          double max_step = max_delta_rad_per_sec_ * dt;
          double delta = lpf_out - prev;
          if (delta >  max_step) lpf_out = prev + max_step;
          if (delta < -max_step) lpf_out = prev - max_step;

          theta_rad_filt_[i] = static_cast<float>(lpf_out);
        }
      }

      applyThetaToModel(theta_rad_filt_);

      // publish
      std_msgs::Float32MultiArray msg;
      msg.data.resize(4);
      for(int i = 0; i < 4; i++) msg.data[i] = theta_deg_[i];
      theta_pub_.publish(msg);

      // ROS_INFO_STREAM_THROTTLE(0.5,
      // "[ThetaEst] P=" << P << " kPa, T=["
      // << latest_thrust_[0] << ", "
      // << latest_thrust_[1] << ", "
      // << latest_thrust_[2] << ", "
      // << latest_thrust_[3] << "] N -> theta_deg=["
      // << theta_rad_filt_[0] << ", "
      // << theta_rad_filt_[1] << ", "
      // << theta_rad_filt_[2] << ", "
      // << theta_rad_filt_[3] << "]");


      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher  theta_pub_;
  ros::Publisher apply_to_joint_pub_, apply_to_joint_sim_pub_;
  ros::Subscriber pressure_sub_, thrust_sub_, mode_sub_;
  hugmy::PressureThrustBendModel model_;
  using BendTrend = hugmy::PressureThrustBendModel::Trend;
  BendTrend pressure_trend_ = BendTrend::NEUTRAL;
  std::array<BendTrend, 4> thrust_trend_{{BendTrend::NEUTRAL, BendTrend::NEUTRAL,
                                          BendTrend::NEUTRAL, BendTrend::NEUTRAL}};
  double previous_model_pressure_ = NAN;
  std::array<double, 4> previous_model_thrust_{{NAN, NAN, NAN, NAN}};

  float latest_pressure_{0.0f};
  std::vector<float> latest_thrust_;
  std::vector<float> theta_deg_;
  std::vector<float> theta_rad_;
  bool apply_estimate_mode_ = false;


  std::vector<float> theta_rad_filt_;
  bool filt_inited_ = false;
  ros::Time last_update_;
  double lpf_tau_sec_ = 0.20; //時定数
  double max_delta_rad_per_sec_ = 0.1;

  float latest_pressure_filt_;
  bool pressure_filt_inited_ = false;
  double pressure_lpf_sec_ = 1.00; //時定数
  double max_pressure_per_sec_ = 5;

  static BendTrend updateTrend(double value, double previous,
                               BendTrend current, double epsilon)
  {
    if (!std::isfinite(previous)) return BendTrend::NEUTRAL;
    if (value > previous + epsilon) return BendTrend::INCREASING;
    if (value < previous - epsilon) return BendTrend::DECREASING;
    return current;
  }


  void pressureCb(const std_msgs::Float32::ConstPtr& msg){
    if (!msg) return;
    latest_pressure_ = msg->data;
  }
  // void thrustCb(const std_msgs::Float32::ConstPtr& msg){
  //   if (!msg) return;
  //   latest_thrust_ = msg->data;
  // }

  void thrustCb(const spinal::FourAxisCommand::ConstPtr& msg){
    if (!msg) return;
    const size_t n = std::min<size_t>(4, msg->base_thrust.size());
    for(int i = 0; i < n; i++){
      latest_thrust_[i] = std::max(0.0f, msg->base_thrust[i]) + 1.8;
    }
  }

  void modeCb(const std_msgs::Empty::ConstPtr& msg){
    if (msg && apply_estimate_mode_){
      apply_estimate_mode_ = false;
    }else if (msg && !apply_estimate_mode_){
      apply_estimate_mode_ = true;
    }else{
      return;
    }
  }


  void applyThetaToModel(const std::vector<float>& theta_rad)
  {
    // KDL::JntArray q = robot_model->getJointPositions();
    // const auto& idx = robot_model->getJointIndexMap();

    sensor_msgs::JointState js, js_sim;
    js.header.stamp = ros::Time::now();
    js.name.reserve(12);
    js.position.reserve(12);

    js_sim.header.stamp = ros::Time::now();
    js_sim.name.reserve(12);
    js_sim.position.reserve(12);
 
    auto clamp = [](double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); };
    const double lim1 = 0.8727, limN = 1.047;

    const size_t arms = std::min<size_t>(4,theta_rad.size());
    for (int id = 1; id <= arms; ++id)
    {
      double theta = static_cast<double>(theta_rad.at(id-1));
      // double theta = static_cast<double>(theta_rad.at(0));
      double d = theta / 3.0;

      std::string j1 = "joint_" + std::to_string(id) + "_1";
      std::string j2 = "joint_" + std::to_string(id) + "_2";
      std::string j3 = "joint_" + std::to_string(id) + "_3";
      std::string j4 = "joint_" + std::to_string(id) + "_4";
      std::string j5 = "joint_" + std::to_string(id) + "_5";

      double v1,v2,v3;
      if (apply_estimate_mode_){
        v1 = clamp(d,0.0,lim1);
        v2 = clamp(d,0.0,limN);
        v3 = clamp(d,0.0,limN);
      }else{
        v1 = 0.0;
        v2 = 0.0;
        v3 = 0.0;
      }
      double v4 = 0.0;
      double v5 = 0.0;
      js.name.push_back(j1); js.position.push_back(v1);
      js.name.push_back(j2); js.position.push_back(v2);
      js.name.push_back(j3); js.position.push_back(v3);
      js.name.push_back(j4); js.position.push_back(v4);
      js.name.push_back(j5); js.position.push_back(v5);

      js_sim.name.push_back(j1); js_sim.position.push_back(v1);
      js_sim.name.push_back(j2); js_sim.position.push_back(v2);
      js_sim.name.push_back(j3); js_sim.position.push_back(v3);
      }
    // robot_model->updateRobotModel(q);
    apply_to_joint_pub_.publish(js);
    apply_to_joint_sim_pub_.publish(js_sim);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_model");
  JointModel node;
  node.spin();
  return 0;
}
