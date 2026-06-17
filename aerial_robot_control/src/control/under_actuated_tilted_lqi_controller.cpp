// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_control/control/under_actuated_tilted_lqi_controller.h>

using namespace aerial_robot_control;

void UnderActuatedTiltedLQIController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
  z_limit_ = pid_controllers_.at(Z).getLimitSum();
  pid_controllers_.at(Z).setLimitSum(1e6); // do not clamp the sum of PID terms for z axis
}

void UnderActuatedTiltedLQIController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  double g_norm = robot_model_->getGravity().norm();

  tf::Vector3 thrust_acc_w = target_acc_w;
  tf::Vector3 attitude_acc_w = target_acc_w;

  double buoy_acc = computeBuoyancyAcc();

  double thrust_acc_norm = thrust_acc_w.length();
  double sign = (thrust_acc_w.z() >= 0) ? 1.0 : -1.0;
  if(!allow_negative_thrust_ && sign < 0.0) {
    sign = 0.0;
  }

  double vertical_ref = std::max(std::abs(g_norm - buoy_acc), min_vertical_ref_);
  attitude_acc_w.setX(sign * attitude_acc_w.x());
  attitude_acc_w.setY(sign * attitude_acc_w.y());
  attitude_acc_w.setZ(vertical_ref);

  tf::Vector3 thrust_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse()* attitude_acc_w;

  tf::Vector3 b3_des = thrust_acc_dash;
  if(b3_des.length() > min_acc_for_attitude_) {
    b3_des.normalize();
  } else {
    b3_des = b3_des_prev_;
  }

  if(b3_des.z() < min_b3_z_) {
    b3_des.setZ(min_b3_z_);
    if(b3_des.length() > 0.0) {
      b3_des.normalize();
    } else {
      b3_des = b3_des_prev_;
    }
  }
  b3_des_prev_ = b3_des;

  target_pitch_ = atan2(b3_des.x(), b3_des.z());
  target_roll_  = atan2(-b3_des.y(), sqrt(b3_des.x() * b3_des.x() + b3_des.z() * b3_des.z()));

  if(navigator_->getForceLandingFlag()) {
    target_pitch_ = 0;
    target_roll_  = 0;
  }

  Eigen::VectorXd allocate_scales;
  if(g_norm > 1e-6) {
    allocate_scales = robot_model_->getStaticThrust() / g_norm;
  } else {
    allocate_scales = getQInv().col(0);
  }

  Eigen::VectorXd target_thrust_z_term = allocate_scales * sign * thrust_acc_norm;

  // constraint z (also I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - z_limit_;

  if(residual > 0) {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
    target_thrust_z_term *= (1 - residual / max_term);
  }

  for(int i = 0; i < motor_num_; i++) {
    target_base_thrust_.at(i) = target_thrust_z_term(i);
    pid_msg_.z.total.at(i)    = target_thrust_z_term(i);
  }

  allocateYawTerm();

  ROS_INFO_THROTTLE(
      1.0,
      "submerged_ratio: %.2f, mass: %.2f, buoy_acc: %.3f, vertical_ref: %.3f, target_acc_w: [%.3f %.3f %.3f], thrust_acc_w: [%.3f %.3f %.3f], norm: %.3f, b3: [%.3f %.3f %.3f], pitch: %.3f, roll: %.3f",
      submerged_ratio_, robot_model_->getMass(), buoy_acc, vertical_ref,
      target_acc_w.x(), target_acc_w.y(), target_acc_w.z(),
      thrust_acc_dash.x(), thrust_acc_dash.y(), thrust_acc_dash.z(), thrust_acc_norm, b3_des.x(), b3_des.y(), b3_des.z(),
      target_pitch_, target_roll_);

  ROS_INFO_THROTTLE(
                    1.0,
                    "z result: %.3f, p: %.3f, i: %.3f, d: %.3f, err_i: %.3f, target_acc_z: %.3f",
                    pid_controllers_.at(Z).result(),
                    pid_controllers_.at(Z).getPTerm(),
                    pid_controllers_.at(Z).getITerm(),
                    pid_controllers_.at(Z).getDTerm(),
                    pid_controllers_.at(Z).getErrI(),
                    target_acc_.z()
                    );
}

bool UnderActuatedTiltedLQIController::optimalGain()
{
  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd P_dash  = inertia.inverse() * P.bottomRows(3); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 9);
  for(int i = 0; i < 3; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_dash.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(6, 0, 3, 9) = -C;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );

  Eigen::VectorXd q_diagonals(9);
  q_diagonals << lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd P_trans = P.topRows(3) / robot_model_->getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  double t = ros::Time::now().toSec();
  bool use_kleinman_method = true;
  if(K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
  if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
    {
      ROS_ERROR_STREAM("error in solver of continuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i,6), -K_(i,1));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i,7), -K_(i,3));
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i,8), -K_(i,5));
    }

  return true;
}

void UnderActuatedTiltedLQIController::publishGain()
{
  UnderActuatedLQIController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void UnderActuatedTiltedLQIController::rosParamInit()
{
  UnderActuatedLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);
  getParam<double>(lqi_nh, "min_acc_for_attitude", min_acc_for_attitude_, 0.3);
  getParam<double>(lqi_nh, "min_vertical_ref", min_vertical_ref_, 1.0);
  getParam<double>(lqi_nh, "min_b3_z", min_b3_z_, 0.1);
  getParam<bool>(nh_, "allow_negative_thrust", allow_negative_thrust_, false);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedLQIController, aerial_robot_control::ControlBase);
