#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sys/select.h>
#include <unistd.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

struct Trial {
  int dir_idx;      // 0..7
  double thrust;    // 6段階の強度
};

std::vector<Trial> trials;
size_t trial_idx = 0;
ros::Time trial_start_time;

Eigen::Vector4d computeAlphaFixedTotal(const Eigen::Vector2d& dir, double total_thrust_c)
{
    Eigen::Vector4d alpha = Eigen::Vector4d::Zero();

    const double E = std::max(0.0, total_thrust_c);
    const double eps = 1e-9;

    double n = dir.norm();
    if (n < eps || E < eps) return alpha;
    Eigen::Vector2d d = dir / n;

    Eigen::Matrix<double,2,4> M;
    M <<  1, -1, -1,  1,
         -1, -1,  1,  1;

    int best_idx = -1;
    double best_cos = -1.0;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d ui = M.col(i).normalized();
        double c = ui.dot(d);
        if (c > best_cos) { best_cos = c; best_idx = i; }
    }

    const double single_thr = 0.999;
    if (best_idx >= 0 && best_cos > single_thr) {
        alpha[best_idx] = E;
        return alpha;
    }

    bool found_pair = false;
    double best_err = 1e9;
    int bi=-1, bj=-1; Eigen::Vector2d bsol(0,0);


    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            Eigen::Matrix2d P;
            P.col(0) = M.col(i);
            P.col(1) = M.col(j);
            double det = P.determinant();
            if (std::abs(det) < eps) continue;

            Eigen::Vector2d beta = P.inverse() * d;
            if (beta[0] < -1e-9 || beta[1] < -1e-9) continue;

            Eigen::Vector2d errv = P * beta - d;
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
    Eigen::Vector2d x = MMt.ldlt().solve(d);
    alpha = M.transpose() * x;

    for (int k=0;k<4;++k) if (alpha[k] < 0 && alpha[k] > -1e-9) alpha[k] = 0.0;

    double na = std::sqrt(std::max(0.0, alpha.squaredNorm()));
    if (na >= eps) alpha *= (E / na); else alpha.setZero();

    return alpha;
}


double calThrustPower(double alpha, double thrust_strength)
{
    double thrust = thrust_strength * std::abs(alpha);
    double pwm = -0.000679 * thrust * thrust + 0.044878 * thrust + 0.5;
    if (pwm > 0.75) pwm = 0.75;
    if (pwm < 0.5) pwm = 0.5;
    return pwm;
}

bool stdinHasData()
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return (ret > 0) && FD_ISSET(STDIN_FILENO, &fds);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_haptics_threshold_node");
    ros::NodeHandle nh("~");

    ros::Publisher pwm_pub = nh.advertise<spinal::PwmTest>("/pwm_test", 1);
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/haptics_thrust", 1);
    ros::Publisher dir_pub = nh.advertise<std_msgs::Int8>("/haptics_direction", 1);
    ros::Publisher alpha_pub = nh.advertise<std_msgs::Float32MultiArray>("/alpha", 1);

    double min_total_thrust, max_total_thrust;
    double inter_trial_interval;
    int    rate_hz;
    std::string log_file;

    bool stim_on = true;
    ros::Time toggle_time = ros::Time::now();
    bool paused = false;
    bool shutdown = false;
    double total_thrust_ = 1.0;

    bool test_mode = false;
    int  manual_dir_idx = 0;
    double manual_total_thrust = 0.0;

    nh.param("total_thrust",       total_thrust_,        1.0);
    nh.param("min_total_thrust",      min_total_thrust,      1.0);
    nh.param("max_total_thrust",      max_total_thrust,      2.0);
    nh.param("inter_trial_interval",  inter_trial_interval,  2.0);
    nh.param("rate_hz",               rate_hz,               50);
    nh.param("log_file",              log_file,              std::string("haptics_threshold_log.csv"));


    std::ofstream ofs(log_file.c_str(), std::ios::app);
    if (!ofs) {
        ROS_WARN("Failed to open log file: %s", log_file.c_str());
    } else {
        ofs << "#dir_idx,angle_deg,total_thrust,response(0/1)\n";
    }

    std::mt19937 rng(static_cast<unsigned int>(ros::Time::now().toNSec() & 0xffffffff));
    // // std::uniform_int_distribution<int> dir_dist(0, 7); // 8方向
    // // int min_thre = static_cast<int>(min_total_thrust * 10);
    // // int max_thre = static_cast<int>(max_total_thrust * 10);
    // // std::uniform_int_distribution<int> thrust_dist(min_thre, max_thre);
    
    // // std::vector<double> thrust_levels;
    // // for (double t = min_total_thrust; t <= max_total_thrust + 1e-6; t += 0.1)
    // //   thrust_levels.push_back(std::round(t * 10.0) / 10.0);

    // // std::uniform_int_distribution<int> thrust_dist(0, thrust_levels.size() - 1);
    // std::vector<int> dir_indices = {0,1,2,3,4,5,6,7};

    // // std::vector<int> thrust_set_even = {8,16,24,32,40};
    // // std::vector<int> thrust_set_odd  = {8,12,16,20,24};
    // std::vector<int> thrust_set_even  = {100,125,150,175,200};
    // std::vector<int> thrust_set_odd  = {200,225,250,275,300};

    // const int LEVELS = 5;
    // std::vector<double> thrust_levels;
    // thrust_levels.reserve(LEVELS);
    // if (max_total_thrust < min_total_thrust) std::swap(max_total_thrust, min_total_thrust);
    // for (int i = 0; i < LEVELS; ++i) {
    //   double v = (LEVELS == 1)
    //     ? min_total_thrust
    //     : min_total_thrust + (max_total_thrust - min_total_thrust) * (double(i) / double(LEVELS - 1));
    //   v = std::round(v * 1000.0) / 1000.0;
    //   thrust_levels.push_back(v);
    // }

    std::vector<double> thrust_levels = {1.0, 1.5, 2.0, 3.5, 4.0};

    // std::vector<double> thrust_levels = {2.25, 2.5, 2.75, 3.0, 3.25};

    trials.clear();
    // trials.reserve(8 * LEVELS);
    trials.reserve(8*thrust_levels.size());
    for (int d = 0; d < 8; ++d) {
      for (double t : thrust_levels) {
        trials.push_back(Trial{d, t});
      }
    }

    std::shuffle(trials.begin(), trials.end(), rng);
    trial_idx = 0;
    ROS_INFO("Prepared %zu randomized unique trials.", trials.size());

    for (size_t i=0;i<trials.size();++i) {
      ROS_INFO("trial[%zu]: dir=%d thrust=%.3f", i, trials[i].dir_idx, trials[i].thrust);
    }

    int current_dir_idx_ = 0;
    int repeat_in_dir_ = 0;
    bool use_even_set = true;

    std::vector<int> thrust_seq(5);

    enum State { INTER_TRIAL, STIMULUS };
    State state = INTER_TRIAL;
    ros::Time state_start = ros::Time::now();

    Eigen::Vector2d current_dir(1.0, 0.0);
    double current_total_thrust = 0.0;

    ros::Rate rate(rate_hz);

    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();

        if (state == INTER_TRIAL) {
            spinal::PwmTest neutral;
            neutral.motor_index = {0,1,2,3};
            neutral.pwms = {0.5,0.5,0.5,0.5};
            pwm_pub.publish(neutral);
 
            if ((now - state_start).toSec() >= inter_trial_interval) {
              if (trial_idx >= trials.size()) {
                ROS_INFO("All %zu trials completed.", trials.size());
                break;
              }

              const Trial& tr = trials[trial_idx];
              current_dir_idx_      = tr.dir_idx;
              current_total_thrust  = tr.thrust;

              double angle = current_dir_idx_ * M_PI / 4.0; // [rad]
              current_dir = Eigen::Vector2d(std::cos(angle), std::sin(angle));

              std_msgs::Int8 dir_msg;
              dir_msg.data = static_cast<int8_t>(current_dir_idx_);
              dir_pub.publish(dir_msg);

              std_msgs::Float32 thrust_msg;
              thrust_msg.data = static_cast<float>(current_total_thrust);
              thrust_pub.publish(thrust_msg);

              trial_start_time = now;
              stim_on = true;
              toggle_time = now;

              state = STIMULUS;
              state_start = now;

              ROS_INFO("Trial %zu/%zu: dir=%d (%.1f deg), thrust=%.3f",
                       trial_idx + 1, trials.size(), current_dir_idx_, angle * 180.0 / M_PI, current_total_thrust);
            }
        }
        else if (state == STIMULUS) {
	  if (!paused) {
	    if ((now - toggle_time).toSec() >= 0.5){
	      stim_on = !stim_on;
	      toggle_time = now;
	    }

	    if (stim_on) {
	      Eigen::Vector2d dir_to_use;
	      double thrust_to_use;

	      if (test_mode) {
                dir_to_use = Eigen::Vector2d(std::cos(manual_dir_idx * M_PI / 4.0), std::sin(manual_dir_idx * M_PI / 4.0));
                thrust_to_use = manual_total_thrust;
	      } else {
                dir_to_use     = current_dir;
                thrust_to_use  = current_total_thrust;
	      }

	      Eigen::Vector4d alpha = computeAlphaFixedTotal(dir_to_use, total_thrust_);

	      spinal::PwmTest msg;
	      msg.motor_index = {0, 1, 2, 3};
	      msg.pwms.resize(4);
	      std_msgs::Float32MultiArray alpha_msg;
	      alpha_msg.data.resize(4);

	      for (int i = 0; i < 4; ++i) {
                msg.pwms[i] = static_cast<float>(calThrustPower(alpha[i], thrust_to_use));
                alpha_msg.data[i] = alpha[i];
	      }
	      pwm_pub.publish(msg);
	      alpha_pub.publish(alpha_msg);
	    } else {
	      spinal::PwmTest neutral;
	      neutral.motor_index = {0, 1, 2, 3};
	      neutral.pwms = {0.5, 0.5, 0.5, 0.5};
	      pwm_pub.publish(neutral);
	    }
	  } else {
	    spinal::PwmTest neutral;
	    neutral.motor_index = {0, 1, 2, 3};
	    neutral.pwms = {0.5, 0.5, 0.5, 0.5};
	    pwm_pub.publish(neutral);
	  }

	  if (stdinHasData()) {
	    std::string line;
	    if (!std::getline(std::cin, line)) {
	      ROS_WARN("Failed to read stdin line.");
	    } else {
	      int response = -1;
	      if (!line.empty()) {
		char c = line[0];

		if (c >= '0' && c <= '8') {
		  response = c - '0';
		} else if (c == 's') {
		  paused = !paused;
		  if (paused) {
		    ROS_WARN("== Paused by user (s pressed) ==");
		    spinal::PwmTest neutral;
		    neutral.motor_index = {0, 1, 2, 3};
		    neutral.pwms = {0.5, 0.5, 0.5, 0.5};
		    pwm_pub.publish(neutral);
		    test_mode = false;
		  } else {
		    ROS_WARN("== Resumed by user (s pressed) ==");
		    stim_on = true;
		    toggle_time = now;
		    test_mode = false;
		  }
		} else if (c == 't') {
		  if (!test_mode) {
		    ROS_WARN("== Enter manual test mode (t pressed) ==");
		    ROS_INFO("Enter dir_idx (0-7) and thrust (e.g. 0.3) separated by space, then Enter:");
		    std::string manual_line;
		    if (std::getline(std::cin, manual_line)) {
		      std::stringstream ss(manual_line);
		      int d_idx;
		      double thrust_val;
		      if (ss >> d_idx >> thrust_val) {
			if (d_idx < 0) d_idx = 0;
			if (d_idx > 7) d_idx = 7;
			manual_dir_idx = d_idx;
			manual_total_thrust = thrust_val;
			test_mode = true;
			paused = false;
			stim_on = true;
			toggle_time = now;
			ROS_INFO("Manual test: dir_idx=%d (%.1f deg), thrust=%.3f",
				 manual_dir_idx,
				 manual_dir_idx * 45.0,
				 manual_total_thrust);
		      } else {
			ROS_WARN("Failed to parse manual input. Format: <dir_idx> <thrust>");
		      }
		    }
		  }
		} else if (c == 'm') {
		  ROS_WARN("== Exit manual test mode (m pressed) ==");
		  test_mode = false;
		  stim_on = true;
		  toggle_time = now;
		}else if (c == 'h') {
		  shutdown = true;
		}
	      }

	      if (!test_mode && response >= 0 && response <= 8) {
                double angle_deg = current_dir_idx_ * 45.0;
                double rt_sec = (ros::Time::now() - trial_start_time).toSec();

                if (ofs) {
                  ofs << (trial_idx + 1) << ","                  // trial number 1..48
                      << current_dir_idx_ << ","
                      << std::fixed << std::setprecision(1) << angle_deg << ","
                      << std::setprecision(3) << current_total_thrust << ","
                      << response << ","
                      << std::setprecision(3) << rt_sec
                      << "\n";
                  ofs.flush();
                }

                spinal::PwmTest neutral;
                neutral.motor_index = {0,1,2,3};
                neutral.pwms = {0.5,0.5,0.5,0.5};
                pwm_pub.publish(neutral);

                ++trial_idx;
                if (trial_idx >= trials.size()) {
                  ROS_INFO("All %zu trials completed.", trials.size());
                  break;
                }

                state = INTER_TRIAL;
                state_start = now;
                stim_on = true;
                toggle_time = now;
              }
	      if (shutdown) {
		ROS_ERROR("== Experiment terminated by user (h pressed) ==");
		spinal::PwmTest neutral;
		neutral.motor_index = {0, 1, 2, 3};
		neutral.pwms = {0.5, 0.5, 0.5, 0.5};
		pwm_pub.publish(neutral);
		ros::shutdown();
		return 0;
	      }
	    }
	  }
        }
        rate.sleep();
    }
    return 0;
}
