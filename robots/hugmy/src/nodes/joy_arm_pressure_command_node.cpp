#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/SetBool.h>

class JoyArmPressureCommand
{
public:
  JoyArmPressureCommand(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("hold_pressure_kpa", hold_pressure_kpa_, 30.0);
    pnh.param("hold_button", hold_button_, 4);
    pnh.param("exhaust_button", exhaust_button_, 5);

    target_pub_ = nh.advertise<std_msgs::Float32MultiArray>(
        "independent_arm_pressure_controller/target_pressure", 1, true);
    enable_client_ = nh.serviceClient<std_srvs::SetBool>(
        "independent_arm_pressure_controller/enable");
    joy_sub_ = nh.subscribe("joy", 1, &JoyArmPressureCommand::joyCallback, this);

    ROS_INFO("Joy arm pressure command ready: button %d = %.1f kPa, button %d = exhaust",
             hold_button_, hold_pressure_kpa_, exhaust_button_);
  }

private:
  bool pressed(const sensor_msgs::Joy& joy, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size() &&
           joy.buttons[index] != 0;
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    const bool hold_pressed = pressed(*msg, hold_button_);
    const bool exhaust_pressed = pressed(*msg, exhaust_button_);

    // Exhaust has priority if both buttons are pressed.
    if (exhaust_pressed && !previous_exhaust_pressed_)
      setAllTargets(0.0, "exhaust");
    else if (hold_pressed && !previous_hold_pressed_)
      setAllTargets(hold_pressure_kpa_, "hold");

    previous_hold_pressed_ = hold_pressed;
    previous_exhaust_pressed_ = exhaust_pressed;
  }

  void setAllTargets(double pressure_kpa, const char* mode)
  {
    std_srvs::SetBool enable;
    enable.request.data = true;
    if (!enable_client_.call(enable) || !enable.response.success)
    {
      ROS_ERROR("Cannot enable pressure controller; target was not changed");
      return;
    }

    std_msgs::Float32MultiArray target;
    target.data.assign(4, static_cast<float>(pressure_kpa));
    target_pub_.publish(target);
    ROS_WARN("Arm pressure mode: %s (target %.1f kPa on all arms)",
             mode, pressure_kpa);
  }

  ros::Subscriber joy_sub_;
  ros::Publisher target_pub_;
  ros::ServiceClient enable_client_;
  double hold_pressure_kpa_ = 30.0;
  int hold_button_ = 4;
  int exhaust_button_ = 5;
  bool previous_hold_pressed_ = false;
  bool previous_exhaust_pressed_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_arm_pressure_command");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JoyArmPressureCommand command(nh, pnh);
  ros::spin();
  return 0;
}
