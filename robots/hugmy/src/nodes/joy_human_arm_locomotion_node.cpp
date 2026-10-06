#include <cmath>
#include <string>

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/SetBool.h>

class JoyHumanArmLocomotion
{
public:
  JoyHumanArmLocomotion(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("enable_button", enable_button_, 4);
    pnh.param("disable_button", disable_button_, 5);
    pnh.param("forward_button", forward_button_, 3);
    pnh.param("stop_button", stop_button_, 1);
    pnh.param("forward_distance_m", forward_distance_m_, 0.100);

    target_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
        "attitude_pressure_controller/target_attitude", 1, true);
    command_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(
        "attitude_pressure_controller/inchworm_command", 1, false);
    enable_client_ = nh.serviceClient<std_srvs::SetBool>(
        "attitude_pressure_controller/enable");
    joy_sub_ = nh.subscribe("joy", 1,
        &JoyHumanArmLocomotion::joyCallback, this);

    publishZeroAttitude();
    ROS_WARN("Human-arm Joy control: button %d=enable, %d=disable, "
             "%d=toward-hand %.0f mm, %d=stop",
             enable_button_, disable_button_, forward_button_,
             1000.0 * forward_distance_m_, stop_button_);
  }

private:
  bool pressed(const sensor_msgs::Joy& joy, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size() &&
           joy.buttons[index] != 0;
  }

  void publishZeroAttitude()
  {
    geometry_msgs::Vector3Stamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "world";
    target.vector.x = 0.0;
    target.vector.y = 0.0;
    target.vector.z = 0.0;
    target_pub_.publish(target);
  }

  void publishStop()
  {
    geometry_msgs::Vector3Stamped command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "body";
    command_pub_.publish(command);
  }

  bool setEnabled(bool enabled)
  {
    if (enabled)
      publishZeroAttitude();
    else
      publishStop();

    std_srvs::SetBool service;
    service.request.data = enabled;
    if (!enable_client_.call(service) || !service.response.success)
      {
        ROS_ERROR("Could not set pressure/locomotion enable=%d: %s",
                  enabled, service.response.message.c_str());
        return false;
      }
    enabled_ = enabled;
    ROS_WARN("Pressure and locomotion control %s: %s",
             enabled ? "ENABLED" : "DISABLED",
             service.response.message.c_str());
    return true;
  }

  void publishForwardCommand()
  {
    if (!enabled_)
      {
        ROS_ERROR("Forward command ignored: press the enable button first");
        return;
      }
    publishZeroAttitude();
    geometry_msgs::Vector3Stamped command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "body_toward_hand";
    command.vector.x = 1.0;
    command.vector.y = 0.0;
    command.vector.z = forward_distance_m_;
    command_pub_.publish(command);
    ROS_WARN("Joy requested %.1f-mm travel toward the hand marker",
             1000.0 * forward_distance_m_);
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    const bool enable = pressed(*msg, enable_button_);
    const bool disable = pressed(*msg, disable_button_);
    const bool forward = pressed(*msg, forward_button_);
    const bool stop = pressed(*msg, stop_button_);

    // Stop/disable has priority when two buttons are pressed together.
    if (disable && !previous_disable_)
      setEnabled(false);
    else if (stop && !previous_stop_)
      {
        publishStop();
        ROS_WARN("Human-arm locomotion stop requested; pressure remains enabled");
      }
    else if (enable && !previous_enable_)
      setEnabled(true);
    else if (forward && !previous_forward_)
      publishForwardCommand();

    previous_enable_ = enable;
    previous_disable_ = disable;
    previous_forward_ = forward;
    previous_stop_ = stop;
  }

  ros::Subscriber joy_sub_;
  ros::Publisher target_pub_, command_pub_;
  ros::ServiceClient enable_client_;
  int enable_button_ = 4;
  int disable_button_ = 5;
  int forward_button_ = 3;
  int stop_button_ = 1;
  double forward_distance_m_ = 0.100;
  bool enabled_ = false;
  bool previous_enable_ = false;
  bool previous_disable_ = false;
  bool previous_forward_ = false;
  bool previous_stop_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_human_arm_locomotion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JoyHumanArmLocomotion command(nh, pnh);
  ros::spin();
  return 0;
}
