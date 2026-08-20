#include <hugmy/control/independent_arm_pressure_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "independent_arm_pressure_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  IndependentArmPressureController controller(nh, pnh);
  ros::spin();
  return 0;
}
