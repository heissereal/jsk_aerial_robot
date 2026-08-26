#include <hugmy/control/attitude_pressure_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "attitude_pressure_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  AttitudePressureController controller(nh, pnh);
  ros::spin();
  return 0;
}
