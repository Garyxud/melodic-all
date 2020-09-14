#include <tf2_server/tf2_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf2_server::TF2Server server(nh, pnh);
  server.start();

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}