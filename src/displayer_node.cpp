#include "dvs_displayer/displayer.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_displayer");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_displayer::Displayer displayer(nh, nh_private);

  ros::spin();

  return 0;
}
