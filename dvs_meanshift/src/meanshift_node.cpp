

#include "dvs_meanshift/meanshift.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_meanshift");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_meanshift::Meanshift meanshift(nh, nh_private);

  ros::spin();

  return 0;
}
