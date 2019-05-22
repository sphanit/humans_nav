#include "humans_nav/humans_nav.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "humans_nav");

  humans_nav::TeleopHumans humans_nav;

  ros::spin();

  return 0;
}
