#include "humans_nav/humans_nav.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "humans_nav");

  tf2_ros::Buffer tf2(ros::Duration(10),true);

  humans_nav::TeleopHumans humans_nav(tf2);
  // if(!humans_nav.external_trajs_->trajectories.empty())
  //   std::cout << "controller called" << '\n';
  // humans_nav.controller();
  ros::spin();

  return 0;
}
