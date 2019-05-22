#include "humans_nav/humans_nav.h"

namespace humans_nav{
TeleopHumans::TeleopHumans():
  linear_(1),
  angular_(0)
{
  ros::NodeHandle nh_;

  nh_.param("control_mode", dual_mode_, dual_mode_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  if(dual_mode_){
    angular_ = 3;
    linear_ = 1;
  }
  else{
    angular_ = 0;
    linear_ = 1;
  }

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("human1/motion", 1);
  hum_pub_ = nh_.advertise<HumanMarker>("human1/marker", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHumans::joyCallback, this);
  human.id=1;
  human.active=false;
  hum_pub_.publish(human);

}

void TeleopHumans::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  hum_pub_.publish(human);
  vel_pub_.publish(twist);
}
};