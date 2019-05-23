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

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("humans/cmd_vel", 1);
  hum_pub_ = nh_.advertise<HumanMarker>("humans/marker", 1);

  timer = nh_.createTimer(ros::Duration(0.1), &TeleopHumans::TimerCallback, this);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHumans::JoyCallback, this);
  human.id=1;
  human.active=false;

}

void TeleopHumans::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  hum_pub_.publish(human);
  vel_pub_.publish(twist);
}

void TeleopHumans::TimerCallback(const ros::TimerEvent& event){
  hum_pub_.publish(human);
}
};