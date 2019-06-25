#include "humans_nav/humans_nav.h"

namespace humans_nav{
TeleopHumans::TeleopHumans():
  linear_x(1),
  linear_y(0),
  angular_(3)
{
  ros::NodeHandle nh_;

  nh_.param("control_mode", dual_mode_, dual_mode_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  for(int num=0;num<3;num++){
    geometry_msgs::Twist twist;
    twist_array.twist.push_back(twist);
  }


  if(dual_mode_){
    angular_ = 3;
    linear_x = 1;
    linear_y = 0;
  }
  else{
    angular_ = 3;
    linear_x = 1;
    linear_y = 0;
  }

  hum_sub_ = nh_.subscribe<humans_msgs::HumanArray>("/humans", 10, &TeleopHumans::HumansCallback, this);
  vel_pub_ = nh_.advertise<humans_msgs::TwistArray>("/humans/cmd_vel", 1);

  // timer = nh_.createTimer(ros::Duration(0.1), &TeleopHumans::TimerCallback, this);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHumans::JoyCallback, this);



}

void TeleopHumans::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  int active_id = 1 ;

  for(auto & human : humans.humans){
    active_id = human.id - 1;
    if(human.active == true){
      twist_array.twist[active_id].angular.z = a_scale_*joy->axes[angular_];
      twist_array.twist[active_id].linear.x = l_scale_*joy->axes[linear_x];
      twist_array.twist[active_id].linear.y = l_scale_*joy->axes[linear_y];
    }
    else{
      twist_array.twist[active_id].angular.z = 0;
      twist_array.twist[active_id].linear.x = 0;
      twist_array.twist[active_id].linear.y = 0;
    }

  }

  vel_pub_.publish(twist_array);
}

// void TeleopHumans::TimerCallback(const ros::TimerEvent& event){
// }

void TeleopHumans::HumansCallback(const humans_msgs::HumanArray::ConstPtr & Humans){
  humans = *Humans;
}

};
