#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <humans_msgs/HumanMarker.h>
#include <humans_msgs/HumanArray.h>
#include <humans_msgs/TwistArray.h>
#include <dynamic_reconfigure/server.h>
#include <humans_nav/HumanConfig.h>

namespace humans_nav{
class TeleopHumans
{
public:
  TeleopHumans();

private:

  humans_msgs::HumanArray humans;
  humans_msgs::TwistArray twist_array;

  double linear_x, linear_y, angular_;
  double l_scale_, a_scale_;
  bool dual_mode_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_,hum_sub_;
  ros::Timer timer;
  // dynamic_reconfigure::Server<humans_nav::HumanConfig> *dsrv_;

  // void TimerCallback(const ros::TimerEvent& event);

  void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void HumansCallback(const humans_msgs::HumanArray::ConstPtr & Humans);

  // void callback(humans_nav::HumanConfig &config, uint32_t level);

};
};
