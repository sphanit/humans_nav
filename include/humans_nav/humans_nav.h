#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <humans_nav/HumanMarker.h>

namespace humans_nav{
class TeleopHumans
{
public:
  TeleopHumans();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  HumanMarker human;

  double linear_, angular_;
  double l_scale_, a_scale_;
  bool dual_mode_;
  ros::Publisher vel_pub_,hum_pub_;
  ros::Subscriber joy_sub_;

};
};