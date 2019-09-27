#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <humans_msgs/HumanMarker.h>
#include <humans_msgs/HumanArray.h>
#include <humans_msgs/TwistArray.h>
#include <dynamic_reconfigure/server.h>
#include <humans_nav/HumanConfig.h>
#include <hanp_msgs/HumanTrajectoryArray.h>
#include <hanp_msgs/HumanPathArray.h>
#include <tf/tf.h>
#include <std_srvs/SetBool.h>

namespace humans_nav{
class TeleopHumans
{
public:
  TeleopHumans();
  hanp_msgs::HumanTrajectoryArrayConstPtr external_trajs_;
  hanp_msgs::HumanTrajectoryArray last_trajs_;
  void controller(const ros::TimerEvent& event);


private:

  humans_msgs::HumanArray humans;
  humans_msgs::TwistArray twist_array;
  humans_msgs::TwistArray zero_twist_array;
  hanp_msgs::HumanPathArray::ConstPtr global_paths_;
  geometry_msgs::PoseStamped global_goal;

  double linear_x, linear_y, angular_;
  double l_scale_, a_scale_;
  bool dual_mode_,joy_;
  bool start_,goal_reached_,reset_time;
  bool got_external_trajs,zeros_published;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_,hum_sub_,external_traj_sub_, global_path_sub_;
  ros::ServiceServer set_goal;
  ros::Timer timer;
  ros::Time last_calc_time_;
  int index,last_index;
  // dynamic_reconfigure::Server<humans_nav::HumanConfig> *dsrv_;

  // void TimerCallback(const ros::TimerEvent& event);
  void controllerPathsCB(const hanp_msgs::HumanTrajectoryArrayConstPtr traj_array);

  void globalPlannerPathsCB(const hanp_msgs::HumanPathArrayConstPtr path_array);

  double normalize_theta(double theta);

  void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void HumansCallback(const humans_msgs::HumanArray::ConstPtr & Humans);

  bool setGoal(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // void callback(humans_nav::HumanConfig &config, uint32_t level);

};
};
