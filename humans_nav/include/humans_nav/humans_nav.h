#ifndef HUMANS_NAV_H
#define HUMANS_NAV_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
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
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include "humans_nav/HumansNavGoal.h"
#include "humans_nav/types.h"
#include "humans_nav/planner_interface.h"
#include <humans_nav/HumanPose.h>

namespace humans_nav{
class TeleopHumans
{
public:
  TeleopHumans(tf2_ros::Buffer &tf2);
  hanp_msgs::HumanTrajectoryArrayConstPtr external_trajs_;
  hanp_msgs::HumanTrajectoryArray last_trajs_;
  void controller(const ros::TimerEvent& event);


private:
  tf2_ros::Buffer &tf2_;
  double count;
  double ctrl_time,prev_control_time;
  double planner_frequency_;
  costmap_2d::Costmap2DROS *planner_costmap_ros_;
  boost::shared_ptr<humans_nav::PlannerInterface> planner_;
  pluginlib::ClassLoader<humans_nav::PlannerInterface> planner_loader_;
  humans_nav::map_pose_vectors *planner_plans_,*latest_plans_;
  humans_nav::pose_vector current_plan;
  bool new_global_plans_, p_freq_change_, run_planner_;
  boost::mutex planner_mutex_, external_trajs_mutex_, external_vels_mutex_, controller_mutex_;
  boost::condition_variable planner_cond_;
  boost::condition_variable controller_cond_;
  humans_nav::map_pose planner_starts_, planner_goals_;
  humans_nav::map_pose_vector planner_sub_goals_;
  boost::thread *planner_thread_, *controller_thread_;

  humans_msgs::HumanArray humans;
  humans_msgs::TwistArray twist_array;
  humans_msgs::TwistArray zero_twist_array;
  hanp_msgs::HumanPathArray global_paths_;
  geometry_msgs::PoseStamped global_goal;

  double linear_x, linear_y, angular_;
  double l_scale_, a_scale_;
  bool dual_mode_,joy_, use_joy;
  bool start_,goal_reached_,reset_time;
  bool got_external_trajs,zeros_published;
  bool goal_set;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_,hum_sub_,external_traj_sub_, global_path_sub_;
  ros::ServiceServer set_goal, set_goal_call, set_goal_srv_;
  ros::Timer timer,wake_timer;
  ros::Time last_calc_time_;
  int index,last_index;

  dynamic_reconfigure::Server<HumanConfig> server;
  dynamic_reconfigure::Server<HumanConfig>::CallbackType f;
  // dynamic_reconfigure::Server<humans_nav::HumanConfig> *dsrv_;

  // void TimerCallback(const ros::TimerEvent& event);
  void controllerPathsCB(const hanp_msgs::HumanTrajectoryArrayConstPtr traj_array);

  void globalPlannerPathsCB(const hanp_msgs::HumanPathArrayConstPtr path_array);

  double normalize_theta(double theta);

  void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  void HumansCallback(const humans_msgs::HumanArray::ConstPtr & Humans);

  void getVelocity(geometry_msgs::Twist &twist);

  bool setGoal(humans_nav::HumansNavGoal::Request &req, humans_nav::HumansNavGoal::Response &res);

  bool setGoal_call(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool set_goal_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void reconfigureCB(HumanConfig &config, uint32_t level);

  void planThread();
  // void controlThread();

  void wakePlanner(const ros::TimerEvent &event);
  void wake_up(const ros::TimerEvent &event);
  geometry_msgs::Twist get_velocity_human(geometry_msgs::Pose pose, geometry_msgs::Pose h_pose);
  // void wakeController(const ros::TimerEvent &event);

  template <typename T>
  bool loadPlugin(const std::string plugin_name, boost::shared_ptr<T> &plugin,
                  pluginlib::ClassLoader<T> &plugin_loader,
                  costmap_2d::Costmap2DROS *plugin_costmap);

  void resetState();


  // void callback(humans_nav::HumanConfig &config, uint32_t level);

};
};
#endif
