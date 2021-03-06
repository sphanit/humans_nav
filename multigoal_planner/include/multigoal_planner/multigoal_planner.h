#ifndef MULTIGOAL_PLANER_H
#define MULTIGOAL_PLANER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>
#include <global_planner/orientation_filter.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <hanp_msgs/HumanPathArray.h>
#include <boost/thread.hpp>
#include <humans_nav/types.h>
#include <humans_nav/planner_interface.h>

#include <multigoal_planner/MultiGoalPlannerConfig.h>

namespace multigoal_planner {
class MultiGoalPlanner : public humans_nav::PlannerInterface {
public:
  MultiGoalPlanner();
  MultiGoalPlanner(std::string name, tf2_ros::Buffer *tf2,
                   costmap_2d::Costmap2DROS *costmap_ros);
  ~MultiGoalPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf2,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool makePlans(const humans_nav::map_pose &starts,
                 const humans_nav::map_pose &goals,
                 humans_nav::map_pose_vectors &plans);

  bool makePlans(const humans_nav::map_pose &starts,
                 const humans_nav::map_pose_vector &sub_goals,
                 const humans_nav::map_pose &goals,
                 humans_nav::map_pose_vectors &plans);

private:
  tf2_ros::Buffer *tf2_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  std::vector<geometry_msgs::Point> robot_prev_pos_costmap;

  // geometry_msgs::Pose robot_pos;
  ros::Publisher plans_pub_, plans_poses_pub_, potential_pub_;
  ros::Subscriber robot_pos_sub_;
  void publishPlans(humans_nav::map_pose_vector &plans);

  dynamic_reconfigure::Server<MultiGoalPlannerConfig> *dsrv_;
  multigoal_planner::MultiGoalPlannerConfig default_config_, last_config_;
  void reconfigureCB(MultiGoalPlannerConfig &config, uint32_t level);
  void RobotPosCB(geometry_msgs::Pose robot_pos);
  boost::mutex configuration_mutex_;
  bool initialized_, setup_, allow_unknown_;
  double default_tolerance_;
  float convert_offset_;
  int publish_scale_;
  double sq_dist_plan_threshold_;

  global_planner::PotentialCalculator *p_calc_;
  global_planner::Expander *planner_;
  global_planner::Traceback *path_maker_, *path_maker_fallback_;
  global_planner::OrientationFilter *orientation_filter_;
  float *potential_array_;

  std::string tf_prefix_, planner_frame_;
  boost::mutex planning_mutex_;

  bool getPlanFromPotential(double start_x, double start_y, double goal_x,
                            double goal_y, humans_nav::pose_vector &plan);

  bool worldToMap(double wx, double wy, double &mx, double &my);
  void mapToWorld(double mx, double my, double &wx, double &wy);
  void outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value);
  void publishPotential(float *potential);
};
};

#endif // MULTIGOAL_PLANER_H
