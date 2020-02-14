#ifndef HUMANS_NAV_PLANNER_INTERFACE_H
#define HUMANS_NAV_PLANNER_INTERFACE_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include "humans_nav/types.h"

namespace humans_nav {

class PlannerInterface {
public:
  virtual ~PlannerInterface() {}

  virtual void initialize(std::string name, tf2_ros::Buffer *tf2,
                          costmap_2d::Costmap2DROS *costmap_ros) = 0;

  virtual bool makePlans(const humans_nav::map_pose &starts,
                         const humans_nav::map_pose &goals,
                         humans_nav::map_pose_vectors &plans) = 0;

  virtual bool makePlans(const humans_nav::map_pose &starts,
                         const humans_nav::map_pose_vector &sub_goals,
                         const humans_nav::map_pose &goals,
                         humans_nav::map_pose_vectors &plans) = 0;

protected:
  PlannerInterface() {}
};
}; // namespace humans_nav

#endif // HUMANS_NAV_PLANNER_INTERFACE_
