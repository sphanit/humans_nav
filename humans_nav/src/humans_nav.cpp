#include "humans_nav/humans_nav.h"
#include <unistd.h>
#include <angles/angles.h>
#define EPS 0.1
#define NODE_NAME "humans_nav"
// #include <dynamic_reconfigure/server.h>
// #include <hanp_msgs/TrackedHumans.h>
// #include <hanp_msgs/TrackedSegmentType.h>
// #define DEFAUTL_SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO

namespace humans_nav{
TeleopHumans::TeleopHumans(tf2_ros::Buffer &tf2):
                          tf2_(tf2),planner_costmap_ros_(NULL),
                          planner_loader_("humans_nav", "humans_nav::PlannerInterface"),
                          planner_plans_(NULL), latest_plans_(NULL),run_planner_(false),
                          p_freq_change_(false), linear_x(1), linear_y(0), angular_(3)

{
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf2_lisn(tf2_);

  std::string planner_name;
  nh_.param("planner", planner_name, std::string("multigoal_planner/MultiGoalPlanner"));
  nh_.param("planner_frequency", planner_frequency_, 0.0);
  nh_.param("control_mode", dual_mode_, dual_mode_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  planner_plans_ = new humans_nav::map_pose_vectors();
  latest_plans_ = new humans_nav::map_pose_vectors();

  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("planner_costmap", tf2_);
  planner_costmap_ros_->pause();

  if (!loadPlugin(planner_name, planner_, planner_loader_,
                  planner_costmap_ros_)) {
    exit(1);
  }
  ROS_INFO("Costmap loaded HumansNav");
  planner_costmap_ros_->start();

  planner_thread_ = new boost::thread(boost::bind(&TeleopHumans::planThread, this));
  // loadPlugin<humans_nav::PlannerInterface>;

  for(int num=0;num<3;num++){
    geometry_msgs::Twist twist;
    twist_array.twist.push_back(twist);
  }
  zero_twist_array=twist_array;

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
  count = 0.0;
  start_=false;
  goal_reached_=true;
  reset_time =true;
  got_external_trajs = false;
  index=0;
  planner_frequency_ = 0.0;
  ctrl_time=0.0;
  prev_control_time=0.0;

  hum_sub_ = nh_.subscribe("/humans", 1, &TeleopHumans::HumansCallback, this);
  vel_pub_ = nh_.advertise<humans_msgs::TwistArray>("/humans/cmd_vel", 1);

  timer = nh_.createTimer(ros::Duration(0.1), &TeleopHumans::controller, this);
  // wake_timer = nh_.createTimer(ros::Duration(0.1), &TeleopHumans::controller, this);
  set_goal = nh_.advertiseService("setGoalHuman", &TeleopHumans::setGoal, this);

  // nh_.param("use_joy", use_joy, use_joy);
  f = boost::bind(&TeleopHumans::reconfigureCB,this, _1, _2);
  server.setCallback(f);


  joy_sub_ = nh_.subscribe("joy", 1, &TeleopHumans::JoyCallback, this);
  external_traj_sub_ = nh_.subscribe("/move_base_node/TebLocalPlannerROS/human_local_trajs", 1, &TeleopHumans::controllerPathsCB, this);
  global_path_sub_= nh_.subscribe("/move_base_node/TebLocalPlannerROS/human_global_plans", 1, &TeleopHumans::globalPlannerPathsCB, this);



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
  joy_ = true;
  // if(joy_ && !goal_reached_)
  //   vel_pub_.publish(zero_twist_array);
  // else
    vel_pub_.publish(twist_array);
}

// void TeleopHumans::TimerCallback(const ros::TimerEvent& event){
// }

void TeleopHumans::HumansCallback(const humans_msgs::HumanArray::ConstPtr & Humans){
  humans = *Humans;
}

void TeleopHumans::controllerPathsCB(const hanp_msgs::HumanTrajectoryArrayConstPtr traj_array)
{
  // if(count<1){
    // std::cout << "I am in the cb" << '\n';
    external_trajs_ = traj_array;
    last_trajs_ = *external_trajs_;
  // }
  // count++;
  got_external_trajs =  true;
  start_=true;
  // goal_reached_=false;
}

void TeleopHumans::globalPlannerPathsCB(const hanp_msgs::HumanPathArrayConstPtr path_array){
  global_paths_ = *path_array;
  global_goal = global_paths_.paths.back().path.poses.back();
  start_=true;
}

void TeleopHumans::getVelocity(geometry_msgs::Twist &twist){
  int active_id = 0;
  auto human = humans.humans[active_id];

  if(global_paths_.paths.back().path.poses.end()==global_paths_.paths.back().path.poses.begin()){
    goal_reached_= true;
    index = 0;
    ROS_INFO("Goal Reached..");
    return;
  }

  global_goal = global_paths_.paths.back().path.poses.back();
  double closest_dist=1e10;
  double dist = 0;

  int i=0;
  for(auto &pose: global_paths_.paths.back().path.poses){
    dist = std::hypot(pose.pose.position.x - human.pose.position.x, pose.pose.position.y- human.pose.position.y);
    if(dist<=closest_dist){
      index=i;
      closest_dist = dist;
    }
    // std::cout << "dist "<<dist << '\n';
    // std::cout << "Index " <<index<< '\n';
    // std::cout << "i " <<i<< '\n';
    i++;
  }

  auto pose = global_paths_.paths.back().path.poses[index].pose;

  twist.linear.x = (pose.position.x-human.pose.position.x);
  twist.linear.y = (pose.position.y-human.pose.position.y);
  twist.angular.z = angles::shortest_angular_distance(tf::getYaw(human.pose.orientation),tf::getYaw(pose.orientation));

  // std::cout <<"velx " << twist.linear.x <<"vely "<<twist.linear.y << "velz "<<twist.angular.z <<'\n';

  // if(closest_dist<EPS)
  dist = std::hypot(pose.position.x - human.pose.position.x, pose.position.y- human.pose.position.y);
  double delta_orient = normalize_theta(tf::getYaw(human.pose.orientation)-tf::getYaw(pose.orientation));

  if(dist<0.1 && delta_orient<0.1 && global_paths_.paths.back().path.poses.size()>1)
    if(index>0)
      global_paths_.paths.back().path.poses.erase(global_paths_.paths.back().path.poses.begin(),global_paths_.paths.back().path.poses.begin()+index);

  else if(index==0)
      global_paths_.paths.back().path.poses.erase(global_paths_.paths.back().path.poses.begin());

}

double TeleopHumans::normalize_theta(double theta)
 {
   if (theta >= -M_PI && theta < M_PI)
     return theta;

   double multiplier = floor(theta / (2*M_PI));
   theta = theta - multiplier*2*M_PI;
   if (theta >= M_PI)
     theta -= 2*M_PI;
   if (theta < -M_PI)
     theta += 2*M_PI;
   return theta;
 }

 geometry_msgs::Twist TeleopHumans::get_velocity_human(geometry_msgs::Pose pose, geometry_msgs::Pose h_pose){
    double theta = tf::getYaw(h_pose.orientation);
    geometry_msgs::Twist velocity;


    velocity.linear.x = 5*((pose.position.x-h_pose.position.x)*std::cos(theta)+(pose.position.x-h_pose.position.x)*std::sin(theta));
    velocity.linear.y = 5*((pose.position.y-h_pose.position.y)*std::sin(theta)+(pose.position.y-h_pose.position.y)*std::cos(theta));
    velocity.angular.z =  5*angles::shortest_angular_distance(tf::getYaw(h_pose.orientation),tf::getYaw(pose.orientation));

    return velocity;

 }

 void TeleopHumans::controller(const ros::TimerEvent& event){
   int active_id = 1;
   double vel_sum=0;
   double dx;
   double dy;
   double delta_orient;
   if(!use_joy){
     for(auto & human : humans.humans){
       active_id = human.id - 1;
       double vel_x = 0.0;
       double vel_y = 0.0;
       double vel_z = 0.0;
       if(human.active == true){
         // std::cout << "start_ " <<start_ << '\n';
         if(!start_ && !goal_reached_){
            for(auto &plan_it : current_plan){
                dx = global_goal.pose.position.x - human.pose.position.x;
                dy = global_goal.pose.position.y - human.pose.position.y;

                delta_orient = normalize_theta(tf::getYaw(global_goal.pose.orientation)-tf::getYaw(human.pose.orientation));
                if((((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <=0.2))&& !goal_reached_) ){
                     reset_time=true;
                     got_external_trajs = false;
                     goal_reached_ = true;
                     last_trajs_.trajectories.clear();
                     ROS_INFO("Goal Reached!");
                     twist_array.twist[active_id].angular.z = vel_z;
                     twist_array.twist[active_id].linear.x = vel_x;
                     twist_array.twist[active_id].linear.y = vel_y;
                     index = 0;
                      break;
                    }
                auto plan = current_plan[1];

                geometry_msgs::Twist velocity;
                velocity = get_velocity_human(plan.pose,human.pose);

                dx = plan.pose.position.x - human.pose.position.x;
                dy = plan.pose.position.y - human.pose.position.y;

                delta_orient = normalize_theta(tf::getYaw(plan.pose.orientation)-tf::getYaw(human.pose.orientation));

                twist_array.twist[active_id] = velocity;
                vel_sum=fabs(velocity.linear.x)+fabs(velocity.linear.y)+fabs(velocity.angular.z);

                if(current_plan.size()>1 && ((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <= 0.2))) {
                  current_plan.erase(current_plan.begin());
                  std::cout << "current_plan.size() " << current_plan.size()<< '\n';
                }
                break;
            }
         }
         else{
           for(auto & traj : last_trajs_.trajectories){
             dx = global_goal.pose.position.x - human.pose.position.x;
             dy = global_goal.pose.position.y - human.pose.position.y;
             delta_orient = normalize_theta(tf::getYaw(global_goal.pose.orientation)-tf::getYaw(human.pose.orientation));
             // double sum_vel = fabs(traj.trajectory.points[1].velocity.linear.x)+fabs(traj.trajectory.points[1].velocity.linear.y)+fabs(traj.trajectory.points[1].velocity.angular.z);
             // std::cout << "traj.trajectory.points.size() " <<traj.trajectory.points.size()<< '\n';
             // std::cout << "goal_reached_ " <<goal_reached_<< '\n';

             if((((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <=0.2)||
                    (traj.trajectory.points.begin()==traj.trajectory.points.end())) && !goal_reached_) ){
                // if((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <=0.1) && !goal_reached_){
                  reset_time=true;
                  got_external_trajs = false;
                  goal_reached_ = true;
                  start_=false;
                  last_trajs_.trajectories.clear();
                  ROS_INFO("Goal Reached!");
                  twist_array.twist[active_id].angular.z = vel_z;
                  twist_array.twist[active_id].linear.x = vel_x;
                  twist_array.twist[active_id].linear.y = vel_y;
                  index = 0;
                   break;
                 }

               if(goal_reached_){
                 last_trajs_.trajectories.clear();
                 break;
               }
               // if(!start_){
               //   ros::Duration(0.1).sleep();
               // }
               if(traj.trajectory.points.size()>1){
                   vel_x = traj.trajectory.points[1].velocity.linear.x;
                   vel_y = traj.trajectory.points[1].velocity.linear.y;
                   vel_z = traj.trajectory.points[1].velocity.angular.z;
                }
                ctrl_time = traj.trajectory.points[1].time_from_start.toSec()-prev_control_time;
                prev_control_time = traj.trajectory.points[1].time_from_start.toSec();

                twist_array.twist[active_id].angular.z = vel_z;
                twist_array.twist[active_id].linear.x = vel_x;
                twist_array.twist[active_id].linear.y = vel_y;

                vel_sum=fabs(vel_x)+fabs(vel_y)+fabs(vel_z);


                if(traj.trajectory.points.begin()==traj.trajectory.points.end())
                {
                  geometry_msgs::Twist velocity;
                  getVelocity(velocity);
                  twist_array.twist[active_id] = velocity;
                  vel_sum=fabs(velocity.linear.x)+fabs(velocity.linear.y)+fabs(velocity.angular.z);
                }
                // double dist = std::hypot(traj.trajectory.points[1].transform.translation.x - human.pose.position.x,traj.trajectory.points[1].transform.translation.y - human.pose.position.y);
                // double rot_dist = normalize_theta(tf::getYaw(human.pose.orientation)-tf::getYaw(traj.trajectory.points[1].transform.rotation));
                if(traj.trajectory.points.begin()!=traj.trajectory.points.end()){
                  last_trajs_.trajectories[0].trajectory.points.erase(last_trajs_.trajectories[0].trajectory.points.begin());
                  break;
                }
                break;
              }
            }
          }
     }
   if(!goal_reached_ && vel_sum<0.00001){
     twist_array.twist[active_id].angular.z = 0;
     twist_array.twist[active_id].linear.x = 0.0;
     twist_array.twist[active_id].linear.y = 0;
   }
   if(!joy_){

     ros::Time last = ros::Time::now();
     vel_pub_.publish(twist_array);
     // ros::Duration(ctrl_time-0.1).sleep();
     // vel_pub_.publish(twist_array);

     // vel_pub_.publish(twist_array);
     // std::cout << "I am in if" << '\n';
   }
   joy_=false;
 }
 // start_=false;
 }


 bool TeleopHumans::setGoal(humans_nav::HumansNavGoal::Request &req, humans_nav::HumansNavGoal::Response &res){

    ROS_DEBUG_NAMED(NODE_NAME, "Received new planning request");
    humans_nav::map_pose starts, goals;
    humans_nav::map_pose_vector sub_goals;

    for (auto & human : humans.humans){
      starts[human.id].pose = human.pose;
      starts[human.id].header.frame_id = "map";
      goals[human.id] = req.goals[human.id-1].pose;
      global_goal = req.goals[human.id-1].pose;
    }

    start_=false;
    goal_reached_ = false;
    count = 0;

    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_starts_ = starts;
    planner_goals_ = goals;
    planner_sub_goals_ = sub_goals;
    // state_ = move_humans::MoveHumansState::PLANNING;
    // ROS_DEBUG_NAMED(NODE_NAME, "Changed to  PLANNING state");
    ROS_INFO("Planning..");
    run_planner_ = true;
    planner_cond_.notify_one();
    lock.unlock();
    res.success=true;
    res.message="Goal has been set.";
    // const humans_nav::HumansNavGoal &hn_goal = *req;
 }

 void TeleopHumans::planThread() {
   ROS_INFO(NODE_NAME "_plan_thread", "Starting planner thread");
   ros::NodeHandle nh;
   ros::Timer timer;
   bool wait_for_wake = false;
   boost::unique_lock<boost::mutex> lock(planner_mutex_);
   while (nh.ok()) {
     while (wait_for_wake || !run_planner_) {
       ROS_DEBUG_NAMED(NODE_NAME "_plan_thread", "Planner thread is suspending");
       planner_cond_.wait(lock);
       wait_for_wake = false;
     }
     ros::Time start_time = ros::Time::now();
     auto human = humans.humans[0];
     planner_starts_[1].pose = human.pose;
     auto planner_starts = planner_starts_;
     auto planner_goals = planner_goals_;
     auto planner_sub_goals = planner_sub_goals_;
     lock.unlock();

     ROS_DEBUG_NAMED(NODE_NAME "_plan_thread", "Planning");
     planner_plans_->clear();
     if (nh.ok()) {
       boost::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(
           *(planner_costmap_ros_->getCostmap()->getMutex()));

       if (planner_costmap_ros_ == NULL) {
         ROS_ERROR_NAMED(NODE_NAME "_plan_thread",
                         "Planner costmap NULL, unable to create plan");
       } else {
         bool planning_success = false;
         if (planner_sub_goals.size() > 0) {
           planning_success =
               planner_->makePlans(planner_starts, planner_sub_goals,
                                   planner_goals, *planner_plans_);
         } else {
           planning_success = planner_->makePlans(planner_starts, planner_goals,
                                                  *planner_plans_);
          }
         if (!planning_success) {
           ROS_DEBUG_NAMED(NODE_NAME "_plan_thread",
                           "Planner plugin failed to find plans");
         }
       }
     }

     lock.lock();

     if (planner_plans_->size() > 0) {
       ROS_DEBUG_NAMED(NODE_NAME "_plan_thread", "Got %lu new plans",
                       planner_plans_->size());
       auto planner_plans = planner_plans_;
       planner_plans_ = latest_plans_;
       latest_plans_ = planner_plans;
       new_global_plans_ = true;

       if (run_planner_) {

        for (auto plan_vector_kv : *latest_plans_) {
          auto human_id = plan_vector_kv.first;
          auto plan_vector = plan_vector_kv.second;
          int active_id = 0;
          auto human = humans.humans[active_id];

          // if (plan_vector.size() > 0) {
          //   current_controller_plans_[human_id] = plan_vector.front();
            current_plan.clear();
            for (auto &plan : plan_vector) {
              current_plan.insert(current_plan.end(), plan.begin(), plan.end());
            }
          }
         ROS_DEBUG_NAMED(NODE_NAME, "Changing to CONTROLLING state");
       }
       if (planner_frequency_ <= 0) {
         run_planner_ = false;
       }
     }
     else {
         run_planner_ = false;

     }

     if (planner_frequency_ > 0) {
       ros::Duration sleep_time =
           (start_time + ros::Duration(1.0 / planner_frequency_)) -
           ros::Time::now();
       if (sleep_time > ros::Duration(0.0)) {
         wait_for_wake = true;
         timer = nh.createTimer(sleep_time, &TeleopHumans::wakePlanner, this);
       }
     }
   }
   lock.unlock();
 }

 void TeleopHumans::wakePlanner(const ros::TimerEvent &event) {
   planner_cond_.notify_one();
 }

 void TeleopHumans::wake_up(const ros::TimerEvent &event){
  ROS_INFO("Woke the timer");
 }


 template <typename T>
 bool TeleopHumans::loadPlugin(const std::string plugin_name,
                             boost::shared_ptr<T> &plugin,
                             pluginlib::ClassLoader<T> &plugin_loader,
                             costmap_2d::Costmap2DROS *plugin_costmap) {
   // std::cout << "I am in MoveHumans::loadPlugin" << '\n';
   boost::shared_ptr<T> old_plugin = plugin;
   ROS_INFO_NAMED(NODE_NAME, "Loading plugin %s", plugin_name.c_str());
   try {
     plugin = plugin_loader.createInstance(plugin_name);

     boost::unique_lock<boost::mutex> lock(planner_mutex_);
     planner_plans_->clear();
     latest_plans_->clear();
     plugin->initialize(plugin_loader.getName(plugin_name), &tf2_,
                        plugin_costmap);
     lock.unlock();

     resetState();
   } catch (const pluginlib::PluginlibException &ex) {
     ROS_FATAL_NAMED(NODE_NAME, "Failed to create plugin %s. Exception: %s",
                     plugin_name.c_str(), ex.what());
     plugin = old_plugin;
     return false;
   }
   return true;
 }


 void TeleopHumans::resetState() {
   boost::unique_lock<boost::mutex> lock(planner_mutex_);
   run_planner_ = false;
   lock.unlock();
 }
 // bool TeleopHumans::validateGoals(const move_humans::MoveHumansGoal &mh_goal,
 //                                move_humans::map_pose &starts,
 //                                move_humans::map_pose_vector &sub_goals,
 //                                move_humans::map_pose &goals) {
 //   // std::cout << "I am in MoveHumans::validateGoals" << '\n';
 //   if (mh_goal.start_poses.size() == 0 || mh_goal.goal_poses.size() == 0 ||
 //       mh_goal.start_poses.size() != mh_goal.goal_poses.size()) {
 //     ROS_ERROR_NAMED(NODE_NAME, "Number of start and goals poses are not equal, "
 //                                "aborting on planning request");
 //     return false;
 //   }
 //
 //   // check frame_id for all start, sub-goal and goal poses
 //   auto frame_id = mh_goal.start_poses[0].pose.header.frame_id;
 //   for (auto &s_pose : mh_goal.start_poses) {
 //     if (s_pose.pose.header.frame_id != frame_id) {
 //       ROS_ERROR_NAMED(
 //           NODE_NAME,
 //           "All start, goal and sub-goal positions must be in same frame");
 //       return false;
 //     }
 //   }
 //   for (auto &g_pose : mh_goal.goal_poses) {
 //     if (g_pose.pose.header.frame_id != frame_id) {
 //       ROS_ERROR_NAMED(
 //           NODE_NAME,
 //           "All start, goal and sub-goal positions must be in same frame");
 //       return false;
 //     }
 //   }
 //   for (auto &sg_poses : mh_goal.sub_goal_poses) {
 //     for (auto &sg_pose : sg_poses.poses) {
 //       if (sg_pose.header.frame_id != frame_id) {
 //         ROS_ERROR_NAMED(
 //             NODE_NAME,
 //             "All start, goal and sub-goal positions must be in same frame");
 //         return false;
 //       }
 //     }
 //   }
 //
 //   // validate quaternions for all start, sub-goal and goal poses
 //   for (auto &start : mh_goal.start_poses) {
 //     if (!isQuaternionValid(start.pose.pose.orientation)) {
 //       ROS_ERROR_NAMED(NODE_NAME, "Not planning for human %lu, start pose was "
 //                                  "sent with an invalid quaternion",
 //                       start.human_id);
 //       continue;
 //     }
 //     starts[start.human_id] = start.pose;
 //   }
 //   for (auto &goal : mh_goal.goal_poses) {
 //     if (!isQuaternionValid(goal.pose.pose.orientation)) {
 //       ROS_ERROR_NAMED(NODE_NAME, "Not planning for human %lu, goal pose was "
 //                                  "sent with an invalid quaternion",
 //                       goal.human_id);
 //       continue;
 //     }
 //     goals[goal.human_id] = goal.pose;
 //   }
 //   for (auto &sub_goal_poses : mh_goal.sub_goal_poses) {
 //     move_humans::pose_vector valid_sub_goals;
 //     for (auto &sub_goal : sub_goal_poses.poses) {
 //       if (!isQuaternionValid(sub_goal.pose.orientation)) {
 //         ROS_ERROR_NAMED(NODE_NAME, "Removing a sub-goals for human %lu, it was "
 //                                    "sent with an invalid quaternion",
 //                         sub_goal_poses.human_id);
 //       } else {
 //         valid_sub_goals.push_back(sub_goal);
 //       }
 //     }
 //     if (valid_sub_goals.size() > 0) {
 //       sub_goals[sub_goal_poses.human_id] = valid_sub_goals;
 //     }
 //   }
 //
 //   // check for all starts there exists a goal and vice-versa
 //   auto its = starts.begin();
 //   while (its != starts.end()) {
 //     if (goals.find(its->first) == goals.end()) {
 //       sub_goals.erase(its->first);
 //       its = starts.erase(its);
 //     } else {
 //       ++its;
 //     }
 //   }
 //   auto itg = goals.begin();
 //   while (itg != goals.end()) {
 //     if (starts.find(itg->first) == starts.end()) {
 //       sub_goals.erase(itg->first);
 //       itg = goals.erase(itg);
 //     } else {
 //       ++itg;
 //     }
 //   }
 //
 //   if (starts.size() == 0 || goals.size() == 0) {
 //     ROS_ERROR_NAMED(NODE_NAME,
 //                     "Aborting on request as not valid start-goal pair found");
 //     return false;
 //   }
 //   return true;
 // }

 void TeleopHumans::reconfigureCB(HumanConfig &config, uint32_t level){
    ROS_INFO("Using JoyStick: %s",config.use_joy?"True":"False");
    use_joy = config.use_joy;

    // if (planner_frequency_ != config.planner_frequency) {
    //   planner_frequency_ = config.planner_frequency;
    //   p_freq_change_ = true;
    // }
    //
    // if (config.planner != last_config_.planner) {
    //   if (!loadPlugin<move_humans::PlannerInterface>(
    //           config.planner, planner_, planner_loader_, planner_costmap_ros_)) {
    //     config.planner = last_config_.planner;
    //   }
    // }
 }

};
