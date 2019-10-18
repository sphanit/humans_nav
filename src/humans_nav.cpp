#include "humans_nav/humans_nav.h"
#include <unistd.h>
#include <angles/angles.h>
#define EPS 0.1
// #include <dynamic_reconfigure/server.h>
// #include <hanp_msgs/TrackedHumans.h>
// #include <hanp_msgs/TrackedSegmentType.h>
// #define DEFAUTL_SEGMENT_TYPE hanp_msgs::TrackedSegmentType::TORSO

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
  start_=true;
  goal_reached_=true;
  reset_time =true;
  got_external_trajs = false;
  index=0;

  hum_sub_ = nh_.subscribe("/humans", 1, &TeleopHumans::HumansCallback, this);
  vel_pub_ = nh_.advertise<humans_msgs::TwistArray>("/humans/cmd_vel", 1);

  timer = nh_.createTimer(ros::Duration(0.1), &TeleopHumans::controller, this);
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
  got_external_trajs =  true;
  external_trajs_ = traj_array;
  last_trajs_ = *external_trajs_;
  start_=true;
}

void TeleopHumans::globalPlannerPathsCB(const hanp_msgs::HumanPathArrayConstPtr path_array){
  global_paths_ = *path_array;
  global_goal = global_paths_.paths.back().path.poses.back();
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

  twist.linear.x = -(pose.position.x-human.pose.position.x);
  twist.linear.y = -(pose.position.y-human.pose.position.y);
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
     for(auto & traj : last_trajs_.trajectories){
       dx = global_goal.pose.position.x - human.pose.position.x;
       dy = global_goal.pose.position.y - human.pose.position.y;
       delta_orient = normalize_theta(tf::getYaw(global_goal.pose.orientation)-tf::getYaw(human.pose.orientation));
       // double sum_vel = fabs(traj.trajectory.points[1].velocity.linear.x)+fabs(traj.trajectory.points[1].velocity.linear.y)+fabs(traj.trajectory.points[1].velocity.angular.z);
       // std::cout << "traj.trajectory.points.size() " <<traj.trajectory.points.size()<< '\n';
       // std::cout << "goal_reached_ " <<goal_reached_<< '\n';

       if((((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <=0.1)||
              (traj.trajectory.points.begin()==traj.trajectory.points.end())) && !goal_reached_) ){
          // if((fabs(std::sqrt(dx*dx+dy*dy)) <=0.2 && fabs(delta_orient) <=0.1) && !goal_reached_){
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
        }
     }

   }
   if(!goal_reached_ && vel_sum<0.00001){
     twist_array.twist[active_id].angular.z = 0;
     twist_array.twist[active_id].linear.x = 0.0;
     twist_array.twist[active_id].linear.y = 0;
   }
   if(!joy_){
     vel_pub_.publish(twist_array);
     // std::cout << "I am in if" << '\n';
   }
   joy_=false;
 }
 start_=false;
 }

 bool TeleopHumans::setGoal(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    std::string message = req.data ? "Goal set" : "Goal not set";
    res.success = true;
    res.message = message;
    goal_reached_ = !req.data;
    ROS_INFO("Goal set!");

 }

 void TeleopHumans::reconfigureCB(HumanConfig &config, uint32_t level){
    ROS_INFO("Using JoyStick: %s",config.use_joy?"True":"False");
    use_joy = config.use_joy;
 }

 // void TeleopHumans::getVelocity(geometry_msgs::Twist &twist){
 //   int active_id = 0;
 //   auto human = humans.humans[active_id];
 //
 //   if(global_paths_.paths.back().path.poses.end()==global_paths_.paths.back().path.poses.begin()){
 //     goal_reached_= true;
 //     index = 0;
 //     ROS_INFO("Goal Reached..");
 //     return;
 //   }
 //
 //   global_goal = global_paths_.paths.back().path.poses.back();
 //   double closest_dist=1e10;
 //   double dist = 0;
 //   // double vel_x,vel_y,vel_z;
 //   if(index<=0){
 //     int i=0;
 //     for(auto &pose: global_paths_.paths.back().path.poses){
 //       dist = std::hypot(pose.pose.position.x - human.pose.position.x, pose.pose.position.y- human.pose.position.y);
 //       if(dist<=closest_dist){
 //         index=i;
 //         closest_dist = dist;
 //       }
 //       std::cout << "dist "<<dist << '\n';
 //       std::cout << "Index " <<index<< '\n';
 //       std::cout << "i " <<i<< '\n';
 //       i++;
 //     }
 //   }
 //
 //   auto pose = global_paths_.paths.back().path.poses[index].pose;
 //   if(closest_dist==1e10)
 //     pose = global_paths_.paths.back().path.poses.front().pose;
 //
 //   twist.linear.x = -(pose.position.x-human.pose.position.x);
 //   twist.linear.y = -(pose.position.y-human.pose.position.y);
 //   twist.angular.z = angles::shortest_angular_distance(tf::getYaw(human.pose.orientation),tf::getYaw(pose.orientation));
 //   if(index>0){
 //     twist.linear.x*=1;
 //     twist.linear.y*=1;
 //     twist.angular.z*=1;
 //   }
 //   // std::cout <<"velx " << twist.linear.x <<"vely "<<twist.linear.y << "velz "<<twist.angular.z <<'\n';
 //
 //   // if(closest_dist<EPS)
 //   dist = std::hypot(pose.position.x - human.pose.position.x, pose.position.y- human.pose.position.y);
 //   double delta_orient = normalize_theta(tf::getYaw(human.pose.orientation)-tf::getYaw(pose.orientation));
 //   if(/*dist<0.1 && delta_orient<0.1 &&*/ global_paths_.paths.back().path.poses.size()>1)
 //     if(closest_dist<1e10)
 //       global_paths_.paths.back().path.poses.erase(global_paths_.paths.back().path.poses.begin(),global_paths_.paths.back().path.poses.begin()+index);
 //     else
 //       global_paths_.paths.back().path.poses.erase(global_paths_.paths.back().path.poses.begin());
 // }

};
