#include "humans_nav/morse_humans_provider.h"

using namespace std;
using namespace uwds_msgs;

namespace humans_nav
{
  void MorseHumansProvider::onInit()
  {
    UwdsClientNodelet::onInit();
    pnh_->param<string>("output_world", output_world_, "morse_humans");
    input_subscriber_ = nh_->subscribe("humans", 1, &MorseHumansProvider::callback, this);

    for(int i=1;i<=3;i++){
      Node node;
      string name = "human" + to_string(i);
      node.name = name;
      node.id = NEW_UUID;
      human_node.push_back(node);
    }
  }

  void MorseHumansProvider::callback(const humans_msgs::HumanArrayConstPtr& msg)
  {
    if(msg->humans.size()>0)
    {
      Changes changes;
      int i = 0;
      for (const auto& human : msg->humans)
      {
        human_node[i].position.pose = human.pose;
        human_node[i].velocity.twist = human.velocity;
        human_node[i].last_observation.data = ros::Time::now();
        changes.nodes_to_update.push_back(human_node[i]);
        i++;
      }
      ctx_->worlds()[output_world_].update(msg->header, changes);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(humans_nav::MorseHumansProvider, nodelet::Nodelet)
