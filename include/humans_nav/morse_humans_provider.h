#ifndef MORSE_HUMANS_PROVIDER_HPP
#define MORSE_HUMANS_PROVIDER_HPP

#include <uwds/uwds.h>
#include <uwds/uwds_client_nodelet.h>
#include <uwds/tools/model_loader.h>
#include <humans_msgs/HumanMarker.h>
#include <humans_msgs/HumanArray.h>
#include <humans_msgs/TwistArray.h>

using namespace uwds_msgs;
using namespace uwds;

namespace humans_nav
{
  class MorseHumansProvider : public uwds::UwdsClientNodelet
  {
    public:
      /**@brief
       * The constructor.
       */
      MorseHumansProvider(): uwds::UwdsClientNodelet(uwds::PROVIDER) {}

      /**@brief
       * The default destructor
       */
      ~MorseHumansProvider() = default;

      /** @brief
       * Initialize method. Subclass should call this method
       * in its onInit method.
       */
      virtual void onInit();

    protected:
      /**@brief
       * This method is called when perception data are received.
       */
      void callback(const humans_msgs::HumanArrayConstPtr& msg);
      /**@brief
       * Input subscriber for perception data.
       */
      std::vector<Node> human_node;

      std::string output_world_;

      ros::Subscriber input_subscriber_;
  };
}

#endif
