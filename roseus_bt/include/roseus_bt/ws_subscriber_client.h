#ifndef BEHAVIOR_TREE_ROSEUS_BT_WS_SUBSCRIBER_CLIENT_
#define BEHAVIOR_TREE_ROSEUS_BT_WS_SUBSCRIBER_CLIENT_

#include <rosbridgecpp/rosbridge_ws_client.hpp>
#include <fmt/format.h>


namespace roseus_bt
{

class RosbridgeSubscriberClient
{
public:
  RosbridgeSubscriberClient(const std::string& master, int port,
                            const std::string& topic_name,
                            const std::string& topic_type):
    rbc_(fmt::format("{}:{}", master, std::to_string(port))),
    topic_name_(topic_name),
    topic_type_(topic_type)
  {
    rbc_.addClient("topic_subscriber");
  }

  ~RosbridgeSubscriberClient() {
    rbc_.removeClient("topic_subscriber");
  }

  void registerCallback(auto callback) {
    rbc_.subscribe("topic_subscriber", topic_name_, callback, "", topic_type_);
  }

protected:
  RosbridgeWsClient rbc_;
  std::string topic_name_;
  std::string topic_type_;
};

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_WS_SUBSCRIBER_CLIENT_
