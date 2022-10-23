#ifndef BEHAVIOR_TREE_EUS_REMOTE_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_EUS_REMOTE_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <roseus_bt/ws_subscriber_client.h>
#include <roseus_bt/copy_document.h>

namespace BT
{

template<typename MessageT>
class EusRemoteSubscriberNode: public BT::ActionNodeBase
{
protected:
    EusRemoteSubscriberNode(const std::string& name, const BT::NodeConfiguration& conf):
    BT::ActionNodeBase(name, conf),
    subscriber_client_(getInput<std::string>("host_name").value(),
                       getInput<int>("host_port").value(),
                       getInput<std::string>("topic_name").value(),
                       getInput<std::string>("message_type").value())
  {
    auto cb = std::bind(&EusRemoteSubscriberNode::topicCallback, this,
                        std::placeholders::_1,
                        std::placeholders::_2);
    subscriber_client_.registerCallback(cb);
  }

public:

  using MessageType = MessageT;

  EusRemoteSubscriberNode() = delete;
  virtual ~EusRemoteSubscriberNode() = default;

  static PortsList providedPorts() {
    return {
      InputPort<std::string>("topic_name", "name of the subscribed topic"),
      OutputPort<rapidjson::CopyDocument>("output_port", "port to where messages are redirected"),
      OutputPort<uint8_t>("received_port", "port set to true every time a message is received"),
      InputPort<std::string>("host_name", "name of the rosbridge_server host"),
      InputPort<int>("host_port", "port of the rosbridge_server host")
        };
  }

  virtual BT::NodeStatus tick() override final
  {
    return NodeStatus::SUCCESS;
  }

  virtual void halt() override final
  {
    setStatus(NodeStatus::IDLE);
  }

protected:
  roseus_bt::RosbridgeSubscriberClient subscriber_client_;

protected:
  virtual void callback(const rapidjson::Value& msg) {
    setOutputFromMessage("output_port", msg);
    setOutput("received_port", (uint8_t)true);
  }

  void topicCallback(std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message)
  {
    std::string message = in_message->string();
#ifdef DEBUG
    std::cout << "topicCallback(): Message Received: " << message << std::endl;
#endif

    rapidjson::Document document, topicMessage;
    document.Parse(message.c_str());
    topicMessage.Swap(document["msg"]);

    callback(topicMessage);
  }

  void setOutputFromMessage(const std::string& name, const rapidjson::Value& message)
  {
    rapidjson::CopyDocument document;
    document.CopyFrom(message, document.GetAllocator());
    setOutput(name, document);
  }
};


/// Method to register the subscriber into a factory.
template <class DerivedT> static
  void RegisterRemoteSubscriber(BT::BehaviorTreeFactory& factory,
                                const std::string& registration_ID)
{
  NodeBuilder builder = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = EusRemoteSubscriberNode<typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_EUS_REMOTE_SUBSCRIBER_NODE_HPP_
