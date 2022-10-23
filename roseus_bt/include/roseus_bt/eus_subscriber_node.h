#ifndef BEHAVIOR_TREE_ROSEUS_BT_EUS_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_ROSEUS_BT_EUS_SUBSCRIBER_NODE_HPP_


namespace BT
{

template<typename MessageT>
class EusSubscriberNode: public BT::ActionNodeBase
{
protected:
  EusSubscriberNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf):
    BT::ActionNodeBase(name, conf), node_(nh)
  {
    const std::string topic_name = getInput<std::string>("topic_name").value();
    sub_ = node_.subscribe(topic_name, 1000, &EusSubscriberNode::callback, this);
  }

public:

  using MessageType = MessageT;

  EusSubscriberNode() = delete;
  virtual ~EusSubscriberNode() = default;

  static PortsList providedPorts() {
    return {
      InputPort<std::string>("topic_name", "name of the subscribed topic"),
      OutputPort<MessageT>("output_port", "port to where messages are redirected"),
      OutputPort<uint8_t>("received_port", "port set to true every time a message is received")
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
  ros::NodeHandle& node_;
  ros::Subscriber sub_;

protected:
  virtual void callback(MessageT msg) {
    setOutput("output_port", msg);
    setOutput("received_port", (uint8_t)true);
  }

};


/// Binds the ros::NodeHandle and register to the BehaviorTreeFactory
template <class DerivedT> static
  void RegisterSubscriberNode(BT::BehaviorTreeFactory& factory,
                             const std::string& registration_ID,
                             ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = EusSubscriberNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_EUS_SUBSCRIBER_NODE_HPP_