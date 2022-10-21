#ifndef BEHAVIOR_TREE_ROSEUS_BT_EUS_REMOTE_CONDITION_NODE_HPP_
#define BEHAVIOR_TREE_ROSEUS_BT_EUS_REMOTE_CONDITION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <roseus_bt/ws_service_client.h>


namespace BT
{

// Helper Node to call a rosbridge websocket inside a BT::ActionNode
template<class ServiceT>
class EusRemoteConditionNode : public BT::ActionNodeBase
{
protected:

  EusRemoteConditionNode(const std::string& name, const BT::NodeConfiguration & conf):
    BT::ActionNodeBase(name, conf),
    service_client_(getInput<std::string>("host_name").value(),
                    getInput<int>("host_port").value(),
                    getInput<std::string>("service_name").value())
  {}

public:

  using BaseClass   = EusRemoteConditionNode<ServiceT>;
  using ServiceType = ServiceT;
  using RequestType = typename ServiceT::Request;

  EusRemoteConditionNode() = delete;
  virtual ~EusRemoteConditionNode() = default;

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("service_name", "name of the ROS service"),
      InputPort<std::string>("host_name", "name of the rosbridge_server host"),
      InputPort<int>("host_port", "port of the rosbridge_server host")
      };
  }

  virtual void sendRequest(rapidjson::Document *request) = 0;
  virtual NodeStatus onResponse(const rapidjson::Value& result) = 0;

  // enum FailureCause{
  //   MISSING_SERVER = 0,
  //   FAILED_CALL = 1
  // };

  // virtual NodeStatus onFailedRequest(FailureCause failure)
  // {
  //   return NodeStatus::FAILURE;
  // }

  virtual void halt() override
  {
    if (service_client_.isActive()) {
      service_client_.cancelRequest();
    }
    setStatus(NodeStatus::IDLE);
    service_client_.waitForResult();
  }

protected:
  roseus_bt::RosbridgeServiceClient service_client_;

  BT::NodeStatus tick() override
  {
    if (BT::TreeNode::status() == BT::NodeStatus::IDLE) {
      BT::TreeNode::setStatus(BT::NodeStatus::RUNNING);

      rapidjson::Document request;
      request.SetObject();
      sendRequest(&request);
      service_client_.call(request);
    }

    // Conditions cannot operate asynchronously
    service_client_.waitForResult();

    return onResponse(service_client_.getResult());
  }
};


/// Method to register the service into a factory.
template <class DerivedT> static
  void RegisterRemoteCondition(BT::BehaviorTreeFactory& factory,
                               const std::string& registration_ID)
{
  NodeBuilder builder = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = EusRemoteConditionNode<typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_EUS_REMOTE_CONDITION_NODE_HPP_
