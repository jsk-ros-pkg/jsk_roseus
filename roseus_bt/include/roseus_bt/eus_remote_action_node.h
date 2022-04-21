#ifndef BEHAVIOR_TREE_EUS_REMOTE_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_EUS_REMOTE_ACTION_NODE_HPP_
#define DEBUG

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <roseus_bt/ws_action_client.h>


namespace BT
{

// Helper Node to call a rosbridge websocket inside a BT::ActionNode
class EusRemoteActionNode : public BT::ActionNodeBase
{
protected:

  EusRemoteActionNode(const std::string& master, int port, const std::string message_type, const std::string& name, const BT::NodeConfiguration & conf): 
    BT::ActionNodeBase(name, conf),
    action_client_(master, port, getInput<std::string>("server_name").value(), message_type)
  {}

public:

  EusRemoteActionNode() = delete;

  virtual ~EusRemoteActionNode() = default;

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "name of the Action Server")
      };
  }

  virtual bool sendGoal(rapidjson::Document& goal) = 0;

  virtual NodeStatus onResult(const rapidjson::Value& res) = 0;

  // virtual NodeStatus onFailedRequest(FailureCause failure)
  // {
  //   return NodeStatus::FAILURE;
  // }

  virtual void halt() override
  {
    if (action_client_.isActive()) {
      action_client_.cancelGoal();
    }
    setStatus(NodeStatus::IDLE);
    action_client_.waitForResult();
  }

protected:
  RosbridgeActionClient action_client_;

  std::string server_name_;
  std::string goal_topic_;
  std::string feedback_topic_;
  std::string result_topic_;

  BT::NodeStatus tick() override
  {
    if (BT::TreeNode::status() == BT::NodeStatus::IDLE) {
      BT::TreeNode::setStatus(BT::NodeStatus::RUNNING);

      rapidjson::Document goal;
      goal.SetObject();
      bool valid_goal = sendGoal(goal);
      if( !valid_goal )
      {
        return NodeStatus::FAILURE;
      }

      action_client_.sendGoal(goal);
    }

    // TODO: wait for result
    // TODO: track feedback
    if (action_client_.isActive()) {
      return NodeStatus::RUNNING;
    }

    // TODO: check slots and raise errors
    // throw BT::RuntimeError("EusRemoteActionNode: ActionResult is not set properly");

    return onResult( action_client_.getResult() );
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
  template <class DerivedT> static
  void RegisterRemoteAction(BT::BehaviorTreeFactory& factory,
                            const std::string& registration_ID)
{
  NodeBuilder builder = [](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = EusRemoteActionNode::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT


#endif  // BEHAVIOR_TREE_BT_REMOTE_ACTION_NODE_HPP_
