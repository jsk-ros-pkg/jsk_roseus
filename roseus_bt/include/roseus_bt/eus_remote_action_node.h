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

  EusRemoteActionNode(const std::string message_type, const std::string& name, const BT::NodeConfiguration & conf):
    BT::ActionNodeBase(name, conf),
    action_client_(getInput<std::string>("host_name").value(),
                   getInput<int>("host_port").value(),
                   getInput<std::string>("server_name").value(),
                   message_type)
  {
    auto cb = std::bind(&EusRemoteActionNode::feedbackCallback, this,
                        std::placeholders::_1,
                        std::placeholders::_2);
    action_client_.registerFeedbackCallback(cb);
  }

public:

  EusRemoteActionNode() = delete;

  virtual ~EusRemoteActionNode() = default;

  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "name of the Action Server"),
      InputPort<std::string>("host_name", "name of the rosbridge_server host"),
      InputPort<int>("host_port", "port of the rosbridge_server host")
      };
  }

  virtual bool sendGoal(rapidjson::Document *goal) = 0;

  virtual NodeStatus onResult(const rapidjson::Value& res) = 0;
  virtual void onFeedback(const rapidjson::Value& feedback) = 0;

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
      bool valid_goal = sendGoal(&goal);
      if( !valid_goal )
      {
        return NodeStatus::FAILURE;
      }

      action_client_.sendGoal(goal);
    }

    if (action_client_.isActive()) {
      return NodeStatus::RUNNING;
    }

    // TODO: check slots and raise errors
    // throw BT::RuntimeError("EusRemoteActionNode: ActionResult is not set properly");

    return onResult( action_client_.getResult() );
  }

  // port related
  // TODO: avoid unnecessary dumping & parsing
  // cannot use rapidjson::Value directly because it is not `CopyConstructible'
  // TODO: translate to ROS message: how to loop through slots?

  void feedbackCallback(std::shared_ptr<WsClient::Connection> connection,
                        std::shared_ptr<WsClient::InMessage> in_message)
  {
    std::string message = in_message->string();
    std::cout << "feedbackCallback(): Message Received: " << message << std::endl;

    rapidjson::Document document, feedbackMessage;
    document.Parse(message.c_str());
    feedbackMessage.Swap(document["msg"]["feedback"]);

    onFeedback(feedbackMessage);
  }

  void setOutputFromMessage(const std::string& name, const rapidjson::Value& message)
  {
    rapidjson::StringBuffer strbuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
    rapidjson::Document document;
    document.CopyFrom(message[name.c_str()], document.GetAllocator());
    document.Accept(writer);
    setOutput(name, strbuf.GetString());
  }
};


/// Method to register the service into a factory.
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
