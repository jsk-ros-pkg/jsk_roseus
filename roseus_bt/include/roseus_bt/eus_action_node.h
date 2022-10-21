#ifndef BEHAVIOR_TREE_ROSEUS_BT_EUS_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_ROSEUS_BT_EUS_ACTION_NODE_HPP_

#include <behaviortree_ros/bt_action_node.h>

namespace BT
{

/**
 * Include feedback callback to BT::RosActionNode
 * Note that the user must implement the additional onFeedback method
 *
 */
template<class ActionT>
class EusActionNode: public BT::RosActionNode<ActionT>
{
protected:
  EusActionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf):
  BT::RosActionNode<ActionT>(nh, name, conf) {
    // check connection at init time
    const unsigned msec = BT::TreeNode::getInput<unsigned>("timeout").value();
    const std::string server_name = BT::TreeNode::getInput<std::string>("server_name").value();
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    ROS_DEBUG("Connecting to action server at '%s'...", server_name.c_str());
    bool connected = BT::RosActionNode<ActionT>::action_client_->waitForServer(timeout);
    if (!connected) {
      throw BT::RuntimeError("Couldn't connect to action server at: ", server_name);
    }
  };

public:
  using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type;
  using Client = actionlib::SimpleActionClient<ActionT>;

  EusActionNode() = delete;
  virtual ~EusActionNode() = default;

  virtual bool sendGoal(typename BT::RosActionNode<ActionT>::GoalType& goal) = 0;
  virtual NodeStatus onResult( const typename BT::RosActionNode<ActionT>::ResultType& res) = 0;
  virtual void onFeedback( const typename FeedbackType::ConstPtr& feedback) = 0;

  virtual void halt() override
  {
    BT::RosActionNode<ActionT>::halt();
    BT::RosActionNode<ActionT>::action_client_->waitForResult();
  }

protected:
 /**
  *  Override Behaviortree.ROS definition to pass the feedback callback
  *
  */
  BT::NodeStatus tick() override
  {
    unsigned msec = BT::TreeNode::getInput<unsigned>("timeout").value();
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    bool connected = BT::RosActionNode<ActionT>::action_client_->waitForServer(timeout);
    if( !connected ){
      return BT::RosActionNode<ActionT>::onFailedRequest(BT::RosActionNode<ActionT>::MISSING_SERVER);
    }

    // first step to be done only at the beginning of the Action
    if (BT::TreeNode::status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      BT::TreeNode::setStatus(BT::NodeStatus::RUNNING);

      typename BT::RosActionNode<ActionT>::GoalType goal;
      bool valid_goal = sendGoal(goal);
      if( !valid_goal )
      {
        return NodeStatus::FAILURE;
      }
      BT::RosActionNode<ActionT>::action_client_->sendGoal(
        goal,
        NULL,  // donecb
        NULL,  // activecb
        boost::bind(&EusActionNode::onFeedback, this, _1));
    }

    // RUNNING
    auto action_state = BT::RosActionNode<ActionT>::action_client_->getState();

    // Please refer to these states

    if( action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE )
    {
      return NodeStatus::RUNNING;
    }
    else if( action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return onResult( *BT::RosActionNode<ActionT>::action_client_->getResult() );
    }
    else if( action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return BT::RosActionNode<ActionT>::onFailedRequest( BT::RosActionNode<ActionT>::ABORTED_BY_SERVER );
    }
    else if( action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return BT::RosActionNode<ActionT>::onFailedRequest( BT::RosActionNode<ActionT>::REJECTED_BY_SERVER );
    }
    else
    {
      // FIXME: is there any other valid state we should consider?
      throw std::logic_error("Unexpected state in RosActionNode::tick()");
    }
  }
};

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_EUS_ACTION_NODE_HPP_
