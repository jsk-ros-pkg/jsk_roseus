#ifndef BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_
#define BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_

#include <behaviortree_ros/bt_service_node.h>

namespace BT
{

template<class ServiceT>
class EusConditionNode : public BT::RosServiceNode<ServiceT>
{
protected:
 EusConditionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf): BT::RosServiceNode<ServiceT>(nh, name, conf) {};

public:
  EusConditionNode() = delete;
  virtual ~EusConditionNode() = default;
};

}  // namespace BT

#endif  // BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_
