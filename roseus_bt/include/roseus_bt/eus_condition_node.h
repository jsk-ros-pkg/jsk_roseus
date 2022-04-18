#ifndef BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_
#define BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_

#include <behaviortree_ros/bt_service_node.h>

namespace BT
{

template<class ServiceT>
class EusConditionNode : public BT::RosServiceNode<ServiceT>
{
protected:
 EusConditionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf): BT::RosServiceNode<ServiceT>(nh, name, conf) {
   const unsigned msec = BT::TreeNode::getInput<unsigned>("timeout").value();
   const std::string server_name = BT::TreeNode::getInput<std::string>("service_name").value();
   ros::Duration timeout(static_cast<double>(msec) * 1e-3);
   BT::RosServiceNode<ServiceT>::service_client_ = nh.serviceClient<ServiceT>( server_name, true );

   ROS_DEBUG("Connecting to service server at '%s'...", server_name.c_str());
   bool connected = BT::RosServiceNode<ServiceT>::service_client_.waitForExistence(timeout);
   if(!connected){
     throw BT::RuntimeError("Couldn't connect to service server at: ", server_name);
   }
 };

public:
  EusConditionNode() = delete;
  virtual ~EusConditionNode() = default;
};

}  // namespace BT

#endif  // BEHAVIOR_TREE_EUS_CONDITION_NODE_HPP_
