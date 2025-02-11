#ifndef BEHAVIOR_TREE_ROSEUS_BT_EUS_NODE_TYPE_HPP_
#define BEHAVIOR_TREE_ROSEUS_BT_EUS_NODE_TYPE_HPP_

#include <behaviortree_cpp_v3/basic_types.h>


namespace roseus_bt
{

enum class NodeType
{
    UNDEFINED = 0,
    ACTION,
    CONDITION,
    CONTROL,
    DECORATOR,
    SUBTREE,
    SUBSCRIBER,
    REMOTE_ACTION,
    REMOTE_CONDITION,
    REMOTE_SUBSCRIBER,
};

std::string toStr(NodeType type);
NodeType convertFromString(BT::StringView str);
std::ostream& operator<<(std::ostream& os, const NodeType& type);

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_EUS_NODE_TYPE_HPP_
