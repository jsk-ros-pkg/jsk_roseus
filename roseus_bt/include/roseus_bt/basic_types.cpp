#include "roseus_bt/basic_types.h"

namespace roseus_bt
{

std::string toStr(NodeType type)
{
    switch (type)
    {
        case NodeType::ACTION:
            return "Action";
        case NodeType::CONDITION:
            return "Condition";
        case NodeType::DECORATOR:
            return "Decorator";
        case NodeType::CONTROL:
            return "Control";
        case NodeType::SUBTREE:
            return "SubTree";
        case NodeType::SUBSCRIBER:
            return "Subscriber";
        case NodeType::REMOTE_SUBSCRIBER:
            return "RemoteSubscriber";
        case NodeType::REMOTE_ACTION:
            return "RemoteAction";
        case NodeType::REMOTE_CONDITION:
            return "RemoteCondition";
        default:
            return "Undefined";
    }
}

NodeType convertFromString(BT::StringView str)
{
    if( str == "Action" )    return NodeType::ACTION;
    if( str == "Condition" ) return NodeType::CONDITION;
    if( str == "Control" )   return NodeType::CONTROL;
    if( str == "Decorator" ) return NodeType::DECORATOR;
    if( str == "SubTree" || str == "SubTreePlus" ) return NodeType::SUBTREE;
    if( str == "Subscriber" )      return NodeType::SUBSCRIBER;
    if( str == "RemoteAction" )    return NodeType::REMOTE_ACTION;
    if( str == "RemoteCondition" ) return NodeType::REMOTE_CONDITION;
    if( str == "RemoteSubscriber" )  return NodeType::REMOTE_SUBSCRIBER;
    return NodeType::UNDEFINED;
}

std::ostream& operator<<(std::ostream& os, const NodeType& type)
{
    os << toStr(type);
    return os;
}


}  // namespace roseus_bt
