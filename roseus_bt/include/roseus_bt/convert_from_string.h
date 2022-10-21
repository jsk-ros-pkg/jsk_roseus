#ifndef BEHAVIOR_TREE_ROSEUS_BT_CONVERT_FROM_STRING_
#define BEHAVIOR_TREE_ROSEUS_BT_CONVERT_FROM_STRING_

#include <boost/lexical_cast.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>


namespace BT
{

template <> short convertFromString(StringView str) {
  return boost::lexical_cast<short>(str);
}

template <> unsigned short convertFromString(StringView str) {
  return boost::lexical_cast<unsigned short>(str);
}

template <> signed char convertFromString(StringView str) {
  // explicitly convert to numbers, not characters
  signed char result = boost::lexical_cast<int>(str);
  return result;
}

template <> unsigned char convertFromString(StringView str) {
  if (str == "true" || str == "True") {
    return true;
  }
  if (str == "false" || str == "False") {
    return false;
  }
  // explicitly convert to numbers, not characters
  unsigned char result = boost::lexical_cast<unsigned int>(str);
  return result;
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_CONVERT_FROM_STRING_
