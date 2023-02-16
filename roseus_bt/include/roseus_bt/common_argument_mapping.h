#ifndef BEHAVIOR_TREE_ROSEUS_BT_COMMON_ARGUMENT_MAPPING_
#define BEHAVIOR_TREE_ROSEUS_BT_COMMON_ARGUMENT_MAPPING_

#include <iostream>
#include <map>
#include <behaviortree_cpp_v3/bt_factory.h>


namespace roseus_bt
{

void register_blackboard_variables(BT::Tree* tree,
                                   const std::map<std::string, std::string>& argument_map)
{
  // New variables are registered as strings.
  // However, if the port has already been registered
  // in the tree, it can be default-casted.
  for (const auto it: argument_map) {
#ifdef DEBUG
    std::cout << "Initializing blackboard variable: " << it.first <<
      " with value: " << it.second << "\n";
#endif
    tree->rootBlackboard()->set<std::string>(it.first, it.second);
  }
}

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_COMMON_ARGUMENT_MAPPING_
