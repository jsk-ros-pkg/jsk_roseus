#ifndef BEHAVIOR_TREE_ROSEUS_BT_ROSPARAM_ARGUMENT_MAPPING_
#define BEHAVIOR_TREE_ROSEUS_BT_ROSPARAM_ARGUMENT_MAPPING_

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <fmt/format.h>
#include <boost/program_options.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpc.h>

#include <roseus_bt/common_argument_mapping.h>

namespace roseus_bt
{

bool parse_rosparam(ros::NodeHandle pnh, std::map<std::string, std::string>& argument_map)
{
  // search ~node_name/parameter_name
  std::vector<std::string> param_names;
  std::string n_name = pnh.resolveName("") + "/";
  pnh.getParamNames(param_names);

  for (int i = 0; i < param_names.size(); ++i)
  {
    std::string p_name = param_names[i];
    XmlRpc::XmlRpcValue p_value;
    if (!(p_name.size() > n_name.size() &&
            std::equal(std::begin(n_name), std::end(n_name), std::begin(p_name))))
    {
      continue;
    }
    bool is_get = pnh.getParam(p_name, p_value);
    if (!is_get) return false;
    p_name = p_name.substr(n_name.size());
    if (p_value.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      argument_map.insert({p_name, std::string(p_value)});
    }
    else if (p_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      argument_map.insert({p_name, std::to_string((int)p_value)});
    }
    else if (p_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      argument_map.insert({p_name, std::to_string((double)p_value)});
    }
    else if (p_value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      argument_map.insert({p_name, std::to_string((bool)p_value)});
    }
    else if (p_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      std::cout << "Array param is not supported, so skipped!\n";
      std::cout <<  p_name << "\n";
      continue;
    }
    else
    {
      std::cerr << "Unknown param type!\n";
      std::cerr <<  p_name << "\n";
      return false;
    }
  }
  return true;
}

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_ROSPARAM_ARGUMENT_MAPPING_
