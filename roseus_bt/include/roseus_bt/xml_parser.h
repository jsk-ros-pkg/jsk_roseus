#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_

#include <map>
#include <string>
#include <vector>
#include <tinyxml2.h>
#include <fmt/format.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/join.hpp>

namespace RoseusBT
{
  using namespace tinyxml2;


class XMLParser
{

public:

  XMLParser(const char* filename) : xml_filename(filename) {
    doc.LoadFile(filename);
  }

  ~XMLParser() {};

protected:

  XMLDocument doc;
  const char* xml_filename;
  std::string port_node_to_message_description(const XMLElement* port_node);
  std::string generate_action_file_contents(const XMLElement* node);
  std::string generate_service_file_contents(const XMLElement* node);
  std::string generate_headers(const char* package_name);
  std::string generate_action_class(const XMLElement* node, const char* package_name);
  std::string generate_condition_class(const XMLElement* node, const char* package_name);
  std::string generate_main_function(const char* roscpp_node_name);

public:

  std::map<std::string, std::string> generate_all_action_files();
  std::map<std::string, std::string> generate_all_service_files();
  std::string generate_cpp_file(const char* package_name,
                                const char* roscpp_node_name);
  std::string generate_cmake_lists(const char* package_name,
                                   const char* target_filename);
  std::string generate_package_xml(const char* package_name,
                                   const char* author_name);

};


std::string XMLParser::port_node_to_message_description(const XMLElement* port_node) {
  if (!port_node->Attribute("type") ||
      !port_node->Attribute("name")) {
    std::string error_str = "Illformed port in ";
    error_str.append(port_node->Name());
    throw std::logic_error(error_str);
  }

  std::string output = fmt::format("{} {}",
                                   port_node->Attribute("type"),
                                   port_node->Attribute("name"));
  return output;
}

std::string XMLParser::generate_action_file_contents(const XMLElement* node) {
  std::vector<std::string> goal, feedback;

  for (auto port_node = node->FirstChildElement();
       port_node != nullptr;
       port_node = port_node->NextSiblingElement())
    {
      std::string name = port_node->Name();
      std::string text = port_node_to_message_description(port_node);

      if (name == "input_port" || name == "inout_port") {
        goal.push_back(text);
      }
      if (name == "output_port" || name == "inout_port") {
        feedback.push_back(text);
      }
    }

  std::string output;
  output.append(boost::algorithm::join(goal, "\n"));
  output.append("\n---\n");
  output.append("bool success");
  output.append("\n---\n");
  output.append(boost::algorithm::join(feedback, "\n"));

  return output;
}

std::string XMLParser::generate_service_file_contents(const XMLElement* node) {
  std::vector<std::string> request;

  for (auto port_node = node->FirstChildElement();
       port_node != nullptr;
       port_node = port_node->NextSiblingElement())
    {
      std::string name = port_node->Name();
      std::string text = port_node_to_message_description(port_node);

      if (name == "input_port") {
        request.push_back(text);
      }
      else {
        throw std::logic_error("Condition Node only accepts input ports!");
      }
    }

  std::string output;
  output.append(boost::algorithm::join(request, "\n"));
  output.append("\n---\n");
  output.append("bool success");

  return output;
}

std::string XMLParser::generate_headers(const char* package_name) {
  auto format_action_node = [](const XMLElement* node, const char* package_name) {
    return fmt::format("#include <{}/{}Action.h>",
                       package_name,
                       node->Attribute("ID"));
  };

  auto format_condition_node = [](const XMLElement* node, const char* package_name) {
    return fmt::format("#include <{}/{}.h>",
                       package_name,
                       node->Attribute("ID"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> headers;

  std::string common_headers = 1 + R"(
#include <ros/ros.h>

#include <roseus_bt/eus_action_node.h>
#include <roseus_bt/eus_condition_node.h>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

)";

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      headers.push_back(format_action_node(action_node, package_name));
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      headers.push_back(format_condition_node(condition_node, package_name));
    }

  std::string output;
  output.append(common_headers);
  output.append(boost::algorithm::join(headers, "\n"));
  output.append("\n\n");
  output.append("using namespace BT;");
  return output;
}

std::string XMLParser::generate_action_class(const XMLElement* node, const char* package_name) {
  auto format_input_port = [](const XMLElement* node) {
    return fmt::format("      InputPort<GoalType::_{0}_type>(\"{0}\")",
                       node->Attribute("name"));
  };
  auto format_output_port = [](const XMLElement* node) {
    return fmt::format("      OutputPort<FeedbackType::_{0}_type>(\"{0}\")",
                       node->Attribute("name"));
  };
  auto format_get_input = [](const XMLElement* node) {
    return fmt::format("    getInput(\"{0}\", goal.{0});",
                       node->Attribute("name"));
  };
  auto format_set_output = [](const XMLElement* node) {
    return fmt::format("    setOutput(\"{0}\", feedback->{0});",
                       node->Attribute("name"));
  };


  std::vector<std::string> provided_input_ports;
  std::vector<std::string> provided_output_ports;
  std::vector<std::string> get_inputs;
  std::vector<std::string> set_outputs;

  for (auto port_node = node->FirstChildElement();
       port_node != nullptr;
       port_node = port_node->NextSiblingElement())
    {
      std::string name = port_node->Name();
      if (name == "input_port" || name == "inout_port") {
        provided_input_ports.push_back(format_input_port(port_node));
        get_inputs.push_back(format_get_input(port_node));
      }
      if (name == "output_port" || name == "inout_port") {
        provided_output_ports.push_back(format_output_port(port_node));
        set_outputs.push_back(format_set_output(port_node));
      }
    }

  std::vector<std::string> provided_ports;
  provided_ports.insert(provided_ports.end(),
                        provided_input_ports.begin(),
                        provided_input_ports.end());
  provided_ports.insert(provided_ports.end(),
                        provided_output_ports.begin(),
                        provided_output_ports.end());


  std::string fmt_string = 1 + R"(
class %2%: public EusActionNode<%1%::%2%Action>
{

public:
  %2%(ros::NodeHandle& handle, const std::string& name, const NodeConfiguration& conf):
EusActionNode<%1%::%2%Action>(handle, name, conf) {}

  static PortsList providedPorts()
  {
    return  {
%3%
    };
  }

  bool sendGoal(GoalType& goal) override
  {
%4%
    return true;
  }

  void onFeedback( const typename FeedbackType::ConstPtr& feedback) override
  {
%5%
    return;
  }

  NodeStatus onResult( const ResultType& result) override
  {
    if (result.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    return NodeStatus::FAILURE;
  }

};
)";
  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    node->Attribute("ID") %
    boost::algorithm::join(provided_ports, ",\n") %
    boost::algorithm::join(get_inputs, "\n") %
    boost::algorithm::join(set_outputs, "\n");

  return bfmt.str();
}

std::string XMLParser::generate_condition_class(const XMLElement* node, const char* package_name) {
  auto format_input_port = [](const XMLElement* node) {
    return fmt::format("      InputPort<RequestType::_{0}_type>(\"{0}\")",
                       node->Attribute("name"));
  };
  auto format_get_input = [](const XMLElement* node) {
    return fmt::format("    getInput(\"{0}\", request.{0});",
                       node->Attribute("name"));
  };

  std::vector<std::string> provided_ports;
  std::vector<std::string> get_inputs;

  for (auto port_node = node->FirstChildElement("input_port");
       port_node != nullptr;
       port_node = port_node->NextSiblingElement("input_port"))
    {
      provided_ports.push_back(format_input_port(port_node));
      get_inputs.push_back(format_get_input(port_node));
    }

  std::string fmt_string = 1 + R"(
class %2%: public EusConditionNode<%1%::%2%>
{

public:
  %2%(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration& conf):
  EusConditionNode<%1%::%2%>(handle, node_name, conf) {}

  static PortsList providedPorts()
  {
    return  {
%3%
    };
  }

  void sendRequest(RequestType& request) override
  {
%4%
  }

  NodeStatus onResponse(const ResponseType& res) override
  {
    if (res.success) return NodeStatus::SUCCESS;
    return NodeStatus::FAILURE;
  }

  virtual NodeStatus onFailedRequest(FailureCause failure) override
  {
    return NodeStatus::FAILURE;
  }

};
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    node->Attribute("ID") %
    boost::algorithm::join(provided_ports, ",\n") %
    boost::algorithm::join(get_inputs, "\n");

  return bfmt.str();
}

std::string XMLParser::generate_main_function(const char* roscpp_node_name) {
  auto format_ros_init = [](const char* roscpp_node_name) {
    return fmt::format("  ros::init(argc, argv, \"{}\");", roscpp_node_name);
  };
  auto format_create_tree = [](const char* xml_filename) {
    return fmt::format("  auto tree = factory.createTreeFromFile(\"{}\");", xml_filename);
  };
  auto format_action_node = [](const XMLElement* node) {
    return fmt::format("  RegisterRosAction<{0}>(factory, \"{0}\", nh);",
                       node->Attribute("ID"));
  };
  auto format_condition_node = [](const XMLElement* node) {
    return fmt::format("  RegisterRosService<{0}>(factory, \"{0}\", nh);",
                       node->Attribute("ID"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> register_actions;
  std::vector<std::string> register_conditions;

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      register_actions.push_back(format_action_node(action_node));
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      register_conditions.push_back(format_condition_node(condition_node));
    }

  std::string fmt_string = 1 + R"(
int main(int argc, char **argv)
{
%1%
  ros::NodeHandle nh;

  BehaviorTreeFactory factory;

%3%
%4%

%2%

  StdCoutLogger logger_cout(tree);
  PublisherZMQ publisher_zmq(tree);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.05);
    sleep_time.sleep();
  }

  return 0;
}
)";

  boost::format bfmt = boost::format(fmt_string) %
    format_ros_init(roscpp_node_name) %
    format_create_tree(xml_filename) %
    boost::algorithm::join(register_actions, "\n") %
    boost::algorithm::join(register_conditions, "\n");

  return bfmt.str();
}


std::map<std::string, std::string> XMLParser::generate_all_action_files() {
  auto format_filename = [](const XMLElement* node) {
    return fmt::format("{}.action", node->Attribute("ID"));
  };

  std::map<std::string, std::string> result;
  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      result[format_filename(action_node)] = generate_action_file_contents(action_node);
    }
  return result;
}

std::map<std::string, std::string> XMLParser::generate_all_service_files() {
  auto format_filename = [](const XMLElement* node) {
    return fmt::format("{}.srv", node->Attribute("ID"));
  };

  std::map<std::string, std::string> result;
  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      result[format_filename(condition_node)] = generate_service_file_contents(condition_node);
    }
  return result;
}

std::string XMLParser::generate_cpp_file(const char* package_name,
                                         const char* roscpp_node_name) {

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::string output;
  output.append(generate_headers(package_name));
  output.append("\n\n");

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      output.append(generate_action_class(action_node, package_name));
      output.append("\n\n");
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      output.append(generate_condition_class(condition_node, package_name));
      output.append("\n\n");
    }

  output.append(generate_main_function(roscpp_node_name));

  return output;
}

std::string XMLParser::generate_cmake_lists(const char* package_name, const char* target_name) {
  auto format_pkg = [](const char* pkg) {
    return fmt::format("  {}", pkg);
  };
  auto format_action_file = [](const XMLElement* node) {
    return fmt::format("  {}.action", node->Attribute("ID"));
  };
  auto format_service_file = [](const XMLElement* node) {
    return fmt::format("  {}.srv", node->Attribute("ID"));
  };
  auto maybe_push_message_package = [format_pkg](const XMLElement* node, std::vector<std::string>* message_packages) {
    std::string msg_type = node->Attribute("type");
    std::size_t pos = msg_type.find('/');
    if (pos != std::string::npos) {
      std::string pkg = format_pkg(msg_type.substr(0, pos).c_str());
      if (std::find(message_packages->begin(), message_packages->end(), pkg) ==
          message_packages->end()) {
        message_packages->push_back(pkg);
      }
    }
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> message_packages;
  std::vector<std::string> action_files;
  std::vector<std::string> service_files;

  message_packages.push_back(format_pkg("std_msgs"));
  message_packages.push_back(format_pkg("actionlib_msgs"));
  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      action_files.push_back(format_action_file(action_node));
      for (auto port_node = action_node->FirstChildElement();
           port_node != nullptr;
           port_node = port_node->NextSiblingElement())
        {
          maybe_push_message_package(port_node, &message_packages);
        }
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      service_files.push_back(format_service_file(condition_node));
      for (auto port_node = condition_node->FirstChildElement();
           port_node != nullptr;
           port_node = port_node->NextSiblingElement())
        {
          maybe_push_message_package(port_node, &message_packages);
        }
    }

  std::string fmt_string = 1+ R"(
cmake_minimum_required(VERSION 3.0.2)
project(%1%)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  behaviortree_ros
  roseus_bt
%3%
)

add_service_files(
  FILES
%4%
)

add_action_files(
  FILES
%5%
)

generate_messages(
  DEPENDENCIES
%3%
)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS
 message_runtime
%3%
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(%2% src/%2%.cpp)
add_dependencies(%2% ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(%2% ${catkin_LIBRARIES})
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    target_name %
    boost::algorithm::join(message_packages, "\n") %
    boost::algorithm::join(service_files, "\n") %
    boost::algorithm::join(action_files, "\n");

  return bfmt.str();
}

std::string XMLParser::generate_package_xml(const char* package_name, const char* author_name) {
  auto format_build_depend = [](std::string pkg) {
    return fmt::format("  <build_depend>{}</build_depend>", pkg);
  };
  auto format_exec_depend = [](std::string pkg) {
    return fmt::format("  <exec_depend>{}</exec_depend>", pkg);
  };
  auto maybe_push_message_package = [](const XMLElement* node, std::vector<std::string>* message_packages) {
    std::string msg_type = node->Attribute("type");
    std::size_t pos = msg_type.find('/');
    if (pos != std::string::npos) {
      std::string pkg = msg_type.substr(0, pos);
      if (std::find(message_packages->begin(), message_packages->end(), pkg) ==
          message_packages->end()) {
        message_packages->push_back(pkg);
      }
    }
  };

  std::string author_email(author_name);
  std::transform(author_email.begin(), author_email.end(), author_email.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  std::replace(author_email.begin(), author_email.end(), ' ', '_');

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> message_packages;

  message_packages.push_back("std_msgs");
  message_packages.push_back("actionlib_msgs");

  for (auto node = root->FirstChildElement();
       node != nullptr;
       node = node->NextSiblingElement())
    {
      for (auto port_node = node->FirstChildElement();
           port_node != nullptr;
           port_node = port_node->NextSiblingElement())
        {
          maybe_push_message_package(port_node, &message_packages);
        }
    }

  std::vector<std::string> build_dependencies;
  std::vector<std::string> exec_dependencies;
  build_dependencies.resize(message_packages.size());
  exec_dependencies.resize(message_packages.size());

  std::transform(message_packages.begin(), message_packages.end(),
                 build_dependencies.begin(), format_build_depend);
  std::transform(message_packages.begin(), message_packages.end(),
                 exec_dependencies.begin(), format_exec_depend);

  std::string fmt_string = 1 + R"(
<?xml version="1.0"?>
<package format="2">
  <name>%1%</name>
  <version>0.0.0</version>
  <description>The %1% package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <maintainer email="%3%@example.com">%2%</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/%1%</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="%3%@example.com">%2%</author> -->


  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>behaviortree_ros</build_depend>
  <build_depend>roseus_bt</build_depend>
%4%

  <exec_depend>message_runtime</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>behaviortree_ros</exec_depend>
  <exec_depend>roseus_bt</exec_depend>
%5%

  <export>
  </export>
</package>
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    author_name %
    author_email %
    boost::algorithm::join(build_dependencies, ",\n") %
    boost::algorithm::join(exec_dependencies, ",\n");

  return bfmt.str();
}

}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
