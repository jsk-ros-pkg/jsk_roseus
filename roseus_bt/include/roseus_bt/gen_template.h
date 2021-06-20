#ifndef BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_
#define BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_

#include <map>
#include <string>
#include <vector>
#include <fmt/format.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/join.hpp>

namespace RoseusBT
{

class GenTemplate
{
public:
  GenTemplate() {};
  ~GenTemplate() {};

  std::string action_file_template(std::vector<std::string> goal,
                                   std::vector<std::string> feedback);
  std::string service_file_template(std::vector<std::string> request);
  std::string headers_template(std::vector<std::string> headers);

  std::string action_class_template(std::string package_name, std::string nodeID,
                                      std::vector<std::string> provided_ports,
                                      std::vector<std::string> get_inputs,
                                      std::vector<std::string> set_outputs);
  std::string  condition_class_template(std::string package_name, std::string nodeID,
                                        std::vector<std::string> provided_ports,
                                        std::vector<std::string> get_inputs);
  std::string main_function_template(std::string roscpp_node_name,
                                       std::string xml_filename,
                                       std::vector<std::string> register_actions,
                                       std::vector<std::string> register_conditions);
  std::string eus_server_template(std::string server_type,
                                    std::string package_name,
                                    std::vector<std::string> callbacks,
                                    std::vector<std::string> instances);
  std::string cmake_lists_template(std::string package_name, std::string target_name,
                                     std::vector<std::string> message_packages,
                                     std::vector<std::string> service_files,
                                     std::vector<std::string> action_files);
  std::string package_xml_template(std::string package_name,
                                     std::string author_name,
                                     std::vector<std::string> build_dependencies,
                                     std::vector<std::string> exec_dependencies);
};


std::string GenTemplate::action_file_template(std::vector<std::string> goal,
                                              std::vector<std::string> feedback) {
  std::string fmt_string = 1 + R"(
%1%
---
bool success
---
%2%
)";

  boost::format bfmt = boost::format(fmt_string) %
    boost::algorithm::join(goal, "\n") %
    boost::algorithm::join(feedback, "\n");

  return bfmt.str();
}


std::string GenTemplate::service_file_template(std::vector<std::string> request) {
  std::string fmt_string = 1 + R"(
%1%
---
bool success
)";

  boost::format bfmt = boost::format(fmt_string) %
    boost::algorithm::join(request, "\n");

  return bfmt.str();
}


std::string GenTemplate::headers_template(std::vector<std::string> headers) {
  std::string fmt_string = 1 + R"(
#include <ros/ros.h>

#include <roseus_bt/eus_action_node.h>
#include <roseus_bt/eus_condition_node.h>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

%1%

using namespace BT;
)";
  boost::format bfmt = boost::format(fmt_string) %
    boost::algorithm::join(headers, "\n");

  return bfmt.str();
}


std::string GenTemplate::action_class_template(std::string package_name, std::string nodeID,
                                               std::vector<std::string> provided_ports,
                                               std::vector<std::string> get_inputs,
                                               std::vector<std::string> set_outputs) {
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
    nodeID %
    boost::algorithm::join(provided_ports, ",\n") %
    boost::algorithm::join(get_inputs, "\n") %
    boost::algorithm::join(set_outputs, "\n");

  return bfmt.str();
}


std::string GenTemplate::condition_class_template(std::string package_name, std::string nodeID,
                                                 std::vector<std::string> provided_ports,
                                                 std::vector<std::string> get_inputs) {
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
    nodeID %
    boost::algorithm::join(provided_ports, ",\n") %
    boost::algorithm::join(get_inputs, "\n");

  return bfmt.str();
}


std::string GenTemplate::main_function_template(std::string roscpp_node_name,
                                                std::string xml_filename,
                                                std::vector<std::string> register_actions,
                                                std::vector<std::string> register_conditions) {
  auto format_ros_init = [roscpp_node_name]() {
    return fmt::format("  ros::init(argc, argv, \"{}\");", roscpp_node_name);
  };
  auto format_create_tree = [xml_filename]() {
    return fmt::format("  auto tree = factory.createTreeFromFile(\"{}\");", xml_filename);
  };

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
    format_ros_init() %
    format_create_tree() %
    boost::algorithm::join(register_actions, "\n") %
    boost::algorithm::join(register_conditions, "\n");

  return bfmt.str();
}


std::string GenTemplate::eus_server_template(std::string server_type,
                                             std::string package_name,
                                             std::vector<std::string> callbacks,
                                             std::vector<std::string> instances) {
  auto format_ros_roseus = [server_type]() {
    return fmt::format("(ros::roseus \"{}_server\")", server_type);
  };
  auto format_load_ros_package = [package_name]() {
    return fmt::format("(ros::load-ros-package \"{}\")\n", package_name);
  };

  std::string fmt_string = 1 + R"(
%2%
%3%
%4%

;; define %1% callbacks
%5%

;; create server instances
%6%

;; spin
(roseus_bt:spin)
)";
  
  boost::format bfmt = boost::format(fmt_string) %
    server_type %
    format_ros_roseus() %
    format_load_ros_package() %
    "(load \"package://roseus_bt/euslisp/nodes.l\")" %
    boost::algorithm::join(callbacks, "\n") %
    boost::algorithm::join(instances, "\n");

  return bfmt.str();
}


std::string GenTemplate::cmake_lists_template(std::string package_name, std::string target_name,
                                              std::vector<std::string> message_packages,
                                              std::vector<std::string> service_files,
                                              std::vector<std::string> action_files) {
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


std::string GenTemplate::package_xml_template(std::string package_name,
                                              std::string author_name,
                                              std::vector<std::string> build_dependencies,
                                              std::vector<std::string> exec_dependencies) {

  std::string author_email(author_name);
  std::transform(author_email.begin(), author_email.end(), author_email.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  std::replace(author_email.begin(), author_email.end(), ' ', '_');

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

#endif  // BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_
