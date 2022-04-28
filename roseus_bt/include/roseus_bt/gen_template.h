#ifndef BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_
#define BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_

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
  std::string remote_action_class_template(std::string package_name, std::string nodeID,
                                           std::vector<std::string> provided_ports,
                                           std::vector<std::string> get_inputs,
                                           std::vector<std::string> set_outputs);
  std::string  condition_class_template(std::string package_name, std::string nodeID,
                                        std::vector<std::string> provided_ports,
                                        std::vector<std::string> get_inputs);
  std::string  remote_condition_class_template(std::string package_name, std::string nodeID,
                                               std::vector<std::string> provided_ports,
                                               std::vector<std::string> get_inputs);
  std::string subscriber_class_template(std::string nodeID, std::string message_type,
                                          std::string message_field);
  std::string main_function_template(std::string roscpp_node_name,
                                       std::string xml_filename,
                                       std::vector<std::string> register_actions,
                                       std::vector<std::string> register_conditions,
                                       std::vector<std::string> register_subscribers);
  std::string eus_server_template(std::string server_type,
                                    std::string package_name,
                                    std::vector<std::string> callbacks,
                                    std::vector<std::string> instances,
                                    std::vector<std::string> load_files);
};


std::string GenTemplate::action_file_template(std::vector<std::string> goal,
                                              std::vector<std::string> feedback) {
  std::string fmt_string = 1 + R"(
%1%
---
bool success
---
string update_field_name
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

#define DEBUG  // rosbridgecpp logging
#include <roseus_bt/eus_nodes.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

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

  void onFeedback(const typename FeedbackType::ConstPtr& feedback) override
  {
%5%
    return;
  }

  NodeStatus onResult(const ResultType& result) override
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


std::string GenTemplate::remote_action_class_template(
     std::string package_name, std::string nodeID,
     std::vector<std::string> provided_ports,
     std::vector<std::string> get_inputs,
     std::vector<std::string> set_outputs)
{
  auto format_send_goal = [](const std::string body) {
    std::string decl = 1 + R"(
    std::string json;
    rapidjson::Document document;
    GoalType ros_msg;
)";
    if (!body.empty()) return fmt::format("{}{}", decl, body);
    return body;
  };

  std::string fmt_string = 1 + R"(
class %2%: public EusRemoteActionNode<%1%::%2%Action>
{

public:
  %2%(const std::string& name, const NodeConfiguration& conf):
EusRemoteActionNode("%1%/%2%Action", name, conf) {}

  static PortsList providedPorts()
  {
    return  {
%3%
    };
  }

  bool sendGoal(rapidjson::Document* goal) override
  {
%4%
    return true;
  }

  void onFeedback(const rapidjson::Value& feedback) override
  {
%5%
    return;
  }

  NodeStatus onResult(const rapidjson::Value& result) override
  {
    if (result.HasMember("success") &&
        result["success"].IsBool() &&
        result["success"].GetBool()) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }

};
)";
  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    nodeID %
    boost::algorithm::join(provided_ports, ",\n") %
    format_send_goal(boost::algorithm::join(get_inputs, "\n")) %
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


std::string GenTemplate::remote_condition_class_template(std::string package_name, std::string nodeID,
                                                         std::vector<std::string> provided_ports,
                                                         std::vector<std::string> get_inputs) {
  auto format_send_request = [](const std::string body) {
    std::string decl = 1 + R"(
    std::string json;
    rapidjson::Document document;
    RequestType ros_msg;
)";
    if (!body.empty()) return fmt::format("{}{}", decl, body);
    return body;
  };

  std::string fmt_string = 1 + R"(
class %2%: public EusRemoteConditionNode<%1%::%2%>
{

public:
  %2%(const std::string& name, const NodeConfiguration& conf):
  EusRemoteConditionNode(name, conf) {}

  static PortsList providedPorts()
  {
    return  {
%3%
    };
  }

  void sendRequest(rapidjson::Document *request) override
  {
%4%
  }

  NodeStatus onResponse(const rapidjson::Value& result) override
  {
    if (result.HasMember("success") &&
        result["success"].IsBool() &&
        result["success"].GetBool()) {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }

};
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    nodeID %
    boost::algorithm::join(provided_ports, ",\n") %
    format_send_request(boost::algorithm::join(get_inputs, "\n"));

  return bfmt.str();
}


std::string GenTemplate::subscriber_class_template(std::string nodeID, std::string message_type,
                                                   std::string message_field) {
  if (!message_field.empty()) {
    std::string fmt_string = R"(
  virtual void callback(%1% msg) {
    setOutput("output_port", msg.%2%);
  }
)";

    boost::format bfmt = boost::format(fmt_string) %
      message_type %
      message_field;

    message_field = bfmt.str();
  }

  std::string fmt_string = 1 + R"(
class %1%: public EusSubscriberNode<%2%>
{
public:
  %1%(ros::NodeHandle& handle, const std::string& node_name, const NodeConfiguration& conf) :
    EusSubscriberNode<%2%>(handle, node_name, conf) {}
%3%};
)";
  boost::format bfmt = boost::format(fmt_string) %
    nodeID %
    message_type %
    message_field;

  return bfmt.str();
}


std::string GenTemplate::main_function_template(std::string roscpp_node_name,
                                                std::string xml_filename,
                                                std::vector<std::string> register_actions,
                                                std::vector<std::string> register_conditions,
                                                std::vector<std::string> register_subscribers) {
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

%3%%4%%5%
%2%

  std::string timestamp = std::to_string(ros::Time::now().toNSec());
  std::string log_filename(fmt::format("%6%", timestamp));

  StdCoutLogger logger_cout(tree);
  FileLogger logger_file(tree, log_filename.c_str());
  PublisherZMQ publisher_zmq(tree);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.005);
    sleep_time.sleep();
  }

  std::cout << "Writed log to file: " << log_filename << std::endl;
  return 0;
}
)";

  if (register_actions.size() != 0) register_actions.push_back("");
  if (register_conditions.size() != 0) register_conditions.push_back("");
  if (register_subscribers.size() != 0) register_subscribers.push_back("");

  boost::format file_format = boost::format("%1%/.ros/%2%_{0}.fbl") %
    getenv("HOME") %
    roscpp_node_name;
  boost::format bfmt = boost::format(fmt_string) %
    format_ros_init() %
    format_create_tree() %
    boost::algorithm::join(register_actions, "\n") %
    boost::algorithm::join(register_conditions, "\n") %
    boost::algorithm::join(register_subscribers, "\n") %
    file_format.str();

  return bfmt.str();
}


std::string GenTemplate::eus_server_template(std::string server_type,
                                             std::string package_name,
                                             std::vector<std::string> callbacks,
                                             std::vector<std::string> instances,
                                             std::vector<std::string> load_files) {
  auto format_ros_roseus = [server_type]() {
    return fmt::format("(ros::roseus \"{}_server\")", server_type);
  };
  auto format_load_ros_package = [package_name]() {
    return fmt::format("(ros::load-ros-package \"{}\")", package_name);
  };
  auto format_load_file = [](std::string filename) {
    return fmt::format("(load \"{}\")", filename);
  };

  std::transform(load_files.begin(), load_files.end(), load_files.begin(), format_load_file);
  if (load_files.size() != 0) load_files.push_back("");

  std::string fmt_string = 1 + R"(
%1%
%2%
%3%
%4%

;; define callbacks
%5%

;; create server instances
%6%

;; set rate
(ros::rate 100)

;; spin
(roseus_bt:spin)
)";
  
  boost::format bfmt = boost::format(fmt_string) %
    format_ros_roseus() %
    format_load_ros_package() %
    "(load \"package://roseus_bt/euslisp/nodes.l\")" %
    boost::algorithm::join(load_files, "\n") %
    boost::algorithm::join(callbacks, "\n") %
    boost::algorithm::join(instances, "\n");

  return bfmt.str();
}


}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_GEN_TEMPLATE_
