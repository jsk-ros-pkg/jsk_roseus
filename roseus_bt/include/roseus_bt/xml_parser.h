#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_

#include <string>
#include <vector>
#include <tinyxml2.h>
#include <fmt/format.h>
#include <boost/algorithm/string/join.hpp>

namespace RoseusBT
{
  using namespace tinyxml2;


class XMLParser
{

public:

  XMLParser(const std::string &filename) {
    doc.LoadFile(filename.c_str());
  }

  ~XMLParser() {};

protected:

  XMLDocument doc;
  std::string port_node_to_message_description(const XMLElement* port_node);
  std::string generate_action_file_contents(const XMLElement* node);
  std::string generate_service_file_contents(const XMLElement* node);

public:

  std::string test_all_actions();
  std::string test_all_conditions();
  std::string generate_headers(const char* package_name);

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

  return output;
}


std::string XMLParser::test_all_actions() {
  std::string result;
  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      result.append(action_node->Attribute("ID"));
      result.append(":\n");
      result.append(generate_action_file_contents(action_node));
      result.append("\n\n");
    }
  return result;
}

std::string XMLParser::test_all_conditions() {
  std::string result;
  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      result.append(condition_node->Attribute("ID"));
      result.append(":\n");
      result.append(generate_service_file_contents(condition_node));
      result.append("\n\n");
    }
  return result;
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
