#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_

#include <string>
#include <vector>
#include <tinyxml2.h>
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

};


std::string XMLParser::port_node_to_message_description(const XMLElement* port_node) {
  if (!port_node->Attribute("type") ||
      !port_node->Attribute("name")) {
    std::string error_str = "Illformed port in ";
    error_str.append(port_node->Name());
    throw std::logic_error(error_str);
  }

  std::string result;
  result.append(port_node->Attribute("type"));
  result.append(" ");
  result.append(port_node->Attribute("name"));
  return result;
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
