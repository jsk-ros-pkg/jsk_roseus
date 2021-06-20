#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_

#include <tinyxml2.h>
#include "gen_template.h"

namespace RoseusBT
{
  using namespace tinyxml2;


class XMLParser
{

public:

  XMLParser(std::string filename) : gen_template() {
    doc.LoadFile(filename.c_str());
  }

  ~XMLParser() {};

protected:

  XMLDocument doc;
  GenTemplate gen_template;
  void collect_node_attribute(const XMLElement* node, const XMLElement* ref_node,
                              const char* attribute, std::vector<std::string>* node_attributes);
  std::string port_node_to_message_description(const XMLElement* port_node);
  std::string generate_action_file_contents(const XMLElement* node);
  std::string generate_service_file_contents(const XMLElement* node);
  std::string generate_headers(std::string package_name);
  std::string generate_action_class(const XMLElement* node, std::string package_name);
  std::string generate_condition_class(const XMLElement* node, std::string package_name);
  std::string generate_main_function(std::string roscpp_node_name, std::string xml_filename);

public:

  std::map<std::string, std::string> generate_all_action_files();
  std::map<std::string, std::string> generate_all_service_files();
  std::string generate_cpp_file(std::string package_name,
                                std::string roscpp_node_name,
                                std::string xml_filename);
  std::string generate_eus_action_server(std::string action_name);
  std::string generate_eus_condition_server(std::string action_name);
  std::string generate_cmake_lists(std::string package_name,
                                   std::string target_filename);
  std::string generate_package_xml(std::string package_name,
                                   std::string author_name);

};


void XMLParser::collect_node_attribute(const XMLElement* node,
                                       const XMLElement* ref_node,
                                       const char* attribute,
                                       std::vector<std::string>* node_attributes) {

  if (!std::strcmp(node->Name(), ref_node->Name()) &&
      !std::strcmp(node->Attribute("ID"), ref_node->Attribute("ID"))) {
    if (node->Attribute(attribute) &&
        std::find(node_attributes->begin(), node_attributes->end(),
                  node->Attribute(attribute)) == node_attributes->end()) {
      node_attributes->push_back(node->Attribute(attribute));
    }
    return;
  }

  for (auto child_node = node->FirstChildElement();
       child_node != nullptr;
       child_node = child_node->NextSiblingElement())
    {
      collect_node_attribute(child_node, ref_node, attribute, node_attributes);
    }
}

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

  return gen_template.action_file_template(goal, feedback);
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

  return gen_template.service_file_template(request);
}

std::string XMLParser::generate_headers(std::string package_name) {
  auto format_action_node = [](const XMLElement* node, std::string package_name) {
    return fmt::format("#include <{}/{}Action.h>",
                       package_name,
                       node->Attribute("ID"));
  };

  auto format_condition_node = [](const XMLElement* node, std::string package_name) {
    return fmt::format("#include <{}/{}.h>",
                       package_name,
                       node->Attribute("ID"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> headers;

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

  return gen_template.headers_template(headers);
}

std::string XMLParser::generate_action_class(const XMLElement* node, std::string package_name) {
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


  return gen_template.action_class_template(package_name, node->Attribute("ID"),
                                            provided_ports, get_inputs, set_outputs);
}

std::string XMLParser::generate_condition_class(const XMLElement* node, std::string package_name) {
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

  return gen_template.condition_class_template(package_name, node->Attribute("ID"),
                                               provided_ports, get_inputs);
}

std::string XMLParser::generate_main_function(std::string roscpp_node_name,
                                              std::string xml_filename) {
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

  return gen_template.main_function_template(roscpp_node_name, xml_filename,
                                             register_actions, register_conditions);
}

std::string XMLParser::generate_eus_action_server(std::string package_name) {
  auto format_callback = [](const XMLElement* node, std::string suffix) {
    std::string fmt_string = 1 + R"(
(roseus_bt:define-action-callback {0}-execute-cb{1} ({2})
  ;; do something
  t)
)";
    std::vector<std::string> param_list;
    for (auto port_node = node->FirstChildElement();
         port_node != nullptr;
         port_node = port_node->NextSiblingElement())
      {
        std::string name = port_node->Name();
        if (name == "input_port" || name == "inout_port") {
          param_list.push_back(port_node->Attribute("name"));
        }
      }

    return fmt::format(fmt_string,
                       node->Attribute("ID"),
                       suffix,
                       boost::algorithm::join(param_list, " "));
  };

  auto format_instance = [package_name](const XMLElement* node, std::string suffix,
                                        std::string server_name) {
    std::string fmt_string = 1 + R"(
(instance roseus_bt:action-node :init
          "{3}" {0}::{1}Action
          :execute-cb '{1}-execute-cb{2})
)";
  return fmt::format(fmt_string,
                     package_name,
                     node->Attribute("ID"),
                     suffix,
                     server_name);
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  const XMLElement* bt_root = doc.RootElement()->FirstChildElement("BehaviorTree");
  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      std::vector<std::string> server_names;
      collect_node_attribute(bt_root, action_node, "server_name", &server_names);

      if (server_names.empty()) {
        callback_definition.push_back(format_callback(action_node, ""));
        instance_creation.push_back(format_instance(action_node, "",
                                                    action_node->Attribute("ID")));
      }
      else if (server_names.size() == 1) {
        callback_definition.push_back(format_callback(action_node, ""));
        instance_creation.push_back(format_instance(action_node, "",
                                                    server_names.at(0)));
      }
      else {
        int suffix_num = 1;
        for (std::vector<std::string>::const_iterator it = server_names.begin();
             it != server_names.end(); ++it, suffix_num++) {
            std::string suffix = fmt::format("-{}", suffix_num);
            callback_definition.push_back(format_callback(action_node, suffix));
            instance_creation.push_back(format_instance(action_node, suffix, *it));
        }
      }
    }

  return gen_template.eus_server_template("action", package_name,
                                          callback_definition, instance_creation);
}

std::string XMLParser::generate_eus_condition_server(std::string package_name) {
  auto format_callback = [](const XMLElement* node, std::string suffix) {
    std::string fmt_string = 1 + R"(
(roseus_bt:define-condition-callback {0}-cb{1} ({2})
  ;; do something
  t)
)";
    std::vector<std::string> param_list;
    for (auto port_node = node->FirstChildElement();
         port_node != nullptr;
         port_node = port_node->NextSiblingElement())
      {
        std::string name = port_node->Name();
        if (name == "input_port" || name == "inout_port") {
          param_list.push_back(port_node->Attribute("name"));
        }
      }

    return fmt::format(fmt_string,
                       node->Attribute("ID"),
                       suffix,
                       boost::algorithm::join(param_list, " "));
  };

  auto format_instance = [package_name](const XMLElement* node, std::string suffix,
                                        std::string service_name) {
    std::string fmt_string = 1 + R"(
(instance roseus_bt:condition-node :init
          "{3}" {0}::{1}
          :execute-cb #'{1}-cb{2})
)";
  return fmt::format(fmt_string,
                     package_name,
                     node->Attribute("ID"),
                     suffix,
                     service_name);
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  const XMLElement* bt_root = doc.RootElement()->FirstChildElement("BehaviorTree");
  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      std::vector<std::string> server_names;
      collect_node_attribute(bt_root, condition_node, "service_name", &server_names);

      if (server_names.empty()) {
        callback_definition.push_back(format_callback(condition_node, ""));
        instance_creation.push_back(format_instance(condition_node, "",
                                                    condition_node->Attribute("ID")));
      }
      else if (server_names.size() == 1) {
        callback_definition.push_back(format_callback(condition_node, ""));
        instance_creation.push_back(format_instance(condition_node, "",
                                                    server_names.at(0)));
      }
      else {
        int suffix_num = 1;
        for (std::vector<std::string>::const_iterator it = server_names.begin();
             it != server_names.end(); ++it, suffix_num++) {
            std::string suffix = fmt::format("-{}", suffix_num);
            callback_definition.push_back(format_callback(condition_node, suffix));
            instance_creation.push_back(format_instance(condition_node, suffix, *it));
        }
      }
    }

  return gen_template.eus_server_template("condition", package_name,
                                          callback_definition, instance_creation);
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

std::string XMLParser::generate_cpp_file(std::string package_name,
                                         std::string roscpp_node_name,
                                         std::string xml_filename) {
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

  output.append(generate_main_function(roscpp_node_name, xml_filename));

  return output;
}

std::string XMLParser::generate_cmake_lists(std::string package_name, std::string target_name) {
  auto format_pkg = [](std::string pkg) {
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
      std::string pkg = format_pkg(msg_type.substr(0, pos));
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

  return gen_template.cmake_lists_template(package_name, target_name,
                                           message_packages,
                                           service_files,
                                           action_files);
}

std::string XMLParser::generate_package_xml(std::string package_name, std::string author_name) {
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

  return gen_template.package_xml_template(package_name, author_name,
                                           build_dependencies,
                                           exec_dependencies);
}

}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
