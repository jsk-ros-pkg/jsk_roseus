#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_

#include <map>
#include <regex>
#include <tinyxml2.h>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <roseus_bt/bt_exceptions.h>
#include <roseus_bt/xml_exceptions.h>
#include <roseus_bt/gen_template.h>

void push_new(std::string elem, std::vector<std::string>* vec) {
  if (std::find(vec->begin(), vec->end(), elem) == vec->end()) {
    vec->push_back(elem);
  }
}

namespace RoseusBT
{
  using namespace tinyxml2;

class XMLParser
{

public:

  XMLParser(std::string filename) {
    if (!boost::filesystem::exists(filename)) {
      throw XMLError::FileNotFound(filename);
    }
    BOOST_LOG_TRIVIAL(debug) << "Initializing XMLParser from " << filename << "...";
    doc.LoadFile(filename.c_str());
    check_xml_file(filename);
  }

  ~XMLParser() {};

protected:

  XMLDocument doc;
  static GenTemplate gen_template;
  void check_xml_file(std::string filename);
  bool is_reactive(const XMLElement* node);
  bool is_reactive_base(const XMLElement* node, const XMLElement* ref_node, bool reactive_parent);
  void collect_param_list(const XMLElement* node, std::vector<std::string>* param_list,
                          std::vector<std::string>* output_list,
                          std::function<std::string(const XMLElement*)> param_fn,
                          std::function<std::string(const XMLElement*)> output_fn);
  void collect_param_list(const XMLElement* node, std::vector<std::string>* param_list,
                          std::vector<std::string>* output_list);
  void collect_eus_actions(const std::string package_name,
                           std::vector<std::string>* callback_definition,
                           std::vector<std::string>* instance_creation);
  void collect_eus_conditions(const std::string package_name,
                              std::vector<std::string>* callback_definition,
                              std::vector<std::string>* instance_creation,
                              std::vector<std::string>* parallel_callback_definition,
                              std::vector<std::string>* parallel_instance_creation);
  void maybe_push_message_package(const XMLElement* node,
                                  std::vector<std::string>* message_packages);
  std::string format_eus_name(const std::string input);
  std::string format_message_description(const XMLElement* port_node);
  std::string generate_action_file_contents(const XMLElement* node);
  std::string generate_service_file_contents(const XMLElement* node);
  std::string generate_headers(const std::string package_name);
  std::string generate_action_class(const XMLElement* node, const std::string package_name);
  std::string generate_remote_action_class(const XMLElement* node, const std::string package_name);
  std::string generate_condition_class(const XMLElement* node, const std::string package_name);
  std::string generate_subscriber_class(const XMLElement* node);
  std::string generate_main_function(const std::string roscpp_node_name, const std::string xml_filename);

  virtual std::string format_node_body(const XMLElement* node, int padding);

public:

  std::map<std::string, std::string> generate_all_action_files();
  std::map<std::string, std::string> generate_all_service_files();
  std::string generate_cpp_file(const std::string package_name,
                                const std::string roscpp_node_name,
                                const std::string xml_filename);
  void push_dependencies(std::vector<std::string>* message_packages,
                         std::vector<std::string>* action_files,
                         std::vector<std::string>* service_files);

  virtual std::string generate_eus_action_server(const std::string package_name);
  virtual std::string generate_eus_condition_server(const std::string package_name);

};

GenTemplate XMLParser::gen_template = GenTemplate();

void XMLParser::check_xml_file(std::string filename) {
  auto check_push = [this](XMLElement* node, std::vector<std::string>* vec,
                           std::vector<XMLElement*> *duplicated_nodes) {
    if (std::find(vec->begin(), vec->end(), node->Attribute("ID")) == vec->end()) {
      vec->push_back(node->Attribute("ID"));
    }
    else {
      duplicated_nodes->push_back(node);
    }
  };

  XMLElement* root = doc.RootElement();
  if (root == nullptr) {
    throw XMLError::ParsingError();
  }
  if (std::string(root->Name()) != "root") {
    throw XMLError::WrongRoot();
  }

  XMLElement* bt_root = root->FirstChildElement("TreeNodesModel");
  std::vector<std::string> actions, conditions, subscribers;
  std::vector<XMLElement*> duplicated_nodes;

  if (bt_root == nullptr) {
    throw XMLError::MissingRequiredNode("TreeNodesModel");
  }

  // check tree model
  for (auto node = bt_root->FirstChildElement();
       node != nullptr;
       node = node->NextSiblingElement()) {

    std::string name = node->Name();

    if (name != "Action" &&
        name != "RemoteAction" &&
        name != "Condition" &&
        name != "Subscriber" &&
        name != "SubTree") {
      throw XMLError::UnknownNode(node);
    }

    if (!node->Attribute("ID")) {
      throw XMLError::MissingRequiredAttribute("ID", node);
    }

    if (name == "Action" || name == "RemoteAction") {
      if (!node->Attribute("server_name")) {
        throw XMLError::MissingRequiredAttribute("server_name", node);
      }
      check_push(node, &actions, &duplicated_nodes);
    }

    if (name == "Condition") {
      if (!node->Attribute("service_name")) {
        throw XMLError::MissingRequiredAttribute("service_name", node);
      }
      check_push(node, &conditions, &duplicated_nodes);
    }

    if (name == "Subscriber") {
      if (!node->Attribute("type")) {
        throw XMLError::MissingRequiredAttribute("type", node);
      }
      check_push(node, &subscribers, &duplicated_nodes);
    }

    // check ports
    for (auto port_node = node->FirstChildElement();
         port_node != nullptr;
         port_node = port_node->NextSiblingElement()) {

      std::string port_name = port_node->Name();

      if (port_name != "input_port" &&
          port_name != "output_port" &&
          port_name != "inout_port") {
        throw XMLError::UnknownPortNode(port_node);
      }

      if (!port_node->Attribute("name")) {
        throw XMLError::MissingRequiredAttribute("name", port_node);
      }

      if (!port_node->Attribute("type")) {
        throw XMLError::MissingRequiredAttribute("type", port_node);
      }
    }
  }

  // delete duplicated nodes
  for (int i = 0; i < duplicated_nodes.size(); i++) {
    XMLElement* node = duplicated_nodes.at(i);
    BOOST_LOG_TRIVIAL(warning) << fmt::format("Ignoring duplicated {} node {} at {} line {}",
       node->Name(), node->Attribute("ID"), filename, node->GetLineNum());
    doc.DeleteNode(node);
  }
}

bool XMLParser::is_reactive(const XMLElement* node) {
  const XMLElement* bt_root = doc.RootElement()->FirstChildElement("BehaviorTree");

  for (auto bt_node = bt_root;
       bt_node != nullptr;
       bt_node = bt_node->NextSiblingElement("BehaviorTree")) {
    if (is_reactive_base(bt_node, node, false))
      return true;
  }
  return false;
}

bool XMLParser::is_reactive_base(const XMLElement* node, const XMLElement* ref_node,
                                 bool reactive_parent) {

  std::string name = node->Name();
  BOOST_LOG_TRIVIAL(trace) << "is_reactive_base: transversing " << name << "...";

  // is possibly reactive control node
  if (name.find("Fallback") != std::string::npos ||
      name.find("Sequence") != std::string::npos) {
    reactive_parent = (name.find("Reactive") != std::string::npos);
  }

  // parent node is reactive, node name and ID matches
  if (reactive_parent  &&
      !std::strcmp(node->Name(), ref_node->Name()) &&
      !std::strcmp(node->Attribute("ID"), ref_node->Attribute("ID"))) {
    return true;
  }

  for (auto child_node = node->FirstChildElement();
       child_node != nullptr;
       child_node = child_node->NextSiblingElement())
    {
      if (is_reactive_base(child_node, ref_node, reactive_parent))
        return true;
    }
  return false;
}

void XMLParser::collect_param_list(const XMLElement* node,
                                   std::vector<std::string>* param_list,
                                   std::vector<std::string>* output_list,
                                   std::function<std::string(const XMLElement*)> param_fn,
                                   std::function<std::string(const XMLElement*)> output_fn) {

  for (auto port_node = node->FirstChildElement();
       port_node != nullptr;
       port_node = port_node->NextSiblingElement())
    {
      std::string name = port_node->Name();
      std::string port_name = port_node->Attribute("name");
      BOOST_LOG_TRIVIAL(trace) << "collect_param_list: transversing " <<
        name << ": " << port_name << "...";

      if (name == "input_port" || name == "inout_port") {
        if (param_list != NULL) {
          BOOST_LOG_TRIVIAL(trace) << "collect_param_list: collecting input: " <<
            port_name << "...";
          param_list->push_back(param_fn(port_node));
        }
      }
      if (name == "output_port" || name == "inout_port") {
        if (output_list != NULL) {
          BOOST_LOG_TRIVIAL(trace) << "collect_param_list: collecting output: " <<
            port_name << "...";
          output_list->push_back(output_fn(port_node));
        }
      }
    }
}

void XMLParser::collect_param_list(const XMLElement* node,
                                   std::vector<std::string>* param_list,
                                   std::vector<std::string>* output_list) {
  auto format_port = [](const XMLElement* port_node) {
    return port_node->Attribute("name");
  };
  collect_param_list(node, param_list, output_list, format_port, format_port);
}

void XMLParser::collect_eus_actions(const std::string package_name,
                                    std::vector<std::string>* callback_definition,
                                    std::vector<std::string>* instance_creation) {

  auto format_action_param = [](const XMLElement* node) {
    return fmt::format("({0} (send goal :goal :{0}))", node->Attribute("name"));
  };
  auto format_callback = [this, format_action_param](const XMLElement* node) {
    std::vector<std::string> param_list;
    collect_param_list(node, &param_list, NULL, format_action_param, NULL);

    // has parameters
    if (param_list.size()) {
      std::string fmt_string = 1 + R"(
(defun {0}-execute-cb (server goal)
  (let ({1})
{2}
    t))
)";
      return fmt::format(fmt_string,
                         format_eus_name(node->Attribute("ID")),
                         boost::algorithm::join(param_list, "\n        "),
                         format_node_body(node, 4));
    }

    // no parameters
    std::string fmt_string = 1 + R"(
(defun {0}-execute-cb (server goal)
{1}
  t)
)";
      return fmt::format(fmt_string,
                         format_eus_name(node->Attribute("ID")),
                         format_node_body(node, 2));
  };

  auto format_instance = [this, package_name](const XMLElement* node, std::string server_name) {
    std::string fmt_string = 1 + R"(
(instance roseus_bt:action-node :init
          "{3}" {0}::{1}Action
          :execute-cb '{2}-execute-cb)
)";
    return fmt::format(fmt_string,
                       package_name,
                       node->Attribute("ID"),
                       format_eus_name(node->Attribute("ID")),
                       server_name);
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");

  for (auto node = root->FirstChildElement();
       node != nullptr;
       node = node->NextSiblingElement())
    {
      std::string name = node->Name();
      if (name == "Action" || name == "RemoteAction") {
        std::string server_name = node->Attribute("server_name");
        BOOST_LOG_TRIVIAL(trace) << "collect_eus_actions: transversing " << server_name << "...";
        push_new(format_callback(node), callback_definition);
        push_new(format_instance(node, server_name), instance_creation);
      }
    }
}

void XMLParser::collect_eus_conditions(const std::string package_name,
                                       std::vector<std::string>* callback_definition,
                                       std::vector<std::string>* instance_creation,
                                       std::vector<std::string>* parallel_callback_definition,
                                       std::vector<std::string>* parallel_instance_creation) {

  auto format_condition_param = [](const XMLElement* node) {
    return fmt::format("({0} (send request :{0}))", node->Attribute("name"));
  };
  auto format_callback = [this, format_condition_param](const XMLElement* node) {
    std::vector<std::string> param_list;
    collect_param_list(node, &param_list, NULL, format_condition_param, NULL);

    // has parameters
    if (param_list.size()) {
      std::string fmt_string = 1 + R"(
(defun {0}-cb (server request)
  (let ({1})
{2}
  ))
)";

      return fmt::format(fmt_string,
                         format_eus_name(node->Attribute("ID")),
                         boost::algorithm::join(param_list, "\n        "),
                         format_node_body(node, 4));
    }

    // no parameters
    std::string fmt_string = 1 + R"(
(defun {0}-cb (server request)
{1}
  )
)";
      return fmt::format(fmt_string,
                         format_eus_name(node->Attribute("ID")),
                         format_node_body(node, 2));
  };

  auto format_instance = [this, package_name](const XMLElement* node, std::string service_name) {
    std::string fmt_string = 1 + R"(
(instance roseus_bt:condition-node :init
          "{3}" {0}::{1}
          :execute-cb '{2}-cb)
)";
    return fmt::format(fmt_string,
                       package_name,
                       node->Attribute("ID"),
                       format_eus_name(node->Attribute("ID")),
                       service_name);
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      std::string service_name = condition_node->Attribute("service_name");
      BOOST_LOG_TRIVIAL(trace) << "collect_eus_conditions: transversing " << service_name << "...";

      if (is_reactive(condition_node)) {
        if (parallel_callback_definition != NULL &&
            parallel_instance_creation != NULL) {
          push_new(format_callback(condition_node), parallel_callback_definition);
          push_new(format_instance(condition_node, service_name), parallel_instance_creation);
        }
      }
      else {
        if (callback_definition != NULL && instance_creation != NULL) {
          push_new(format_callback(condition_node), callback_definition);
          push_new(format_instance(condition_node, service_name), instance_creation);
        }
      }
    }
}

void XMLParser::maybe_push_message_package(const XMLElement* node,
                                           std::vector<std::string>* message_packages) {
  std::string msg_type = node->Attribute("type");
  std::size_t pos = msg_type.find('/');
  if (pos != std::string::npos) {
    std::string pkg = msg_type.substr(0, pos);
    push_new(pkg, message_packages);
  }
}

std::string XMLParser::format_eus_name(const std::string input) {
  std::regex e ("([^A-Z]+)([A-Z]+)");
  std::string out = std::regex_replace(input, e, "$1-$2");
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return out;
}

std::string XMLParser::format_node_body(const XMLElement* node, int padding) {
  auto format_setoutput = [padding](const XMLElement* port_node) {
    return fmt::format(";; (send server :set-output \"{0}\" <{1}>)",
                       port_node->Attribute("name"),
                       port_node->Attribute("type")).
    insert(0, padding, ' ');
  };

  std::vector<std::string> output_list;
  output_list.push_back(std::string(";; do something").insert(0, padding, ' '));
  collect_param_list(node, NULL, &output_list, NULL, format_setoutput);

  return boost::algorithm::join(output_list, "\n");
}

std::string XMLParser::format_message_description(const XMLElement* port_node) {
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
      std::string text = format_message_description(port_node);

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
      std::string text = format_message_description(port_node);

      if (name == "input_port") {
        request.push_back(text);
      }
      else {
        throw InvalidOutputPort();
      }
    }

  return gen_template.service_file_template(request);
}

std::string XMLParser::generate_headers(const std::string package_name) {
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

  auto format_subscriber_node = [](const XMLElement* node) {
    return fmt::format("#include <{}.h>", node->Attribute("type"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> headers;

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      headers.push_back(format_action_node(action_node, package_name));
    }

   for (auto action_node = root->FirstChildElement("RemoteAction");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("RemoteAction"))
    {
      headers.push_back(format_action_node(action_node, package_name));
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      headers.push_back(format_condition_node(condition_node, package_name));
    }

  for (auto subscriber_node = root->FirstChildElement("Subscriber");
       subscriber_node != nullptr;
       subscriber_node = subscriber_node->NextSiblingElement("Subscriber"))
    {
      headers.push_back(format_subscriber_node(subscriber_node));
    }

  return gen_template.headers_template(headers);
}

std::string XMLParser::generate_action_class(const XMLElement* node, const std::string package_name) {
  auto format_server_name = [](const XMLElement* node) {
    return fmt::format("      InputPort<std::string>(\"server_name\", \"{0}\", \"name of the Action Server\")",
                       node->Attribute("server_name"));
  };
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

  provided_input_ports.push_back(format_server_name(node));

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

std::string XMLParser::generate_remote_action_class(const XMLElement* node, const std::string package_name) {
  auto format_server_name = [](const XMLElement* node) {
    return fmt::format("      InputPort<std::string>(\"server_name\", \"{0}\", \"name of the Action Server\")",
                       node->Attribute("server_name"));
  };
  auto format_input_port = [](const XMLElement* node) {
    return fmt::format("      InputPort<std::string>(\"{0}\")",
                         node->Attribute("name"));
  };
  auto format_output_port = [](const XMLElement* node) {
    return fmt::format("      OutputPort<std::string>(\"{0}\")",
                         node->Attribute("name"));
  };
  auto format_get_input = [](const XMLElement* node) {
    return fmt::format(R"(
    getInput("{0}", json);
    document.Parse(json.c_str());
    rapidjson::Value {0}(document, document.GetAllocator());
    goal->AddMember("{0}", {0}, goal->GetAllocator());)",
    node->Attribute("name"));
  };
  auto format_set_output = [](const XMLElement* node) {
    return fmt::format("    setOutputFromMessage(\"{0}\", feedback);",
                         node->Attribute("name"));
   };

  std::vector<std::string> provided_input_ports;
  std::vector<std::string> provided_output_ports;
  std::vector<std::string> get_inputs;
  std::vector<std::string> set_outputs;

  provided_input_ports.push_back(format_server_name(node));

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


  return gen_template.remote_action_class_template(
             package_name, node->Attribute("ID"),
             node->Attribute("host_name"),
             std::atoi(node->Attribute("host_port")),
             provided_ports, get_inputs, set_outputs);
}

std::string XMLParser::generate_condition_class(const XMLElement* node, const std::string package_name) {
  auto format_service_name = [](const XMLElement* node) {
    return fmt::format("      InputPort<std::string>(\"service_name\", \"{0}\", \"name of the ROS service\")",
                       node->Attribute("service_name"));
  };
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

  provided_ports.push_back(format_service_name(node));

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

std::string XMLParser::generate_subscriber_class(const XMLElement* node) {
  auto format_type = [](const XMLElement* node) {
    std::string type = node->Attribute("type");
    std::size_t pos = type.find('/');
    if (pos == std::string::npos) {
      throw XMLError::InvalidTopicType(type, node);
    }
    return fmt::format("{}::{}", type.substr(0, pos), type.substr(1+pos));
  };
  auto format_field = [](const XMLElement* node) {
    if (!node->Attribute("field")) return "";
    return node->Attribute("field");
  };

  return gen_template.subscriber_class_template(node->Attribute("ID"),
                                                format_type(node),
                                                format_field(node));
}


std::string XMLParser::generate_main_function(const std::string roscpp_node_name,
                                              const std::string xml_filename) {
  auto format_action_node = [](const XMLElement* node) {
    return fmt::format("  RegisterRosAction<{0}>(factory, \"{0}\", nh);",
                       node->Attribute("ID"));
  };
  auto format_remote_action_node = [](const XMLElement* node) {
    return fmt::format("  RegisterRemoteAction<{0}>(factory, \"{0}\");",
                       node->Attribute("ID"));
  };
  auto format_condition_node = [](const XMLElement* node) {
    return fmt::format("  RegisterRosService<{0}>(factory, \"{0}\", nh);",
                       node->Attribute("ID"));
  };
  auto format_subscriber_node = [](const XMLElement* node) {
    return fmt::format("  RegisterSubscriberNode<{0}>(factory, \"{0}\", nh);",
                       node->Attribute("ID"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");
  std::vector<std::string> register_actions;
  std::vector<std::string> register_conditions;
  std::vector<std::string> register_subscribers;

  for (auto action_node = root->FirstChildElement("Action");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("Action"))
    {
      register_actions.push_back(format_action_node(action_node));
    }

  for (auto action_node = root->FirstChildElement("RemoteAction");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("RemoteAction"))
    {
      register_actions.push_back(format_remote_action_node(action_node));
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      register_conditions.push_back(format_condition_node(condition_node));
    }

  for (auto subscriber_node = root->FirstChildElement("Subscriber");
       subscriber_node != nullptr;
       subscriber_node = subscriber_node->NextSiblingElement("Subscriber"))
    {
      register_subscribers.push_back(format_subscriber_node(subscriber_node));
    }

  return gen_template.main_function_template(roscpp_node_name, xml_filename,
                                             register_actions,
                                             register_conditions,
                                             register_subscribers);
}

std::string XMLParser::generate_eus_action_server(const std::string package_name) {

  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  collect_eus_actions(package_name, &callback_definition, &instance_creation);
  collect_eus_conditions(package_name, &callback_definition, &instance_creation,
                         NULL, NULL);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("action", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
}

std::string XMLParser::generate_eus_condition_server(const std::string package_name) {
  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  collect_eus_conditions(package_name, NULL, NULL,
                         &callback_definition, &instance_creation);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("condition", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
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

  for (auto action_node = root->FirstChildElement("RemoteAction");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("RemoteAction"))
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

std::string XMLParser::generate_cpp_file(const std::string package_name,
                                         const std::string roscpp_node_name,
                                         const std::string xml_filename) {
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

  for (auto action_node = root->FirstChildElement("RemoteAction");
       action_node != nullptr;
       action_node = action_node->NextSiblingElement("RemoteAction"))
    {
      output.append(generate_remote_action_class(action_node, package_name));
      output.append("\n\n");
    }

  for (auto condition_node = root->FirstChildElement("Condition");
       condition_node != nullptr;
       condition_node = condition_node->NextSiblingElement("Condition"))
    {
      output.append(generate_condition_class(condition_node, package_name));
      output.append("\n\n");
    }

  for (auto subscriber_node = root->FirstChildElement("Subscriber");
       subscriber_node != nullptr;
       subscriber_node = subscriber_node->NextSiblingElement("Subscriber"))
    {
      output.append(generate_subscriber_class(subscriber_node));
      output.append("\n\n");
    }

  output.append(generate_main_function(roscpp_node_name, xml_filename));

  return output;
}

void XMLParser::push_dependencies(std::vector<std::string>* message_packages,
                                  std::vector<std::string>* service_files,
                                  std::vector<std::string>* action_files) {
  auto format_action_file = [](const XMLElement* node) {
    return fmt::format("  {}.action", node->Attribute("ID"));
  };
  auto format_service_file = [](const XMLElement* node) {
    return fmt::format("  {}.srv", node->Attribute("ID"));
  };

  const XMLElement* root = doc.RootElement()->FirstChildElement("TreeNodesModel");

   for (auto node = root->FirstChildElement();
       node != nullptr;
       node = node->NextSiblingElement()) {

      std::string name = node->Name();

      if (name == "Action" || name == "RemoteAction") {
        push_new(format_action_file(node), action_files);
      }
      if (name == "Condition") {
        push_new(format_service_file(node), service_files);
      }
      if (name == "Action" || name == "RemoteAction" || name == "Condition") {
        for (auto port_node = node->FirstChildElement();
             port_node != nullptr;
             port_node = port_node->NextSiblingElement())
          {
            maybe_push_message_package(port_node, message_packages);
          }
      }
      if (name == "Subscriber") {
        maybe_push_message_package(node, message_packages);
      }
   }
}


}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_XML_PARSER_
