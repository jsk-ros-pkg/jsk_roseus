#ifndef BEHAVIOR_TREE_ROSEUS_BT_TUTORIAL_PARSER_
#define BEHAVIOR_TREE_ROSEUS_BT_TUTORIAL_PARSER_

#include <tinyxml2.h>
#include <roseus_bt/xml_parser.h>

namespace RoseusBT
{
  using namespace tinyxml2;

class TutorialParser : public XMLParser
{

public:

  TutorialParser(std::string filename): XMLParser(filename) {};

  ~TutorialParser() {};

protected:
  virtual std::string format_node_body(const XMLElement* node, int padding) override;

public:
  virtual std::string generate_eus_action_server(const std::string package_name) override;
  virtual std::string generate_eus_remote_action_server(const std::string package_name,
                                                        const std::string remote_host) override;
  virtual std::string generate_eus_condition_server(const std::string package_name) override;
  virtual std::string generate_eus_remote_condition_server(const std::string package_name,
                                                           const std::string remote_host) override;
};


std::string TutorialParser::format_node_body(const XMLElement* node, int padding) {
  std::string pad(padding, ' ');
  std::string id = node->Attribute("ID");
  std::vector<std::string> param_list, output_list;
  collect_param_list(node, &param_list, &output_list);

  // Conditions
  if (id == "CheckTrue") return std::string("(send value :data)").insert(0, padding, ' ');
  if (id == "atTable")   return std::string("(at-spot \"table-front\")").insert(0, padding, ' ');
  if (id == "atSpot" || id == "atTableSpot" || id == "atBroomSpot")
    return fmt::format("{}(at-spot {})", pad, param_list.at(0));
  if (id == "CheckCoords") return fmt::format(
    "{}{}", pad, param_list.at(0));

  // Actions
  if (id == "Init")  return std::string("(init nil t)").insert(0, padding, ' ');
  if (id == "InitWithBroom") return  std::string("(init t t)").insert(0, padding, ' ');
  if (id == "MoveToTable")   return  std::string("(go-to-spot \"table-front\")").insert(0, padding, ' ');
  if (id == "MoveToBroom")   return  std::string("(go-to-spot \"broom-front\")").insert(0, padding, ' ');
  if (id == "PickBottle")    return  std::string("(pick-sushi-bottle)").insert(0, padding, ' ');
  if (id == "PourBottle")    return  std::string("(pour-sushi-bottle)").insert(0, padding, ' ');

  if (id == "PlaceBottle")
    return  fmt::format("{0}(place-sushi-bottle)\n{0}(reset-pose)", pad);
  if (id == "MoveTo")
    return fmt::format("{}(go-to-spot {})", pad, param_list.at(0));
  if (id == "PickBottleAt")
    return fmt::format("{}(pick-sushi-bottle (ros::tf-pose->coords {}))", pad, param_list.at(0));
  if (id == "setCoords") {
    std::string fmt_string = 1 + R"(
{0}(send server :set-output "{1}"
{0}      (ros::coords->tf-pose (make-coords :pos #f(1850 400 700)))))";
    return fmt::format(fmt_string, pad, output_list.at(0));
  }
  if (id == "SweepFloor") {
      std::string fmt_string = 1 + R"(
{0}(handler-case (sweep-floor)
{0}  (roseus_bt:cancel-action () nil))
{0}(reset-pose))";
      return fmt::format(fmt_string, pad);
  }

  throw XMLError::UnknownNode(node);
}

std::string TutorialParser::generate_eus_action_server(const std::string package_name) {

  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  // Add load files
  load_files.push_back("package://roseus_bt/sample/sample-task.l");

  collect_eus_actions(package_name, &callback_definition, &instance_creation);
  collect_eus_conditions(package_name, &callback_definition, &instance_creation,
                         NULL, NULL);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("action", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
}

std::string TutorialParser::generate_eus_remote_action_server(const std::string package_name,
                                                              const std::string remote_host) {

  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  // Add load files
  load_files.push_back("package://roseus_bt/sample/sample-task.l");

  collect_eus_actions(package_name, &callback_definition, &instance_creation, remote_host);
  collect_eus_conditions(package_name, &callback_definition, &instance_creation,
                         NULL, NULL,
                         remote_host);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("action", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
}

std::string TutorialParser::generate_eus_condition_server(const std::string package_name) {
  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  // Add load files
  load_files.push_back("package://roseus_bt/sample/sample-task.l");

  collect_eus_conditions(package_name, NULL, NULL,
                         &callback_definition, &instance_creation);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("condition", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
}

std::string TutorialParser::generate_eus_remote_condition_server(const std::string package_name,
                                                                 const std::string remote_host) {
  std::vector<std::string> callback_definition;
  std::vector<std::string> instance_creation;
  std::vector<std::string> load_files;

  // Add load files
  load_files.push_back("package://roseus_bt/sample/sample-task.l");

  collect_eus_conditions(package_name, NULL, NULL,
                         &callback_definition, &instance_creation,
                         remote_host);

  if (callback_definition.empty()) return "";

  return gen_template.eus_server_template("condition", package_name,
                                          callback_definition, instance_creation,
                                          load_files);
}

}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_TUTORIAL_PARSER_
