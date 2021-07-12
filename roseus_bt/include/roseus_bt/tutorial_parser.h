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
  virtual std::string format_node_body(const XMLElement* node) override;

public:
  virtual std::string generate_eus_action_server(const std::string package_name) override;
  virtual std::string generate_eus_condition_server(const std::string package_name) override;
};


std::string TutorialParser::format_node_body(const XMLElement* node) {
  std::string id = node->Attribute("ID");
  std::vector<std::string> param_list, output_list;
  collect_param_list(node, &param_list, &output_list);

  // Conditions
  if (id == "CheckTrue") return "  (send value :data)";
  if (id == "atTable")   return "  (at-spot \"table-front\")";
  if (id == "atSpot")    return fmt::format("  (at-spot {})", param_list.at(0));
  if (id == "CheckCoords") return fmt::format(
    "  (not (equal (instance geometry_msgs::Pose :init) {}))", param_list.at(0));

  // Actions
  if (id == "Init")  return "  (init nil t)";
  if (id == "InitWithBroom") return  "  (init t t)";
  if (id == "MoveToTable")   return  "  (go-to-spot \"table-front\")";
  if (id == "PickBottle")    return  "  (pick-sushi-bottle)";
  if (id == "PourBottle")    return  "  (pour-sushi-bottle)";
  if (id == "PlaceBottle")   return  "  (place-sushi-bottle)\n  (reset-pose)";
  if (id == "SweepFloor")    return  "  (sweep-floor #'roseus_bt:ok)\n  (reset-pose)";

  if (id == "MoveTo")
    return fmt::format("  (go-to-spot {})", param_list.at(0));
  if (id == "PickBottleAt")
    return fmt::format("  (pick-sushi-bottle (ros::tf-pose->coords {}))", param_list.at(0));
  if (id == "setCoords") {
    std::string fmt_string = 1 + R"(
  (roseus_bt:set-output
   "{}" (ros::coords->tf-pose (make-coords :pos #f(1850 400 700)))))";
    return fmt::format(fmt_string, output_list.at(0));
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


}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_TUTORIAL_PARSER_
