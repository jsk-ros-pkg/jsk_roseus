#ifndef BEHAVIOR_TREE_ROSEUS_BT_COMMAND_LINE_ARGUMENT_MAPPING_
#define BEHAVIOR_TREE_ROSEUS_BT_COMMAND_LINE_ARGUMENT_MAPPING_

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <fmt/format.h>
#include <boost/program_options.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/init.h>

#include <roseus_bt/common_argument_mapping.h>

namespace roseus_bt
{

namespace po = boost::program_options;

bool parse_command_line(int argc, char** argv,
                        const std::string& program_description,
                        std::map<std::string, std::string>& argument_map)
{

  std::vector<std::string> argv_vec;
  ros::removeROSArgs(argc, argv, argv_vec);

  std::string example_description =
    fmt::format("example:\n  {0} --arg var1 hello --arg var2 world\n", argv[0]);

  po::options_description desc("usage");
  desc.add_options()
    ("help,h", "show this help message and exit")
    ("arg", po::value<std::vector<std::string>>()->multitoken(),
     "Initial blackboard variable-value pairs");

  po::positional_options_description pos;
  // when initializing from std::vector, we need to drop the program name
  po::parsed_options parsed_options = po::command_line_parser(
      std::vector<std::string>(argv_vec.begin() + 1, argv_vec.end())).
    options(desc).
    positional(pos).
    style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).
    run();

  po::variables_map vm;
  po::store(parsed_options, vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << "\n" << program_description << "\n";
    std::cout << desc << "\n";
    std::cout << example_description << "\n";
    return false;
  }

  for (const auto option : parsed_options.options) {
    if (option.string_key == "arg") {
      if (option.value.size() != 2) {
        std::cerr << "Each argument must have exactly one name and one value!\n";
        std::cerr << desc << "\n";
        std::cerr << example_description << "\n";
        return false;
      }
      argument_map.insert( {option.value.at(0), option.value.at(1)} );
    }
  }
  return true;
}

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_COMMAND_LINE_ARGUMENT_MAPPING_
