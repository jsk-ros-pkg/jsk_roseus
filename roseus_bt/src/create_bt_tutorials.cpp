#include <iostream>
#include <ros/package.h>
#include <roseus_bt/tutorial_parser.h>
#include <roseus_bt/package_generator.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  std::string package_name = "roseus_bt_tutorials";
  std::string author = "Guilherme Affonso";
  std::vector<std::string> model_files, executable_names;
  std::string path = ros::package::getPath("roseus_bt") + "/sample/models/";

  po::options_description desc("usage");
  desc.add_options()
    ("help,h", "show this help message and exit")
    ("overwrite,y", "overwrite all existing files")
    ("verbose,v", "print all logging messages");

  po::variables_map args;
  po::store(po::parse_command_line(argc, argv, desc), args);
  po::notify(args);

  // Initialize Logger
  auto logger_level = boost::log::trivial::warning;
  if (args.count("verbose")) {
    logger_level = boost::log::trivial::debug;
  }

  boost::log::core::get()->set_filter(
     boost::log::trivial::severity >= logger_level);

  // Help
  if (args.count("help")) {
    std::cout << "\n" << "Create behavior tree package." << "\n";
    std::cout << desc << std::endl;
    return 0;
  }

  // Set model files
  model_files.push_back(path + "t01_simple_tree.xml");
  model_files.push_back(path + "t02_conditions.xml");
  model_files.push_back(path + "t03_ports.xml");
  model_files.push_back(path + "t04_subscriber.xml");
  model_files.push_back(path + "t05_subtrees.xml");
  model_files.push_back(path + "t06_reactive.xml");
  // model_files.push_back(path + "t07_xacro.xml");

  // Set executable names
  executable_names.resize(model_files.size());
  for (auto model_it = model_files.begin(), exec_it = executable_names.begin();
       model_it != model_files.end();
       ++model_it, ++exec_it) {
    std::string model_file = *model_it;
    std::string executable = *exec_it;
    *exec_it = boost::filesystem::path(model_file).stem().string();
  }

  // Generate files
  RoseusBT::PackageGenerator<RoseusBT::TutorialParser> pg(package_name,
                                                          model_files, executable_names,
                                                          author, args.count("overwrite"));
  pg.write_all_files();

  return 0;
}
