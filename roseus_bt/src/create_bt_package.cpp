#include <iostream>
#include <roseus_bt/xml_parser.h>
#include <roseus_bt/package_generator.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  std::string package_name, author;
  std::vector<std::string> model_files, executable_names;

  po::options_description desc("usage");
  desc.add_options()
    ("help,h", "show this help message and exit")
    ("package_name", po::value<std::string>(&package_name), "package name")
    ("model_file", po::value<std::vector<std::string>>(&model_files), "model file")
    ("executable,e", po::value<std::vector<std::string>>(&executable_names),
     "executable name (defaults to model filename)")
    ("author,a", po::value<std::string>(&author)->default_value("The Author"),
     "author name")
    ("overwrite,y", "overwrite all existing files")
    ("verbose,v", "print all logging messages");

  po::positional_options_description positional_arguments;
  positional_arguments.add("package_name", 1);
  positional_arguments.add("model_file", -1);

  po::variables_map args;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(positional_arguments).run(), args);
  po::notify(args);

  // Initialize Logger
  auto logger_level = boost::log::trivial::warning;
  if (args.count("verbose")) {
    logger_level = boost::log::trivial::trace;
  }
  boost::log::core::get()->set_filter(
     boost::log::trivial::severity >= logger_level);

  // Help
  if (args.count("help")) {
    std::cout << "\n" << "Create behavior tree package." << "\n";
    std::cout << desc << std::endl;
    return 0;
  }

  // Wrong usage
  if (package_name.empty()) {
    std::cout << desc << std::endl;
    return 1;
  }

  // resize up to the number of model_files
  executable_names.resize(model_files.size());

  // fill executable_names with defaults
  for (auto model_it = model_files.begin(), exec_it = executable_names.begin();
       model_it != model_files.end();
       ++model_it, ++exec_it) {
    std::string model_file = *model_it;
    std::string executable = *exec_it;

    if (executable.empty())
      *exec_it = boost::filesystem::path(model_file).stem().string();
  }

  RoseusBT::PackageGenerator<RoseusBT::XMLParser> pg(package_name,
                                                     model_files, executable_names,
                                                     author, args.count("overwrite"));
  pg.write_all_files();

  return 0;
}
