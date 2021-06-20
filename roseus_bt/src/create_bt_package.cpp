#include <iostream>
#include <roseus_bt/package_generator.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
  std::string package_name, model_file, author, executable, node_name;

  po::options_description desc("usage");
  desc.add_options()
    ("help,h", "show this help message and exit")
    ("package_name", po::value<std::string>(&package_name), "package name")
    ("model_file", po::value<std::string>(&model_file), "model file")
    ("executable,e", po::value<std::string>(&executable)->default_value("mynode"),
     "executable name")
    ("author,a", po::value<std::string>(&author)->default_value("The Author"),
     "author name");

  po::positional_options_description positional_arguments;
  positional_arguments.add("package_name", 1);
  positional_arguments.add("model_file", 1);

  po::variables_map args;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(positional_arguments).run(), args);
  po::notify(args);

  // Help
  if (args.count("help")) {
    std::cout << "\n" << "Create behavior tree package." << "\n";
    std::cout << desc << std::endl;
    return 0;
  }

  // Wrong usage
  if (package_name.empty() || model_file.empty()) {
    std::cout << desc << std::endl;
    return 1;
  }

  RoseusBT::PackageGenerator pg(package_name, model_file, executable, author);
  pg.write_all_files();

  return 0;
}
