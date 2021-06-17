#include <iostream>
#include <roseus_bt/xml_parser.h>

int main(int argc, char **argv)
{
  RoseusBT::XMLParser parser(argv[1]);

  // std::cout << parser.generate_cpp_file("my_package", "test", argv[1]) << std::endl;
  // std::cout << parser.generate_cmake_lists("my_package", "test") << std::endl;
  // std::cout << parser.generate_package_xml("my_package", "Guilherme Affonso") << std::endl;

  std::map<std::string, std::string> action_files, service_files;
  std::map<std::string, std::string>::iterator it;

  action_files = parser.generate_all_action_files();
  service_files = parser.generate_all_service_files();

  for (it=action_files.begin(); it!=action_files.end(); ++it) {
    std::cout << it->first << std::endl;
    std::cout << it->second << std::endl << std::endl;
  }

  for (it=service_files.begin(); it!=service_files.end(); ++it) {
    std::cout << it->first << std::endl;
    std::cout << it->second << std::endl << std::endl;
  }

  return 0;
}
