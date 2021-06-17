#include <iostream>
#include <roseus_bt/xml_parser.h>


int main(int argc, char **argv)
{
  RoseusBT::XMLParser parser(argv[1]);

  // std::cout << parser.generate_cpp_file("my_package", "test", argv[1]) << std::endl;
  std::cout << parser.generate_cmake_lists("my_package", "test") << std::endl;

  // std::cout << "Actions:" << std::endl;
  // std::cout << parser.test_all_actions() << std::endl;

  // std::cout << "Conditions:" << std::endl;
  // std::cout << parser.test_all_conditions() << std::endl;

  return 0;
}
