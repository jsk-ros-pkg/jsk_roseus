#include <iostream>
#include <roseus_bt/xml_parser.h>


int main(int argc, char **argv)
{
  RoseusBT::XMLParser parser(argv[1]);

  std::cout << parser.generate_headers("my_package") << std::endl;
  std::cout << std::endl << std::endl;
  std::cout << parser.test_all_action_classes("my_package") << std::endl;
  std::cout << std::endl;
  std::cout << parser.test_all_condition_classes("my_package") << std::endl;

  // std::cout << "Actions:" << std::endl;
  // std::cout << parser.test_all_actions() << std::endl;

  // std::cout << "Conditions:" << std::endl;
  // std::cout << parser.test_all_conditions() << std::endl;

  return 0;
}
