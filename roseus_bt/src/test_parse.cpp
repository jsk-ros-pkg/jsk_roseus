#include <iostream>
#include <roseus_bt/xml_parser.h>


int main(int argc, char **argv)
{
  std::cout << "Start..." << std::endl;
  RoseusBT::XMLParser parser(argv[1]);

  std::cout << parser.test_all_actions() << std::endl;
  std::cout << "...End" << std::endl;

  return 0;
}
