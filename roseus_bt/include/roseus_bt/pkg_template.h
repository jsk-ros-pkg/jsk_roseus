#ifndef BEHAVIOR_TREE_ROSEUS_BT_PKG_TEMPLATE_
#define BEHAVIOR_TREE_ROSEUS_BT_PKG_TEMPLATE_

#include <string>
#include <vector>
#include <fmt/format.h>
#include <boost/format.hpp>
#include <boost/algorithm/string/join.hpp>

namespace RoseusBT
{

class PkgTemplate
{
public:
  PkgTemplate() {};
  ~PkgTemplate() {};

protected:
  std::string cmake_lists_template(std::string package_name,
                                   std::vector<std::string> message_packages,
                                   std::vector<std::string> service_files,
                                   std::vector<std::string> action_files,
                                   std::vector<std::string> add_executables);
  std::string package_xml_template(std::string package_name,
                                   std::string author_name,
                                   std::vector<std::string> build_dependencies,
                                   std::vector<std::string> exec_dependencies);

public:
  std::string generate_cmake_lists(std::string package_name,
                                   std::vector<std::string> executables,
                                   std::vector<std::string> message_packages,
                                   std::vector<std::string> service_files,
                                   std::vector<std::string> action_files);
  std::string generate_package_xml(std::string package_name,
                                   std::string author_name,
                                   std::vector<std::string> message_packages);
};


std::string PkgTemplate::cmake_lists_template(std::string package_name,
                                              std::vector<std::string> message_packages,
                                              std::vector<std::string> service_files,
                                              std::vector<std::string> action_files,
                                              std::vector<std::string> add_executables) {
  std::string fmt_string = 1+ R"(
cmake_minimum_required(VERSION 3.0.2)
project(%1%)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  behaviortree_ros
  roseus_bt
%2%
)

add_service_files(
  FILES
%3%
)

add_action_files(
  FILES
%4%
)

generate_messages(
  DEPENDENCIES
%2%
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  message_runtime
%2%
)


include_directories(${catkin_INCLUDE_DIRS})

%5%
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    boost::algorithm::join(message_packages, "\n") %
    boost::algorithm::join(service_files, "\n") %
    boost::algorithm::join(action_files, "\n") %
    boost::algorithm::join(add_executables, "\n");

  return bfmt.str();      
}


std::string PkgTemplate::package_xml_template(std::string package_name,
                                              std::string author_name,
                                              std::vector<std::string> build_dependencies,
                                              std::vector<std::string> exec_dependencies) {

  std::string author_email(author_name);
  std::transform(author_email.begin(), author_email.end(), author_email.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  std::replace(author_email.begin(), author_email.end(), ' ', '_');

  std::string fmt_string = 1 + R"(
<?xml version="1.0"?>
<package format="2">
  <name>%1%</name>
  <version>0.0.0</version>
  <description>The %1% package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <maintainer email="%3%@example.com">%2%</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/%1%</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="%3%@example.com">%2%</author> -->


  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>behaviortree_ros</build_depend>
  <build_depend>roseus_bt</build_depend>
%4%

  <exec_depend>message_runtime</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>behaviortree_ros</exec_depend>
  <exec_depend>roseus_bt</exec_depend>
%5%

  <export>
  </export>
</package>
)";

  boost::format bfmt = boost::format(fmt_string) %
    package_name %
    author_name %
    author_email %
    boost::algorithm::join(build_dependencies, ",\n") %
    boost::algorithm::join(exec_dependencies, ",\n");

  return bfmt.str();
}


std::string PkgTemplate::generate_cmake_lists(std::string package_name,
                                              std::vector<std::string> executables,
                                              std::vector<std::string> message_packages,
                                              std::vector<std::string> service_files,
                                              std::vector<std::string> action_files) {
  auto format_pkg = [](std::string pkg) {
    return fmt::format("  {}", pkg);
  };

  auto format_executable = [](std::string target_name) {
    std::string fmt_string = 1+ R"(
add_executable(%1% src/%1%.cpp)
add_dependencies(%1% ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(%1% ${catkin_LIBRARIES})

)";

    boost::format bfmt = boost::format(fmt_string) %
    target_name;
    return bfmt.str();
  };


  std::transform(message_packages.begin(), message_packages.end(),
                 message_packages.begin(), format_pkg);
  std::transform(executables.begin(), executables.end(),
                 executables.begin(), format_executable);

  return cmake_lists_template(package_name, message_packages,
                              service_files, action_files,
                              executables);
}


std::string PkgTemplate::generate_package_xml(std::string package_name,
                                              std::string author_name,
                                              std::vector<std::string> message_packages)  {
  auto format_build_depend = [](std::string pkg) {
    return fmt::format("  <build_depend>{}</build_depend>", pkg);
  };
  auto format_exec_depend = [](std::string pkg) {
    return fmt::format("  <exec_depend>{}</exec_depend>", pkg);
  };
  std::vector<std::string> build_dependencies;
  std::vector<std::string> exec_dependencies;
  build_dependencies.resize(message_packages.size());
  exec_dependencies.resize(message_packages.size());

  std::transform(message_packages.begin(), message_packages.end(),
                 build_dependencies.begin(), format_build_depend);
  std::transform(message_packages.begin(), message_packages.end(),
                 exec_dependencies.begin(), format_exec_depend);

  return package_xml_template(package_name, author_name,
                              build_dependencies, exec_dependencies);
}


}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_PKG_TEMPLATE_
