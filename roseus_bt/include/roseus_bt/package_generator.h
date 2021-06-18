#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <fmt/format.h>
#include <boost/filesystem.hpp>
#include <roseus_bt/xml_parser.h>


namespace RoseusBT
{

class PackageGenerator
{
public:
  PackageGenerator(const char* package_name,
                   const char* xml_filename,
                   const char* roscpp_node_name,
                   const char* target_filename,
                   const char* author_name) :
    parser(xml_filename),
    xml_filename(xml_filename),
    package_name(package_name),
    roscpp_node_name(roscpp_node_name),
    target_filename(target_filename),
    author_name(author_name) {};

  ~PackageGenerator() {};

private:
  XMLParser parser;
  const char* package_name;
  std::string xml_filename;
  const char* roscpp_node_name;
  const char* target_filename;
  const char* author_name;

public:
  void copy_xml_file();
  void write_action_files();
  void write_service_files();
  void write_cpp_file();
  void write_cmake_lists();
  void write_package_xml();
  void write_all_files();

};

void PackageGenerator::copy_xml_file() {
  std::string base_dir = fmt::format("{}/models", package_name);
  std::string dest_file = fmt::format("{}/{}",
      base_dir,
      boost::filesystem::path(xml_filename).filename().c_str());

  boost::filesystem::create_directories(base_dir);
  boost::filesystem::copy_file(xml_filename, dest_file,
                               boost::filesystem::copy_option::overwrite_if_exists);

  xml_filename = dest_file;
}

void PackageGenerator::write_action_files() {
  std::string base_dir = fmt::format("{}/action", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> action_files;
  std::map<std::string, std::string>::iterator it;
  action_files = parser.generate_all_action_files();

  for (it=action_files.begin(); it!=action_files.end(); ++it) {
    std::ofstream output_file(fmt::format("{}/{}", base_dir, it->first));
    output_file << it->second;
    output_file.close();
  }
}

void PackageGenerator::write_service_files() {
  std::string base_dir = fmt::format("{}/srv", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> action_files;
  std::map<std::string, std::string>::iterator it;
  action_files = parser.generate_all_service_files();

  for (it=action_files.begin(); it!=action_files.end(); ++it) {
    std::ofstream output_file(fmt::format("{}/{}", base_dir, it->first));
    output_file << it->second;
    output_file.close();
  }
}

void PackageGenerator::write_cpp_file() {
  std::string base_dir = fmt::format("{}/src", package_name);
  boost::filesystem::create_directories(base_dir);

  std::ofstream output_file(fmt::format("{}/{}.cpp", base_dir, target_filename));

  output_file << parser.generate_cpp_file(package_name, roscpp_node_name,
                                          boost::filesystem::absolute(xml_filename).c_str());
  output_file.close();
}

void PackageGenerator::write_cmake_lists() {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::ofstream output_file(fmt::format("{}/CMakeLists.txt", base_dir));
  output_file << parser.generate_cmake_lists(package_name, target_filename);
  output_file.close();
}

void PackageGenerator::write_package_xml() {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::ofstream output_file(fmt::format("{}/package.xml", base_dir));
  output_file << parser.generate_package_xml(package_name, author_name);
  output_file.close();
}

void PackageGenerator::write_all_files() {
  copy_xml_file();
  write_action_files();
  write_service_files();
  write_cpp_file();
  write_cmake_lists();
  write_package_xml();
}

}  // namespaceRoseusBT
