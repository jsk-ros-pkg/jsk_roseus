#include <map>
#include <string>
#include <regex>
#include <vector>
#include <fstream>
#include <iostream>
#include <fmt/format.h>
#include <boost/filesystem.hpp>
#include <roseus_bt/xml_parser.h>


namespace RoseusBT
{

class Query
{
private:
  const std::regex yes, no;
public:
  Query() :
    yes("y|yes", std::regex::icase | std::regex::optimize),
    no("n|no", std::regex::icase | std::regex::optimize)
  {};
  ~Query() {};

  bool yn(const std::string message);
};

class PackageGenerator
{
public:
  PackageGenerator(const std::string package_name,
                   const std::string xml_filename,
                   const std::string target_filename,
                   const std::string author_name,
                   bool force_overwrite) :
    query(),
    parser(xml_filename),
    xml_filename(xml_filename),
    package_name(package_name),
    target_filename(target_filename),
    author_name(author_name),
    force_overwrite(force_overwrite)
  {};

  ~PackageGenerator() {};

private:
  Query query;
  XMLParser parser;
  std::string package_name;
  std::string xml_filename;
  std::string target_filename;
  std::string author_name;
  bool force_overwrite;

protected:
  bool overwrite(const std::string filename);

public:
  void copy_xml_file();
  void write_action_files();
  void write_service_files();
  void write_cpp_file();
  void write_eus_action_server();
  void write_eus_condition_server();
  void write_cmake_lists();
  void write_package_xml();
  void write_all_files();
};


bool Query::yn(const std::string message) {
  auto prompt = [message]() {
    std::cout << message << " [Y/n] ";
  };

  std::string answer;

  while(prompt(), std::cin >> answer) {
    if (std::regex_match(answer, yes)) return true;
    if (std::regex_match(answer, no)) return false;
  }

  throw std::logic_error("Invalid input");
}

bool PackageGenerator::overwrite(const std::string filename) {
  return force_overwrite || query.yn(fmt::format("Overwrite {}?", filename));
}

void PackageGenerator::copy_xml_file() {
  std::string base_dir = fmt::format("{}/models", package_name);
  std::string dest_file = fmt::format("{}/{}",
      base_dir,
      boost::filesystem::path(xml_filename).filename().c_str());

  if (dest_file == xml_filename)
    return;

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

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

  std::string roscpp_node_name = fmt::format("{}_engine", target_filename);
  std::string dest_file = fmt::format("{}/{}.cpp", base_dir, target_filename);

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << parser.generate_cpp_file(package_name, roscpp_node_name,
                                          boost::filesystem::absolute(xml_filename).c_str());
  output_file.close();
}

void PackageGenerator::write_eus_action_server() {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/{}-action-server.l", base_dir, target_filename);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::string body = parser.generate_eus_action_server(package_name);
  if (body.empty()) return;

  std::ofstream output_file(dest_file);
  output_file << body;
  output_file.close();
}

void PackageGenerator::write_eus_condition_server() {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/{}-condition-server.l", base_dir, target_filename);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::string body = parser.generate_eus_condition_server(package_name);
  if (body.empty()) return;

  std::ofstream output_file(dest_file);
  output_file << body;
  output_file.close();
}

void PackageGenerator::write_cmake_lists() {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/CMakeLists.txt", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << parser.generate_cmake_lists(package_name, target_filename);
  output_file.close();
}

void PackageGenerator::write_package_xml() {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/package.xml", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << parser.generate_package_xml(package_name, author_name);
  output_file.close();
}

void PackageGenerator::write_all_files() {
  copy_xml_file();
  write_action_files();
  write_service_files();
  write_cpp_file();
  write_eus_action_server();
  write_eus_condition_server();
  write_cmake_lists();
  write_package_xml();
}

}  // namespaceRoseusBT
