#include <map>
#include <string>
#include <regex>
#include <vector>
#include <fstream>
#include <iostream>
#include <fmt/format.h>
#include <boost/filesystem.hpp>
#include <roseus_bt/xml_parser.h>
#include <roseus_bt/pkg_template.h>


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
                   const std::vector<std::string> xml_filenames,
                   const std::vector<std::string> target_filenames,
                   const std::string author_name,
                   bool force_overwrite) :
    query(),
    pkg_template(),
    parser_vector(xml_filenames.begin(), xml_filenames.end()),
    package_name(package_name),
    xml_filenames(xml_filenames),
    target_filenames(target_filenames),
    author_name(author_name),
    force_overwrite(force_overwrite)
  {};

  ~PackageGenerator() {};

private:
  Query query;
  PkgTemplate pkg_template;
  std::vector<XMLParser> parser_vector;
  std::string package_name;
  std::vector<std::string> xml_filenames;
  std::vector<std::string> target_filenames;
  std::string author_name;
  bool force_overwrite;

protected:
  bool overwrite(const std::string filename);

public:
  void copy_xml_file(std::string* xml_filename);
  void write_action_files(XMLParser* parser);
  void write_service_files(XMLParser* parser);
  void write_cpp_file(XMLParser* parser,
                      const std::string target_filename, const std::string xml_filename);
  void write_eus_action_server(XMLParser* parser, const std::string target_filename);
  void write_eus_condition_server(XMLParser* parser, const std::string target_filename);
  void write_cmake_lists(const std::vector<std::string> message_packages,
                         const std::vector<std::string> service_files,
                         const std::vector<std::string> action_files);
  void write_package_xml(const std::vector<std::string> message_packages);
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

void PackageGenerator::copy_xml_file(std::string* xml_filename) {
  std::string base_dir = fmt::format("{}/models", package_name);
  std::string dest_file = fmt::format("{}/{}",
      base_dir,
      boost::filesystem::path(*xml_filename).filename().c_str());

  if (dest_file == *xml_filename)
    return;

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  boost::filesystem::create_directories(base_dir);
  boost::filesystem::copy_file(*xml_filename, dest_file,
                               boost::filesystem::copy_option::overwrite_if_exists);
  *xml_filename = dest_file;
}

void PackageGenerator::write_action_files(XMLParser* parser) {
  std::string base_dir = fmt::format("{}/action", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> action_files = parser->generate_all_action_files();

  for (auto it=action_files.begin(); it!=action_files.end(); ++it) {
    std::ofstream output_file(fmt::format("{}/{}", base_dir, it->first));
    output_file << it->second;
    output_file.close();
  }
}

void PackageGenerator::write_service_files(XMLParser* parser) {
  std::string base_dir = fmt::format("{}/srv", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> service_files = parser->generate_all_service_files();

  for (auto it=service_files.begin(); it!=service_files.end(); ++it) {
    std::ofstream output_file(fmt::format("{}/{}", base_dir, it->first));
    output_file << it->second;
    output_file.close();
  }
}

void PackageGenerator::write_cpp_file(XMLParser* parser,
                                      const std::string target_filename,
                                      const std::string xml_filename) {
  std::string base_dir = fmt::format("{}/src", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string roscpp_node_name = fmt::format("{}_engine", target_filename);
  std::string dest_file = fmt::format("{}/{}.cpp", base_dir, target_filename);

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << parser->generate_cpp_file(package_name, roscpp_node_name,
                        boost::filesystem::absolute(xml_filename).c_str());
  output_file.close();
}

void PackageGenerator::write_eus_action_server(XMLParser* parser,
                                               const std::string target_filename) {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/{}-action-server.l", base_dir, target_filename);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::string body = parser->generate_eus_action_server(package_name);
  if (body.empty()) return;

  std::ofstream output_file(dest_file);
  output_file << body;
  output_file.close();
}

void PackageGenerator::write_eus_condition_server(XMLParser* parser,
                                                  const std::string target_filename) {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/{}-condition-server.l", base_dir, target_filename);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::string body = parser->generate_eus_condition_server(package_name);
  if (body.empty()) return;

  std::ofstream output_file(dest_file);
  output_file << body;
  output_file.close();
}

void PackageGenerator::write_cmake_lists(const std::vector<std::string> message_packages,
                                         const std::vector<std::string> service_files,
                                         const std::vector<std::string> action_files) {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/CMakeLists.txt", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << pkg_template.generate_cmake_lists(package_name, target_filenames,
                                                   message_packages,
                                                   service_files,
                                                   action_files);
  output_file.close();
}

void PackageGenerator::write_package_xml(const std::vector<std::string> message_packages) {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/package.xml", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  std::ofstream output_file(dest_file);
  output_file << pkg_template.generate_package_xml(package_name, author_name,
                                                   message_packages);
  output_file.close();
}

void PackageGenerator::write_all_files() {
  std::vector<std::string> message_packages;
  std::vector<std::string> action_files;
  std::vector<std::string> service_files;

  message_packages.push_back("std_msgs");
  message_packages.push_back("actionlib_msgs");

  for (int i=0; i<parser_vector.size(); i++) {
    XMLParser* parser = &parser_vector.at(i);
    std::string xml_filename = xml_filenames.at(i);
    std::string target_filename = target_filenames.at(i);

    parser->push_dependencies(&message_packages, &service_files, &action_files);
    copy_xml_file(&xml_filename);
    write_action_files(parser);
    write_service_files(parser);
    write_cpp_file(parser, target_filename, xml_filename);
    write_eus_action_server(parser, target_filename);
    write_eus_condition_server(parser, target_filename);
  }

  write_cmake_lists(message_packages, service_files, action_files);
  write_package_xml(message_packages);
}

}  // namespaceRoseusBT
