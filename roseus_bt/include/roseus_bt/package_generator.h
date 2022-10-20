#ifndef BEHAVIOR_TREE_ROSEUS_BT_PACKAGE_GENERATOR_
#define BEHAVIOR_TREE_ROSEUS_BT_PACKAGE_GENERATOR_

#include <map>
#include <string>
#include <regex>
#include <vector>
#include <fstream>
#include <iostream>
#include <fmt/format.h>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <roseus_bt/pkg_template.h>
#include <roseus_bt/bt_exceptions.h>


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

template<class Parser>
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
  {
    // Check package name
    std::regex valid_naming ("^[a-zA-Z0-9][a-zA-Z0-9_-]*$");
    if (!std::regex_match(package_name, valid_naming)) {
      throw InvalidPackageName(package_name);
    }
  };

  ~PackageGenerator() {};

private:
  Query query;
  PkgTemplate pkg_template;
  std::vector<Parser> parser_vector;
  std::string package_name;
  std::vector<std::string> xml_filenames;
  std::vector<std::string> target_filenames;
  std::vector<std::string> euslisp_filenames;
  std::string author_name;
  bool force_overwrite;

protected:
  bool overwrite(const std::string filename);

public:
  void copy_xml_file(std::string* xml_filename);
  void write_action_files(Parser* parser);
  void write_service_files(Parser* parser);
  void write_launch_file(Parser* parser,
                         const std::string target_filename);
  void write_cpp_file(Parser* parser,
                      const std::string target_filename, const std::string xml_filename);
  void write_eus_action_server(Parser* parser, const std::string target_filename);
  void write_eus_condition_server(Parser* parser, const std::string target_filename);
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

  throw InvalidInput();
}

template<class Parser>
bool PackageGenerator<Parser>::overwrite(const std::string filename) {
  return force_overwrite || query.yn(fmt::format("Overwrite {}?", filename));
}

template<class Parser>
void PackageGenerator<Parser>::copy_xml_file(std::string* xml_filename) {
  std::string base_dir = fmt::format("{}/models", package_name);
  std::string dest_file = fmt::format("{}/{}",
      base_dir,
      boost::filesystem::path(*xml_filename).filename().c_str());

  if (dest_file == *xml_filename)
    return;

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
  boost::filesystem::create_directories(base_dir);
  boost::filesystem::copy_file(*xml_filename, dest_file,
                               boost::filesystem::copy_option::overwrite_if_exists);
  *xml_filename = dest_file;
}

template<class Parser>
void PackageGenerator<Parser>::write_action_files(Parser* parser) {
  std::string base_dir = fmt::format("{}/action", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> action_files = parser->generate_all_action_files();

  for (auto it=action_files.begin(); it!=action_files.end(); ++it) {
    std::string dest_file = fmt::format("{}/{}", base_dir, it->first);
    BOOST_LOG_TRIVIAL(debug) << "Writing " << dest_file << "...";
    std::ofstream output_file(dest_file);
    output_file << it->second;
    output_file.close();
  }
}

template<class Parser>
void PackageGenerator<Parser>::write_service_files(Parser* parser) {
  std::string base_dir = fmt::format("{}/srv", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> service_files = parser->generate_all_service_files();

  for (auto it=service_files.begin(); it!=service_files.end(); ++it) {
    std::string dest_file = fmt::format("{}/{}", base_dir, it->first);
    BOOST_LOG_TRIVIAL(debug) << "Writing " << dest_file << "...";
    std::ofstream output_file(dest_file);
    output_file << it->second;
    output_file.close();
  }
}

template<class Parser>
void PackageGenerator<Parser>::write_launch_file(Parser* parser,
                                                 const std::string target_filename) {
  std::string base_dir = fmt::format("{}/launch", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/{}_server.launch", base_dir, target_filename);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
  std::ofstream output_file(dest_file);
  output_file << parser->generate_launch_file(package_name, euslisp_filenames);
  output_file.close();
}

template<class Parser>
void PackageGenerator<Parser>::write_cpp_file(Parser* parser,
                                              const std::string target_filename,
                                              const std::string xml_filename) {
  std::string base_dir = fmt::format("{}/src", package_name);
  boost::filesystem::create_directories(base_dir);

  std::string roscpp_node_name = fmt::format("{}_engine", target_filename);
  std::string dest_file = fmt::format("{}/{}.cpp", base_dir, target_filename);

  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
  std::ofstream output_file(dest_file);
  output_file << parser->generate_cpp_file(package_name, roscpp_node_name,
                        boost::filesystem::absolute(xml_filename).c_str());
  output_file.close();
}

template<class Parser>
void PackageGenerator<Parser>::write_eus_action_server(Parser* parser,
                                                       const std::string target_filename) {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> server_files = parser->generate_all_eus_action_servers(package_name);

  for (auto it=server_files.begin(); it!=server_files.end(); ++it) {
    std::string remote_host = it->first;
    std::string body = it->second;
    std::string euslisp_filename = fmt::format("{}{}-action-server",
                                               target_filename, remote_host);
    std::replace(euslisp_filename.begin(), euslisp_filename.end(), '_', '-');
    std::string dest_file = fmt::format("{}/{}.l",
                                        base_dir, euslisp_filename);
    if (body.empty()) continue;
    euslisp_filenames.push_back(euslisp_filename);
    if (boost::filesystem::exists(dest_file) && !overwrite(dest_file)) continue;

    BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
    std::ofstream output_file(dest_file);
    output_file << body;
    output_file.close();
  }
}

template<class Parser>
void PackageGenerator<Parser>::write_eus_condition_server(Parser* parser,
                                                          const std::string target_filename) {
  std::string base_dir = fmt::format("{}/euslisp", package_name);
  boost::filesystem::create_directories(base_dir);

  std::map<std::string, std::string> server_files = parser->generate_all_eus_condition_servers(package_name);

  for (auto it=server_files.begin(); it!=server_files.end(); ++it) {
    std::string remote_host = it->first;
    std::string body = it->second;
    std::string euslisp_filename = fmt::format("{}{}-condition-server",
                                               target_filename, remote_host);
    std::replace(euslisp_filename.begin(), euslisp_filename.end(), '_', '-');
    std::string dest_file = fmt::format("{}/{}.l",
                                        base_dir, euslisp_filename);
    if (body.empty()) continue;
    euslisp_filenames.push_back(euslisp_filename);
    if (boost::filesystem::exists(dest_file) && !overwrite(dest_file)) continue;

    BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
    std::ofstream output_file(dest_file);
    output_file << body;
    output_file.close();
  }
}

template<class Parser>
void PackageGenerator<Parser>::write_cmake_lists(const std::vector<std::string> message_packages,
                                                 const std::vector<std::string> service_files,
                                                 const std::vector<std::string> action_files) {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/CMakeLists.txt", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
  std::ofstream output_file(dest_file);
  output_file << pkg_template.generate_cmake_lists(package_name, target_filenames,
                                                   message_packages,
                                                   service_files,
                                                   action_files);
  output_file.close();
}

template<class Parser>
void PackageGenerator<Parser>::write_package_xml(const std::vector<std::string> message_packages) {
  std::string base_dir = package_name;
  boost::filesystem::create_directories(base_dir);

  std::string dest_file = fmt::format("{}/package.xml", base_dir);
  if (boost::filesystem::exists(dest_file) && !overwrite(dest_file))
    return;

  BOOST_LOG_TRIVIAL(info) << "Writing " << dest_file << "...";
  std::ofstream output_file(dest_file);
  output_file << pkg_template.generate_package_xml(package_name, author_name,
                                                   message_packages);
  output_file.close();
}

template<class Parser>
void PackageGenerator<Parser>::write_all_files() {
  std::vector<std::string> message_packages;
  std::vector<std::string> action_files;
  std::vector<std::string> service_files;

  message_packages.push_back("std_msgs");
  message_packages.push_back("actionlib_msgs");

  for (int i=0; i<parser_vector.size(); i++) {
    Parser* parser = &parser_vector.at(i);
    std::string xml_filename = xml_filenames.at(i);
    std::string target_filename = target_filenames.at(i);
    euslisp_filenames.clear();

    BOOST_LOG_TRIVIAL(info) << "Generating " << xml_filename << " files...";

    BOOST_LOG_TRIVIAL(info) << "Checking dependencies...";
    parser->push_dependencies(&message_packages, &service_files, &action_files);

    copy_xml_file(&xml_filename);
    write_action_files(parser);
    write_service_files(parser);
    write_cpp_file(parser, target_filename, xml_filename);
    write_eus_action_server(parser, target_filename);
    write_eus_condition_server(parser, target_filename);
    write_launch_file(parser, target_filename);
  }

  write_cmake_lists(message_packages, service_files, action_files);
  write_package_xml(message_packages);
}


}  // namespaceRoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_PACKAGE_GENERATOR_
