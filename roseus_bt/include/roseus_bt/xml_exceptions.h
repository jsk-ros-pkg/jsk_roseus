#ifndef BEHAVIOR_TREE_ROSEUS_BT_XML_EXCEPTIONS_
#define BEHAVIOR_TREE_ROSEUS_BT_XML_EXCEPTIONS_

#include <string>
#include <stdexcept>
#include <tinyxml2.h>
#include <fmt/format.h>


namespace XMLError
{

using namespace tinyxml2;

class XMLError: public std::exception {
public:
  XMLError(std::string message):  message(message) {}

  const char* what() const noexcept {
    return message.c_str();
  }

  std::string message;
};

std::string get_place(const XMLElement* node) {
  XMLPrinter printer;
  node->Accept(&printer);
  return fmt::format(" at line {}: {}", node->GetLineNum(), printer.CStr());
};

class FileNotFound: public XMLError {
public:
  FileNotFound(std::string filename) :
    XMLError(fmt::format("File not found: {}", filename)) {};
};

class ParsingError: public XMLError {
public:
  ParsingError() : XMLError("Could not parse the XML file") {};
};

class WrongRoot: public XMLError {
public:
  WrongRoot() : XMLError("The XML must have a root node called <root>") {};
};

class MissingRequiredNode: public XMLError {
public:
  MissingRequiredNode(std::string node_type) :
    XMLError(fmt::format("The XML must have a <{}> node", node_type)) {};
};

class MissingRequiredAttribute: public XMLError {
public:
  MissingRequiredAttribute(std::string attribute, const XMLElement* node) :
    XMLError(fmt::format("Missing \"{}\" attribute{}", attribute, get_place(node)))
  {};
};

class UnknownNode: public XMLError {
public:
  UnknownNode(const XMLElement* node) :
    XMLError(fmt::format("Unknown node type {}{}", node->Name(), get_place(node))) {};
};

class UnknownPortNode: public XMLError {
public:
  UnknownPortNode(const XMLElement* node) :
    XMLError(fmt::format("Unknown port node {}{}", node->Name(), get_place(node))) {};
};

class InvalidTopicType: public XMLError {
public:
  InvalidTopicType(std::string type, const XMLElement* node) :
    XMLError(fmt::format("Invalid topic type {}{}", type, get_place(node))) {};
};

class InvalidPortType: public XMLError {
public:
  InvalidPortType(std::string type, const XMLElement* node) :
    XMLError(fmt::format("Invalid port type {}{}", type, get_place(node))) {};
};

}  // namespace XMLError

#endif  // BEHAVIOR_TREE_ROSEUS_BT_XML_EXCEPTIONS_
