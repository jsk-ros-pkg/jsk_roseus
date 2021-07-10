#ifndef BEHAVIOR_TREE_ROSEUS_BT_EXCEPTIONS_
#define BEHAVIOR_TREE_ROSEUS_BT_EXCEPTIONS_

#include <string>
#include <stdexcept>

namespace XMLError
{

class XMLError: public std::exception {
public:
  XMLError(std::string message):  message(message) {}

  const char* what() const noexcept {
    return message.c_str();
  }

  std::string message;
};

class NoAttribute: public XMLError {
public:
  NoAttribute(std::string message) : XMLError(message) {};
};

class NoNode: public XMLError {
public:
  NoNode(std::string message) : XMLError(message) {};
};

class UnknownNode: public XMLError {
public:
  UnknownNode(std::string message) : XMLError(message) {};
};

}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_EXCEPTIONS_
