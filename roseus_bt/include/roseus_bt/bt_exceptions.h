#ifndef BEHAVIOR_TREE_ROSEUS_BT_BT_EXCEPTIONS_
#define BEHAVIOR_TREE_ROSEUS_BT_BT_EXCEPTIONS_

#include <string>
#include <stdexcept>
#include <fmt/format.h>

namespace RoseusBT
{

class BTError: public std::exception {
public:
  BTError(std::string message):  message(message) {}

  const char* what() const noexcept {
    return message.c_str();
  }

  std::string message;
};

class InvalidOutputPort: public BTError {
public:
  InvalidOutputPort() : BTError("Condition Node only accepts input ports!") {};
};

class InvalidPackageName: public BTError {
public:
  InvalidPackageName(std::string name) :
    BTError(fmt::format("Package name {} does not follow naming conventions", name)) {};
};

class InvalidInput: public BTError {
public:
  InvalidInput() : BTError("Invalid Input") {};
};


}  // namespace RoseusBT

#endif  // BEHAVIOR_TREE_ROSEUS_BT_BT_EXCEPTIONS_
