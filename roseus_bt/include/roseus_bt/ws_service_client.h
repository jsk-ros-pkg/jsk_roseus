#ifndef BEHAVIOR_TREE_ROSEUS_BT_WS_SERVICE_CLIENT_
#define BEHAVIOR_TREE_ROSEUS_BT_WS_SERVICE_CLIENT_

#include <rosbridgecpp/rosbridge_ws_client.hpp>
#include <behaviortree_cpp_v3/exceptions.h>
#include <fmt/format.h>


namespace roseus_bt
{

class RosbridgeServiceClient
{
public:
  RosbridgeServiceClient(const std::string& master, int port, const std::string& service_name):
    rbc_(fmt::format("{}:{}", master, std::to_string(port))),
    service_name_(service_name),
    result_(rapidjson::kObjectType),
    is_active_(false)
  {
    if (service_name_.front() != '/') {
      service_name_ = '/' + service_name_;
    }
  }

  ~RosbridgeServiceClient() {}

  bool call(const rapidjson::Document& request) {
    auto service_cb = std::bind(&RosbridgeServiceClient::serviceCallback, this,
                                std::placeholders::_1,
                                std::placeholders::_2);
    is_active_ = true;
    rbc_.callService(service_name_, service_cb, request);
    return true;
  }

  void cancelRequest() {
    // connection->send_close(1000);
  }

  bool isActive() {
    return is_active_;
  }

  rapidjson::Value getResult() {
    if (!(result_["result"].GetBool())) {
      std::string err = "Error calling remote service: " + service_name_;
      if (result_["values"].IsString()) {
        err += "\n  what():  ";
        err += result_["values"].GetString();
      }
      throw BT::RuntimeError(err);
    }
    // TODO: reset result after getting
    return result_["values"].GetObject();
  }

  void waitForResult() {
#ifdef DEBUG
    std::cout << "RemoteService: waiting for result: " << service_name_ << std::endl;
#endif
    while (is_active_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

// waitForServer

protected:
  RosbridgeWsClient rbc_;

  bool is_active_;
  rapidjson::Document result_;

  std::string service_name_;

protected:

  void serviceCallback(std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message)
  {
    std::string message = in_message->string();
#ifdef DEBUG
    std::cout << "serviceResponseCallback(): Message Received: " << message << std::endl;
#endif

    result_.Parse(message.c_str());

    is_active_ = false;
    connection->send_close(1000);
  }
};

}  // namespace roseus_bt

#endif  // BEHAVIOR_TREE_ROSEUS_BT_WS_SERVICE_CLIENT_
