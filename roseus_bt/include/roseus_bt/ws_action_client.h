#ifndef WS_ACTION_CLIENT_
#define WS_ACTION_CLIENT_

#include <rosbridgecpp/rosbridge_ws_client.hpp>
#include <fmt/format.h>

class RosbridgeActionClient
{
public:
  RosbridgeActionClient(const std::string& master, int port, const std::string& server_name, const std::string& action_type):
    rbc_(fmt::format("{}:{}", master, std::to_string(port))),
    server_name_(server_name)
  {
    goal_topic_ = fmt::format("{}/goal", server_name_);
    result_topic_ = fmt::format("{}/result", server_name_);
    cancel_topic_ = fmt::format("{}/cancel", server_name_);

    action_goal_type_ = fmt::format("{}Goal", action_type);

    rbc_.addClient("goal_publisher");
    rbc_.addClient("goal_advertiser");
    rbc_.addClient("cancel_advertiser");
    rbc_.addClient("result_subscriber");
    rbc_.advertise("goal_advertiser", goal_topic_, action_goal_type_);
    rbc_.advertise("cancel_advertiser", cancel_topic_, "actionlib_msgs/GoalID");
    auto res_sub = std::bind(&RosbridgeActionClient::resultCallback, this,
                             std::placeholders::_1,
                             std::placeholders::_2);
    rbc_.subscribe("result_subscriber", result_topic_, res_sub);
  }

  ~RosbridgeActionClient() {
    rbc_.removeClient("goal_publisher");
    rbc_.removeClient("goal_advertiser");
    rbc_.removeClient("cancel_advertiser");
    rbc_.removeClient("result_subscriber");
  }

  void sendGoal(const rapidjson::Value& goal) {
    // TODO: add header and goal_id

    rapidjson::Document action_goal;
    action_goal.SetObject();
    rapidjson::Value g(goal, action_goal.GetAllocator());
    action_goal.AddMember("goal", g, action_goal.GetAllocator());

    is_active_ = true;
    rbc_.publish(goal_topic_, action_goal);
  }

  void cancelGoal() {
    rapidjson::Document msg;
    msg.SetObject();
    is_active_ = false;
    rbc_.publish(cancel_topic_, msg);
  }

  bool isActive() {
    return is_active_;
  }

  rapidjson::Value getResult() {
    // TODO: reset result after getting
    return result_["msg"].GetObject()["result"].GetObject();
  }

// waitForServer

protected:
  RosbridgeWsClient rbc_;

  bool is_active_;
  rapidjson::Value result_;

  std::string server_name_;
  std::string goal_topic_;
  std::string result_topic_;
  std::string cancel_topic_;

  std::string action_goal_type_;

protected:

  void resultCallback(std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message)
  {
    std::string message = in_message->string();
    std::cout << "resultCallback(): Message Received: " << message << std::endl;

    rapidjson::Document document(rapidjson::kObjectType);
    document.Parse(message.c_str());
    rapidjson::Value res(document, document.GetAllocator());
    result_ = res;

    is_active_ = false;
  }

};

#endif  // WS_ACTION_CLIENT_
