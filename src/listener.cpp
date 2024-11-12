// Copyright 2024 Piyush Goenka

/**
 * @file listener.cpp
 * @author Piyush Goenka
 * @brief The listener node listens to the messages that are being published
 * into the chatter topic
 * @version 1.0
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief Listener node class
 *
 * @details This node listens to the chatter topic and outputs the message on
 * the terminal
 */

class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the Node
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function for the chatter topic
   */
  void topic_callback(const std_msgs::msg::String &msg) const {
    if (msg.data == "I'm not set") {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"),
          "String is not set. Please call string-change service!");
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Chatter string: " << msg.data.c_str());
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main code where execution takes place
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
