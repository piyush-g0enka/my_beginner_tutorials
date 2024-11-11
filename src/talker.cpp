// Copyright 2024 Piyush Goenka

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"){
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Declare the 'publish_frequency' parameter with a default value of 10.0 Hz
        this->declare_parameter<double>("publish_frequency", 10.0);

        // Retrieve the 'publish_frequency' parameter value
        double publish_frequency;
        this->get_parameter("publish_frequency", publish_frequency);


// Create the service with std::bind to bind `this`
service_ =
    this->create_service<example_interfaces::srv::SetBool>(
        "string_changer",
        std::bind(&MinimalPublisher::string_change_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency)), std::bind(&MinimalPublisher::timer_callback, this));
  }


  void string_change_cb(   const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
{
  response->success = true;
  response->message = "String changed!";

  if (request->data)
  chatter_string = "I'm publishing TRUE";

  else
  chatter_string = "I'm publishing FALSE";



  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);


}

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = chatter_string;
    RCLCPP_INFO(this->get_logger(), "publishing");
    publisher_->publish(message);
  }

rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  std::string chatter_string = {"I'm not set"};
  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
