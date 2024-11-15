// Copyright 2024 Piyush Goenka

/**
 * @file talker.cpp
 * @author Piyush Goenka
 * @brief The talker node publishes a string to the chatter topic and publishes static TF transform between /world and /talk frames.
 * It also spins a service which when called changes the string contents.
 * @version 1.0
 */

#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <catch2/catch.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @brief Talker node class
 *
 * @details The talker node publishes a string to the chatter topic and publishes static TF transform between /world and /talk frames.
 * It also spins a service which when called changes the string contents.
 */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher") {
    tf_static_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Publish static transforms once at startup
    this->make_transforms();

    // create publisher to chatter
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Declare the 'publish_frequency' parameter with a default value of 10.0 Hz
    this->declare_parameter<double>("publish_frequency", 10.0);

    // Retrieve the 'publish_frequency' parameter value
    double publish_frequency;
    this->get_parameter("publish_frequency", publish_frequency);

    // Create the service with std::bind to bind `this`
    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "string_changer",
        std::bind(&MinimalPublisher::string_change_cb, this,
                  std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency)),
        std::bind(&MinimalPublisher::timer_callback, this));
  }




  /**
   * @brief Callback function for the service which sets the string value to be
   * published on chatter topic
   */
  void string_change_cb(
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    response->success = true;
    response->message = "String changed!";

    if (request->data)
      chatter_string = "I'm publishing TRUE";

    else
      chatter_string = "I'm publishing FALSE";

    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "Warning! String changed to '" << chatter_string << "'");
  }


 private:
  /**
   * @brief Callback of a timer which is used to publish string data onto
   * chatter at a certain rate
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = chatter_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Publishing to Chatter: " << chatter_string);
    publisher_->publish(message);
  }

/**
   * @brief Function which defines the transform between /world parent and /talk child
   */
  void make_transforms() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 1.8;
    t.transform.translation.z = 0.7;
    tf2::Quaternion q;
    q.setRPY(0.1, 0.3, 0.8);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_br->sendTransform(t);
  }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_br;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  std::string chatter_string = {"I'm not set"};
};

/**
 * @brief Main code where execution takes place
 */

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
