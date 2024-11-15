#define CATCH_CONFIG_RUNNER
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

class TestTalker : public rclcpp::Node {
public:
    TestTalker() : Node("test_talker") {
        // Create a subscription to the Talker node's topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10, [this](std_msgs::msg::String::SharedPtr msg) {
                last_message_ = msg->data;
                message_received_ = true;
            }
        );
    }

    bool wait_for_message(const std::chrono::seconds &timeout) {
        auto start = std::chrono::steady_clock::now();
        while (!message_received_ && std::chrono::steady_clock::now() - start < timeout) {
            rclcpp::spin_some(this->get_node_base_interface());
        }
        return message_received_;
    }

    std::string get_last_message() const {
        return last_message_;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string last_message_;
    bool message_received_ = false;
};

TEST_CASE("Talker Node publishes expected messages", "[Talker]") {
    auto test_node = std::make_shared<TestTalker>();

    // Check if a message is received within the timeout
    REQUIRE(test_node->wait_for_message(std::chrono::seconds(5)) == true);

    // Capture the received message for better visibility in test output
    auto received_message = test_node->get_last_message();
    CAPTURE(received_message);

    // Expected message content
    std::string expected_message = "I'm not set";
    CAPTURE(expected_message);

    // Test that the received message matches the expected one
    REQUIRE(received_message == expected_message);

    // Output success details if the test passed
    INFO("Test passed! Received message: " << received_message);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    int result = Catch::Session().run(argc, argv);
    rclcpp::shutdown();
    return result;
}
