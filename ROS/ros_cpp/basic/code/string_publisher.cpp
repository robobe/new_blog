#include "cpp_demo/string_publisher.hpp"
#include <fmt/core.h>

using namespace std::chrono_literals;

StringPublisher::StringPublisher()
: Node("string_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&StringPublisher::timer_callback, this));
}

void StringPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = fmt::format("Hello, ROS 2! {}", count_++);
    RCLCPP_INFO(this->get_logger(), "%s", fmt::format("Publishing: {}", message.data).c_str());
    publisher_->publish(message);
}
