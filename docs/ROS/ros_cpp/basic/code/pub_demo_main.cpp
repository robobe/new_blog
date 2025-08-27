#include "cpp_demo/string_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringPublisher>());
    rclcpp::shutdown();
    return 0;
}
