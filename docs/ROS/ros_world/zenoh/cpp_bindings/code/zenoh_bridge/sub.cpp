#include <iostream>
#include <thread>
#include "zenoh.hxx"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
// sudo apt install libfastcdr-dev


using namespace zenoh;

std_msgs::msg::Int32 deserialize_ros_message(const std::vector<uint8_t> &buffer)
{
    rclcpp::SerializedMessage serialized_msg;

    // Resize internal buffer and copy
    serialized_msg.get_rcl_serialized_message().buffer_capacity = buffer.size();
    serialized_msg.get_rcl_serialized_message().buffer_length = buffer.size();
    serialized_msg.get_rcl_serialized_message().buffer = reinterpret_cast<uint8_t *>(malloc(buffer.size()));
    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, buffer.data(), buffer.size());

    rclcpp::Serialization<std_msgs::msg::Int32> serializer;
    std_msgs::msg::Int32 msg;
    serializer.deserialize_message(&serialized_msg, &msg);

    

    return msg;
}

int main(int argc, char **argv) {
   Config config = Config::create_default();
   auto session = Session::open(std::move(config));
   auto subscriber = session.declare_subscriber(
      KeyExpr("my_int32_topic"),
      [](const Sample& sample) {
        auto buffer = sample.get_payload().as_vector();
            auto msg = deserialize_ros_message(buffer);
            std::cout << "Received: " << msg.data << std::endl;

      },
      closures::none
   );
   // Wait for a key press to exit
   char c = getchar();
}