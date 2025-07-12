#include <iostream>
#include <thread>
#include "zenoh.hxx"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

using namespace zenoh;

std::vector<uint8_t> serialize_ros_message(const std_msgs::msg::Int32 & msg)
{
    rclcpp::Serialization<std_msgs::msg::Int32> serializer;
    rclcpp::SerializedMessage serialized_msg;
    
    serializer.serialize_message(&msg, &serialized_msg);

    // Copy message block of memory to std::vector
    std::vector<uint8_t> buffer(
        serialized_msg.get_rcl_serialized_message().buffer,
        serialized_msg.get_rcl_serialized_message().buffer +
        serialized_msg.get_rcl_serialized_message().buffer_length);

    return buffer;
}

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

void task1()
{
    Config config = Config::create_default();
    auto session = Session::open(std::move(config));
    // auto publisher = session.declare_publisher(KeyExpr("demo/example/simple"));

    for (int i = 0; i < 5; ++i) {
        std_msgs::msg::Int32 msg;
        msg.data = i;
        auto buffer = serialize_ros_message(msg);

        // Publish using session.put
        session.put(KeyExpr("demo/example/simple"), buffer);

        // Publish using publisher.put
        // publisher.put("Simple from publisher.put!");

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}



int main()
{
    std::thread t1(task1);
    Config config = Config::create_default();
    auto session = Session::open(std::move(config));
    auto subscriber = session.declare_subscriber(
        KeyExpr("demo/example/simple"),
        [](const Sample &sample)
        {
            auto buffer = sample.get_payload().as_vector();
            auto msg = deserialize_ros_message(buffer);
            std::cout << "Received: " << msg.data << std::endl;
        },
        closures::none);

    t1.join(); // Wait for task1 to finish

    std::cout << "Both tasks completed.\n";
    return 0;
}
