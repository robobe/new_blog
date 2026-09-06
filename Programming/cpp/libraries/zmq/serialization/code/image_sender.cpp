#include "image_metadata.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zmq.hpp>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

namespace
{
cv::Mat make_test_image()
{
    cv::Mat image(64, 96, CV_8UC3);
    for (int row = 0; row < image.rows; ++row)
    {
        for (int column = 0; column < image.cols; ++column)
        {
            image.at<cv::Vec3b>(row, column) = {
                static_cast<std::uint8_t>(column),
                static_cast<std::uint8_t>(row),
                static_cast<std::uint8_t>((row + column) % 256)};
        }
    }
    return image;
}
}

int main(int argc, char* argv[])
{
    cv::Mat image = argc == 2 ? cv::imread(argv[1], cv::IMREAD_UNCHANGED)
                              : make_test_image();
    if (image.empty())
    {
        std::cerr << "Could not load the image\n";
        return 1;
    }
    if (image.depth() != CV_8U || (image.channels() != 1 && image.channels() != 3))
    {
        std::cerr << "Only uint8 grayscale and BGR images are supported\n";
        return 1;
    }
    if (!image.isContinuous())
        image = image.clone();

    const auto byte_count = image.total() * image.elemSize();
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const ImageMetadata metadata{
        .version = 1,
        .frame_id = 1,
        .timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count(),
        .width = static_cast<std::uint32_t>(image.cols),
        .height = static_cast<std::uint32_t>(image.rows),
        .channels = static_cast<std::uint8_t>(image.channels()),
        .dtype = "uint8",
        .encoding = "raw",
        .payload_size = byte_count,
    };

    msgpack::sbuffer metadata_buffer;
    msgpack::pack(metadata_buffer, metadata);

    zmq::context_t context{1};
    zmq::socket_t client{context, zmq::socket_type::req};
    client.connect("tcp://localhost:5555");
    client.send(zmq::buffer(metadata_buffer.data(), metadata_buffer.size()),
                zmq::send_flags::sndmore);
    client.send(zmq::buffer(image.data, byte_count), zmq::send_flags::none);

    zmq::message_t reply;
    const auto received = client.recv(reply);
    if (!received)
    {
        std::cerr << "No reply received\n";
        return 1;
    }

    std::cout << reply.to_string() << '\n';
}
