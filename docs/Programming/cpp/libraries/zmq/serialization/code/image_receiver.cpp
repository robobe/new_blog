#include "image_metadata.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <zmq.hpp>

#include <exception>
#include <iostream>
#include <string>

int main()
{
    zmq::context_t context{1};
    zmq::socket_t server{context, zmq::socket_type::rep};
    server.bind("tcp://*:5555");

    zmq::message_t metadata_frame;
    zmq::message_t pixel_frame;
    const auto metadata_size = server.recv(metadata_frame);
    const auto pixel_size = metadata_size && metadata_frame.more()
                          ? server.recv(pixel_frame) : zmq::recv_result_t{};

    try
    {
        if (!metadata_size || !pixel_size || pixel_frame.more())
            throw std::runtime_error{"expected exactly two frames"};

        const auto handle = msgpack::unpack(
            static_cast<const char*>(metadata_frame.data()), metadata_frame.size());
        ImageMetadata metadata;
        handle.get().convert(metadata);

        if (const auto error = validate(metadata, pixel_frame.size()))
            throw std::runtime_error{*error};

        const int type = metadata.channels == 1 ? CV_8UC1 : CV_8UC3;
        const cv::Mat view{static_cast<int>(metadata.height),
                           static_cast<int>(metadata.width), type,
                           pixel_frame.data()};
        const cv::Mat owned_image = view.clone();

        if (!cv::imwrite("received_image.png", owned_image))
            throw std::runtime_error{"could not save received_image.png"};

        const std::string reply = "OK frame_id=" + std::to_string(metadata.frame_id);
        server.send(zmq::buffer(reply), zmq::send_flags::none);
        std::cout << reply << " saved received_image.png\n";
    }
    catch (const std::exception& error)
    {
        const std::string reply = "ERROR: " + std::string{error.what()};
        server.send(zmq::buffer(reply), zmq::send_flags::none);
        std::cerr << reply << '\n';
        return 1;
    }
}
