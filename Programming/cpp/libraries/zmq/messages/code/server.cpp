#include <zmq.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

int main()
{
    zmq::context_t context{1};
    zmq::socket_t server{context, zmq::socket_type::rep};
    server.bind("tcp://*:5555");

    zmq::message_t header;
    const auto header_size = server.recv(header);
    if (!header_size || !header.more())
    {
        const std::string reply{"ERROR: expected header and payload"};
        server.send(zmq::buffer(reply), zmq::send_flags::none);
        return 1;
    }

    zmq::message_t payload;
    const auto payload_size = server.recv(payload);
    if (!payload_size || payload.more())
    {
        const std::string reply{"ERROR: expected exactly two frames"};
        server.send(zmq::buffer(reply), zmq::send_flags::none);
        return 1;
    }

    const auto* bytes = static_cast<const std::uint8_t*>(payload.data());
    std::cout << "Header: " << header.to_string() << '\n';
    std::cout << "Payload bytes:";
    for (std::size_t index = 0; index < payload.size(); ++index)
        std::cout << ' ' << static_cast<unsigned int>(bytes[index]);
    std::cout << '\n';

    const std::string reply{"OK: received 4 bytes"};
    server.send(zmq::buffer(reply), zmq::send_flags::none);
}
