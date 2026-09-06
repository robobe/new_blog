#include <zmq.hpp>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

int main()
{
    zmq::context_t context{1};
    zmq::socket_t client{context, zmq::socket_type::req};
    client.connect("tcp://localhost:5555");

    const std::string header{"sensor-frame"};
    const std::vector<std::uint8_t> payload{0x10, 0x20, 0x00, 0xFF};

    client.send(zmq::buffer(header), zmq::send_flags::sndmore);
    client.send(zmq::buffer(payload), zmq::send_flags::none);

    zmq::message_t reply;
    const auto received = client.recv(reply);
    if (!received)
    {
        std::cerr << "No reply received\n";
        return 1;
    }

    std::cout << reply.to_string() << '\n';
}
