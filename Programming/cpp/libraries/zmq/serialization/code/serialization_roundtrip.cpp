#include "image_metadata.hpp"

#include <msgpack.hpp>

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

struct Telemetry
{
    std::uint64_t sequence{};
    double temperature{};
    std::string status;

    MSGPACK_DEFINE_MAP(sequence, temperature, status)
};

int main()
{
    const Telemetry telemetry{7, 22.5, "ready"};
    msgpack::sbuffer telemetry_buffer;
    msgpack::pack(telemetry_buffer, telemetry);
    const auto telemetry_handle = msgpack::unpack(
        telemetry_buffer.data(), telemetry_buffer.size());
    Telemetry decoded_telemetry;
    telemetry_handle.get().convert(decoded_telemetry);
    assert(decoded_telemetry.sequence == telemetry.sequence);
    assert(decoded_telemetry.temperature == telemetry.temperature);
    assert(decoded_telemetry.status == telemetry.status);

    const std::array<std::uint8_t, 6> pixels{1, 2, 3, 4, 5, 6};
    const ImageMetadata original{
        .version = 1,
        .frame_id = 42,
        .timestamp_ns = 123456,
        .width = 3,
        .height = 2,
        .channels = 1,
        .dtype = "uint8",
        .encoding = "raw",
        .payload_size = pixels.size(),
    };

    msgpack::sbuffer buffer;
    msgpack::pack(buffer, original);
    const auto handle = msgpack::unpack(buffer.data(), buffer.size());
    ImageMetadata decoded;
    handle.get().convert(decoded);

    assert(decoded.version == original.version);
    assert(decoded.frame_id == original.frame_id);
    assert(decoded.width == original.width);
    assert(decoded.height == original.height);
    assert(decoded.channels == original.channels);
    assert(!validate(decoded, pixels.size()));
    assert(validate(decoded, pixels.size() - 1));

    decoded.version = 2;
    assert(validate(decoded, pixels.size()));
    decoded = original;
    decoded.channels = 2;
    assert(validate(decoded, pixels.size()));
    decoded = original;
    decoded.dtype = "float32";
    assert(validate(decoded, pixels.size()));

    std::cout << "struct and metadata round trips passed\n";
}
