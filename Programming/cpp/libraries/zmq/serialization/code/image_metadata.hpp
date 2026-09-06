#pragma once

#include <msgpack.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

struct ImageMetadata
{
    std::uint32_t version{1};
    std::uint64_t frame_id{};
    std::int64_t timestamp_ns{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint8_t channels{};
    std::string dtype{"uint8"};
    std::string encoding{"raw"};
    std::uint64_t payload_size{};

    MSGPACK_DEFINE_MAP(version, frame_id, timestamp_ns, width, height, channels,
                       dtype, encoding, payload_size)
};

inline std::optional<std::string> validate(const ImageMetadata& metadata,
                                           std::size_t actual_payload_size)
{
    if (metadata.version != 1)
        return "unsupported version";
    if (metadata.width == 0 || metadata.height == 0)
        return "image dimensions must be positive";
    if (metadata.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
        || metadata.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        return "image dimensions exceed OpenCV limits";
    if (metadata.channels != 1 && metadata.channels != 3)
        return "only one or three channels are supported";
    if (metadata.dtype != "uint8" || metadata.encoding != "raw")
        return "only raw uint8 images are supported";

    const auto expected = static_cast<std::uint64_t>(metadata.width)
                        * metadata.height * metadata.channels;
    if (metadata.payload_size != expected || actual_payload_size != expected)
        return "payload size does not match image metadata";

    return std::nullopt;
}
