#include <charconv>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>

std::optional<int> read_retry_count(int argc, char* argv[])
{
    if (argc < 2)
        return std::nullopt;

    const std::string_view text{argv[1]};
    int value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);

    if (error != std::errc{} || end != text.data() + text.size() || value < 0)
        throw std::invalid_argument{"retry count must be a non-negative integer"};

    return value;
}

int main(int argc, char* argv[])
{
    try
    {
        const auto configured = read_retry_count(argc, argv);
        std::cout << "Retry count: " << configured.value_or(3)
                  << (configured ? " (configured)\n" : " (default)\n");
    }
    catch (const std::invalid_argument& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
