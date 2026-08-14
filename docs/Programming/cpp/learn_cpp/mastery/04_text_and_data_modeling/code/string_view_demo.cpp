#include <algorithm>
#include <cstddef>
#include <iostream>
#include <string>
#include <string_view>

void print_prefix(std::string_view text, std::size_t length)
{
    const auto prefix_length = std::min(length, text.size());
    std::cout << "Prefix: " << text.substr(0, prefix_length) << '\n';
}

int main()
{
    std::string topic{"camera calibration"};
    print_prefix(topic, 6);
    print_prefix("flight controller", 6);
}
