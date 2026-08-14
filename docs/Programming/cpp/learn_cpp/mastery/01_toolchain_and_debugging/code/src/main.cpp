#include "greeter/greeter.hpp"

#include <iostream>
#include <string_view>

int main(int argc, char* argv[])
{
    const std::string_view name = argc > 1 ? argv[1] : "world";
    std::cout << greeter::make_greeting(name) << '\n';
    return 0;
}
