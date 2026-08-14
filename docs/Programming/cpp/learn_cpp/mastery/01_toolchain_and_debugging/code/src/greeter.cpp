#include "greeter/greeter.hpp"

namespace greeter {

std::string make_greeting(const std::string_view name)
{
    return "Hello, " + std::string{name} + "!";
}

} // namespace greeter
