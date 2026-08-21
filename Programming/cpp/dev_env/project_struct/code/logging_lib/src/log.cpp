#include "logging/log.hpp"

#include <iostream>

namespace logging {

void log(const std::string_view message)
{
    std::cout << "[log] " << message << '\n';
}

}
