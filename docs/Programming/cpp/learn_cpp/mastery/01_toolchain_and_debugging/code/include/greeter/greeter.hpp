#pragma once

#include <string>
#include <string_view>

namespace greeter {

[[nodiscard]] std::string make_greeting(std::string_view name);

} // namespace greeter
