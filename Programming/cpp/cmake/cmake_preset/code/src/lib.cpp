#include "demo/lib.hpp"

namespace demo {

int add(int a, int b) {
    return a + b;
}

std::string greet(const std::string &name) {
    if (name.empty()) return "Hello, World!";
    return "Hello, " + name + "!";
}

} // namespace demo
