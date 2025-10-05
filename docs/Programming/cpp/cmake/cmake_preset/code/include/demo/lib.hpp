#pragma once
#include <string>

namespace demo {

// Adds two integers and returns the sum.
int add(int a, int b);

// Returns a greeting message ("Hello, <name>!"). If name empty => "Hello, World!".
std::string greet(const std::string &name);

} // namespace demo
