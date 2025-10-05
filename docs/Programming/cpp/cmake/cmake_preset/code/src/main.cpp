#include <iostream>
#include "demo/lib.hpp"

int main(int argc, char** argv) {
    std::string name = (argc > 1) ? argv[1] : "";
    std::cout << demo::greet(name) << "\n";
    std::cout << "2 + 3 = " << demo::add(2, 3) << "\n";
    return 0;
}
