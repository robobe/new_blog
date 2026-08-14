#include <iostream>

#define APPLICATION_NAME "toolchain-demo"

int main()
{
    std::cout << "Application: " << APPLICATION_NAME << '\n';

#ifdef NDEBUG
    std::cout << "Build mode: Release\n";
#else
    std::cout << "Build mode: Debug\n";
#endif
}
