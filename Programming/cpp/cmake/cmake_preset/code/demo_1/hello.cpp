// hello.cpp
#include <iostream>

int main()
{
#ifdef USE_FAST_MODE
    std::cout << "Fast mode build\n";
#else
    std::cout << "Normal build\n";
#endif
}