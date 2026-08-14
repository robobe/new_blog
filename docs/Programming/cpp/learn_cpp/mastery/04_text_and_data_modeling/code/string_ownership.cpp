#include <iostream>
#include <string>

int main()
{
    std::string original{"rover"};
    std::string copy{original};
    copy += "-2";

    std::cout << "Original: " << original << '\n';
    std::cout << "Copy: " << copy << '\n';
}
