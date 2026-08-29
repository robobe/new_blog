#include <iostream>

constexpr int square(int value)
{
    return value * value;
}

constexpr int absolute(int value)
{
    if (value < 0)
        return -value;

    return value;
}

static_assert(square(5) == 25);
static_assert(absolute(-7) == 7);

int main()
{
    constexpr int fixed = square(6);
    const int runtime_input = 4;

    std::cout << "Compile-time square: " << fixed << '\n';
    std::cout << "Runtime square: " << square(runtime_input) << '\n';
}
