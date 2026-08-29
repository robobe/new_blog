#include <iostream>

constexpr int square(int value)
{
    return value * value;
}

static_assert(square(5) == 25);

int main()
{
    // Exercise: add is_even() and clamp_percentage(), then check them with
    // static_assert before making a runtime call here.
    std::cout << "Starter result: " << square(5) << '\n';
}
