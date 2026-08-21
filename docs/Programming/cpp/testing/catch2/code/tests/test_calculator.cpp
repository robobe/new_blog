#include <catch2/catch_test_macros.hpp>

#include "calculator.hpp"

#include <stdexcept>

TEST_CASE("add two numbers")
{
    REQUIRE(add(2, 3) == 5);
    REQUIRE(add(-1, 1) == 0);
}

TEST_CASE("divide two numbers")
{
    REQUIRE(divide(10, 2) == 5);
}

TEST_CASE("division by zero throws")
{
    REQUIRE_THROWS_AS(
        divide(10, 0),
        std::invalid_argument
    );
}
