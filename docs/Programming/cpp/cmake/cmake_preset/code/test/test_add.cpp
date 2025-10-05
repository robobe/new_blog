#include <gtest/gtest.h>
#include "demo/lib.hpp"

TEST(AddTests, BasicPositive) {
    EXPECT_EQ(demo::add(2,3), 5);
}

TEST(AddTests, NegativeAndPositive) {
    EXPECT_EQ(demo::add(-1,1), 0);
}

TEST(GreetTests, EmptyName) {
    EXPECT_EQ(demo::greet("") , "Hello, World!");
}

TEST(GreetTests, NonEmptyName) {
    EXPECT_EQ(demo::greet("Alice"), "Hello, Alice!");
}

// main provided by GTest::gtest_main
