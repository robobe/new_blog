#include <gtest/gtest.h>

// function for which we write unit tests
int add(int num1, int num2) {
    return num1 + num2;
}

// to test addition of positive numbers
TEST(SumTest, ForPositiveNumbers) {
    EXPECT_EQ(35, add(23, 12));
}

// to test addition of negative numbers
TEST(SumTest, ForNegativeNumbers) {
    EXPECT_EQ(-1, add(-1, 0));
}