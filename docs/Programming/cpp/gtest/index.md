---
title: GTest
tags:
    - gtest
    - cpp
---


{{ page_folder_links() }}


## Core Concepts

- **ASSERT_\* macros**: Fatal failures. If an ASSERT_ fails, current function test will abort immidiately.
- **EXPECT_\* macro**:  Non-fatal failures. If an EXPECT_ fails, current function test continues and allow to find further failures.
- **TEST() macro**:  defines a function contains assertions to verify a behavior.
    `TEST(TestSuiteName, TestName)`

- **TEST_F() macro**: Used to define a test fixture. When we need to perform multiple tests on same object
    `TEST_F(FixtureName, TestName)`

- **Test Suite**: Group of related test
   
---

## Simple demo

Minimal gtest and CMakeList, for simplicity add the function on test to the test file

```cpp title="test/test_demo.cpp"
--8<-- "docs/Programming/cpp/gtest/code/test/test_demo.cpp"
```

```cmake
--8<-- "docs/Programming/cpp/gtest/code/CMakeLists.txt"
```

### usage

```bash
# -S: source folder
# -B: build folder
cmake -S . -B build

# compile
cmake --build build

cd build
# Test
ctest
```

---

## Disabled test

Add `DISABLED` prefix to test suit name

```
// to test addition of negative numbers
TEST(DISABLED_SumTest, ForNegativeNumbers) {
    EXPECT_EQ(-1, add(-1, 0));
}
```

```
Test project /home/user/projects/cmake_presets/build
    Start 1: SumTest.ForPositiveNumbers
1/2 Test #1: SumTest.ForPositiveNumbers .......   Passed    0.00 sec
    Start 2: SumTest.ForNegativeNumbers
2/2 Test #2: SumTest.ForNegativeNumbers .......***Not Run (Disabled)   0.00 sec
```