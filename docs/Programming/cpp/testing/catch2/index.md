---
title: Catch2
tags:
    - catch2
    - cpp
    - testing
---

Catch2 is a C++ unit-testing framework with CTest and CMake integration.


## Install

```bash
sudo apt install catch2
```

---

## Demo

```bash
calculator/
├── CMakeLists.txt
├── include/
│   └── calculator.hpp
├── src/
│   └── calculator.cpp
└── tests/
    └── test_calculator.cpp
```

```cpp title="include/calculator.hpp"
--8<-- "docs/Programming/cpp/testing/catch2/code/include/calculator.hpp"
```

```cpp title="src/calculator.cpp"
--8<-- "docs/Programming/cpp/testing/catch2/code/src/calculator.cpp"
```

```cpp title="tests/test_calculator.cpp"
--8<-- "docs/Programming/cpp/testing/catch2/code/tests/test_calculator.cpp"
```

```cmake title="CMakeLists.txt"
--8<-- "docs/Programming/cpp/testing/catch2/code/CMakeLists.txt"
```

---

### Build and usage

```bash
cmake -S code -B code/build
cmake --build code/build
```

```bash title="run tests"
./code/build/calculator_tests
```

```bash title="run test using ctest"
ctest --test-dir code/build --output-on-failure
```

Expected result:

```text
100% tests passed, 0 tests failed out of 3
```

Running `calculator_tests` directly should finish with this summary:

```text
All tests passed (4 assertions in 3 test cases)
```

---

## Basic Catch2 API

### TEST_CASE

`TEST_CASE` defines an independently runnable test. Give it a description that
states the behavior being verified. An optional tag, such as `[calculator]`,
can be used to select related tests from the command line.

```cpp
TEST_CASE("addition returns the sum", "[calculator]")
{
    REQUIRE(add(2, 3) == 5);
}
```

### CHECK and REQUIRE

Both macros verify a condition, but they handle failure differently:

| Macro | Failure behavior |
|---|---|
| `CHECK(expression)` | Records the failure and continues the current test case. |
| `REQUIRE(expression)` | Records the failure and stops the current test case immediately. |

### CHECK

```cpp
TEST_CASE("example")
{
    CHECK(1 == 2);     // fails, but continues
    CHECK(2 == 3);     // this still runs
}
```

The example intentionally fails to demonstrate that the second `CHECK` still
runs. It is not included in the buildable companion project.

### REQUIRE

```cpp
TEST_CASE("example")
{
    REQUIRE(1 == 2);   // fails and stops this test case
    REQUIRE(2 == 3);   // never reached
}
```

This example also intentionally fails. The second assertion is not evaluated,
so it is kept out of the passing companion test suite.

### Exception assertions

| Macro | Verifies |
|---|---|
| `REQUIRE_THROWS(expression)` | The expression throws any exception. |
| `REQUIRE_THROWS_AS(expression, type)` | The expression throws the expected exception type. |
| `REQUIRE_NOTHROW(expression)` | The expression completes without throwing. |

Use `REQUIRE_THROWS_AS` when the exception type is part of the behavior being
tested:

```cpp
TEST_CASE("division by zero throws")
{
    REQUIRE_THROWS_AS(
        divide(10, 0),
        std::invalid_argument
    );
}
```

This assertion fails if `divide(10, 0)` does not throw or if the thrown
exception does not match `std::invalid_argument`. The same passing test is
included in `code/tests/test_calculator.cpp`.


---

## More Advance API

