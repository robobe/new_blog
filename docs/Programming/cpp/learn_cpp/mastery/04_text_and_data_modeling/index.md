---
title: Text and data modeling
tags:
    - cpp
    - cpp20
    - string
    - string-view
    - data-modeling
---

# Text and data modeling

Programs become easier to understand when their types describe the problem. This lesson uses owning strings, non-owning string views, scoped enumerations, and small data types to express intent directly.

!!! info "Lesson 04"
    This is Phase 1, Subject 4 of the [C++ mastery plan](../plan.md). Like [Lesson 01](../01_toolchain_and_debugging/index.md), it contains runnable examples, an exercise, tests, a quiz, and a phase-mission contribution.

## Prerequisites

You should be able to build a CMake target, use functions and basic control flow, and read compiler diagnostics.

## Learning goals

By the end of this lesson, you should be able to:

- choose between `std::string` and `std::string_view`;
- recognize and avoid a dangling `std::string_view`;
- use `enum class` for a closed set of states;
- group related values into a type;
- identify and protect a simple invariant.

---

## `std::string` owns its text

A `std::string` owns a resizable sequence of characters. Use it when an object must store or modify text, or return text with a lifetime independent of the input.

```cpp
std::string original{"rover"};
std::string copy{original};
copy += "-2";
```

The copy owns separate storage, so changing it does not change `original`:

```text
Original: rover
Copy: rover-2
```

Common operations are `empty()`, `size()`, `+=`, comparison, and `substr()`. See the complete [string ownership example](code/string_ownership.cpp).

---

## `std::string_view` observes existing text

A `std::string_view` is a small, non-owning view of characters. Conceptually, it holds a pointer and a length; copying it does not copy the characters.

```cpp
void print_prefix(std::string_view text, std::size_t length);

std::string topic{"camera calibration"};
print_prefix(topic, 6);
print_prefix("flight controller", 6);
```

One read-only parameter now accepts a `std::string`, a string literal, or another view without making an owning text copy. See [string_view_demo.cpp](code/string_view_demo.cpp).

!!! warning "A view does not own the characters"
    The viewed text must outlive the view. Never return a view into a local `std::string`, and do not retain a view after its owner is destroyed or modified in a way that reallocates its storage.

This view dangles at the end of the statement:

```cpp
std::string_view label = std::string{"temporary"}; // Wrong
```

!!! tip "A view is not necessarily null-terminated"
    A view knows its length, but the next character is not guaranteed to be `\0`. If an API requires a C string, pass an appropriate owning `std::string` using `c_str()` rather than blindly passing `view.data()`.

### Which text type should I use?

| Need | Prefer | Reason |
| --- | --- | --- |
| Store or modify text | `std::string` | It owns mutable storage. |
| Return newly created text | `std::string` | The caller safely receives ownership. |
| Read text only during a call | `std::string_view` | It avoids copying the characters. |
| Store text for later | Usually `std::string` | A view's owner may disappear. |
| Call an API requiring a C string | `std::string::c_str()` | A general view may not end with `\0`. |

---

## `enum class` models a closed set of choices

Use `enum class` when a value must be exactly one of a small, named set:

```cpp
enum class FlightMode
{
    manual,
    altitude_hold,
    position_hold
};
```

The names are scoped, so you write `FlightMode::manual`. Unlike an old unscoped `enum`, an `enum class` does not silently convert to an integer. Both properties prevent accidental mixing of unrelated values.

Convert an enumerator to display text explicitly:

```cpp
[[nodiscard]] constexpr std::string_view mode_name(FlightMode mode)
{
    switch (mode)
    {
        case FlightMode::manual: return "manual";
        case FlightMode::altitude_hold: return "altitude hold";
        case FlightMode::position_hold: return "position hold";
    }
    return "unknown";
}
```

A compiler warning can then reveal a missing case when you add another mode.

---

## Group related values into a type

A `struct` gives a meaningful name to values that belong together:

```cpp
struct Position
{
    double north_m{};
    double east_m{};
    double altitude_m{};
};

const Position home{0.0, 0.0, 12.5};
```

The member names carry meaning and units that three unrelated `double` variables cannot express. A `struct` can also contain functions and constructors; it is not limited to passive data.

As a useful convention, choose a `struct` for transparent data whose members should be accessed directly, and a `class` when hidden state and controlled operations matter. At language level, their main difference is default access: `struct` begins `public`, while `class` begins `private`.

---

## Protect simple invariants

An **invariant** is a condition that must remain true for every valid object. A vehicle configuration may require a non-empty name and a positive, finite altitude limit.

```cpp
VehicleConfig(std::string name, FlightMode mode, double altitude_limit_m)
    : name_{std::move(name)}, mode_{mode}, altitude_limit_m_{altitude_limit_m}
{
    if (name_.empty())
        throw std::invalid_argument{"vehicle name must not be empty"};

    if (!std::isfinite(altitude_limit_m_) || altitude_limit_m_ <= 0.0)
        throw std::invalid_argument{"altitude limit must be positive and finite"};
}
```

The constructor rejects bad input before a valid object exists. Private members prevent callers from bypassing the checks later. See the complete [domain model](code/domain_model.cpp).

!!! tip "Make invalid states difficult to represent"
    Use types for rules the compiler can enforce, such as `FlightMode` instead of an arbitrary integer. Use runtime validation for rules that depend on values, such as an altitude being positive.

---

## Build and run

The local [CMakeLists.txt](code/CMakeLists.txt) enables C++20, compiler warnings, and small output checks with CTest.

```bash
cd docs/Programming/cpp/learn_cpp/mastery/04_text_and_data_modeling/code
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

Run the combined example:

```bash
./build/domain_model
```

```text
Vehicle: scout
Mode: altitude hold
Limit: 120 m
Home altitude: 12.5 m
```

---

## Exercise: model a sensor configuration

Open [exercise_starter.cpp](code/exercise_starter.cpp) and complete its two `TODO` comments.

1. Map every `SensorState` to `offline`, `ready`, or `fault`.
2. Reject an empty name with `std::invalid_argument`.
3. Reject a sample rate that is zero or negative.
4. Keep the program warning-free.
5. Make it print `imu ready at 100 Hz`.

After it works, add a `PASS_REGULAR_EXPRESSION` property to `exercise_starter_runs` so CTest verifies the output.

!!! tip "Exercise the failure path"
    Temporarily construct `SensorConfig{"", 100.0}` and `SensorConfig{"imu", 0.0}`. Both should fail clearly. Remove those calls or turn them into deliberate tests afterward.

---

## Quiz

### 1. Predict the result

What does this print, and why?

```cpp
std::string text{"rover"};
std::string_view view{text};
text[0] = 'R';
std::cout << view << '\n';
```

??? success "Answer"
    It prints `Rover`. The view observes the characters owned by `text`; it does not contain a copy. Replacing one character does not reallocate the string, so the view remains valid and sees the change.

### 2. Explain the concept

Why is returning a `std::string_view` into a local `std::string` incorrect?

??? success "Answer"
    The local string is destroyed when the function returns. The view does not extend its lifetime, so it points to storage that no longer exists. Return an owning `std::string` instead.

### 3. Find the bug

```cpp
std::string_view component = std::string{"navigation"};
std::cout << component << '\n';
```

??? success "Answer"
    The temporary string is destroyed at the first semicolon. `component` is dangling before it is printed. Own the text with `std::string`, or view a string literal whose lifetime is long enough.

### 4. Write a small model

Define `enum class LogLevel` with `debug`, `info`, and `error`. Define a `LoggerConfig` that owns a component name, stores a level, and rejects an empty name.

??? success "One possible answer"
    ```cpp
    enum class LogLevel { debug, info, error };

    class LoggerConfig
    {
    public:
        LoggerConfig(std::string component, LogLevel level)
            : component_{std::move(component)}, level_{level}
        {
            if (component_.empty())
                throw std::invalid_argument{"component must not be empty"};
        }

    private:
        std::string component_;
        LogLevel level_;
    };
    ```

---

## Phase 1 mission contribution

Extend the Phase 1 text analyzer with an explicit model instead of passing loose values between functions:

- an `enum class AnalysisMode` with `characters`, `words`, and `lines`;
- an `AnalysisRequest` that owns its input text and stores the mode;
- an `AnalysisResult` struct with named count fields;
- functions accepting `std::string_view` when they only inspect text;
- constructor validation for rules required by your design.

Add tests for empty input, repeated whitespace, and a final line without a trailing newline. This contribution will later combine with the other Phase 1 subjects.

## Further reading

- [cppreference: `std::basic_string`](https://en.cppreference.com/w/cpp/string/basic_string){:target="_blank"}
- [cppreference: `std::basic_string_view`](https://en.cppreference.com/w/cpp/string/basic_string_view){:target="_blank"}
- [cppreference: enumeration declaration](https://en.cppreference.com/w/cpp/language/enum){:target="_blank"}
- [C++ Core Guidelines: class rules](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-class){:target="_blank"}

<!-- post-content-skill: 1.0.0 -->
