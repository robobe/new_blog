---
title: C++ auto type deduction
tags:
    - cpp
    - cpp11
    - type-deduction
---

# C++ `auto` type deduction

`auto` asks the compiler to deduce a type from an initializer. The variable still has one fixed compile-time type; C++ does not become dynamically typed.

## Brief example

```cpp
#include <iostream>
#include <string>

int main()
{
    auto count = 3;                 // int
    auto temperature = 21.5;        // double
    auto name = std::string{"imu"}; // std::string

    std::cout << name << ": " << count << " samples at "
              << temperature << " C\n";
}
```

Every declaration needs an initializer so the compiler has a type to deduce:

```cpp
auto count = 3; // Valid: int
auto missing;   // Error: no initializer
```

## Why use `auto`?

Use `auto` when the initializer already makes the type clear or when spelling the type adds noise:

```cpp
const auto iterator = readings.find("temperature");
const auto result = calculate_position();
```

An explicit type is often clearer when the type communicates an important unit, range, or conversion:

```cpp
std::uint32_t timeout_ms = 1000;
double distance_m = sensor.read_distance();
```

`auto` is a readability tool, not a rule that every type must be hidden.

## Value, reference, and `const` deduction

Plain `auto` creates a new value. It drops top-level `const` and does not keep a reference from the initializer.

```cpp
const int original = 5;
auto copy = original; // int
copy = 9;             // Does not change original
```

Add qualifiers according to the behavior you need:

| Declaration | Meaning |
| --- | --- |
| `auto value = expression;` | Make a copy or move a new value. |
| `auto& value = expression;` | Create a modifiable reference. |
| `const auto& value = expression;` | Read without copying; can bind to a temporary. |
| `auto&& value = expression;` | Preserve the expression's value category during deduction. |

```cpp
std::string label{"camera"};

auto copy = label;
auto& reference = label;
const auto& read_only = label;

reference = "lidar"; // Changes label
copy = "gps";        // Does not change label
```

!!! tip "Start with intent"
    Use `auto` for an independent value, `auto&` to modify the original, and `const auto&` to read a potentially expensive value without copying it.

## Loops

`const auto&` is a useful default for reading container elements:

```cpp
for (const auto& reading : readings)
    std::cout << reading << '\n';
```

Use `auto&` when the loop must modify each element:

```cpp
for (auto& reading : readings)
    reading *= 2.0;
```

Use plain `auto` only when you intentionally want a copy of every element.

## Function return types

Since C++14, a function can deduce its return type from its `return` statements:

```cpp
auto square(int value)
{
    return value * value;
}
```

All return paths must deduce the same type. The function definition also normally needs to be visible before code calls it because the compiler must see the body to know the return type.

Use an explicit return type when it documents the interface better or when implicit conversions between return expressions are intended.

## Generic lambdas

C++14 also allows `auto` in lambda parameters:

```cpp
const auto add = [](const auto& left, const auto& right) {
    return left + right;
};

const auto integer_sum = add(2, 3);
const auto text = add(std::string{"front"}, std::string{" camera"});
```

The compiler creates a suitable call operator for each compatible argument-type combination.

## `auto` and structured bindings

Structured bindings use `auto` to deduce the decomposed element types:

```cpp
const std::pair<int, double> reading{4, 22.5};
const auto [sensor_id, temperature] = reading;
```

Use `auto&` or `const auto&` when the bindings should refer to the original object. See [C++ structured bindings](structure_binding.md) for the complete lesson.

## Braced initialization

These similar-looking declarations deduce different types:

```cpp
auto first{1};    // int
auto second = {1}; // std::initializer_list<int>
```

Mixed element types cannot produce one initializer-list type:

```cpp
auto values = {1, 2.5}; // Error: int and double do not match
```

Prefer direct initialization such as `auto value = Type{...}` when the intended type should be obvious.

## `decltype(auto)`

`decltype(auto)` follows `decltype` rules and can preserve references that plain `auto` would discard:

```cpp
decltype(auto) first(std::vector<int>& values)
{
    return (values.front()); // Returns int& because the expression is parenthesized
}
```

This is useful in forwarding code, but it is easy to return a dangling reference. Prefer an explicit return type unless preserving the exact expression type is necessary.

## Common mistakes

- Declaring `auto` without an initializer.
- Assuming plain `auto` keeps a reference or top-level `const`.
- Copying large elements in a loop when `const auto&` was intended.
- Hiding a meaningful conversion or unit behind `auto`.
- Expecting one `auto` variable to change type after declaration.
- Accidentally deducing `std::initializer_list` with `auto value = {...}`.

## Quiz

### 1. Does `copy` modify `original`?

```cpp
const int original = 4;
auto copy = original;
copy = 9;
std::cout << original;
```

<form class="auto-quiz" data-answer="4" data-explanation="Plain auto creates an int copy and drops top-level const. Assigning to copy does not change original.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="aq1" value="4"> <code>4</code></label>
    <label><input type="radio" name="aq1" value="9"> <code>9</code></label>
    <label><input type="radio" name="aq1" value="error"> A compiler error</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="auto-quiz-result" aria-live="polite"></p>
</form>

### 2. Which declaration reads a large element without copying it?

<form class="auto-quiz" data-answer="const-ref" data-explanation="const auto&amp; refers to the existing element without copying it and prevents modification through that reference.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="aq2" value="value"> <code>auto value = element;</code></label>
    <label><input type="radio" name="aq2" value="const-ref"> <code>const auto&amp; value = element;</code></label>
    <label><input type="radio" name="aq2" value="pointer"> <code>auto* value = element;</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="auto-quiz-result" aria-live="polite"></p>
</form>

### 3. What type does `values` have?

```cpp
auto values = {1, 2, 3};
```

<form class="auto-quiz" data-answer="initializer-list" data-explanation="Copy-list initialization with auto deduces std::initializer_list&lt;int&gt; because all elements have type int.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="aq3" value="array"> <code>int[3]</code></label>
    <label><input type="radio" name="aq3" value="vector"> <code>std::vector&lt;int&gt;</code></label>
    <label><input type="radio" name="aq3" value="initializer-list"> <code>std::initializer_list&lt;int&gt;</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="auto-quiz-result" aria-live="polite"></p>
</form>

<style>
.auto-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.auto-quiz-result { padding: .6rem; }
.auto-quiz-result:empty { display: none; }
.auto-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.auto-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.auto-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.auto-quiz-result');

    if (!selected) {
      result.className = 'auto-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `auto-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Further reading

- [cppreference: placeholder type specifiers](https://en.cppreference.com/w/cpp/language/auto){:target="_blank"}

<!-- post-content-skill: 1.0.0 -->
