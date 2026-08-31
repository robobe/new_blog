---
title: C++ structured bindings
tags:
    - cpp
    - cpp17
    - structured-bindings
---

# C++ structured bindings

Structured bindings give names to the individual parts of an array, tuple-like object, or simple structure. They were introduced in C++17.

## Simple example

```cpp
#include <iostream>
#include <utility>

int main()
{
    const std::pair<int, int> position{4, 7};
    const auto [x, y] = position;

    std::cout << "x=" << x << ", y=" << y << '\n';
}
```

`auto [x, y]` creates two names from the two elements of `position`. Compile it as C++17 or newer:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic example.cpp -o example
./example
```

```text
x=4, y=7
```

## Supported types

### Structures

```cpp
struct SensorReading
{
    int id;
    double temperature;
};

const SensorReading reading{3, 24.5};
const auto [id, temperature] = reading;
```

The names follow member declaration order, not their names. The number of bindings must match the number of decomposed members.

### Arrays

```cpp
int coordinates[]{10, 20, 30};
auto [x, y, z] = coordinates;
```

### Pairs and tuples

```cpp
#include <tuple>

const auto record = std::make_tuple(7, "camera", true);
const auto [id, name, enabled] = record;
```

`std::pair`, `std::tuple`, and other tuple-like types can be decomposed when their element count and access operations are defined.

## Copies, references, and `const`

Choose the qualifier according to whether you need a copy or access to the original object:

| Binding | Behavior |
| --- | --- |
| `auto [x, y] = value;` | Work with a copy. |
| `auto& [x, y] = value;` | Modify the original elements. |
| `const auto& [x, y] = value;` | Read the original elements without copying. |
| `auto&& [x, y] = expression;` | Preserve whether the expression is an lvalue or rvalue. |

```cpp
std::pair<int, int> position{4, 7};

auto& [x, y] = position;
x = 10;

std::cout << position.first; // 10
```

!!! tip "Use `const auto&` in read-only loops"
    It avoids copying each element while preventing accidental modification.

## Structured bindings in loops

They make key-value iteration easier to read:

```cpp
#include <iostream>
#include <map>
#include <string>

const std::map<std::string, int> scores{
    {"Ada", 10},
    {"Bjarne", 12}
};

for (const auto& [name, score] : scores)
    std::cout << name << ": " << score << '\n';
```

For a map element, the first binding is the key and the second is the mapped value.

## Returning multiple values

A function can return a small structure or tuple and let the caller name each result:

```cpp
struct DivisionResult
{
    int quotient;
    int remainder;
};

DivisionResult divide(int value, int divisor)
{
    return {value / divisor, value % divisor};
}

const auto [quotient, remainder] = divide(17, 5);
```

Prefer a named structure when the fields have domain meaning. A pair or tuple is suitable for a small, obvious local result.

## Use in an `if` statement

A structured binding can appear in an `if` initializer. This is common with container insertion:

```cpp
std::map<std::string, int> scores;

if (const auto [iterator, inserted] = scores.insert({"Ada", 10}); inserted)
    std::cout << "Added " << iterator->first << '\n';
else
    std::cout << "The key already exists\n";
```

Both names exist only inside the `if` and `else` statements.

## Common mistakes

- Using the wrong number of names causes a compile error.
- Omitting `&` creates a copy, so changes do not affect the original object.
- Binding names are chosen by position; renaming structure members does not rename the bindings.
- Large values can be expensive to copy. Prefer `const auto&` when only reading them.
- `_` is an ordinary variable name in C++; it does not discard an unwanted element.

## Quiz

### 1. Does this modify the pair?

```cpp
std::pair<int, int> point{2, 3};
auto [x, y] = point;
x = 9;
std::cout << point.first;
```

<form class="structured-quiz" data-answer="2" data-explanation="auto [x, y] decomposes a copy of point. Use auto&amp; [x, y] to modify the original pair.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="sq1" value="2"> <code>2</code></label>
    <label><input type="radio" name="sq1" value="9"> <code>9</code></label>
    <label><input type="radio" name="sq1" value="error"> A compiler error</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="structured-quiz-result" aria-live="polite"></p>
</form>

### 2. Which binding is best for a read-only map loop?

<form class="structured-quiz" data-answer="const-ref" data-explanation="const auto&amp; avoids copying each map element and prevents the loop from modifying it.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="sq2" value="copy"> <code>auto [key, value]</code></label>
    <label><input type="radio" name="sq2" value="const-ref"> <code>const auto&amp; [key, value]</code></label>
    <label><input type="radio" name="sq2" value="pointer"> <code>auto* [key, value]</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="structured-quiz-result" aria-live="polite"></p>
</form>

### 3. What happens when the binding count is wrong?

```cpp
std::tuple<int, int, int> values{1, 2, 3};
auto [first, second] = values;
```

<form class="structured-quiz" data-answer="compile-error" data-explanation="A structured binding must provide exactly one name for each decomposed element, so this code does not compile.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="sq3" value="ignore"> The third value is ignored.</label>
    <label><input type="radio" name="sq3" value="zero"> <code>second</code> becomes zero.</label>
    <label><input type="radio" name="sq3" value="compile-error"> The program fails to compile.</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="structured-quiz-result" aria-live="polite"></p>
</form>

<style>
.structured-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.structured-quiz-result { padding: .6rem; }
.structured-quiz-result:empty { display: none; }
.structured-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.structured-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.structured-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.structured-quiz-result');

    if (!selected) {
      result.className = 'structured-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `structured-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Further reading

- [cppreference: structured binding declaration](https://en.cppreference.com/w/cpp/language/structured_binding){:target="_blank"}

<!-- post-content-skill: 1.0.0 -->
