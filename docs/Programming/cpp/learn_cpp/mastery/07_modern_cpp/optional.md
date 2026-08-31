---
title: C++ std::optional
tags:
    - cpp
    - cpp17
    - optional
---

# C++ `std::optional`

`std::optional<T>` represents either a value of type `T` or no value. It is useful when absence is a normal result, not an error.

## Basic example

```cpp
#include <iostream>
#include <optional>

std::optional<int> find_answer(bool available)
{
    if (available)
        return 42;

    return std::nullopt;
}

int main()
{
    const auto answer = find_answer(true);

    if (answer)
        std::cout << "Answer: " << *answer << '\n';
}
```

The function returns `42` when a value is available and `std::nullopt` when it is not. The `if` checks whether the optional contains a value; `*answer` reads that value.

Include `<optional>` and compile as C++17 or newer:

```bash
g++ -std=c++17 optional_demo.cpp -o optional_demo
```

## How to use it

Create an optional with a value or without one:

```cpp
std::optional<int> retries{3};
std::optional<int> missing{std::nullopt};
```

Use `if`, `has_value()`, or `value_or()` depending on what the program needs:

```cpp
if (retries)
    std::cout << *retries << '\n';

if (retries.has_value())
    std::cout << retries.value() << '\n';

std::cout << missing.value_or(5) << '\n';
```

| Operation | Meaning |
| --- | --- |
| `if (value)` | Check whether a value is present. |
| `*value` | Read the contained value after checking. |
| `value.value()` | Read the value, or throw `std::bad_optional_access` if empty. |
| `value.value_or(default_value)` | Read the value, or use a fallback. |
| `value.reset()` | Remove the contained value. |

!!! warning "Check before dereferencing"
    Dereferencing an empty optional with `*value` is undefined behavior. Check it first, or use `value_or()` when a default is appropriate.

## Why not use a sentinel value?

A function returning `-1` for “missing” cannot also treat `-1` as valid data. An optional stores presence separately, so every `int`, including `0` and negative values, remains available to the program.

```cpp
std::optional<int> configured_limit{0};

if (configured_limit)
    std::cout << "Configured limit: " << *configured_limit << '\n';
```

The condition is true because the optional contains a value. It does not test whether that value is nonzero.

## Hands-on: optional retry count

The complete [optional_demo.cpp](code/optional_demo.cpp) reads an optional retry count from the command line. With no argument, it uses a default of `3`; with an argument, it parses and uses that value.

```bash
cd docs/Programming/cpp/learn_cpp/mastery/07_modern_cpp
g++ -std=c++17 -Wall -Wextra -pedantic code/optional_demo.cpp -o optional_demo
./optional_demo
./optional_demo 0
./optional_demo 5
```

Expected output:

```text
Retry count: 3 (default)
Retry count: 0 (configured)
Retry count: 5 (configured)
```

Now change the program:

1. Change the default retry count from `3` to `2`.
2. Reject configured values greater than `10`.
3. Print a clear error and return a nonzero exit status for rejected input.
4. Confirm that `0`, `2`, and no argument still work.

## Quiz

### 1. Is zero present?

What does this print?

```cpp
std::optional<int> count{0};
std::cout << (count ? "present" : "missing");
```

<form class="optional-quiz" data-answer="present" data-explanation="The condition checks whether the optional contains a value, not whether the contained integer is nonzero.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="q1" value="zero"> <code>0</code></label>
    <label><input type="radio" name="q1" value="missing"> <code>missing</code></label>
    <label><input type="radio" name="q1" value="present"> <code>present</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="optional-quiz-result" aria-live="polite"></p>
</form>

### 2. What does `value_or()` return?

```cpp
std::optional<int> count;
std::cout << count.value_or(7);
```

<form class="optional-quiz" data-answer="7" data-explanation="count is empty, so value_or(7) returns the fallback. If count held a value, it would return that value instead.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="q2" value="0"> <code>0</code></label>
    <label><input type="radio" name="q2" value="7"> <code>7</code></label>
    <label><input type="radio" name="q2" value="error"> A compiler error</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="optional-quiz-result" aria-live="polite"></p>
</form>

### 3. Why is `std::optional<int>` better than returning `-1` for a missing integer?

<form class="optional-quiz" data-answer="separate" data-explanation="std::optional represents absence separately from the integer. It keeps -1 available as valid data and makes the missing case explicit.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="q3" value="faster"> It always runs faster.</label>
    <label><input type="radio" name="q3" value="separate"> It represents absence separately from every valid integer.</label>
    <label><input type="radio" name="q3" value="positive"> It only permits positive integers.</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="optional-quiz-result" aria-live="polite"></p>
</form>

<style>
.optional-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.optional-quiz-result { padding: .6rem; }
.optional-quiz-result:empty { display: none; }
.optional-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.optional-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.optional-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.optional-quiz-result');

    if (!selected) {
      result.className = 'optional-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `optional-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

---

## Advanced topics

### Must know: lifetime, `emplace()`, and `reset()`

An optional contains its value directly. `emplace()` constructs a value inside it, replacing any current value, while `reset()` destroys the value and makes the optional empty.

```cpp
std::optional<std::string> label;
label.emplace(3, 'x'); // Contains "xxx"
label.reset();         // Empty again
```

Use `emplace()` when constructing the value in place is clearer or avoids a temporary. Normal assignment is usually simpler for basic values.

### Must know: copy and move behavior

Copying an optional copies its contained value. Moving it moves the contained value, but does not guarantee that the source optional becomes empty.

```cpp
std::optional<std::string> source{"camera"};
auto destination = std::move(source);
```

After the move, `destination` contains `"camera"`. `source` may still report that it has a value, but that string is in a valid, unspecified moved-from state. Do not use `has_value()` to test whether an object was moved from.

An optional can also contain a move-only type. Use that only when “no object” and the contained type's own empty state have different meanings.

### Must know: C++23 optional pipelines

C++23 adds three operations for processing a value without repeatedly writing `if` checks:

| Operation | Runs when | Callable returns |
| --- | --- | --- |
| `transform()` | A value is present | A plain value, wrapped in an optional automatically |
| `and_then()` | A value is present | Another `std::optional` |
| `or_else()` | The optional is empty | A replacement `std::optional` |

```cpp
const auto result = parse_number(text)
    .transform([](int value) { return value * 2; })
    .and_then(validate_limit)
    .or_else([] { return std::optional<int>{0}; });
```

Compile this style with C++23:

```bash
g++ -std=c++23 example.cpp -o example
```

Keep a direct `if` when it is easier to read. Pipelines are most useful when several optional-producing steps must be chained.

### Must know: choosing the right return type

| Situation | Prefer |
| --- | --- |
| A value may normally be absent | `std::optional<T>` |
| Failure needs an explanation | `std::expected<T, E>` in C++23 |
| Failure is exceptional and cannot be handled locally | An exception |
| Shared or transferred ownership is required | A smart pointer |
| A function parameter has a common default | A default argument or overload |

An optional parameter can be appropriate when the caller must explicitly distinguish “not supplied” from every possible value. Do not use it merely to avoid writing an overload.

### Common traps

- `std::optional<bool>` has three states: empty, `false`, and `true`. Test presence separately from the contained Boolean.
- `std::optional<T&>` is not allowed. Use `std::reference_wrapper<T>` when a non-owning optional reference is genuinely needed.
- Nested optionals represent more than two states, but are usually harder to understand; use them only when each level has a distinct meaning.
- `value_or()` eagerly evaluates its fallback argument, even when the optional contains a value. Avoid putting expensive work directly in that argument.




---

## Further reading

- [cppreference: `std::optional`](https://en.cppreference.com/w/cpp/utility/optional){:target="_blank"}

<!-- post-content-skill: 1.0.0 -->
