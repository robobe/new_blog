---
title: C++ std::span
tags:
    - cpp
    - cpp20
    - span
    - containers
---

# C++ `std::span`

`std::span<T>` is a small, non-owning view over a contiguous sequence of `T` objects. It carries both a pointer and an element count, making it a clearer function parameter than separate pointer-and-size arguments.

## Simple example

```cpp
#include <array>
#include <iostream>
#include <span>

void double_values(std::span<int> values)
{
    for (int& value : values)
        value *= 2;
}

int main()
{
    std::array<int, 4> values{1, 2, 3, 4};
    double_values(values);

    for (const int value : values)
        std::cout << value << ' ';
}
```

The span refers to the elements owned by `values`, so the function modifies the original array.

Compile with C++20 or newer:

```bash
g++ -std=c++20 -Wall -Wextra -pedantic example.cpp -o example
./example
```

```text
2 4 6 8
```

## What can create a span?

A span can view contiguous storage such as a built-in array, `std::array`, or `std::vector`:

```cpp
int raw[]{1, 2, 3};
std::array<int, 3> fixed{4, 5, 6};
std::vector<int> dynamic{7, 8, 9};

std::span<int> raw_view{raw};
std::span<int> fixed_view{fixed};
std::span<int> dynamic_view{dynamic};
```

Containers such as `std::list` are not supported because their elements are not stored contiguously.

## Read-only and writable spans

Use `std::span<const T>` when a function only reads elements:

```cpp
int sum(std::span<const int> values)
{
    int total{};

    for (const int value : values)
        total += value;

    return total;
}
```

A writable container can be passed to either `std::span<T>` or `std::span<const T>`. A const container can only be viewed through `std::span<const T>`.

## Dynamic and fixed extent

The default span has a size known at runtime:

```cpp
std::span<int> values;
```

A fixed-extent span includes the required element count in its type:

```cpp
void update_rgb(std::span<int, 3> channels);
```

Fixed extent is useful when an operation always requires an exact number of elements. Prefer dynamic extent for general-purpose sequence functions.

## Access and subviews

A span supports iteration, indexing, and lightweight subviews:

```cpp
std::span<int> values{buffer};

const auto first_three = values.first(3);
const auto last_two = values.last(2);
const auto middle = values.subspan(1, 2);
```

Useful operations include:

| Operation | Purpose |
| --- | --- |
| `size()` | Return the number of elements. |
| `size_bytes()` | Return the viewed size in bytes. |
| `empty()` | Check whether the span has no elements. |
| `data()` | Return a pointer to the first element. |
| `front()` / `back()` | Access the first or last element. |
| `first()`, `last()`, `subspan()` | Create a smaller view without copying elements. |

!!! warning "Indexing is not bounds checked"
    Check sizes before using `operator[]`, `front()`, `back()`, or a runtime subview count. A span knows its size, but these operations do not automatically make invalid access safe.

## Possible uses

- Accept arrays, `std::array`, and `std::vector` through one function interface.
- Replace a pointer-and-count parameter such as `process(data, size)`.
- Pass a read-only sequence with `std::span<const T>`.
- Modify a caller-owned buffer without copying it.
- Work on a window of a larger buffer using `subspan()`.
- Pass contiguous data to a C API through `data()` and `size()`.
- Inspect an object's byte representation with `std::as_bytes()`.

```cpp
void send(std::span<const std::byte> bytes);

std::array<int, 4> values{1, 2, 3, 4};
send(std::as_bytes(std::span{values}));
```

`std::as_bytes()` exposes object representation; it does not define a portable serialization format.

## Lifetime rules

A span does not own or extend the lifetime of its elements. The viewed storage must remain alive and must not move while the span is used.

```cpp
std::span<int> invalid_view()
{
    int local[]{1, 2, 3};
    return local; // Wrong: local is destroyed on return
}
```

Operations that reallocate a vector also invalidate spans into that vector:

```cpp
std::vector<int> values{1, 2, 3};
std::span<int> view{values};

values.push_back(4); // May reallocate; view may now dangle
```

Recreate the span after an operation that may replace or reallocate its underlying storage.

## When not to use a span

- Use a container when the function must own or resize the sequence.
- Use `std::string_view` for read-only character strings.
- Use an iterator pair or a range when the data is not contiguous.
- Use a smart pointer when ownership must be shared or transferred.

## Quiz

### 1. What does the function modify?

```cpp
void clear(std::span<int> values)
{
    for (int& value : values)
        value = 0;
}
```

<form class="span-quiz" data-answer="original" data-explanation="A span is a non-owning view. Its elements refer to the caller's original contiguous storage.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="spq1" value="copy"> A private copy owned by the span</label>
    <label><input type="radio" name="spq1" value="original"> The caller's original elements</label>
    <label><input type="radio" name="spq1" value="nothing"> Nothing, because spans are always const</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="span-quiz-result" aria-live="polite"></p>
</form>

### 2. Which parameter accepts readable data from both const and non-const contiguous containers?

<form class="span-quiz" data-answer="const-span" data-explanation="std::span&lt;const int&gt; provides read-only access and can view either const or writable integer storage.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="spq2" value="const-span"> <code>std::span&lt;const int&gt;</code></label>
    <label><input type="radio" name="spq2" value="span"> <code>std::span&lt;int&gt;</code></label>
    <label><input type="radio" name="spq2" value="vector"> <code>std::vector&lt;int&gt;</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="span-quiz-result" aria-live="polite"></p>
</form>

### 3. Why can `view` become invalid?

```cpp
std::vector<int> values{1, 2, 3};
std::span<int> view{values};
values.push_back(4);
```

<form class="span-quiz" data-answer="reallocation" data-explanation="push_back may reallocate the vector's storage. A span does not follow moved storage, so it must be recreated after reallocation.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="spq3" value="size"> A span can only contain three elements.</label>
    <label><input type="radio" name="spq3" value="reallocation"> The vector may move its storage during reallocation.</label>
    <label><input type="radio" name="spq3" value="const"> <code>push_back()</code> makes the vector const.</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="span-quiz-result" aria-live="polite"></p>
</form>

<style>
.span-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.span-quiz-result { padding: .6rem; }
.span-quiz-result:empty { display: none; }
.span-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.span-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.span-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.span-quiz-result');

    if (!selected) {
      result.className = 'span-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `span-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Further reading

- [cppreference: `std::span`](https://en.cppreference.com/w/cpp/container/span){:target="_blank"}

<!-- post-content-skill: 1.0.0 -->
