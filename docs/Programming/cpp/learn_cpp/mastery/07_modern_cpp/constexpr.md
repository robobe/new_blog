---
title: C++ constexpr
tags:
    - cpp
    - cpp11
    - cpp20
    - constexpr
    - compile-time
---

# C++ `constexpr`

`constexpr` means **can be evaluated at compile time when used with compile-time
inputs**. This lets the compiler calculate and validate values before the
program starts.

`constexpr` was introduced in C++11 and became more capable in later standards.
The examples in this lesson use C++20.

## Goal and prerequisites

After this lesson, you should be able to:

- create compile-time constants;
- write a function that works at compile time and runtime;
- verify a result with `static_assert`;
- choose between `const`, `constexpr`, and a normal function;
- recognize when `constexpr` does not guarantee compile-time evaluation.

You should already understand functions, return values, and `const` variables.

## Brief example

```cpp
constexpr int square(int value)
{
    return value * value;
}

constexpr int area = square(5);
static_assert(area == 25);
```

The compiler can evaluate `square(5)` while building the program. If the
assertion is false, compilation fails.

## Why use `constexpr`?

Before `constexpr`, programmers often used macros or repeated literal values:

```cpp
#define BUFFER_SIZE (64 * 4)
```

A typed compile-time constant is clearer and follows normal C++ rules:

```cpp
constexpr int samples_per_block = 64;
constexpr int channel_count = 4;
constexpr int buffer_size = samples_per_block * channel_count;
```

Unlike a macro, a `constexpr` variable has a real type, scope, and compiler
diagnostics.

Useful compile-time values include:

- fixed array sizes;
- protocol constants;
- unit conversions;
- small lookup tables;
- values checked by `static_assert`;
- calculations shared by compile-time and runtime code.

## `const` versus `constexpr`

Both prevent changing a variable after initialization, but they express
different guarantees:

| Declaration | Meaning |
| --- | --- |
| `const int value` | `value` cannot be changed through this name. |
| `constexpr int value` | `value` is a compile-time constant and is also const. |

```cpp
int read_sensor();

const int latest = read_sensor(); // Valid: value can be known at runtime.
constexpr int limit = 100;        // Must be known at compile time.
```

This does not compile because a runtime result cannot initialize a `constexpr`
variable:

```cpp
constexpr int latest = read_sensor(); // Error
```

Use `const` for a runtime value that must not change. Use `constexpr` when the
value must be available during compilation.

## A `constexpr` function can run at runtime

Declaring a function `constexpr` does not force every call to happen during
compilation:

```cpp
#include <iostream>

constexpr int double_value(int value)
{
    return value * 2;
}

int main()
{
    constexpr int fixed = double_value(4); // Compile-time evaluation

    int input{};
    std::cin >> input;
    const int result = double_value(input); // Runtime evaluation

    std::cout << fixed << ' ' << result << '\n';
}
```

The same function supports both calls. The arguments and the surrounding
context determine whether compile-time evaluation is required.

!!! tip "Require a compile-time result"
    Store the result in a `constexpr` variable, use it in `static_assert`, or
    use it in another context that requires a constant expression.

## Compile-time validation with `static_assert`

`static_assert` checks a Boolean expression while compiling:

```cpp
constexpr int kilobytes(int count)
{
    return count * 1024;
}

static_assert(kilobytes(2) == 2048);
static_assert(kilobytes(0) == 0);
```

It creates no runtime test code. Use it for rules that the compiler can prove.
Keep runtime tests for values that arrive from files, users, sensors, or the
network.

## More than one statement

In C++20, a `constexpr` function can contain normal control flow when every
operation used during compile-time evaluation is allowed in a constant
expression:

```cpp
constexpr int absolute(int value)
{
    if (value < 0)
        return -value;

    return value;
}

static_assert(absolute(-7) == 7);
static_assert(absolute(4) == 4);
```

Do not make a function complicated merely because modern `constexpr` permits
it. A small, pure calculation is easiest to understand and test.

## `constexpr` objects

A class can support compile-time construction and member functions:

```cpp
class Duration
{
public:
    constexpr explicit Duration(int seconds) : seconds_(seconds) {}

    [[nodiscard]] constexpr int seconds() const
    {
        return seconds_;
    }

private:
    int seconds_;
};

constexpr Duration timeout{30};
static_assert(timeout.seconds() == 30);
```

This is useful for small value types. It does not mean every class should be
rewritten for compile-time use.

## Related C++20 keywords

These keywords solve different problems:

| Keyword | Meaning |
| --- | --- |
| `constexpr` | A value or function can participate in compile-time evaluation. |
| `consteval` | Every call to the function must be evaluated at compile time. |
| `constinit` | A static or thread-local variable must be statically initialized. |

```cpp
consteval int protocol_version()
{
    return 3;
}

constexpr int version = protocol_version();
```

Use `consteval` only when a runtime call would be meaningless or invalid.
`constinit` does not make a variable immutable; it controls initialization.

## When not to use `constexpr`

Keep an ordinary function when:

- its inputs only exist at runtime;
- it performs input/output;
- compile-time use provides no practical value;
- adding compile-time constraints makes the code harder to understand.

`constexpr` is a guarantee and design signal, not a command to move as much work
as possible into compilation.

## Predict the behavior

Does this program compile, and what does it print?

```cpp
#include <iostream>

constexpr int increment(int value)
{
    return value + 1;
}

int main()
{
    constexpr int first = increment(4);
    int value = 9;
    const int second = increment(value);

    std::cout << first << ' ' << second << '\n';
}
```

<details>
<summary>Show the answer</summary>

It compiles and prints:

```text
5 10
```

`first` is calculated at compile time because it initializes a `constexpr`
variable. `second` is calculated at runtime because `value` is not a constant
expression.

</details>

## Compile the examples

Use the complete [`constexpr_demo.cpp`](code/constexpr/constexpr_demo.cpp) and
[`exercise_starter.cpp`](code/constexpr/exercise_starter.cpp) files.

```bash
cmake -S code/constexpr -B code/constexpr/build
cmake --build code/constexpr/build
ctest --test-dir code/constexpr/build --output-on-failure
```

## Small coding exercise

Open `code/constexpr/exercise_starter.cpp`. Add:

1. A `constexpr bool is_even(int value)` function.
2. A `constexpr int clamp_percentage(int value)` function that returns a value
   from 0 through 100.
3. At least two `static_assert` checks for each function.
4. One runtime call using a value entered by the user.

<details>
<summary>Show one possible solution</summary>

```cpp
constexpr bool is_even(int value)
{
    return value % 2 == 0;
}

constexpr int clamp_percentage(int value)
{
    if (value < 0)
        return 0;
    if (value > 100)
        return 100;
    return value;
}

static_assert(is_even(8));
static_assert(!is_even(7));
static_assert(clamp_percentage(-5) == 0);
static_assert(clamp_percentage(120) == 100);
```

</details>

## Quiz

### 1. What does `constexpr` mean for a function?

<form class="constexpr-quiz" data-answer="possible" data-explanation="A constexpr function can be evaluated at compile time when its arguments and context allow it, but it may also run at runtime.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="cxq1" value="always"> Every call must run at compile time</label>
    <label><input type="radio" name="cxq1" value="possible"> The function can run at compile time in a constant-expression context</label>
    <label><input type="radio" name="cxq1" value="inline"> The function is replaced by a macro</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="constexpr-quiz-result" aria-live="polite"></p>
</form>

### 2. Which declaration requires a compile-time value?

<form class="constexpr-quiz" data-answer="constexpr" data-explanation="A constexpr variable must be initialized with a constant expression. A const variable may be initialized at runtime.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="cxq2" value="const"><code>const int value</code></label>
    <label><input type="radio" name="cxq2" value="constexpr"><code>constexpr int value</code></label>
    <label><input type="radio" name="cxq2" value="auto"><code>auto value</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="constexpr-quiz-result" aria-live="polite"></p>
</form>

### 3. What happens if a `static_assert` condition is false?

<form class="constexpr-quiz" data-answer="compile-error" data-explanation="static_assert is checked during compilation, so a false condition stops the build with a diagnostic.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="cxq3" value="exception"> The program throws an exception</label>
    <label><input type="radio" name="cxq3" value="warning"> The program compiles with only a warning</label>
    <label><input type="radio" name="cxq3" value="compile-error"> Compilation fails</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="constexpr-quiz-result" aria-live="polite"></p>
</form>

### 4. Which keyword requires every function call to be evaluated at compile time?

<form class="constexpr-quiz" data-answer="consteval" data-explanation="A consteval function is an immediate function: every potentially evaluated call must produce a compile-time result.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="cxq4" value="const"><code>const</code></label>
    <label><input type="radio" name="cxq4" value="constexpr"><code>constexpr</code></label>
    <label><input type="radio" name="cxq4" value="consteval"><code>consteval</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="constexpr-quiz-result" aria-live="polite"></p>
</form>

### 5. Which value should usually remain a runtime `const` value?

<form class="constexpr-quiz" data-answer="sensor" data-explanation="A sensor reading is obtained while the program runs, so it cannot be a compile-time constant. const can still prevent later modification.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="cxq5" value="channels"> A fixed channel count written in the program</label>
    <label><input type="radio" name="cxq5" value="sensor"> A sensor reading received while the program runs</label>
    <label><input type="radio" name="cxq5" value="conversion"> The number of bytes in a kilobyte</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="constexpr-quiz-result" aria-live="polite"></p>
</form>

<style>
.constexpr-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.constexpr-quiz-result { padding: .6rem; }
.constexpr-quiz-result:empty { display: none; }
.constexpr-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.constexpr-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.constexpr-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.constexpr-quiz-result');

    if (!selected) {
      result.className = 'constexpr-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `constexpr-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Code-review challenge

A programmer says this calculation is guaranteed to happen during compilation:

```cpp
constexpr int square(int value)
{
    return value * value;
}

int input{};
std::cin >> input;
const int result = square(input);
```

Are they correct?

<details>
<summary>Show the review</summary>

No. `input` is only known at runtime, so this call to `square` runs at runtime.
The function is allowed to run at compile time, but `constexpr` does not require
all calls to do so.

If the input is fixed and the result must be computed during compilation, use a
constant expression:

```cpp
constexpr int result = square(6);
static_assert(result == 36);
```

Do not replace the original code with `consteval`: a value read from the user
cannot be evaluated during compilation.

</details>

## Completion check

You should now be able to:

- explain the difference between `const` and `constexpr`;
- write a `constexpr` function usable at compile time and runtime;
- require compile-time validation with `static_assert`;
- explain why a particular call runs at runtime;
- distinguish `constexpr`, `consteval`, and `constinit`.

Next, continue with compile-time programming or learn how
[`std::optional`](optional.md) represents a value that may be absent.

## Further reading

- [cppreference: `constexpr`](https://en.cppreference.com/w/cpp/language/constexpr){:target="_blank" rel="noopener noreferrer"}
- [cppreference: constant expressions](https://en.cppreference.com/w/cpp/language/constant_expression){:target="_blank" rel="noopener noreferrer"}

<!-- post-content-skill: 1.0.0 -->
