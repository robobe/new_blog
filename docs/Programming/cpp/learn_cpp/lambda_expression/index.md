---
title: C++ lambda expression
tags:
    - cpp
    - learn
    - lambda
    - function
---

# C++ lambda expression

A lambda expression is a small function you can write directly inside your code.

Use a lambda when you need a short function for one local task.

Basic shape:

```cpp
[capture](arguments) {
    body
};
```

Parts:

- `[]`: capture list, controls which outside variables the lambda can use
- `()`: function arguments
- `{}`: function body

## Simple example

```cpp
#include <iostream>

int main()
{
    auto say_hello = []() {
        std::cout << "Hello from lambda\n";
    };

    say_hello();

    return 0;
}
```

Output:

```text
Hello from lambda
```

The lambda is stored in the variable `say_hello`.

Then we call it like a normal function:

```cpp
say_hello();
```

## Lambda with arguments

A lambda can receive arguments like a normal function.

```cpp
#include <iostream>

int main()
{
    auto add = [](int a, int b) {
        return a + b;
    };

    int result = add(3, 4);

    std::cout << result << '\n';

    return 0;
}
```

Output:

```text
7
```

Here:

```cpp
[](int a, int b) {
    return a + b;
}
```

means: create a small function that receives two `int` values and returns their
sum.

## Pass a lambda to a function

Lambdas are often passed to other functions.

Example with `std::sort`:

```cpp
#include <algorithm>
#include <iostream>
#include <vector>

int main()
{
    std::vector<int> numbers = {4, 1, 3, 2};

    std::sort(numbers.begin(), numbers.end(), [](int left, int right) {
        return left < right;
    });

    for (int number : numbers) {
        std::cout << number << ' ';
    }

    std::cout << '\n';

    return 0;
}
```

Output:

```text
1 2 3 4
```

The lambda tells `std::sort` how to compare two numbers.

## Scope and capture

A lambda has its own scope.

Variables created inside the lambda are local to the lambda:

```cpp
auto print_value = []() {
    int value = 10;
    std::cout << value << '\n';
};
```

The variable `value` exists only inside the lambda body.

Outside variables are not available unless you capture them.

This does not compile:

```cpp
int x = 5;

auto print_x = []() {
    std::cout << x << '\n'; // Error: x is outside the lambda scope
};
```

To use `x`, capture it.

## Capture by value

Capture by value copies the outside variable into the lambda.

```cpp
#include <iostream>

int main()
{
    int x = 5;

    auto print_x = [x]() {
        std::cout << x << '\n';
    };

    x = 10;

    print_x();

    return 0;
}
```

Output:

```text
5
```

The lambda copied `x` when it was created. Later changes to the outside `x` do
not change the lambda copy.

## Capture by reference

Capture by reference lets the lambda use the original outside variable.

```cpp
#include <iostream>

int main()
{
    int x = 5;

    auto print_x = [&x]() {
        std::cout << x << '\n';
    };

    x = 10;

    print_x();

    return 0;
}
```

Output:

```text
10
```

The lambda uses the same `x` from the outer scope.

## Modify captured values

By default, a value captured by copy is read-only inside the lambda.

This does not compile:

```cpp
int x = 5;

auto change_x = [x]() {
    x = 10; // Error: copied capture is read-only
};
```

Use `mutable` if you want to modify the lambda copy:

```cpp
#include <iostream>

int main()
{
    int x = 5;

    auto change_copy = [x]() mutable {
        x = 10;
        std::cout << "inside lambda: " << x << '\n';
    };

    change_copy();

    std::cout << "outside lambda: " << x << '\n';

    return 0;
}
```

Output:

```text
inside lambda: 10
outside lambda: 5
```

`mutable` changes only the lambda copy. The outside `x` is still `5`.

## Capture everything

C++ also supports short capture forms.

Capture all used outside variables by value:

```cpp
[=]() {
    std::cout << x << '\n';
}
```

Capture all used outside variables by reference:

```cpp
[&]() {
    x = 10;
}
```

These are convenient, but for beginner code it is usually clearer to capture
only what you need:

```cpp
[x, &total]() {
    std::cout << x << '\n';
    total += x;
}
```

Here:

- `x` is captured by value
- `total` is captured by reference

## Key points

- A lambda is a small local function.
- Use `auto` to store a lambda.
- Put arguments in `()`.
- Use `[]` to capture outside variables.
- `[x]` captures `x` by value.
- `[&x]` captures `x` by reference.
- Use `mutable` only when you need to modify a captured copy.
- Prefer explicit captures when learning.
