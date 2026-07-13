---
title: Cpp function arguments
tags:
    - cpp
    - function
    - reference
    - pointer
    - const
---

# Passing variables to functions in C++

When you call a function, you choose how the function receives the variable.

The most common options are:

| Method | Syntax | Can change caller variable? | Can be null? | Common use |
| ------ | ------ | --------------------------- | ------------ | ---------- |
| Pass by value | `void f(int x)` | No | No | Small values, copy is OK |
| Pass by reference | `void f(int& x)` | Yes | No | Function must modify the variable |
| Pass by const reference | `void f(const std::string& s)` | No | No | Read large object without copy |
| Pass by pointer | `void f(int* p)` | Yes, if not null | Yes | Optional object, C API style |
| Pass by const pointer | `void f(const int* p)` | No | Yes | Optional read-only object |
| Pass by rvalue reference | `void f(std::string&& s)` | Function can steal/move | No | Move semantics |

---

## Pass by value

The function gets a copy.

Changing the parameter does not change the original variable.

```cpp
#include <iostream>

void add_one(int x)
{
    x = x + 1;
    std::cout << "inside function: " << x << "\n";
}

int main()
{
    int number = 10;

    add_one(number);

    std::cout << "after function: " << number << "\n";
}
```

Output:

```text
inside function: 11
after function: 10
```

Use pass by value when:

- the variable is small, like `int`, `float`, `double`, `bool`, `char`
- the function needs its own copy
- you do not want to change the caller variable

---

## Pass by reference

The function receives another name for the same variable.

Changing the parameter changes the original variable.

```cpp
#include <iostream>

void add_one(int& x)
{
    x = x + 1;
}

int main()
{
    int number = 10;

    add_one(number);

    std::cout << number << "\n"; // 11
}
```

`int& x` means:

> `x` is a reference to an existing `int`.

Use pass by reference when the function should modify the original variable.

```cpp
void reset_to_zero(int& value)
{
    value = 0;
}
```

!!! warning
    A non-const reference makes it clear that the function may change the input.

---

## Pass by const reference

Pass by const reference means:

- do not copy the object
- do not allow the function to modify it

This is very common for large objects like `std::string`, `std::vector`, and custom classes.

```cpp
#include <iostream>
#include <string>

void print_name(const std::string& name)
{
    std::cout << name << "\n";

    // name = "new name"; // error: name is const
}

int main()
{
    std::string user = "Alice";
    print_name(user);
}
```

Use `const T&` when the function only reads a large object.

```cpp
void print_numbers(const std::vector<int>& numbers);
void draw_robot(const Robot& robot);
void send_message(const std::string& message);
```

---

## Pass by pointer

A pointer stores an address.

```cpp
int value = 10;
int* p = &value;
```

`p` points to `value`.

Inside a function, use `*p` to access the value.

```cpp
#include <iostream>

void add_one(int* x)
{
    if (x == nullptr) {
        return;
    }

    *x = *x + 1;
}

int main()
{
    int number = 10;

    add_one(&number);

    std::cout << number << "\n"; // 11
}
```

Important syntax:

| Syntax | Meaning |
| ------ | ------- |
| `int* p` | `p` is a pointer to an `int` |
| `&number` | address of `number` |
| `*p` | value at the address |
| `nullptr` | pointer points to nothing |

Use pointers when:

- the argument is optional
- you need to pass `nullptr`
- you are working with C libraries
- the codebase already uses pointer style for this API

If the argument must exist, prefer a reference.

```cpp
void must_have_value(int& value);  // cannot be null
void maybe_has_value(int* value);  // can be null
```

---

## Const and pointers

Pointer const syntax is confusing because `const` can apply to the value, the pointer, or both.

Read the declaration from right to left.

### Pointer to const value

```cpp
void print_value(const int* p)
{
    if (p == nullptr) {
        return;
    }

    std::cout << *p << "\n";

    // *p = 5; // error: cannot change value through this pointer
}
```

`const int* p` means:

- the value is const
- the pointer can point somewhere else

### Const pointer to mutable value

```cpp
void change_value(int* const p)
{
    *p = 5;       // OK
    // p = nullptr; // error: p itself is const
}
```

`int* const p` means:

- the value can change
- the pointer cannot point somewhere else

### Const pointer to const value

```cpp
void print_value(const int* const p)
{
    std::cout << *p << "\n";

    // *p = 5;       // error
    // p = nullptr; // error
}
```

`const int* const p` means:

- the value cannot change
- the pointer cannot point somewhere else

---

## Passing arrays

A C-style array usually becomes a pointer when passed to a function.

```cpp
#include <iostream>

void print_array(const int* data, int size)
{
    for (int i = 0; i < size; ++i) {
        std::cout << data[i] << "\n";
    }
}

int main()
{
    int numbers[] = {1, 2, 3};
    print_array(numbers, 3);
}
```

In modern C++, prefer `std::vector`, `std::array`, or `std::span`.

```cpp
#include <iostream>
#include <span>
#include <vector>

void print_numbers(std::span<const int> numbers)
{
    for (int value : numbers) {
        std::cout << value << "\n";
    }
}

int main()
{
    std::vector<int> numbers = {1, 2, 3};
    print_numbers(numbers);
}
```

`std::span` is available from C++20.

---

## Return value instead of output parameter

Often the cleanest method is not to modify an argument.

Return the result instead.

```cpp
int add_one(int x)
{
    return x + 1;
}

int main()
{
    int number = 10;
    int result = add_one(number);
}
```

For multiple return values, use a struct.

```cpp
#include <string>

struct UserInfo {
    std::string name;
    int age;
};

UserInfo create_user()
{
    return {"Alice", 30};
}
```

This is usually easier to read than output parameters.

---

## Output parameters

An output parameter is an argument that the function writes into.

```cpp
bool divide(double a, double b, double& result)
{
    if (b == 0.0) {
        return false;
    }

    result = a / b;
    return true;
}

int main()
{
    double result = 0.0;

    if (divide(10.0, 2.0, result)) {
        // result is 5.0
    }
}
```

This style is useful when:

- the function can fail
- you do not want exceptions
- the codebase uses this pattern

For new code, also consider `std::optional`.

```cpp
#include <optional>

std::optional<double> divide(double a, double b)
{
    if (b == 0.0) {
        return std::nullopt;
    }

    return a / b;
}
```

---

## Pass by rvalue reference and move

This is more advanced, but you will see it in real C++.

`T&&` usually means the function can take ownership of a temporary object.

```cpp
#include <string>
#include <vector>

void add_name(std::vector<std::string>& names, std::string&& name)
{
    names.push_back(std::move(name));
}

int main()
{
    std::vector<std::string> names;

    add_name(names, "Alice");
}
```

`std::move` does not move by itself. It allows moving from an object.

After moving, the object is still valid, but its value should not be trusted.

For beginner code, first learn:

1. value
2. reference
3. const reference
4. pointer

Then learn move semantics.

---

## Function parameters in class methods

Class methods use the same rules.

```cpp
#include <iostream>
#include <string>

class Robot {
public:
    void set_name(std::string name)
    {
        name_ = name;
    }

    void set_battery(double battery)
    {
        battery_ = battery;
    }

    void print() const
    {
        std::cout << name_ << ": " << battery_ << "\n";
    }

private:
    std::string name_;
    double battery_ = 0.0;
};
```

`void print() const` means this method does not change the object.

This is different from `const std::string& name`.

| Const location | Meaning |
| -------------- | ------- |
| `void print() const` | method does not modify `this` object |
| `const std::string& name` | function does not modify `name` |
| `const int* p` | function does not modify `*p` |

### Class method with const reference

```cpp
#include <string>

class Robot {
public:
    void set_name(const std::string& name)
    {
        name_ = name;
    }

private:
    std::string name_;
};
```

This avoids copying the argument when calling the function.

### Class method with value and move

Another common modern style is pass by value, then move into the class member.

```cpp
#include <string>
#include <utility>

class Robot {
public:
    void set_name(std::string name)
    {
        name_ = std::move(name);
    }

private:
    std::string name_;
};
```

This works well when the class needs to keep its own copy anyway.

---

## Quick rules

Use these rules most of the time:

| Situation | Recommended parameter |
| --------- | --------------------- |
| Small read-only value | `int x` |
| Need to modify caller variable | `int& x` |
| Large read-only object | `const std::string& s` |
| Optional object | `Robot* robot` |
| Optional read-only object | `const Robot* robot` |
| Function takes ownership | `std::unique_ptr<T>` |
| Function stores a copy | `std::string name`, then `std::move(name)` |

---

## Common mistakes

### Mistake: expecting value parameter to modify original

```cpp
void reset(int x)
{
    x = 0;
}

int number = 5;
reset(number);
// number is still 5
```

Fix:

```cpp
void reset(int& x)
{
    x = 0;
}
```

### Mistake: forgetting to check pointer

```cpp
void print(int* p)
{
    std::cout << *p << "\n"; // crash if p is nullptr
}
```

Fix:

```cpp
void print(int* p)
{
    if (p == nullptr) {
        return;
    }

    std::cout << *p << "\n";
}
```

### Mistake: copying large objects for no reason

```cpp
void print(std::vector<int> numbers); // copies the vector
```

Fix:

```cpp
void print(const std::vector<int>& numbers); // no copy
```

---

## Practice

Try to answer before checking:

```cpp
void a(int x);
void b(int& x);
void c(const int& x);
void d(int* x);
void e(const int* x);
```

Which functions can change the original `int`?

Answer:

- `a` cannot
- `b` can
- `c` cannot
- `d` can, if `x != nullptr`
- `e` cannot

