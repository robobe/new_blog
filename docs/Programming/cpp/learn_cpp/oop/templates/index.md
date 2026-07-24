---
title: C++ template class, types, and simple tricks
tags:
    - cpp
    - oop
    - templates
---

# C++ template class, types, and simple tricks

A template class lets you write one class pattern and let the compiler create
real classes from it for the types you use.

Without templates, you may write the same class many times:

```cpp
class IntBox {
public:
    explicit IntBox(int value)
        : value_(value)
    {
    }

    int get() const
    {
        return value_;
    }

private:
    int value_;
};

class DoubleBox {
public:
    explicit DoubleBox(double value)
        : value_(value)
    {
    }

    double get() const
    {
        return value_;
    }

private:
    double value_;
};
```

Both classes have the same idea. Only the stored type is different.

With a template class, write the idea once:

```cpp
#include <iostream>
#include <string>

template <typename T>
class Box {
public:
    explicit Box(T value)
        : value_(value)
    {
    }

    T get() const
    {
        return value_;
    }

private:
    T value_;
};

int main()
{
    Box<int> number_box(10);
    Box<std::string> name_box("robot");

    std::cout << number_box.get() << '\n';
    std::cout << name_box.get() << '\n';

    return 0;
}
```

Output:

```text
10
robot
```

## What the syntax means

```cpp
template <typename T>
class Box {
```

This means `Box` is not one finished class yet. It is a class template.

`T` is a type parameter. It is a placeholder for a real type such as:

- `int`
- `double`
- `std::string`
- your own class type

Inside the class, `T` is used like a normal type:

```cpp
T value_;
```

When the code uses:

```cpp
Box<int> number_box(10);
```

the compiler replaces `T` with `int`.

When the code uses:

```cpp
Box<std::string> name_box("robot");
```

the compiler replaces `T` with `std::string`.

## What the compiler does

The compiler does not compile `Box<T>` as one normal class.

It waits until it sees which types are used, then it generates real classes
from the template.

For this code:

```cpp
Box<int> number_box(10);
Box<std::string> name_box("robot");
```

the compiler acts like it created two separate classes.

Conceptually, it creates something like this for `Box<int>`:

```cpp
class Box_int {
public:
    explicit Box_int(int value)
        : value_(value)
    {
    }

    int get() const
    {
        return value_;
    }

private:
    int value_;
};
```

And something like this for `Box<std::string>`:

```cpp
class Box_string {
public:
    explicit Box_string(std::string value)
        : value_(value)
    {
    }

    std::string get() const
    {
        return value_;
    }

private:
    std::string value_;
};
```

These names are only for explanation. The compiler uses its own internal names.

This process is called template instantiation.

## Each type gets its own class

`Box<int>` and `Box<double>` are different types.

This means this code does not compile:

```cpp
Box<int> int_box(5);
Box<double> double_box(5.5);

int_box = double_box; // Error: different types
```

Even though both come from the same template, the compiler treats them as
different classes after instantiation.

## Errors happen when the template is used

A template is checked fully only when the compiler creates a real class from it.

For example:

```cpp
template <typename T>
class Printer {
public:
    explicit Printer(T value)
        : value_(value)
    {
    }

    void print_twice() const
    {
        std::cout << value_ + value_ << '\n';
    }

private:
    T value_;
};
```

This works with `int`:

```cpp
Printer<int> printer(10);
printer.print_twice(); // Prints 20
```

It also works with `std::string`:

```cpp
Printer<std::string> printer("hi");
printer.print_twice(); // Prints hihi
```

But it fails for a type that does not support `operator+`:

```cpp
struct User {
    std::string name;
};

Printer<User> printer({"alice"});
printer.print_twice(); // Error: User + User is not defined
```

The template code asks for `value_ + value_`. The compiler can only generate
that code for types where `+` is valid.

---

## What types can a template use?

A template type parameter can use almost any C++ type.

Common examples:

- built-in types: `int`, `double`, `bool`, `char`
- standard library types: `std::string`, `std::vector<int>`
- pointer types: `int*`, `User*`
- reference types: `int&`, `const std::string&`
- your own classes and structs: `Robot`, `User`, `Point`

Example:

```cpp
#include <iostream>
#include <string>

template <typename T>
class Holder {
public:
    explicit Holder(T value)
        : value_(value)
    {
    }

    T get() const
    {
        return value_;
    }

private:
    T value_;
};

struct Point {
    int x;
    int y;
};

int main()
{
    Holder<int> number(42);
    Holder<std::string> text("hello");
    Holder<Point> point({3, 4});

    std::cout << number.get() << '\n';
    std::cout << text.get() << '\n';
    std::cout << point.get().x << ", " << point.get().y << '\n';

    return 0;
}
```

The important rule is simple: the type must support the operations used inside
the template.

If the template only stores and returns the value, many types work.

If the template uses `+`, the type must support `+`.

If the template uses `<`, the type must support `<`.

If the template prints with `std::cout << value`, the type must support
`operator<<`.

## Template with multiple parameters

A template can have more than one type parameter.

```cpp
#include <iostream>
#include <string>

template <typename Key, typename Value>
class Pair {
public:
    Pair(Key key, Value value)
        : key_(key), value_(value)
    {
    }

    Key key() const
    {
        return key_;
    }

    Value value() const
    {
        return value_;
    }

private:
    Key key_;
    Value value_;
};

int main()
{
    Pair<std::string, int> age("Alice", 30);
    Pair<int, double> sensor_reading(7, 23.5);

    std::cout << age.key() << ": " << age.value() << '\n';
    std::cout << sensor_reading.key() << ": " << sensor_reading.value() << '\n';

    return 0;
}
```

Output:

```text
Alice: 30
7: 23.5
```

Here the compiler creates one class for:

```cpp
Pair<std::string, int>
```

and another class for:

```cpp
Pair<int, double>
```

Each different combination of template arguments creates a different type.

## Template with a non-type parameter

Templates can also receive values, not only types.

This is called a non-type template parameter.

```cpp
#include <iostream>

template <typename T, int Size>
class Array {
public:
    T& operator[](int index)
    {
        return data_[index];
    }

    int size() const
    {
        return Size;
    }

private:
    T data_[Size];
};

int main()
{
    Array<int, 3> numbers;

    numbers[0] = 10;
    numbers[1] = 20;
    numbers[2] = 30;

    for (int i = 0; i < numbers.size(); ++i) {
        std::cout << numbers[i] << '\n';
    }

    return 0;
}
```

`Array<int, 3>` and `Array<int, 5>` are different types because the size is part
of the template arguments.

## Simple template tricks and usages

### Use a template for a reusable function

Templates are not only for classes. A common use is a function that works with
different types.

```cpp
template <typename T>
T max_value(T a, T b)
{
    if (a > b) {
        return a;
    }

    return b;
}
```

Usage:

```cpp
std::cout << max_value<int>(3, 8) << '\n';
std::cout << max_value<double>(2.5, 1.2) << '\n';
```

In many cases, the compiler can detect the type automatically:

```cpp
std::cout << max_value(3, 8) << '\n';       // T is int
std::cout << max_value(2.5, 1.2) << '\n';   // T is double
```

This is called template argument deduction.

### Use `using` to make a template easier to read

Long template names can become noisy.

```cpp
template <typename T>
class Box {
public:
    explicit Box(T value)
        : value_(value)
    {
    }

    T get() const
    {
        return value_;
    }

private:
    T value_;
};

using IntBox = Box<int>;
using TextBox = Box<std::string>;
```

Now the usage is shorter:

```cpp
IntBox id(100);
TextBox name("motor");
```

`IntBox` is only an alias. The real type is still `Box<int>`.

### Give a template a default type

A template parameter can have a default value.

```cpp
template <typename T = int>
class Counter {
public:
    void add(T value)
    {
        total_ += value;
    }

    T total() const
    {
        return total_;
    }

private:
    T total_ = 0;
};
```

Usage:

```cpp
Counter<> normal_counter;       // Uses int
Counter<double> precise_counter; // Uses double
```

The empty `<>` means: use the default template argument.

### Use `static_assert` for a clear compile error

Sometimes a template should only accept certain kinds of types.

For example, this class should only work with numbers:

```cpp
#include <type_traits>

template <typename T>
class NumberBox {
public:
    static_assert(std::is_arithmetic_v<T>, "NumberBox requires a numeric type");

    explicit NumberBox(T value)
        : value_(value)
    {
    }

    T doubled() const
    {
        return value_ + value_;
    }

private:
    T value_;
};
```

This works:

```cpp
NumberBox<int> number(10);
```

This fails with a clear error:

```cpp
NumberBox<std::string> text("hello");
```

`static_assert` is useful because it tells the user of the template what the
rule is.

## Important idea

A template class is a recipe.

`Box<T>` is the recipe.

`Box<int>` is a real class generated from the recipe.

`Box<std::string>` is another real class generated from the same recipe.

The compiler creates only the versions that your program actually uses.
