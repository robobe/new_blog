---
title: Cpp operator overload
tags:
    - cpp
    - oop
    - operator-overload
---

# C++ operator overload

Operator overloading lets a class define how normal C++ operators work with
objects of that class.

For example, if `Number` is a class, C++ does not automatically know what this
means:

```cpp
Number sum = a + b;
```

By writing `operator+`, we teach the class how `+` should behave.

---

## Full example

```cpp
#include <iostream>

class Number {
public:
    explicit Number(int initial_value)
        : value_(initial_value)
    {
    }

    Number& operator=(const Number& other)
    {
        value_ = other.value_;
        return *this;
    }

    Number operator+(const Number& other) const
    {
        return Number(value_ + other.value_);
    }

    int get_value() const
    {
        return value_;
    }

private:
    int value_;
};

std::ostream& operator<<(std::ostream& out, const Number& number)
{
    return out << number.get_value();
}

Number operator-(const Number& left, const Number& right)
{
    return Number(left.get_value() - right.get_value());
}

bool operator==(const Number& left, const Number& right)
{
    return left.get_value() == right.get_value();
}

int main()
{
    Number a(10);
    Number b(20);

    // Calls a.operator+(b).
    Number sum = a + b;

    // Calls operator-(b, a).
    Number difference = b - a;

    a = sum; // Calls our copy assignment operator.

    // Calls our operator<< function for each Number.
    std::cout << "Sum: " << sum << '\n';
    std::cout << "Difference: " << difference << '\n';
    std::cout << "Are they equal? " << (sum == difference ? "Yes" : "No") << '\n';

    return 0;
}
```

Output:

```text
Sum: 30
Difference: 10
Are they equal? No
```

---

## Member operator overload

A member operator is written inside the class.

```cpp
Number operator+(const Number& other) const
{
    return Number(value_ + other.value_);
}
```

This means:

```cpp
Number sum = a + b;
```

is translated by the compiler into:

```cpp
Number sum = a.operator+(b);
```

So with a member operator:

- the left side is the current object, `*this`
- the right side is passed as a function argument
- the function can access private members of the class

Inside `operator+`, the object on the left side is `a`, and the object on the
right side is `b`.

```cpp
return Number(value_ + other.value_);
```

This is the same idea as:

```cpp
return Number(a.value_ + b.value_);
```

but inside the member function, `a.value_` is written as just `value_`.

---

## Assignment operator must be a member

Some operators must be overloaded as member functions.

The assignment operator is one of them:

```cpp
Number& operator=(const Number& other)
{
    value_ = other.value_;
    return *this;
}
```

This code:

```cpp
a = sum;
```

calls:

```cpp
a.operator=(sum);
```

The function returns `Number&` so chained assignment can work:

```cpp
a = b = Number(30);
```

The expression `b = Number(30)` returns `b`, then `a = b` can run.

Important operators that must be members include:

- `operator=`
- `operator[]`
- `operator()`
- `operator->`

Use a member function for these operators because C++ requires it.

---

## Non-member operator overload

A non-member operator is written outside the class.

```cpp
Number operator-(const Number& left, const Number& right)
{
    return Number(left.get_value() - right.get_value());
}
```

This means:

```cpp
Number difference = b - a;
```

is translated by the compiler into:

```cpp
Number difference = operator-(b, a);
```

With a non-member operator:

- both sides are normal function arguments
- there is no `this`
- the function cannot access private members unless it is a `friend`
- it usually uses public functions like `get_value()`

In this example, `operator-` is outside the class, so it cannot directly use
`value_`.

This would not compile:

```cpp
Number operator-(const Number& left, const Number& right)
{
    return Number(left.value_ - right.value_); // error: value_ is private
}
```

So the function uses the public getter:

```cpp
return Number(left.get_value() - right.get_value());
```

---

## Non-member operator for output

The stream operator `<<` is normally written as a non-member.

```cpp
std::ostream& operator<<(std::ostream& out, const Number& number)
{
    return out << number.get_value();
}
```

This code:

```cpp
std::cout << sum;
```

calls:

```cpp
operator<<(std::cout, sum);
```

It should not be a member of `Number`, because the left side is not a `Number`.
The left side is `std::cout`, which is a `std::ostream`.

If `operator<<` were a member of `Number`, the syntax would need to look like
this:

```cpp
sum << std::cout; // wrong direction for normal printing
```

So `operator<<` is almost always a non-member operator.

It returns `std::ostream&` so printing can be chained:

```cpp
std::cout << "Sum: " << sum << '\n';
```

First this runs:

```cpp
std::cout << "Sum: "
```

Then the same stream is returned, so this can run next:

```cpp
<< sum
```

Then the stream is returned again, so this can run:

```cpp
<< '\n'
```

---

## Non-member operator for equality

The equality operator is also often written as a non-member.

```cpp
bool operator==(const Number& left, const Number& right)
{
    return left.get_value() == right.get_value();
}
```

This code:

```cpp
sum == difference
```

calls:

```cpp
operator==(sum, difference);
```

For comparison operators, non-member functions are often preferred because the
left and right sides are treated equally.

---

## Member vs non-member

Use a member operator when:

- C++ requires the operator to be a member, like `=`, `[]`, `()`, or `->`
- the operator naturally modifies the left object, like `+=`
- the operation is strongly tied to the class internals

Use a non-member operator when:

- the left side is not your class, like `std::cout << number`
- both sides should be treated equally, like `a == b`
- you want conversions to work on the left side too
- the operator can be implemented using the public interface

Example:

```cpp
Number operator-(const Number& left, const Number& right)
```

This is a good non-member because subtraction uses two `Number` objects equally.

Example:

```cpp
Number& operator=(const Number& other)
```

This must be a member because assignment changes the object on the left side.

---

## Implicit conversion and non-member operators

Implicit conversion on the left side only works with a non-member operator.

In the main example, the constructor is marked `explicit`:

```cpp
explicit Number(int initial_value)
```

That is usually a good choice because it prevents accidental conversions from
`int` to `Number`.

For this example only, remove `explicit` so C++ is allowed to convert an `int`
into a `Number` automatically.

### Member operator

With a member `operator+`:

```cpp
class Number {
public:
    Number(int initial_value)
        : value_(initial_value)
    {
    }

    Number operator+(const Number& other) const
    {
        return Number(value_ + other.value_);
    }

private:
    int value_;
};
```

This works:

```cpp
Number a(10);
Number result = a + 5;
```

The compiler sees:

```cpp
Number result = a.operator+(5);
```

The left side is already a `Number`, and the right side `5` can be converted to
`Number`.

But this does not work:

```cpp
Number result = 5 + a; // error
```

For a member operator, the left side must already be the class object, because
C++ needs something to call the member function on.

The compiler would need this:

```cpp
5.operator+(a); // impossible: int has no Number::operator+
```

So the compiler cannot use `Number::operator+` here.

### Non-member operator

With a non-member `operator+`:

```cpp
class Number {
public:
    Number(int initial_value)
        : value_(initial_value)
    {
    }

    int get_value() const
    {
        return value_;
    }

private:
    int value_;
};

Number operator+(const Number& left, const Number& right)
{
    return Number(left.get_value() + right.get_value());
}
```

Both directions work:

```cpp
Number a(10);

Number x = a + 5; // operator+(a, Number(5))
Number y = 5 + a; // operator+(Number(5), a)
```

Because `operator+` is a normal function, both arguments can use implicit
conversion.

That is why symmetric operators like `+`, `-`, `==`, and `!=` are often better
as non-member functions.

---

## Conversion operator

A conversion operator lets an object convert itself to another type.

The syntax is:

```cpp
operator target_type() const
```

Example:

```cpp
#include <iostream>

class Number {
public:
    explicit Number(int value)
        : value_(value)
    {
    }

    operator int() const
    {
        return value_;
    }

private:
    int value_;
};

int main()
{
    Number number(42);

    int x = number; // calls number.operator int()

    std::cout << x << '\n';

    return 0;
}
```

Output:

```text
42
```

This function:

```cpp
operator int() const
```

means:

> a `Number` object can be converted to an `int`.

So this line:

```cpp
int x = number;
```

is similar to:

```cpp
int x = number.operator int();
```

Notice that a conversion operator has no return type before the function name.
The target type is written after the word `operator`.

Do not write this:

```cpp
int operator int() const // error
```

Write this:

```cpp
operator int() const
```

### Convert to another class type

A conversion operator can also convert an object to another class type.

Example:

```cpp
#include <iostream>
#include <string>

class Text {
public:
    explicit Text(std::string value)
        : value_(value)
    {
    }

    void print() const
    {
        std::cout << value_ << '\n';
    }

private:
    std::string value_;
};

class Number {
public:
    explicit Number(int value)
        : value_(value)
    {
    }

    operator Text() const
    {
        return Text(std::to_string(value_));
    }

private:
    int value_;
};

int main()
{
    Number number(42);

    Text text = number; // calls number.operator Text()

    text.print();

    return 0;
}
```

Output:

```text
42
```

This function:

```cpp
operator Text() const
```

means:

> a `Number` object can be converted to a `Text` object.

So this line:

```cpp
Text text = number;
```

is similar to:

```cpp
Text text = number.operator Text();
```

The conversion operator belongs to the source type.

In this example, `Number` knows how to convert itself into `Text`, so
`operator Text()` is written inside `Number`.

### Explicit conversion operator

Implicit conversion can sometimes make code unclear.

You can make the conversion operator `explicit`:

```cpp
class Number {
public:
    explicit Number(int value)
        : value_(value)
    {
    }

    explicit operator int() const
    {
        return value_;
    }

private:
    int value_;
};
```

Now this does not work:

```cpp
Number number(42);
int x = number; // error
```

You must write the conversion clearly:

```cpp
int x = static_cast<int>(number);
```

Use `explicit operator type()` when automatic conversion could surprise the
reader.

---

## Const correctness

This operator is marked `const`:

```cpp
Number operator+(const Number& other) const
```

The final `const` means `operator+` does not modify the left object.

That is important because addition should create a new value:

```cpp
Number sum = a + b;
```

It should not change `a` or `b`.

The parameter is also a const reference:

```cpp
const Number& other
```

This avoids copying `other` and promises not to modify it.

---

## Friend functions

Sometimes a **non-member** operator needs access to private data.

One option is to make it a `friend`.

```cpp
class Number {
public:
    explicit Number(int initial_value)
        : value_(initial_value)
    {
    }

    friend Number operator-(const Number& left, const Number& right);

private:
    int value_;
};

Number operator-(const Number& left, const Number& right)
{
    return Number(left.value_ - right.value_);
}
```

A `friend` function is not a member function, but it is allowed to access private
members.

This line inside the class is only a declaration:

```cpp
friend Number operator-(const Number& left, const Number& right);
```

It tells C++:

> this non-member function is allowed to access private members of `Number`.

The function body can be written outside the class:

```cpp
Number operator-(const Number& left, const Number& right)
{
    return Number(left.value_ - right.value_);
}
```

You can also write the body directly inside the class:

```cpp
class Number {
public:
    explicit Number(int initial_value)
        : value_(initial_value)
    {
    }

    friend Number operator-(const Number& left, const Number& right)
    {
        return Number(left.value_ - right.value_);
    }

private:
    int value_;
};
```

This is still a non-member function. The `friend` keyword gives access to private
members, but it does not make the function a class member.

Use `friend` when the operator really belongs outside the class but needs direct
access to private state.

Do not use `friend` automatically. If the operator can be cleanly implemented
with public getters, that is often simpler.

---

## Common rule

A practical rule:

- operators that change the left object are often members
- symmetric operators are often non-members
- stream operators are non-members
- assignment-like operators are members

In this example:

```cpp
a + b       // member: a.operator+(b)
b - a       // non-member: operator-(b, a)
a = sum     // member: a.operator=(sum)
std::cout << sum // non-member: operator<<(std::cout, sum)
sum == difference // non-member: operator==(sum, difference)
```

The syntax looks similar, but the function call behind the syntax is different.

---

## Demo: conversion operation

This example shows a conversion operator from one class type to another class
type.

```cpp
#include <iostream>
using namespace std;

class A
{
public:
    friend ostream& operator<<(ostream& out, const A& other)
    {
        out << "1";
        return out;
    }
};

class Op
{
public:
    operator A() const
    {
        return A();
    }
};

int main()
{
    Op op;
    cout << (A)op << endl;
}
```

Output:

```text
1
```

This function is the conversion operator:

```cpp
operator A() const
{
    return A();
}
```

It means:

> an `Op` object can be converted to an `A` object.

This line:

```cpp
cout << (A)op << endl;
```

first converts `op` to `A`:

```cpp
(A)op
```

That cast calls:

```cpp
op.operator A()
```

After the conversion, the expression becomes similar to:

```cpp
cout << A() << endl;
```

Then C++ calls the `operator<<` function for `A`:

```cpp
friend ostream& operator<<(ostream& out, const A& other)
{
    out << "1";
    return out;
}
```

So the full flow is:

```cpp
Op op;
cout << (A)op << endl;
```

1. `(A)op` calls `Op::operator A() const`
2. `operator A()` returns a new `A` object
3. `cout << A_object` calls `operator<<(ostream&, const A&)`
4. `operator<<` prints `1`

The conversion operator belongs to the source type, so `operator A()` is written
inside `Op`.

The stream operator belongs outside `A` because the left side is `cout`, not an
`A` object.
