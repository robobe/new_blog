---
title: Cpp const in function signatures
tags:
    - cpp
    - oop
    - const
    - functions
---

# C++ `const` in function signatures

`const` can appear in several places in a member function signature.

Each place controls something different:

- `const X&` before the function name: the return value is a read-only reference
- `const X& x` in the argument list: the parameter is a read-only reference
- `const` after the argument list: the function cannot modify `*this`
- `X&`: a writable reference

Example:

```cpp
const X& f1(const X& x) const
```

Read it like this:

> `f1` is a const member function. It receives `x` as a const reference and
> returns a const reference to `X`.

The `const` words do not all mean the same object is const. Each one belongs to
the type or function part next to it.

---

## The example class

```cpp
class X {
public:
    const X& f1(const X& x) const {
        return *this;
    }

    const X& f2(X& x) const {
        return *this;
    }

    X& f3(X& x) const {
        return x;
    }

    X& f4(X& x) {
        return *this;
    }
};
```

---

## How to read the signature

Use this order:

1. Read the function name and parameters.
2. Check if there is `const` after `)`.
3. Check whether each parameter is writable or read-only.
4. Check whether the return value is writable or read-only.

For member functions, the suffix `const` is very important:

```cpp
void g() const
```

This means:

> inside `g`, `this` behaves like `const X*`.

So `g` cannot modify the current object and cannot return `*this` as `X&`.

---

## `f1`: const return, const argument, const function

```cpp
const X& f1(const X& x) const {
    return *this;
}
```

Meaning:

- `const X&` return: the caller receives a read-only reference
- `const X& x`: the function can read `x` but cannot modify it
- suffix `const`: the function can read `*this` but cannot modify it

This is the most read-only version.

It can be called on both non-const and const objects:

```cpp
X a;
const X ca;

a.f1(a);   // OK
ca.f1(a);  // OK
```

But the returned reference is const:

```cpp
X a;
X b;

const X& r = a.f1(b); // OK
// X& bad = a.f1(b);  // error: cannot bind X& to const X
```

---

## `f2`: const return, writable argument, const function

```cpp
const X& f2(X& x) const {
    return *this;
}
```

Meaning:

- `const X&` return: the caller receives a read-only reference
- `X& x`: the argument must be a non-const `X`
- suffix `const`: the function cannot modify `*this`

The function itself is const, but the parameter is not const.

So this is allowed:

```cpp
const X owner;
X value;

owner.f2(value); // OK: owner is const, value is writable
```

But this is not allowed:

```cpp
const X owner;
const X value;

// owner.f2(value); // error: f2 needs X&, not const X&
```

Even though `f2` does not modify `x` in this code, its signature says it could.
That is why a const argument cannot be passed.

---

## `f3`: writable return, writable argument, const function

```cpp
X& f3(X& x) const {
    return x;
}
```

Meaning:

- `X&` return: the caller receives a writable reference
- `X& x`: the argument must be a non-const `X`
- suffix `const`: the function cannot modify `*this`

This function is const with respect to the current object, but it can still
return another object as writable.

```cpp
const X owner;
X value;

X& r = owner.f3(value); // OK
r = X{};               // OK: r refers to value, not owner
```

This works because `f3` returns `x`, not `*this`.

This would not compile:

```cpp
class Bad {
public:
    Bad& f() const {
        // return *this; // error: *this is const inside a const member function
    }
};
```

Inside a const member function, `*this` is treated as const. You cannot return it
as a writable `X&`.

---

## `f4`: writable return, writable argument, non-const function

```cpp
X& f4(X& x) {
    return *this;
}
```

Meaning:

- `X&` return: the caller receives a writable reference
- `X& x`: the argument must be a non-const `X`
- no suffix `const`: the function may modify `*this`

Because `f4` is not a const member function, it cannot be called on a const
object:

```cpp
X a;
X b;
const X ca;

a.f4(b);    // OK
// ca.f4(b); // error: f4 is not const
```

The returned reference is writable:

```cpp
X a;
X b;

X& r = a.f4(b); // OK: r refers to a
r = X{};        // OK: modifies a
```

---

## Full test program

Uncomment the error lines one by one to see what the compiler rejects.

```cpp
#include <iostream>

class X {
public:
    int value = 0;

    const X& f1(const X& x) const {
        std::cout << "f1 reads x.value = " << x.value << "\n";
        return *this;
    }

    const X& f2(X& x) const {
        x.value = 20; // OK: x is not const
        return *this;
    }

    X& f3(X& x) const {
        x.value = 30; // OK: x is not const
        return x;
    }

    X& f4(X& x) {
        value = 40;   // OK: this function is not const
        x.value = 50; // OK: x is not const
        return *this;
    }
};

int main()
{
    X a;
    X b;
    const X ca;
    const X cb;

    a.f1(b);   // OK
    a.f1(cb);  // OK
    ca.f1(b);  // OK
    ca.f1(cb); // OK

    a.f2(b);   // OK
    ca.f2(b);  // OK
    // a.f2(cb);  // error: f2 needs X&, but cb is const
    // ca.f2(cb); // error: f2 needs X&, but cb is const

    a.f3(b);   // OK
    ca.f3(b);  // OK
    // a.f3(cb);  // error: f3 needs X&, but cb is const

    a.f4(b);   // OK
    // ca.f4(b); // error: f4 is not a const member function

    const X& read_only = a.f1(b);
    // X& writable1 = a.f1(b); // error: f1 returns const X&
    std::cout << "read_only.value = " << read_only.value << "\n";

    X& writable2 = ca.f3(b); // OK: returns b as writable
    writable2.value = 100;

    X& writable3 = a.f4(b); // OK: returns a as writable
    writable3.value = 200;

    std::cout << "a.value = " << a.value << "\n";
    std::cout << "b.value = " << b.value << "\n";
}
```

Compile:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic const_functions.cpp -o const_functions
./const_functions
```

---

## Quick table

| Function | Can be called on const object? | Can receive const argument? | Return is writable? | Can modify `*this`? |
|---|---:|---:|---:|---:|
| `const X& f1(const X& x) const` | yes | yes | no | no |
| `const X& f2(X& x) const` | yes | no | no | no |
| `X& f3(X& x) const` | yes | no | yes | no |
| `X& f4(X& x)` | no | no | yes | yes |

---

## Other `const` roles in classes

`const` is also common in these class-related places.

### Const data members

A data member can be const:

```cpp
class User {
public:
    User(int id) : id_(id) {}

private:
    const int id_;
};
```

A const data member must be initialized in the constructor initializer list.
After construction, it cannot be assigned a new value.

### Const objects

```cpp
const X x;
```

A const object can only call const member functions.

### Const pointers

```cpp
const X* p1; // pointer to const X
X* const p2 = nullptr; // const pointer to X
const X* const p3 = nullptr; // const pointer to const X
```

Read pointer declarations from right to left:

- `const X*`: the `X` is const
- `X* const`: the pointer itself is const
- `const X* const`: both are const

### Mutable data members

`mutable` is the exception to const member functions.

```cpp
class Counter {
public:
    int get() const {
        ++read_count_; // OK because read_count_ is mutable
        return value_;
    }

private:
    int value_ = 0;
    mutable int read_count_ = 0;
};
```

Use `mutable` carefully. It is usually for cache values, counters, or internal
bookkeeping that does not change the logical value of the object.

---

## Rule of thumb

Use `const` to describe what the function promises:

- use `const T&` for parameters that should only be read
- put `const` after a member function when it should not modify the object
- return `const T&` when callers should not modify the returned object through
  that reference
- return `T&` only when callers are allowed to modify the returned object

The suffix `const` protects the current object. Parameter `const` protects the
argument. Return `const` protects the value returned to the caller.
