---
title: Cpp const and variables
tags:
    - cpp
    - const
    - variables
    - pointers
---

# C++ `const` and variables

`const` means:

> this value cannot be changed through this name.

The important part is **through this name**. With pointers and references, the
original object may still be writable through another non-const name.

---

## Const value

```cpp
const int a = 10;

// a = 20; // error
```

`a` is a constant integer. After initialization, you cannot assign a new value
to it.

This is useful for values that should not change:

```cpp
const int max_speed = 120;
const double pi = 3.14159;
```

---

## Const reference

```cpp
int x = 10;

const int& r = x;

// r = 20; // error: cannot modify x through r
x = 20;    // OK: x itself is not const
```

`r` is a read-only view of `x`.

The reference does not make the original variable permanently const. It only
prevents modification through the reference name `r`.

This is common in function arguments:

```cpp
void print_value(const int& value)
{
    // value = 5; // error
}
```

For small types like `int`, pass by value is usually fine. For larger objects,
`const T&` avoids copying and prevents modification.

---

## Pointer to const value

```cpp
int x = 10;
int y = 20;

const int* p = &x;

// *p = 30; // error: cannot change the int through p
p = &y;     // OK: p can point somewhere else
```

`const int* p` means:

> `p` is a pointer to a const `int`.

The pointed value is read-only through `p`, but the pointer itself can change.

This declaration has the same meaning:

```cpp
int const* p = &x;
```

`const int*` and `int const*` are identical.

---

## Const pointer

```cpp
int x = 10;
int y = 20;

int* const p = &x;

*p = 30;  // OK: can change x through p
// p = &y; // error: p itself cannot point somewhere else
```

`int* const p` means:

> `p` is a const pointer to an `int`.

The pointer address is fixed after initialization, but the value it points to is
still writable.

---

## Const pointer to const value

```cpp
int x = 10;
int y = 20;

const int* const p = &x;

// *p = 30; // error: cannot change the value through p
// p = &y;  // error: cannot change the pointer
```

`const int* const p` means:

> `p` is a const pointer to a const `int`.

Both things are protected:

- the pointer cannot move to another address
- the value cannot be modified through the pointer

---

## Quick table

| Declaration | Pointer can change? | Value through pointer can change? |
|---|---:|---:|
| `const int* p` | yes | no |
| `int const* p` | yes | no |
| `int* const p` | no | yes |
| `const int* const p` | no | no |

---

## How to read pointer const

Read from right to left:

```cpp
const int* p;
```

`p` is a pointer to const `int`.

```cpp
int* const p = nullptr;
```

`p` is a const pointer to `int`.

```cpp
const int* const p = nullptr;
```

`p` is a const pointer to const `int`.

Another useful rule:

> `const` applies to the thing directly on its left. If there is nothing on its
> left, it applies to the thing directly on its right.

Examples:

```cpp
const int* p; // const applies to int
int const* p; // const applies to int
int* const p; // const applies to p
```

---

## Full example

Copy this code into the runtime below and uncomment the error lines one by one.

```cpp
#include <iostream>

void print_line(const char* label, int value)
{
    std::cout << label << " = " << value << "\n";
}

int main()
{
    std::cout << "const value\n";
    const int a = 10;
    print_line("a", a);
    // a = 20; // error: a is const

    std::cout << "\nconst reference\n";
    int x = 10;
    const int& r = x;
    print_line("r before x changes", r);
    // r = 20; // error: cannot modify through const reference
    x = 20;
    print_line("r after x changes", r);

    std::cout << "\npointer to const value\n";
    int p_value1 = 100;
    int p_value2 = 200;
    const int* pointer_to_const = &p_value1;
    print_line("*pointer_to_const", *pointer_to_const);
    // *pointer_to_const = 150; // error: value is read-only through pointer
    pointer_to_const = &p_value2; // OK: pointer can change
    print_line("*pointer_to_const after re-point", *pointer_to_const);

    std::cout << "\nconst pointer\n";
    int writable_value = 300;
    int other_value = 400;
    int* const const_pointer = &writable_value;
    *const_pointer = 350; // OK: pointed value is writable
    print_line("writable_value", writable_value);
    print_line("other_value", other_value);
    // const_pointer = &other_value; // error: pointer itself is const

    std::cout << "\nconst pointer to const value\n";
    int locked_value = 500;
    const int* const locked_pointer = &locked_value;
    print_line("*locked_pointer", *locked_pointer);
    // *locked_pointer = 600; // error: cannot modify value through pointer
    // locked_pointer = &other_value; // error: cannot move pointer
}
```

Expected output:

```text
const value
a = 10

const reference
r before x changes = 10
r after x changes = 20

pointer to const value
*pointer_to_const = 100
*pointer_to_const after re-point = 200

const pointer
writable_value = 350
other_value = 400

const pointer to const value
*locked_pointer = 500
```

---

## Run online

Copy the full example and run it in the embedded OneCompiler C++ runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

---

## Run locally

Save the full example as `main.cpp`:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic main.cpp -o const_variables_demo
./const_variables_demo
```

---

## Rule of thumb

Use `const` when code should only read a value.

- `const int a`: the variable `a` cannot change
- `const int& r`: cannot modify the object through `r`
- `const int* p`: cannot modify the object through `p`
- `int* const p`: cannot move `p` to another object
- `const int* const p`: cannot move `p` and cannot modify the object through `p`

For function parameters, prefer `const T&` for larger objects that should not be
copied or modified.
