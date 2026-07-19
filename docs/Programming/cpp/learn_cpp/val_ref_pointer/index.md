---
title: by_val by_ref and pointer
tags:
    - cpp
    - by_val
    - by_ref
    - pointers
---

## by_val
Copy the value from the caller to callee

```cpp title="classic example"
#include <iostream>

void increment(int x)
{
    x++;
    std::cout << x << std::endl;
}

int main()
{
    int a = 5;
    increment(a);
    std::cout << a << std::endl;
}
```

### return by_val from a function

Return by value means the function returns a new value to the caller.
The caller receives its own object, so changing it does not change the local
variable that was inside the function.

```cpp title="return by value"
#include <iostream>

int make_number()
{
    int x = 10;
    return x; // returns a value
}

int main()
{
    int a = make_number();
    a++;

    std::cout << a << std::endl;
}
```

Output:

```text
11
```

In this example, `x` is a local variable inside `make_number`.
After the function returns, `x` is gone, but its value was used to initialize
`a` in `main`.

## ref

A reference is another name (alias) for an existing variable.
It does not create a new `int`; it points to the same object.

```cpp title="reference"
#include <iostream>

int main()
{
    int a = 5;
    int& r = a;

    r++;

    std::cout << a << std::endl;
}
```

Output:

```text
6
```

`r` and `a` are two names for the same value.
Changing `r` changes `a`.

## by_ref

Pass by reference means the function parameter is a reference to the caller
variable. The function can modify the original variable.

```cpp title="pass by reference"
#include <iostream>

void increment(int& x)
{
    x++;
}

int main()
{
    int a = 5;
    increment(a);

    std::cout << a << std::endl;
}
```

Output:

```text
6
```

In `void increment(int& x)`, `x` is not a copy.
It is a reference to `a`, so `x++` changes `a`.

### return by_ref from a function

Return by reference means the function returns a reference to an existing object.
The caller can use the returned value to modify that original object.

```cpp title="return by reference"
#include <iostream>

int& first_item(int values[])
{
    return values[0];
}

int main()
{
    int numbers[] = {10, 20, 30};

    first_item(numbers) = 99;

    std::cout << numbers[0] << std::endl;
}
```

Output:

```text
99
```

`first_item(numbers)` returns a reference to `numbers[0]`.
Because the return type is `int&`, assigning to the function result changes the
array element itself.

Do not return a reference to a local variable:

```cpp
int& bad()
{
    int x = 10;
    return x; // wrong: x is destroyed when the function ends
}
```

---

<details>
<summary>Run section</summary>


<div class="cpp-runner" style="--cpp-editor-height: 230px;">

<textarea class="cpp-editor" spellcheck="false">
#include <iostream>

int make_number()
{
    int x = 10;
    return x;
}

int main()
{
    int a = make_number();
    a++;

    std::cout << a << std::endl;
}
</textarea>

<button class="cpp-run-button">
Run
</button>

<pre class="cpp-output" hidden></pre>

</div>
</details>
