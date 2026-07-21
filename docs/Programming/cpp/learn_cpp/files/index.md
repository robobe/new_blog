---
title: C++ files and streams
tags:
    - cpp
    - files
    - streams
    - fstream
---

# C++ files and streams

In C++, a **stream** is an object that moves data from one place to another.

You already use streams when printing to the terminal:

```cpp
std::cout << "Hello\n";
```

`std::cout` is an output stream.

File streams work in a similar way, but the data goes to or comes from a file.

---

## Basic stream types

Include the file stream library:

```cpp
#include <fstream>
```

Common file stream classes:

| Stream | Meaning | Usage |
|---|---|---|
| `std::ofstream` | output file stream | write to a file |
| `std::ifstream` | input file stream | read from a file |
| `std::fstream` | file stream | read and write |

---

## Write to a file

This creates a file called `hello.txt` and writes text into it.

```cpp
#include <fstream>

int main()
{
    std::ofstream file("hello.txt");

    file << "Hello from C++\n";
    file << "This line is written to a file\n";

    file.close();
}
```

After running the program, `hello.txt` contains:

```text
Hello from C++
This line is written to a file
```

`file << value` works like `std::cout << value`, but the output goes to the
file instead of the terminal.

---

## Read from a file

This reads text from `hello.txt` line by line.

```cpp
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main()
{
    std::ifstream file("hello.txt");
    std::string line;

    while (std::getline(file, line)) {
        std::cout << line << "\n";
    }
}
```

Output:

```text
Hello from C++
This line is written to a file
```

`std::getline(file, line)` reads one full line from the file into the string
`line`.

---

## Check if the file opened

Always check that the file opened successfully.

```cpp
#include <fstream>
#include <iostream>

int main()
{
    std::ifstream file("data.txt");

    if (!file) {
        std::cout << "Could not open file\n";
        return 1;
    }

    std::cout << "File opened\n";
}
```

This is useful when the file does not exist or the program does not have
permission to read it.

---

## Append to a file

By default, `std::ofstream` replaces the old file content.

Use `std::ios::app` to append to the end of the file:

```cpp
#include <fstream>

int main()
{
    std::ofstream file("log.txt", std::ios::app);

    file << "New log line\n";
}
```

Each time the program runs, it adds another line to `log.txt`.

---

## Read simple values

You can also read values using `>>`.

Assume `numbers.txt` contains:

```text
10 20
```

Read the two numbers:

```cpp
#include <fstream>
#include <iostream>

int main()
{
    std::ifstream file("numbers.txt");

    int a;
    int b;

    file >> a >> b;

    std::cout << "sum = " << a + b << "\n";
}
```

Output:

```text
sum = 30
```

Use `>>` for simple whitespace-separated values.

Use `std::getline()` when you want to read full text lines.

---

## Complete simple usage

This program writes a file, then reads it back.

```cpp
#include <fstream>
#include <iostream>
#include <string>

int main()
{
    {
        std::ofstream output("message.txt");
        output << "C++ file stream example\n";
    }

    {
        std::ifstream input("message.txt");
        std::string line;

        while (std::getline(input, line)) {
            std::cout << line << "\n";
        }
    }
}
```

The file streams close automatically when they go out of scope.

---

## Key points

- A stream moves data.
- `std::cout` writes to the terminal.
- `std::ofstream` writes to a file.
- `std::ifstream` reads from a file.
- Use `<<` to write data.
- Use `>>` to read simple values.
- Use `std::getline()` to read full lines.
- Check `if (!file)` before using a file.

---

## Exceptions and closing files

In C++, file streams close themselves automatically when the stream object is
destroyed.

This is called **RAII**:

- The file opens when the stream object is created.
- The file closes when the stream object goes out of scope.

Because of this, you usually do not need to call `file.close()` manually.

Example:

```cpp
#include <fstream>
#include <iostream>
#include <stdexcept>

int main()
{
    try {
        std::ofstream file("output.txt");

        if (!file) {
            throw std::runtime_error("Could not open file");
        }

        file << "Hello\n";

        throw std::runtime_error("Something went wrong");

        file << "This line will not run\n";
    } catch (const std::exception& error) {
        std::cout << "Error: " << error.what() << "\n";
    }
}
```

Even though an exception happens, `file` still closes automatically.

Why?

Because `file` is a local object inside the `try` block. When the exception is
thrown, C++ leaves the block and destroys local objects. The `std::ofstream`
destructor closes the file.

This is safer than manually calling `close()` at the end, because manual code may
be skipped when an exception occurs.

---

## Enable file stream exceptions

By default, file streams usually do not throw exceptions when an operation
fails. They set error flags instead.

You can tell the stream to throw exceptions:

```cpp
#include <fstream>
#include <iostream>

int main()
{
    try {
        std::ifstream file;

        file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        file.open("missing_file.txt");

        std::cout << "File opened\n";
    } catch (const std::ios_base::failure& error) {
        std::cout << "File error: " << error.what() << "\n";
    }
}
```

If `missing_file.txt` does not exist, `file.open()` throws an exception.

The file still closes automatically when the `file` object goes out of scope.

---

## Simple recommended style

For most beginner code, use this style:

```cpp
#include <fstream>
#include <iostream>
#include <string>

int main()
{
    try {
        std::ifstream file("data.txt");

        if (!file) {
            throw std::runtime_error("Could not open data.txt");
        }

        std::string line;

        while (std::getline(file, line)) {
            std::cout << line << "\n";
        }
    } catch (const std::exception& error) {
        std::cout << "Error: " << error.what() << "\n";
        return 1;
    }
}
```

Important points:

- Put the file stream inside a scope.
- Check if the file opened.
- Throw or return when opening fails.
- Let the stream close itself automatically.
- Avoid raw file handles when `std::ifstream` and `std::ofstream` are enough.
