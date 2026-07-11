---
title: C++ Destructor and Cleanup
tags:
    - cpp
    - oop
    - destructor
    - cleanup
    - lifetime
    - exception-safety
---

# C++ Destructor and Cleanup

**Series navigation:** Previous: [The Rule of Three / Rule of Five](rule_of_three_five) | Next: [RAII Resource Acquisition Is Initialization](raii)

A destructor runs when an object lifetime ends.

Use it to release resources owned by the object.

Examples:

- delete heap memory
- close a file
- close a socket
- unlock or release a handle
- free a C library resource

## Destructor syntax

The destructor has the class name with `~`.

It has no return type and no parameters.

```cpp
class Robot {
public:
    ~Robot()
    {
        std::cout << "destructor\n";
    }
};
```

## Scope cleanup

For stack objects, the destructor runs automatically when the scope ends.

```cpp
#include <iostream>

class Robot {
public:
    Robot()
    {
        std::cout << "constructor\n";
    }

    ~Robot()
    {
        std::cout << "destructor\n";
    }
};

int main()
{
    std::cout << "start\n";

    {
        Robot robot;
        std::cout << "inside scope\n";
    }

    std::cout << "end\n";
    return 0;
}
```

Expected output:

```text
start
constructor
inside scope
destructor
end
```

## Cleanup when exception happens

If an exception leaves a scope, C++ destroys stack objects before leaving that
scope.

This is called stack unwinding.

```cpp
#include <iostream>
#include <stdexcept>

class Resource {
public:
    Resource()
    {
        std::cout << "open resource\n";
    }

    ~Resource()
    {
        std::cout << "close resource\n";
    }
};

void run()
{
    Resource resource;
    std::cout << "before throw\n";
    throw std::runtime_error("something failed");
}

int main()
{
    try {
        run();
    } catch (const std::exception& error) {
        std::cout << "caught: " << error.what() << "\n";
    }

    return 0;
}
```

Expected output:

```text
open resource
before throw
close resource
caught: something failed
```

The important part:

```text
close resource
```

The destructor still ran, even though `run()` did not finish normally.

## Heap object cleanup

For raw heap objects, the destructor runs only when you call `delete`.

```cpp
Robot* robot = new Robot();
delete robot;
```

If you forget `delete`, the destructor does not run and the resource leaks.

Modern C++ usually avoids raw `new` and `delete`.

Prefer:

```cpp
auto robot = std::make_unique<Robot>();
```

`std::unique_ptr` deletes the object automatically when it goes out of scope.

## Destructors should not throw

A destructor should not throw exceptions.

Bad idea:

```cpp
~Resource()
{
    throw std::runtime_error("cleanup failed");
}
```

If another exception is already active and the destructor throws, the program can
terminate.

Use destructors for cleanup that must always happen.

If cleanup can fail, handle the error inside the destructor or provide an
explicit `close()` function that the caller can check.

## Full example

```cpp
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

class FileWriter {
public:
    FileWriter(const std::string& path)
        : file_(std::fopen(path.c_str(), "w"))
    {
        if (file_ == nullptr) {
            throw std::runtime_error("failed to open file");
        }

        std::cout << "file opened\n";
    }

    ~FileWriter()
    {
        if (file_ != nullptr) {
            std::fclose(file_);
            std::cout << "file closed\n";
        }
    }

    void write_line(const std::string& text)
    {
        std::fprintf(file_, "%s\n", text.c_str());
    }

    FileWriter(const FileWriter&) = delete;
    FileWriter& operator=(const FileWriter&) = delete;

private:
    std::FILE* file_;
};

void save()
{
    FileWriter writer("/tmp/destructor_demo.txt");
    writer.write_line("hello from destructor cleanup");

    throw std::runtime_error("error after writing");
}

int main()
{
    try {
        save();
    } catch (const std::exception& error) {
        std::cout << "caught: " << error.what() << "\n";
    }

    return 0;
}
```

Expected idea:

```text
file opened
file closed
caught: error after writing
```

The file is closed even though `save()` throws.

## Rule of thumb

- Put cleanup in the destructor.
- Stack objects are destroyed automatically at scope exit.
- Stack objects are also destroyed when an exception leaves the scope.
- Raw heap objects need `delete`.
- Prefer smart pointers and standard library RAII types.
- Do not throw from destructors.

## Run online

Copy the exception cleanup example and run it in the embedded OneCompiler C++
runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the full example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o destructor_demo
./destructor_demo
cat /tmp/destructor_demo.txt
```

**Series navigation:** Previous: [The Rule of Three / Rule of Five](rule_of_three_five) | Next: [RAII Resource Acquisition Is Initialization](raii)
