---
title: Develop a C++ Application with Sibling Repositories
tags:
    - cpp
    - projects
    - cmake
---

Sibling repositories keep an application and its libraries in separate Git
repositories under one development directory. Each project keeps its own
history and release cycle, while CMake builds the local checkouts together.

```text
~/projects/
├── my_app/
│   └── .git/
└── logging_lib/
    └── .git/
```

This layout is useful when a library is shared by several applications or must
be versioned independently. It also means every developer must check out the
required repositories before configuring the application.

## Minimal example

The example contains one application and one static library:

```text
code/
├── sibling-projects.code-workspace
├── my_app/
│   ├── CMakeLists.txt
│   └── src/main.cpp
└── logging_lib/
    ├── CMakeLists.txt
    ├── include/logging/log.hpp
    └── src/log.cpp
```

In a real development directory, `my_app` and `logging_lib` would each be a
separate Git repository. The example keeps them together only so they can be
downloaded from this page.

### The library

```cpp title="logging_lib/include/logging/log.hpp"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/logging_lib/include/logging/log.hpp"
```

```cpp title="logging_lib/src/log.cpp"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/logging_lib/src/log.cpp"
```

```cmake title="logging_lib/CMakeLists.txt"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/logging_lib/CMakeLists.txt"
```

`PUBLIC` makes the library's include directory available to targets that link
`logging_lib::logging_lib`.

### The application

```cpp title="my_app/src/main.cpp"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/my_app/src/main.cpp"
```

```cmake title="my_app/CMakeLists.txt"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/my_app/CMakeLists.txt"
```

`LOGGING_LIB_SOURCE` is a CMake cache path supplied by the developer.
`add_subdirectory()` adds that source tree to the same build, and its second
argument gives the library a build directory outside its source repository.

## Build and run

From the application repository, configure CMake with the path to the sibling
library:

```bash
cd ~/projects/my_app
cmake -S . -B build \
    -DLOGGING_LIB_SOURCE="$HOME/projects/logging_lib"
cmake --build build
./build/my_app
```

Expected output:

```text
[log] Hello from my_app
```

The path may be relative too. From the bundled example, use:

```bash
cd docs/Programming/cpp/dev_env/project_struct/code/my_app
cmake -S . -B build -DLOGGING_LIB_SOURCE=../logging_lib
cmake --build build
./build/my_app
```

`add_subdirectory()` only combines the projects for this CMake build. It does
not combine their Git histories: changes to the application and library are
still committed in their respective repositories.

---

## Use the sibling projects in VS Code

A [multi-root workspace](https://code.visualstudio.com/docs/editing/workspaces/multi-root-workspaces)
opens both repositories in one VS Code window. The workspace also passes the
library path to the
[CMake Tools](https://github.com/microsoft/vscode-cmake-tools) extension, so
the graphical configure action is equivalent to the command-line example.

```json title="sibling-projects.code-workspace"
--8<-- "docs/Programming/cpp/dev_env/project_struct/code/sibling-projects.code-workspace"
```

Place the workspace file beside the two repositories:

```text
~/projects/
├── sibling-projects.code-workspace
├── my_app/
└── logging_lib/
```

Then open it:

```bash
cd ~/projects
code sibling-projects.code-workspace
```

Install the recommended extensions when VS Code prompts, then run these
commands from the Command Palette:

1. **CMake: Select a Kit** — choose the compiler.
2. **CMake: Configure** — configure `my_app` and add `logging_lib` to its build.
3. **CMake: Build** — build the application and library.
4. **CMake: Run Without Debugging** — run `my_app`.

`${workspaceFolder:logging_lib}` resolves to the `logging_lib` folder named in
the workspace file. This avoids storing a developer-specific absolute path in
either Git repository.


---
