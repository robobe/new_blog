---
title: Command-Line Arguments with CLI11
tags:
    - cpp
    - cli11
    - command-line
    - cmake
---

CLI11 is a header-only C++ library for defining command-line options, parsing
arguments, converting values into C++ types, validating input, and generating
help output. This minimal C++17 example adds a `--name` option to a greeting
program.

## Install on Ubuntu 24.04

```bash
sudo apt update
sudo apt install build-essential cmake libcli11-dev
```

Ubuntu 24.04 provides CLI11 `2.4.1`. Confirm the installed package with:

```bash
dpkg-query -W libcli11-dev
```

Because CLI11 is header-only, the package primarily provides headers and CMake
metadata rather than a runtime library.

---

## Minimal example

[Download `main.cpp`](code/main.cpp).

```cpp title="main.cpp"
--8<-- "docs/Programming/cpp/libraries/cli11/code/main.cpp"
```

Create the command-line application and its description:

```cpp
CLI::App app{"A minimal CLI11 hello-world example"};
```

Define a normal C++ variable and bind an option to it:

```cpp
std::string name = "world";
app.add_option("-n,--name", name, "Name to greet");
```

The option has two equivalent names:

- `-n` is the short form;
- `--name` is the readable long form.

If the user supplies the option, CLI11 stores its value in `name`. Otherwise,
the original value `"world"` remains unchanged.

```cpp
CLI11_PARSE(app, argc, argv);
```

`CLI11_PARSE` parses the arguments and handles ordinary parsing failures, such
as an unknown option or a missing value. It also enables automatically generated
`--help` output.

!!! tip "Define options before parsing"
    Register every option and flag on `app` before calling `CLI11_PARSE`. Read
    the bound variables only after parsing succeeds.

---

## CMake configuration

[Download `CMakeLists.txt`](code/CMakeLists.txt).

```cmake title="CMakeLists.txt"
--8<-- "docs/Programming/cpp/libraries/cli11/code/CMakeLists.txt"
```

`find_package` locates the Ubuntu development package. Link the executable to
the imported header-only target:

```cmake
find_package(CLI11 CONFIG REQUIRED)
target_link_libraries(cli11_hello PRIVATE CLI11::CLI11)
```

The target provides CLI11's include directory and usage requirements.

---

## Build and run

Download `main.cpp` and `CMakeLists.txt` into one directory, then run:

```bash
cmake -S . -B build
cmake --build build
```

Run without an option to use the default value:

```bash
./build/cli11_hello
```

```text
Hello, world!
```

Use either option name to provide another value:

```bash
./build/cli11_hello --name Alice
./build/cli11_hello -n Bob
```

```text
Hello, Alice!
Hello, Bob!
```

Display the generated help page:

```bash
./build/cli11_hello --help
```

```text
A minimal CLI11 hello-world example
Usage: ./build/cli11_hello [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -n,--name TEXT              Name to greet
```

CLI11 uses the bound variable's C++ type to convert input. For example, binding
an option to an `int` asks CLI11 to parse an integer and report invalid text to
the user.

---

Demo:

```cpp
#include <CLI/CLI.hpp>

  #include <iostream>
  #include <string>

  struct Options {
      std::string name = "world";
      int count = 1;
      bool verbose = false;
  };

  int main(int argc, char* argv[])
  {
      CLI::App app{"Bind CLI11 options to a struct"};

      Options options;

      app.add_option("-n,--name", options.name, "Name to greet")
          ->capture_default_str();

      app.add_option("-c,--count", options.count, "Number of greetings")
          ->check(CLI::Range(1, 10))
          ->capture_default_str();

      app.add_flag("-v,--verbose", options.verbose, "Enable verbose output");

      CLI11_PARSE(app, argc, argv);

      if (options.verbose) {
          std::cout << "Generating " << options.count << " greetings\n";
      }

      for (int index = 0; index < options.count; ++index) {
          std::cout << "Hello, " << options.name << "!\n";
      }
  }
```

---

## Common CLI11 features

The following examples focus on one feature at a time. Add the options before
`CLI11_PARSE`, then use their bound variables after parsing succeeds.

### 1. Flags

A flag represents an on/off choice and does not consume a value:

```cpp
bool verbose = false;
app.add_flag("-v,--verbose", verbose, "Enable verbose output");

CLI11_PARSE(app, argc, argv);
if (verbose) {
    std::cout << "Verbose output enabled\n";
}
```

Enable it by including its name:

```bash
./app --verbose
```

The bound Boolean remains `false` when the flag is absent and becomes `true`
when it is present.

### 2. Required options

Call `required()` when the application cannot run without an option:

```cpp
std::string config_path;
app.add_option("-c,--config", config_path, "Configuration file")
    ->required();
```

```bash
./app --config config.yaml
```

CLI11 prints an error and usage information when `--config` is missing.

!!! warning "Required means the user must provide it"
    Giving the bound variable a C++ default value does not satisfy
    `required()`. Use a required option only when silently choosing a default
    would be incorrect.

### 3. Default-value display

`capture_default_str()` records the variable's current value and displays it in
the generated help:

```cpp
int port = 8080;
app.add_option("-p,--port", port, "Server port")
    ->capture_default_str();
```

The help output then includes the default:

```text
-p,--port INT [8080]          Server port
```

Call `capture_default_str()` after assigning the intended C++ default.

### 4. Validators

Validators reject values that have the correct type but are not acceptable for
the application:

```cpp
int port = 8080;
app.add_option("-p,--port", port, "Server port")
    ->check(CLI::Range(1, 65535))
    ->capture_default_str();
```

```bash
./app --port 70000
```

This fails because `70000` is outside the permitted range. Useful built-in
validators include:

```cpp
CLI::ExistingFile
CLI::ExistingDirectory
CLI::ExistingPath
CLI::NonexistentPath
CLI::PositiveNumber
CLI::IsMember({"trace", "debug", "info", "warn", "error"})
```

Type conversion happens as well: an option bound to `int` rejects text that is
not an integer.

### 5. Positional arguments

An option name without `-` or `--` creates a positional argument:

```cpp
std::string input_path;
app.add_option("input", input_path, "Input file")
    ->required()
    ->check(CLI::ExistingFile);
```

The value is written without an option name:

```bash
./app image.png
```

Positional arguments are convenient for the primary input or output. Named
options are usually clearer for optional settings.

### 6. Vectors and repeated options

Bind an option to a vector when it may receive several values:

```cpp
std::vector<std::string> inputs;
app.add_option("-i,--input", inputs, "Input files")
    ->expected(1, -1);
```

The user can provide several values together or repeat the option:

```bash
./app --input first.txt second.txt third.txt
./app --input first.txt --input second.txt
```

After parsing, `inputs` contains every supplied filename. Here, `expected(1,
-1)` means at least one value and no fixed upper limit.

### 7. Environment variables

`envname()` supplies an option from an environment variable when it was not
provided on the command line:

```cpp
std::string log_level = "info";
app.add_option("--log-level", log_level, "Logging level")
    ->envname("APP_LOG_LEVEL")
    ->check(CLI::IsMember({"debug", "info", "warn", "error"}))
    ->capture_default_str();
```

```bash
APP_LOG_LEVEL=debug ./app
```

The effective precedence is:

```text
C++ default < environment variable < command-line option
```

Therefore, `./app --log-level warn` overrides `APP_LOG_LEVEL=debug`.

### 8. Dependencies and exclusions

`needs()` requires another option when the first option is used:

```cpp
std::string username;
std::string token;

auto* username_option = app.add_option("--username", username);
auto* token_option = app.add_option("--token", token);
token_option->needs(username_option);
```

`--token secret` now requires `--username Alice` as well.

`excludes()` prevents incompatible options from appearing together:

```cpp
bool json = false;
bool yaml = false;

auto* json_option = app.add_flag("--json", json, "Write JSON");
auto* yaml_option = app.add_flag("--yaml", yaml, "Write YAML");
json_option->excludes(yaml_option);
```

CLI11 rejects `./app --json --yaml`. One exclusion declaration is sufficient
for this pair.

### 9. Subcommands

Subcommands divide one executable into separate operations, similar to
`git clone`, `git status`, and `git log`. Each subcommand can have its own
description, options, positional arguments, and help page.

```cpp
CLI::App app{"Service controller"};

auto* start = app.add_subcommand("start", "Start the service");
int port = 8080;
start->add_option("--port", port, "Listening port")
    ->check(CLI::Range(1, 65535))
    ->capture_default_str();

auto* stop = app.add_subcommand("stop", "Stop the service");
bool force = false;
stop->add_flag("-f,--force", force, "Force shutdown");

app.require_subcommand(1);
CLI11_PARSE(app, argc, argv);

if (*start) {
    std::cout << "Starting on port " << port << '\n';
} else if (*stop) {
    std::cout << (force ? "Forcing shutdown\n" : "Stopping\n");
}
```

Usage follows the structure of the program:

```bash
./app start --port 9000
./app stop
./app stop --force
```

`app.require_subcommand(1)` requires exactly one subcommand. Remove it when the
program has useful behavior without a subcommand.

The pointer returned by `add_subcommand` behaves as a Boolean after parsing:

```cpp
if (*start) {
    // The user selected "start".
}
```

Options added to `start` are local to `start`; they normally appear after the
subcommand name. Options added directly to the parent `app` are global. Help is
available at both levels:

```bash
./app --help
./app start --help
```

The parent help lists available subcommands, while the second command shows the
options specific to `start`. Subcommands may also have aliases and nested
subcommands, but one level is easier to understand and is sufficient for most
small tools.

!!! tip "Keep dispatch after parsing"
    For a simple application, parse once and dispatch with `if (*subcommand)`.
    Callbacks are useful later, but explicit dispatch makes control flow and
    shared initialization easier to see.

---

## Reference

- [CLI11 project](https://github.com/CLIUtils/CLI11){:target="_blank" rel="noopener noreferrer"}
- [CLI11 documentation](https://cliutils.github.io/CLI11/book/){:target="_blank" rel="noopener noreferrer"}

<!-- post-content-skill: 1.0.0 -->
