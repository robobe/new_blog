---
title: Linux Toolchains
tags:
    - embedded
    - linux
    - compiler
    - cross-compile
    - glibc
    - musl
    - static-linking
---

# Linux Toolchains



Short notes about native compilers, cross compilers, libc, dynamic linking, and static linking.

## Terms

| term | meaning |
|---|---|
| Compiler | Translate source code to object files or a binary. |
| Linker | Connect object files with libraries and create the final executable. |
| Host compiler | Compiler that builds a binary for the same machine you are using. |
| Cross compiler | Compiler that runs on one machine but builds a binary for another target. |
| libc | The C runtime library used by Linux programs, for example glibc or musl. |
| Dynamic binary | Loads shared libraries at runtime. |
| Static binary | Copies the needed libraries into the executable at link time. |

## Compile and link

Compile only:

```bash
g++ -c code/hello.cpp -o hello.o
```

Link:

```bash
g++ hello.o -o hello
```

Compile and link in one command:

```bash
g++ code/hello.cpp -o hello
```

`g++` calls the compiler and then the linker. For C++ it also links the C++ standard library.

## Why linker is used when the binary runs

For a dynamic binary, linking is not fully finished when you build the program.

The binary contains:

- the program code
- the shared libraries it needs
- the runtime loader path, for example `/lib64/ld-linux-x86-64.so.2`

When you run the binary, Linux starts the loader first. The loader finds the shared libraries, maps them into memory, resolves symbols, and then starts `main()`.

Check it:

```bash
readelf -l hello | grep interpreter
ldd hello
```

For a static binary, there is no runtime loader dependency for normal libc libraries:

```bash
file hello-static
ldd hello-static
```

## glibc vs musl

| libc | common use |
|---|---|
| glibc | Default libc on most Debian, Ubuntu, Fedora, and many desktop/server Linux systems. |
| musl | Small libc used by Alpine Linux and useful for static embedded deployment. |

glibc is common and very compatible with normal Linux distributions.

musl is designed to be small, simple, and friendly to static linking. It is useful when you want one binary that is easy to copy to an embedded target.

## Static compiler

"Static compiler" usually means a compiler toolchain or build command that creates a statically linked binary.

It is not a different C++ language. The main difference is the link step:

```bash
g++ code/hello.cpp -o hello-dynamic
g++ code/hello.cpp -static -o hello-static
```

## Install glibc host tools

Install native build tools on Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential file binutils
```

Build with the host glibc compiler:

```bash
g++ code/hello.cpp -o hello-glibc-dynamic
g++ code/hello.cpp -static -o hello-glibc-static
file hello-glibc-dynamic hello-glibc-static
```

## Install glibc cross compiler

For AArch64 target:

```bash
sudo apt update
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

Verify:

```bash
which aarch64-linux-gnu-g++
aarch64-linux-gnu-g++ --version
```

Build AArch64 binaries with glibc:

```bash
aarch64-linux-gnu-g++ code/hello.cpp -o hello-aarch64-glibc-dynamic
aarch64-linux-gnu-g++ code/hello.cpp -static -o hello-aarch64-glibc-static
file hello-aarch64-glibc-dynamic hello-aarch64-glibc-static
```

Useful package names for other targets:

| target | packages |
|---|---|
| AArch64 | `gcc-aarch64-linux-gnu g++-aarch64-linux-gnu` |
| ARM hard-float | `gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf` |
| ARM soft-float | `gcc-arm-linux-gnueabi g++-arm-linux-gnueabi` |
| x86_64 | `gcc-x86-64-linux-gnu g++-x86-64-linux-gnu` |

## Install musl cross compiler from musl.cc

[musl.cc](https://musl.cc/) provides prebuilt musl toolchains. It is a community source, not the official musl project.

Choose the archive by the target CPU, not by the host CPU.

For an x86_64 Ubuntu host building for AArch64 Linux, use:

```text
aarch64-linux-musl-cross.tgz
```

Do not use this unless the target is x86_64 Linux with musl:

```text
x86_64-linux-musl-cross.tgz
```

Download and install in `/opt`:

```bash
cd /tmp
wget https://musl.cc/aarch64-linux-musl-cross.tgz
tar -xf aarch64-linux-musl-cross.tgz
sudo mv aarch64-linux-musl-cross /opt/
```

Add it to the shell path:

```bash
echo 'export PATH=/opt/aarch64-linux-musl-cross/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
which aarch64-linux-musl-g++
aarch64-linux-musl-g++ --version
```

## Build with musl cross compiler

Dynamic musl build:

```bash
aarch64-linux-musl-g++ code/hello.cpp -o hello-aarch64-musl-dynamic
file hello-aarch64-musl-dynamic
```

Static musl build:

```bash
aarch64-linux-musl-g++ -static code/hello.cpp -o hello-aarch64-musl-static
file hello-aarch64-musl-static
```

Static musl binaries are useful for embedded targets because they do not require the target root filesystem to provide matching shared libc files.

## Test AArch64 binary on x86_64 host

Install QEMU user emulation:

```bash
sudo apt update
sudo apt install qemu-user
```

Run the static binary:

```bash
qemu-aarch64 ./hello-aarch64-musl-static
```

Expected output:

```text
hello toolchain
```
