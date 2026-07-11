---
title: Build mavlink-router as Static AArch64 musl Binary
tags:
    - mavlink
    - mavlink-router
    - cross-compile
    - musl
    - aarch64
    - meson
    - static-linking
---

# Build mavlink-router as Static AArch64 musl Binary

`mavlink-router` forwards MAVLink packets between UART, UDP, and TCP endpoints.
It is useful on a companion computer or embedded Linux board when one MAVLink
source should be shared by several clients.

- <a href="https://github.com/mavlink-router/mavlink-router" target="_blank" rel="noopener noreferrer">mavlink-router GitHub</a>
- <a href="https://mavlink-router.github.io/" target="_blank" rel="noopener noreferrer">mavlink-router GitHub Pages</a>

This guide builds `mavlink-routerd` as a static AArch64 Linux binary using the
musl cross compiler from `musl.cc`.

Read more on linux toolchain and cross compiler [linux cross compiler toolchain](docs/Embedded/linux_toolchains)

## Step 1: install host tools

Install the tools used to download and build the source:

```bash
sudo apt update
sudo apt install git meson ninja-build pkg-config file wget
```

- `git`: download `mavlink-router`.
- `meson`: configure the build.
- `ninja`: run the build.
- `pkg-config`: helper used by many C/C++ build systems.
- `file`: check the output binary type.
- `wget`: download the musl cross compiler archive.

## Step 2: install aarch64 musl cross compiler

Download the prebuilt musl.cc toolchain:

```bash
cd /tmp
wget https://musl.cc/aarch64-linux-musl-cross.tgz
tar -xf aarch64-linux-musl-cross.tgz
sudo mv aarch64-linux-musl-cross /opt/
```

!!! tip target cpu
    The archive name is selected by the target CPU, not by the host CPU.

    For an x86_64 Ubuntu host that builds for aarch64 Linux, use:

    ```text
    aarch64-linux-musl-cross.tgz
    ```



Add the compiler to your shell path:

```bash
echo 'export PATH=/opt/aarch64-linux-musl-cross/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

## Step 3: verify the compiler

Check that the C and C++ compilers are available:

```bash
/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-gcc --version
/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-g++ --version
```

These tools run on the host machine and create AArch64 Linux binaries linked
against musl.

## Step 4: download mavlink-router

From your workspace root:

```bash
mkdir -p external
git clone --depth 1 https://github.com/mavlink-router/mavlink-router.git external/mavlink-router
git -C external/mavlink-router submodule update --init --recursive --depth 1
```

The submodule step is required because `mavlink-router` uses the MAVLink C
library as a dependency.



## Step 5: record the source revision

Record the exact source version used for the build:

!!! tip ""
    `git -C <path> ...` means: run the Git command as if you first changed directory to <path>.

    **Example**:

    ```bash
    git -C <path> status
    ```

    is equivalent to:

    ```bash
    cd <path>
    git status
    ```

```bash
git -C external/mavlink-router rev-parse HEAD
git -C external/mavlink-router submodule status
```

This makes the build easier to reproduce later.

## Step 6: create the Meson cross file

Create the toolchain folder:

```bash
mkdir -p toolchains
```

Create `toolchains/aarch64-linux-musl.ini`:

```ini
[binaries]
c = '/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-gcc'
cpp = '/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-g++'
ar = '/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-ar'
strip = '/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-strip'

[host_machine]
system = 'linux'
cpu_family = 'aarch64'
cpu = 'aarch64'
endian = 'little'

[built-in options]
c_args = ['-Os']
cpp_args = ['-Os']
c_link_args = ['-static', '-no-pie']
cpp_link_args = ['-static', '-no-pie']
default_library = 'static'
b_lto = true
b_pie = false
```

Important static build settings:

- `default_library = 'static'`: prefer static libraries.
- `c_link_args = ['-static', '-no-pie']`: statically link C code.
- `cpp_link_args = ['-static', '-no-pie']`: statically link C++ code.
- `b_pie = false`: avoid position-independent executable mode for this static build.

## Step 7: configure with Meson

Remove any old build folder:

```bash
rm -rf build/mavlink-router-aarch64-musl-static
```

Configure the cross build:

```bash
meson setup \
  build/mavlink-router-aarch64-musl-static \
  external/mavlink-router \
  --cross-file toolchains/aarch64-linux-musl.ini \
  --buildtype release \
  --strip \
  -Dsystemdsystemunitdir=/usr/lib/systemd/system
```

What this command does:

- `build/mavlink-router-aarch64-musl-static`: output build directory.
- `external/mavlink-router`: source directory.
- `--cross-file`: tells Meson which compiler and target machine to use.
- `--buildtype release`: optimized release build.
- `--strip`: strip symbols during install/build output where supported.
- `-Dsystemdsystemunitdir=...`: sets the systemd unit install path.

## Step 8: build with Ninja

Build the project:

```bash
ninja -C build/mavlink-router-aarch64-musl-static
```

The main binary is:

```text
build/mavlink-router-aarch64-musl-static/src/mavlink-routerd
```

Strip the final binary:

```bash
/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-strip \
  build/mavlink-router-aarch64-musl-static/src/mavlink-routerd
```

## Step 9: verify the binary

Check that the output is an **aarch64** executable:

```bash
file build/mavlink-router-aarch64-musl-static/src/mavlink-routerd
```

Expected output should include:

```text
ELF 64-bit LSB executable, ARM aarch64, statically linked
```

Check the ELF header:

```bash
/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-readelf \
  -h build/mavlink-router-aarch64-musl-static/src/mavlink-routerd
```

Check that there is no dynamic interpreter:

```bash
/opt/aarch64-linux-musl-cross/bin/aarch64-linux-musl-readelf \
  -l build/mavlink-router-aarch64-musl-static/src/mavlink-routerd | grep INTERP || echo "no dynamic interpreter"
```

Expected interpreter check:

```text
no dynamic interpreter
```

## Step 10: test with QEMU

Install QEMU user emulation:

```bash
sudo apt install qemu-user
```

Run the help command:

```bash
QEMU_AARCH64="$(command -v qemu-aarch64-static || command -v qemu-aarch64)"
"$QEMU_AARCH64" build/mavlink-router-aarch64-musl-static/src/mavlink-routerd -h
```

The command should print the `mavlink-routerd` usage text.

## Step 11: copy to Raspberry Pi

Copy the static binary to a Raspberry Pi target:

```bash
scp build/mavlink-router-aarch64-musl-static/src/mavlink-routerd pi5:/tmp/mavlink-routerd
```

Check it on the Raspberry Pi:

```bash
ssh pi5 'chmod +x /tmp/mavlink-routerd && file /tmp/mavlink-routerd && /tmp/mavlink-routerd -h | head -40'
```

If `file` is not installed on the Raspberry Pi, run only:

```bash
ssh pi5 'chmod +x /tmp/mavlink-routerd && /tmp/mavlink-routerd -h | head -40'
```

---

## Download pre build

- [mavlink-router musl aarch static build](code/mavlink-routerd)