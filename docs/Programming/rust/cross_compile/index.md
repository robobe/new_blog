---
title: Rust static binary with musl
tags:
    - rust
    - cross-compile
    - musl
---

# Build a static Rust binary with musl

This example builds a static ARM64 Linux binary for Raspberry Pi 5 from an
x86_64 Linux development machine.

The target used here is:

```text
aarch64-unknown-linux-musl
```

Use this target when you want:

- ARM64 Linux output.
- A musl libc binary.
- A static executable that can run without target-side shared libraries.

## Step 1: install the musl Rust target

Check installed targets:

```bash
rustup target list --installed
```

Add the ARM64 musl target:

```bash
rustup target add aarch64-unknown-linux-musl
```

For a local x86_64 static Linux binary, use:

```bash
rustup target add x86_64-unknown-linux-musl
```

## Step 2: create the Rust project

```bash
cargo new hello_static_pi --bin
cd hello_static_pi
mkdir -p .cargo
```



## Step 3: source files

<details>
<summary>src/main.rs</summary>

```rust
use std::env;
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock should be after Unix epoch")
        .as_secs();

    println!("hello from a static Rust binary");
    println!("package: {}", env!("CARGO_PKG_NAME"));
    println!("version: {}", env!("CARGO_PKG_VERSION"));
    println!("target arch: {}", env::consts::ARCH);
    println!("target os: {}", env::consts::OS);
    println!("unix time: {now}");
}
```

</details>

<details>
<summary>Cargo.toml</summary>

```toml
[package]
name = "hello_static_pi"
version = "0.1.0"
edition = "2024"

[dependencies]
```

</details>

<details>
<summary>.cargo/config.toml</summary>

```toml
[build]
target = "aarch64-unknown-linux-musl"

[target.aarch64-unknown-linux-musl]
linker = "rust-lld"
rustflags = ["-C", "target-feature=+crt-static"]
```

</details>

## Step 4: understand the configuration

`Cargo.toml` describes the Rust package:

- package name
- version
- Rust edition
- dependencies

`.cargo/config.toml` controls how Cargo builds the project.

```toml
[build]
target = "aarch64-unknown-linux-musl"
```

This makes `cargo build` use `aarch64-unknown-linux-musl` by default.

```toml
[target.aarch64-unknown-linux-musl]
linker = "rust-lld"
```

This tells Cargo to link the ARM64 musl binary with Rust's bundled LLVM linker.
For a pure Rust application, this avoids needing a separate ARM64 musl GCC
toolchain.

```toml
rustflags = ["-C", "target-feature=+crt-static"]
```

This passes the static C runtime flag to `rustc`.

For musl targets this is the important static binary flag:

```bash
-C target-feature=+crt-static
```

## Step 5: build the application

Because `.cargo/config.toml` sets the default target, this is enough:

```bash
cargo build --release
```

The binary is created here:

```bash
target/aarch64-unknown-linux-musl/release/hello_static_pi
```

You can also build with a specific target from the command line:

```bash
cargo build --release --target aarch64-unknown-linux-musl
```

Command-line `--target` is useful when the project does not define a default
target in `.cargo/config.toml`, or when you want to override the default target.

## Step 6: check the binary

Use `file`:

```bash
file target/aarch64-unknown-linux-musl/release/hello_static_pi
```

Expected result:

```text
ELF 64-bit LSB executable, ARM aarch64, statically linked
```

Use `ldd`:

```bash
ldd target/aarch64-unknown-linux-musl/release/hello_static_pi
```

Expected result:

```text
not a dynamic executable
```

## Step 7: run the ARM64 binary with QEMU

Install QEMU user emulation:

```bash
sudo apt update
sudo apt install qemu-user-static
```

Run the ARM64 binary on the development machine:

```bash
qemu-aarch64-static \
target/aarch64-unknown-linux-musl/release/hello_static_pi
```

Expected output:

```text
hello from a static Rust binary
package: hello_static_pi
version: 0.1.0
target arch: aarch64
target os: linux
```






