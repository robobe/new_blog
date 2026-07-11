---
title: Rust programming
tags:
  - rust
---

{{ page_folder_links() }}

<div class="grid-container">
    <div class="grid-item">
      <a href="dev_env">
        <img src="images/vscode.png" width="150" height="150">
        <p>VSCode dev env.</p>
      </a>
    </div>
    <div class="grid-item">
      <a href="rust">
        <img src="../images/rust.png" width="150" height="150">
        <p>Rust</p>
      </a>
    </div>
</div>

---

## Install and rustup

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

- **rustup**: toolchain manager
- **rustc**: compiler
- **cargo**: build & package manager
- **crate**: smallest compilation unit in Rust (lib/bin)
- **crates.io**: rust's official package registry

```bash title="load rust into shell"
source ~/.cargo/env
```

## rust toolchain
A toolchain is a complete Rust environment consisting:
- rustc
- cargo
- Rust standard library
- optional components (clippy, rustfmt)

### rustup
rustup manages Rust versions, targets, and components on your system.

!!! tip "autocomplete"
  Add rustup autocomplete

  ```bash
  source <(rustup completions bash)
  # add to bashrc
  echo 'source <(rustup completions bash)' >> ~/.bashrc
  ```
    
#### update
update and upgrade toolchain and components

```bash
rustup update
# its like apt update + apt upgrade
```

#### Components
A component is an optional tool that plugs into a Rust toolchain.

- rustfmt: Formatting
- clippy: Linting


```bash
# add to active toolchain
rustup component add rustfmt
# for specific toolchain
rustup component add rustfmt --toolchain nightly
```

---

## Hello world
Simple `hello world` build and run
- Add `aarch` target and build it for ARM

```bash
cargo new hello_world
```

```bash
cargo run        # build + run
cargo build      # build only
cargo check      # fast compile check
cargo fmt        # format code
cargo clippy     # lint code
```

### Add target (cross compile)

Cross compilation builds a binary for a different CPU or OS than the machine
running `cargo build`.

Rust needs two things:

- **Rust target**: standard library for the target platform.
- **Linker / C toolchain**: only needed when the crate links C code, system
  libraries, or uses a target that needs an external linker.

Target name format:

```text
<cpu>-<vendor>-<os>-<abi>
```

Common Linux targets:

| Target | Use case | libc | Static friendly |
|---|---|---|---|
| `x86_64-unknown-linux-gnu` | normal Linux x86_64 | glibc | partial |
| `aarch64-unknown-linux-gnu` | 64-bit ARM Linux | glibc | partial |
| `x86_64-unknown-linux-musl` | portable Linux x86_64 | musl | yes |
| `aarch64-unknown-linux-musl` | portable 64-bit ARM Linux | musl | yes |

#### GNU target example: build for ARM64 Linux

GNU targets use glibc. The binary usually depends on shared libraries from the
target system.

```bash title="install rust target"
rustup target add aarch64-unknown-linux-gnu
```

```bash title="install cross compiler"
sudo apt update
sudo apt install gcc-aarch64-linux-gnu
```

```init title="./.cargo/config.toml"
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
```

```bash title="build with aarch target"
cargo build --release --target aarch64-unknown-linux-gnu
```

The output binary will be here:

```bash
target/aarch64-unknown-linux-gnu/release/hello_world
```

Check the binary:

```bash
file target/aarch64-unknown-linux-gnu/release/hello_world
```

#### musl target: create static Linux binary

`musl` is an alternative C standard library.

Why use musl:

- Produces mostly or fully static Linux binaries.
- Easier to copy one executable to another Linux machine.
- Good for containers, small deployments, and servers without matching glibc.

Install musl tools:

```bash title="install musl compiler"
sudo apt update
sudo apt install musl-tools
```

Add the Rust musl target:

```bash title="x86_64 static binary target"
rustup target add x86_64-unknown-linux-musl
```

Build:

```bash title="build static binary"
cargo build --release --target x86_64-unknown-linux-musl
```

Output:

```bash
target/x86_64-unknown-linux-musl/release/hello_world
```

Check if the binary is static:

```bash
file target/x86_64-unknown-linux-musl/release/hello_world
ldd target/x86_64-unknown-linux-musl/release/hello_world
```

For a static binary, `ldd` usually prints:

```text
not a dynamic executable
```

#### ARM64 static binary with musl

For ARM64 Linux:

```bash
rustup target add aarch64-unknown-linux-musl
```

On Ubuntu/Debian, the native `musl-tools` package usually gives a musl compiler
for the host architecture. For cross-compiling musl to ARM64, the easiest
practical option is often `cross`, which builds inside a prepared container.

Install `cross`:

```bash
cargo install cross
```

Build:

```bash
cross build --release --target aarch64-unknown-linux-musl
```

Output:

```bash
target/aarch64-unknown-linux-musl/release/hello_world
```


#### Strip and reduce binary size

Strip debug symbols:

```bash
strip target/x86_64-unknown-linux-musl/release/hello_world
```

Or configure release builds in `Cargo.toml`:

```toml title="Cargo.toml"
[profile.release]
strip = true
lto = true
codegen-units = 1
panic = "abort"
```

#### Important notes

- Pure Rust crates are easiest to cross compile.
- Crates using C libraries may need matching target libraries and headers.
- `musl-tools` is the musl compiler package on Ubuntu/Debian.
- `cross` is usually simpler when building for another CPU architecture.
- Use `file` and `ldd` to verify what kind of binary was produced.


---

## Reference
- [Let's Get Rusty](https://www.youtube.com/@letsgetrusty/playlists)
- [Learn rust](https://www.youtube.com/@smartcontractprogrammer)
- [brown rust programming book](https://rust-book.cs.brown.edu/)
