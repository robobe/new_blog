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
</div>

---

## Install and rustup

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

- **rustup**: toolchain manager
- **rustc**: compiler
- **cargo**: build & package manager

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

### Add target

```bash title="install rust target"
rustup target add aarch64-unknown-linux-gnu
```

```bash title="install cross compiler"
sudo apt update
sudo apt install gcc-arm-linux-gnueabihf
```

```init title="./.cargo/config.toml"
[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"
```

```bash title="build with aarch target"
cargo build --release --target aarch64-unknown-linux-gnu
```



---

## Reference
- [Let's Get Rusty](https://www.youtube.com/@letsgetrusty/playlists)