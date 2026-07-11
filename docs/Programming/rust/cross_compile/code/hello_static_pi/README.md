# hello_static_pi

Small Rust demo that builds a static ARM64 Linux binary for Raspberry Pi 5.

The Cargo config defaults builds to:

```text
aarch64-unknown-linux-musl
```

Build:

```bash
cargo build --release
```

Local checks:

```bash
file target/aarch64-unknown-linux-musl/release/hello_static_pi
ldd target/aarch64-unknown-linux-musl/release/hello_static_pi
```

Expected `ldd` output:

```text
not a dynamic executable
```

Copy to Pi:

```bash
scp target/aarch64-unknown-linux-musl/release/hello_static_pi pi5:/tmp/hello_static_pi
```

Run on Pi:

```bash
ssh pi5 /tmp/hello_static_pi
```
