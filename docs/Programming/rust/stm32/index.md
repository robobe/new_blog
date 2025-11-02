---
tags:
    - rust
    - stm32
---


{{ page_folder_links() }}

```
rustup target list
```

![alt text](images/rust_target_list_mcu.png)


```
rustup target add thumbv7em-none-eabihf
```
- thumbv7em: ARM Cortex-M Thumb instruction set
- none: Without OS
- enbihf: Embedded Application Binary Interface with hardware floating point support

```
cargo build --target thumbv7em-none-eabihf
```

```
mkdir .cargo
touch config.toml
```

```toml
[build]
target = "thumbv7em-none-eabihf"
```

---

# Reference
- [Udemy](https://www.udemy.com/course/embedded-rust-for-absolute-beginners/?couponCode=KEEPLEARNING)
- [Rust on an STM32 microcontroller](https://medium.com/digitalfrontiers/rust-on-a-stm32-microcontroller-90fac16f6342)
- [ Rust Embedded for STM32 Microcontrollers: Intro ](https://youtu.be/o_alVYMBBco)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/intro/index.html)
- [A Foray Into Embedded Rust ](https://joeyh.dev/blog/rust_arduino/)