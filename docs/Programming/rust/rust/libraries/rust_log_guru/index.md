---
title: Rust loguru
tags:
    - rust
    - log
---

```toml title="Cargo.toml"
[dependencies]
rust-loguru = "0.1"
```

```rust title="console handler with color"
use rust_loguru::handler::{console::ConsoleHandler, new_handler_ref};
use rust_loguru::{error, info, init, LogLevel, Logger};

fn main() {
    let mut logger = Logger::new(LogLevel::Info);

    let console_handler = ConsoleHandler::stdout(LogLevel::Info)
        .with_colors(true)
        .with_pattern("{timestamp} {level} [{location}] {message}");

    logger.add_handler(new_handler_ref(console_handler));
    init(logger);

    info!("Hello from Rust Loguru!");
    error!("This is an error message.");
}

```


### log with format

!!! warning "format override the color settings"
    Add color manual

    ```rust
    LogLevel::Info => format!("\x1b[32m{}\x1b[0m", record.level())
    ```
    
```rust
use rust_loguru::handler::{console::ConsoleHandler, new_handler_ref};
use rust_loguru::{error, info, init, LogLevel, Logger, Record};

fn main() {
    let mut logger = Logger::new(LogLevel::Info);

    let console_handler = ConsoleHandler::stdout(LogLevel::Info)
        .with_colors(false)
        .with_format(|record: &Record| {
            let level = match record.level() {
                LogLevel::Info => format!("\x1b[32m{}\x1b[0m", record.level()),
                LogLevel::Error => format!("\x1b[31m{}\x1b[0m", record.level()),
                _ => record.level().to_string(),
            };

            format!(
                "{} {:<5} [{}:{}] {}\n",
                record.timestamp().format("%Y-%m-%d %H:%M:%S%.3f"),
                level,
                record.file(),
                record.line(),
                record.message()
            )
        });

    logger.add_handler(new_handler_ref(console_handler));
    init(logger);

    info!("Hello from Rust Loguru!");
    error!("This is an error message.");
}
```