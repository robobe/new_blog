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
