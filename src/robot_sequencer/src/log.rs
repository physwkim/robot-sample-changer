//! Timestamped stdout logging, matching the C++ node's operator-facing
//! format (`YYYY-MM-DD HH:MM:SS.mmm`, local time).

use chrono::Local;

pub fn timestamp() -> String {
    Local::now().format("%Y-%m-%d %H:%M:%S%.3f").to_string()
}

pub fn info(message: &str) {
    println!("[{}] {message}", timestamp());
}

pub fn warn(message: &str) {
    println!("[{}] WARN: {message}", timestamp());
}

pub fn error(message: &str) {
    eprintln!("[{}] ERROR: {message}", timestamp());
}
