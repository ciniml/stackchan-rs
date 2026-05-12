//! Stackchan firmware for the M5Stack family. Choose the board variant via Cargo features:
//!
//!   `--features core2`  (default) — M5Stack Core2, ESP32, AXP192 + ILI9341
//!   `--no-default-features --features cores3` — M5Stack CoreS3 / SE, ESP32-S3, AXP2101 +
//!                                               AW9523B + ILI9342
//!
//! Build target must match the chip:
//!   Core2  -> `--target xtensa-esp32-none-elf`
//!   CoreS3 -> `--target xtensa-esp32s3-none-elf`

#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_alloc as _;

#[cfg(all(feature = "core2", feature = "cores3"))]
compile_error!("Enable only one of `core2` or `cores3`.");

#[cfg(not(any(feature = "core2", feature = "cores3")))]
compile_error!("Enable one of `core2` or `cores3`.");

#[cfg(feature = "core2")]
mod core2;

#[cfg(feature = "cores3")]
mod cores3;
