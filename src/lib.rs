//! Communication Device Class Network Control Model (CDC-NCM) class for [usb-device](https://crates.io/crates/usb-device).

#![no_std]
#![warn(clippy::pedantic)]
#![warn(clippy::style)]
#![warn(clippy::cargo)]
#![allow(clippy::multiple_crate_versions)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![warn(clippy::use_self)]

mod buffer;
mod bytes;
mod ethernet;

pub use ethernet::*;
