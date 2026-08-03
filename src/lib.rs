//! ioCan — general-purpose I/O board firmware for the vehicle CAN bus.
//!
//! Four high current outputs and two I2C sensor buses, on an STM32F105RC. One node on this bus is
//! the master and makes every decision; this board executes, reports, and looks after itself when
//! the master goes quiet.
//!
//! # Shape of the firmware
//!
//! ```text
//!   CAN  --> can::sdo    ---\                      /--- control  --> high current outputs
//!                            >--  store (the OD) --<
//!   CAN  <-- can::tpdo   ---/                      \--- sensors  <-- I2C amplifier boards
//!                                     |
//!                             config::persist  --> external NOR flash
//! ```
//!
//! [`store`] is the only shared state, and it *is* the object dictionary described in
//! `device-conf/can-io.toml`. Tasks communicate by reading and writing it, never directly.
//! [`control`] is the only task that touches an output, which is what makes the valve model's
//! measured position mean anything.
//!
//! # Where the raw PAC is still used, and why
//!
//! Everything reachable through embassy goes through embassy. Three places do not:
//!
//! - [`panic::safe_outputs`] writes timer and GPIO registers directly. It runs from a panic or a
//!   HardFault, where the `HcoControl` borrow may be mid-mutation and taking a lock is not an
//!   option, but the outputs still have to be de-energised before the reset.
//! - [`board::pet_watchdog`] kicks the IWDG by register during the long blocking flash operations
//!   in the cancan confirm path, where the watchdog task cannot be scheduled.
//! - The rev2 high current output controller software-PWMs HCO1 and HCO2 from a TIM2 interrupt,
//!   because on that revision those pins are plain GPIO with no timer channel behind them. rev3
//!   fixed the routing and uses `SimplePwm` throughout.
//!
//! `unstable-pac` is also enabled for one read of `DBGMCU.idcode()`, which cancan reports as the
//! chip identity, and for the bxCAN receive FIFO overrun check in [`can`].

//! # Testing
//!
//! The `hardware` feature (on by default) gates everything that only compiles for the target: the
//! HAL, the executor, the drivers. Everything else — the object dictionary, the valve model, the
//! fallback state machine, sensor calibration, the identifier layout — is plain logic over plain
//! data and runs on the host:
//!
//! ```sh
//! cargo test --no-default-features --target x86_64-unknown-linux-gnu
//! ```
//!
//! That split is why the interesting decisions live in pure functions taking `&Config` and an
//! `Instant` rather than reaching for hardware directly.

#![cfg_attr(not(test), no_std)]

pub mod config;
pub mod hco;
pub mod index;
pub mod leds;
pub mod rail_sense;
pub mod relief;
pub mod safety;
pub mod store;
pub mod valves;
pub mod zenith_mapping;

pub mod can;
pub mod sensors;

#[cfg(feature = "hardware")]
pub mod board;
#[cfg(any(feature = "hardware", test))]
pub mod control;
#[cfg(feature = "hardware")]
pub mod node;
#[cfg(any(feature = "hardware", test))]
pub mod outputs;
#[cfg(feature = "hardware")]
pub mod panic;
#[cfg(feature = "hardware")]
pub mod utils;

#[cfg(feature = "hardware")]
mod firmware {
    use cancan::CanCanChannels;
    use defmt_rtt as _;
    use embassy_executor::InterruptExecutor;
    use embassy_stm32::can::Frame;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    // Firmware metadata generated using `cancan-build`
    include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

    /// Channels for firmware updates during runtime.
    pub(crate) static CANCAN: CanCanChannels<CriticalSectionRawMutex, Frame> = CanCanChannels::new();

    pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
}

#[cfg(feature = "hardware")]
pub use firmware::*;
