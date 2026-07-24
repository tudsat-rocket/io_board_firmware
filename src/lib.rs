#![no_std]

use embassy_executor::InterruptExecutor;
use embassy_stm32::can::Frame;
use embassy_stm32::flash::Flash;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use cancan::{CanCan, CanCanChannels};

#[cfg(feature = "rev3")]
use crate::board::{CurrentSens, OnboardSensRev3, TemperatureSens, VoltageSens};

use {defmt_rtt as _, panic_probe as _};

pub mod board;
pub mod can;
pub mod canopen_interface;
pub mod ereg;
pub mod ext_adc;
pub mod node;
pub mod sensors;
pub mod store;
pub mod tpdo;
pub mod utils;
pub mod valves;
pub mod zenith_mapping;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

// pub const NODE_ID: u8 = 0xff;

// FIXME: felix?
#[cfg(feature = "rev2")]
pub const NODE_NAME: &str = "I/O [rev2]";
#[cfg(feature = "rev3")]
pub const NODE_NAME: &str = "I/O [rev3]";

// Channels for firmware updates during runtime
static CANCAN: CanCanChannels<CriticalSectionRawMutex, Frame> = CanCanChannels::new();

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
