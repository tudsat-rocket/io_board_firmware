//! Node 16 — the OX stepper node (rev2).
//!
//! A clock/direction actuator on COM3 with its ENABLE switched from HCO2, and a solenoid on HCO1.
//! Rev2 board: build with `--features rev2`. See [`io_board::zenith_mapping::NODE16_OX_STEPPER`].

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use io_board::zenith_mapping;

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, zenith_mapping::NODE16_OX_STEPPER).await;
}
