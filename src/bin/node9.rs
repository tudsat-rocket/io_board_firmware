//! Node 9 — the stepper node.
//!
//! One clock/direction actuator on COM4, commanded like any other valve on the bus. See
//! [`io_board::zenith_mapping::NODE9_STEPPER`] for the wiring it expects and
//! [`io_board::stepper`] for why its reported position is exact where a servo's is a guess.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use io_board::zenith_mapping;

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, zenith_mapping::NODE9_STEPPER).await;
}
