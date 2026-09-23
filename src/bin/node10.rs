//! Node 10 — the dual stepper node.
//!
//! Two clock/direction actuators on one board, commanded like any other pair of valves. Needs the
//! `dual-stepper` feature, which is what puts the second step clock on PA3 and moves both
//! direction lines onto plain GPIO. See [`io_board::zenith_mapping::NODE10_DUAL_STEPPER`] for the
//! wiring and `io_board::board::stepper` for what the two actuators share.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use io_board::zenith_mapping;

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, zenith_mapping::NODE10_DUAL_STEPPER).await;
}
