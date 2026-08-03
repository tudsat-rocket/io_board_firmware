//! Node 8 — the self-regulating relief node.
//!
//! One pressure transducer, one valve, and the authority to open that valve on its own when the
//! vessel it is bolted to gets away from the master. See [`io_board::relief`] for the reasoning
//! and [`io_board::zenith_mapping::NODE8_REG`] for the wiring it expects.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use io_board::zenith_mapping;

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, zenith_mapping::NODE8_REG).await;
}
