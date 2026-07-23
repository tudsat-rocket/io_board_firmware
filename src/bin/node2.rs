#![no_std]
#![no_main]

use embassy_executor::Spawner;
use io_board::zenith_mapping;

use {defmt_rtt as _, panic_probe as _};

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, zenith_mapping::NODE2).await;
}
