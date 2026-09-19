#![no_std]
#![no_main]

use io_board::zenith_mapping;

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[cortex_m_rt::entry]
fn main() -> ! {
    io_board::node::run(zenith_mapping::NODE3)
}
