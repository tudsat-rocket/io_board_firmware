#![no_std]
#![no_main]

use cortex_m_rt::entry;

use defmt_rtt as _;

use embassy_stm32::flash::{BANK1_REGION, Flash};

// The cancan bootloader for the IO board. It runs only at reset: cancan_boot inspects the swap
// state, performs the A/B swap (or revert) if one is pending, then jumps to the application in
// the ACTIVE partition.
#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(embassy_stm32::Config::default());

    // TODO: watchdog (required after application uses it)

    let region = Flash::new_blocking(p.FLASH).into_blocking_regions().bank1_region;

    cancan_boot::run::<_, _, 2048>(region, BANK1_REGION.base(), || {
        // TODO: pet watchdog
    })
}

cancan_boot::runtime_handlers!();
