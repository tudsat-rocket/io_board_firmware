pub use crate::leds::*;

use embassy_stm32::gpio::Output;

#[embassy_executor::task]
pub async fn run_leds(
    mut leds: (Output<'static>, Output<'static>, Output<'static>),
    initial: LedsState,
    mut leds_state_sub: StateLedSub,
) -> ! {
    leds.0.set_level((!initial.red).into());
    leds.1.set_level((!initial.yellow).into());
    leds.2.set_level((!initial.white).into());
    loop {
        let leds_state = leds_state_sub.next_message_pure().await;
        leds.0.set_level((!leds_state.red).into());
        leds.1.set_level((!leds_state.yellow).into());
        leds.2.set_level((!leds_state.white).into());
    }
}
