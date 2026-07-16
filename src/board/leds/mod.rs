use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use static_cell::StaticCell;

pub type StateLedSub = Subscriber<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;
pub type StateLedPub = Publisher<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;

pub static STATE_LED_PUB_SUB: StaticCell<PubSubChannel<CriticalSectionRawMutex, LedsState, 4, 1, 1>> =
    StaticCell::new();

#[derive(Default, Clone, Copy)]
pub struct LedsState {
    pub red: bool,
    pub yellow: bool,
    pub white: bool,
}
impl From<[bool; 3]> for LedsState {
    fn from(value: [bool; 3]) -> Self {
        Self {
            red: value[0],
            yellow: value[1],
            white: value[2],
        }
    }
}

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
