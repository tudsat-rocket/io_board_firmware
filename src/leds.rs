//! The status LED state, and the pubsub plumbing `crate::control::Control` publishes it on.
//!
//! Unconditional (not behind the `hardware` feature) even though only `crate::board::leds`'s
//! `run_leds` task ever turns [`LedsState`] into real GPIO levels: the type and the
//! `embassy_sync::pubsub` channel around it are plain data, and keeping them reachable on the
//! host is what lets `Control` be built and tested without hardware.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use static_cell::StaticCell;

pub type StateLedSub = Subscriber<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;
pub type StateLedPub = Publisher<'static, CriticalSectionRawMutex, LedsState, 4, 1, 1>;

pub static STATE_LED_PUB_SUB: StaticCell<PubSubChannel<CriticalSectionRawMutex, LedsState, 4, 1, 1>> =
    StaticCell::new();

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub struct LedsState {
    pub red: bool,
    pub yellow: bool,
    pub white: bool,
}

impl LedsState {
    /// The wire encoding of object 0x2030: red is bit 0, yellow bit 1, white bit 2.
    ///
    /// Lives here rather than at the store write so the bit layout sits next to the fields it
    /// packs — nothing else in the firmware is allowed to know which bit is which.
    pub const fn as_byte(self) -> u8 {
        (self.red as u8) | (self.yellow as u8) << 1 | (self.white as u8) << 2
    }
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
