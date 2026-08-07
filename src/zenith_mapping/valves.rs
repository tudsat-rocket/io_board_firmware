//! Servo valve travel calibration for the Zenith vehicle.
//!
//! Factory defaults, same as [`super::sensors`]: a board configured over the bus and saved
//! ignores these.
//!
//! `travel_ms` is the time for a full closed-to-open sweep, which is what the measured-position
//! estimator integrates against and what sets the settle deadline before a fallback releases the
//! servo. Measure it once per valve type on the bench; a wrong value does not move the valve
//! anywhere different, it just makes `measured_state` lead or lag reality.
//!
//! `stall_ma` is left at 0 (stall detection off) for every valve here. Turning it on needs a
//! bench measurement of running current versus locked-rotor current for that specific valve, and
//! guessing a threshold would either cry stall on every stroke or never fire.

use crate::config::ValveConfig;
use crate::index::HcoPair;

/// Travel time until somebody measures the real ones. Deliberately the same for every valve so
/// that a value which has been measured is obvious by being different.
const UNMEASURED_TRAVEL_MS: u16 = 1500;

/// A servo on an HCO pair ([`HcoPair::A`] = outputs 1 and 2, [`HcoPair::B`] = 3 and 4), wired the way the
/// vehicle harness does it: the lower output powers the servo, the upper one carries the signal.
const fn servo(pair: HcoPair, closed_us: u16, open_us: u16) -> ValveConfig {
    ValveConfig::servo_on_pair(pair, closed_us, open_us, UNMEASURED_TRAVEL_MS)
}

pub const fn ox_fill_and_dump(pair: HcoPair) -> ValveConfig {
    servo(pair, 1980, 850)
}

pub const fn pressurant_vent(pair: HcoPair) -> ValveConfig {
    servo(pair, 2000, 1082)
}

pub const fn main_valve(pair: HcoPair) -> ValveConfig {
    servo(pair, 2470, 700)
}

pub const fn pressurization(pair: HcoPair) -> ValveConfig {
    servo(pair, 2200, 1080)
}

/// Mid-range travel for bench work with an uncharacterised servo.
pub const fn placeholder_servo(pair: HcoPair) -> ValveConfig {
    servo(pair, 2000, 1000)
}
