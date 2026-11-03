//! The bench node: a board that is not installed in the vehicle.
//!
//! Everything is left uncalibrated on purpose. Sensor slots report raw ADC counts, which is what
//! you want while working out a calibration, and every amplifier address that answers shows up in
//! the presence bitmap (TPDO kind 14) whether or not a slot is mapped to it — which is how an
//! address-strap mistake gets caught during assembly.
//!
//! Configure it over the bus and write "save" to 0x1010 to make a setup stick; the constants here
//! are only what a freshly flashed board falls back to.

#![no_std]
#![no_main]

use io_board::config::{Config, SensorSlotConfig, ValveConfig};
use io_board::index::{BridgeChannel, HcoPair, I2cBus, I2cBus::*, SensorSlot::*, ValveId::*};
use io_board::node::NodeSettings;
use io_board::zenith_mapping::sensors::{Transducer, bridge_counts, wika_ttf1};

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[cortex_m_rt::entry]
fn main() -> ! {
    io_board::node::run(BENCH)
}

const PL_VALVE_CONFIG: ValveConfig = ValveConfig::servo_on_pair(HcoPair::B, 1750, 1940, 1500);

// WIKA TTF-1s by range, at nominal span and an untrimmed zero — see [`wika_ttf1`] for how far off
// that can be. The range is on the sensor's label, not in the datasheet. Pick one per slot below;
// the 400 bar one reports decibar, since 400 bar does not fit an i16 of centibar.
#[allow(dead_code)]
const TTF1_40BAR: Transducer = wika_ttf1(40.0);
#[allow(dead_code)]
const TTF1_100BAR: Transducer = wika_ttf1(100.0);
#[allow(dead_code)]
const TTF1_400BAR: Transducer = wika_ttf1(400.0);

pub const fn wika_testing(range_bar: f32) -> Transducer {
    // bridge(range_bar, 2.0, 170.0)
    bridge_counts(range_bar, -2.0, 65400, 32)
}

/// PGA gain for both NAU7802s: full scale +-15.6 mV/V.
///
/// Gain 128 (+-3.9 mV/V) would hold a TTF-1's span plus its worst zero offset, but clip on
/// overpressure — up to 2x the range, about 5.6 mV/V with that offset — which is the one reading
/// that must not clip; 64 (+-7.8 mV/V) would just about hold it. 32 leaves margin on top, and
/// about 18 noise-free bits (datasheet, gain 32) still resolve the 2 mV/V span into some 20 000
/// steps — far finer than the sensor's own 0.25 % non-linearity.
const GAIN: u8 = 32;

/// One uncharacterised servo on HCO pair B, and a TTF-1 on each of the four NAU7802 channels —
/// both channels on both buses — on slots 0..4, which go out as TPDO sensor channels 0..4
/// (`Sensor0`). The 24-bit counts are on 0x2008 and the `RawBridge` TPDOs.
const BENCH: NodeSettings = NodeSettings::new(
    6,
    Config::new()
        .with_valve(Valve0, PL_VALVE_CONFIG)
        .with_bridge_gain(Bus0, GAIN)
        .with_bridge_gain(Bus1, GAIN)
        .with_sensor(Slot0, bridge(Bus0, BridgeChannel::A, wika_testing(40.0)))
        // .with_sensor(Slot1, bridge(Bus0, BridgeChannel::B, TTF1_100BAR))
        .with_sensor(Slot2, bridge(Bus1, BridgeChannel::A, wika_testing(40.0))), // .with_sensor(Slot3, bridge(Bus1, BridgeChannel::B, TTF1_100BAR)),
);

const fn bridge(bus: I2cBus, channel: BridgeChannel, transducer: Transducer) -> SensorSlotConfig {
    SensorSlotConfig::bridge_pressure(bus, channel, transducer.unit, transducer.calib)
}
