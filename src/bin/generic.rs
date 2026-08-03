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

use embassy_executor::Spawner;

use io_board::config::Config;
use io_board::index::{AmplifierId, AmplifierId::*, HcoPair, I2cBus, I2cBus::*, SensorSlot::*, ValveId::*};
use io_board::node::NodeSettings;
use io_board::zenith_mapping::{sensors::PLACEHOLDER_P, valves};

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, BENCH).await;
}

/// Two uncharacterised servos on the two HCO pairs, and the first four amplifier positions of
/// bus 0 mapped straight through as raw counts.
const BENCH: NodeSettings = NodeSettings::new(
    6,
    Config::new()
        .with_valve(Valve0, valves::placeholder_servo(HcoPair::A))
        .with_valve(Valve1, valves::placeholder_servo(HcoPair::B))
        .with_sensor(Slot0, raw(Bus0, Amp0))
        .with_sensor(Slot1, raw(Bus0, Amp1))
        .with_sensor(Slot2, raw(Bus0, Amp2))
        .with_sensor(Slot3, raw(Bus1, Amp0)),
);

const fn raw(bus: I2cBus, amplifier: AmplifierId) -> io_board::config::SensorSlotConfig {
    io_board::config::SensorSlotConfig::pressure(bus, amplifier, PLACEHOLDER_P.unit, PLACEHOLDER_P.calib)
}
