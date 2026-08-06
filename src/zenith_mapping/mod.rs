//! Per-node factory defaults for the Zenith vehicle.
//!
//! One [`NodeSettings`] per physical board, selected by the matching `src/bin/nodeN.rs`. These
//! are only what a board comes up with when its NOR flash holds no valid configuration — see
//! [`crate::config`]. Reconfiguring a board in the field is an SDO write plus a save, not a
//! rebuild; this file is the fallback, and the place to record a configuration once it has been
//! proven.

use crate::config::{Config, NodeSettings, ReliefConfig, SensorSlotConfig, ValveConfig};
use crate::index::{AmplifierId::*, HcoId, HcoPair, I2cBus::*, SensorSlot::*, ValveId::*};
use crate::zenith_mapping::sensors::Transducer;

pub mod sensors;
pub mod valves;

/// A pressure slot fed by amplifier `amplifier` on `bus`, with the transducer's own unit.
const fn pressure(bus: crate::index::I2cBus, amplifier: crate::index::AmplifierId, t: Transducer) -> SensorSlotConfig {
    SensorSlotConfig::pressure(bus, amplifier, t.unit, t.calib)
}

const fn pt1000(bus: crate::index::I2cBus, amplifier: crate::index::AmplifierId) -> SensorSlotConfig {
    SensorSlotConfig::pt1000(bus, amplifier)
}

/// Node 2 — nosecone / recovery. One temperature probe, no valves.
pub const NODE2: NodeSettings = NodeSettings::new(2, Config::new().with_sensor(Slot0, pt1000(Bus0, Amp0)));

/// Node 3 — payload avionics. Nothing wired yet.
pub const NODE3: NodeSettings = NodeSettings::new(3, Config::new());

/// Node 4 — upper propulsion. Oxidizer vent solenoid on HCO1.
pub const NODE4: NodeSettings =
    NodeSettings::new(4, Config::new().with_valve(Valve0, ValveConfig::solenoid_on(HcoId::Hco0)));

/// Node 5 — upper propulsion: pressurization and pressurant vent, tank and regulator sensing.
pub const NODE5: NodeSettings = NodeSettings::new(
    5,
    Config::new()
        .with_valve(Valve0, valves::pressurization(HcoPair::A))
        .with_valve(Valve1, valves::pressurant_vent(HcoPair::B))
        // regulator temperature
        .with_sensor(Slot0, pt1000(Bus0, Amp0))
        // regulator, upper and lower
        .with_sensor(Slot1, pressure(Bus0, Amp1, sensors::REG_2_P))
        .with_sensor(Slot2, pressure(Bus0, Amp2, sensors::REG_1_P))
        // upper oxidizer tank
        .with_sensor(Slot3, pressure(Bus1, Amp0, sensors::OX_TANK_UPPER_P))
        // pressurant (N2) tank — 400 bar
        .with_sensor(Slot4, pressure(Bus1, Amp1, sensors::PRESSURANT_TANK_P)),
);

/// Node 6 — lower propulsion, valve control: main valve and oxidizer fill/dump.
pub const NODE6: NodeSettings = NodeSettings::new(
    6,
    Config::new()
        .with_valve(Valve0, valves::main_valve(HcoPair::A))
        .with_valve(Valve1, valves::ox_fill_and_dump(HcoPair::B))
        .with_sensor(Slot0, pressure(Bus0, Amp0, sensors::OX_TANK_LOWER_P))
        .with_sensor(Slot1, pressure(Bus0, Amp1, sensors::COMB_CHAMBER_1_P))
        .with_sensor(Slot2, pt1000(Bus0, Amp2))
        .with_sensor(Slot3, pressure(Bus1, Amp1, sensors::COMB_CHAMBER_2_P)),
);

/// Node 7 — lower propulsion, igniter control. Nothing wired yet.
pub const NODE7: NodeSettings = NodeSettings::new(7, Config::new());

/// 60 bar, in the centibar that a 100 bar transducer slot reports.
pub const RELIEF_THRESHOLD_60_BAR: i16 = 6000;

/// Node 8 — self-regulating relief node.
///
/// A tank that is being heated with every valve shut keeps rising in pressure on its own, and the
/// master may be slow to react or briefly off the bus when it happens. This node watches one
/// transducer and bleeds its own valve when the pressure gets away from it — see
/// [`crate::relief`]. The rest of the time it is an ordinary slave.
///
/// The relief valve is a **solenoid** on HCO1 rather than a servo, deliberately: the relief pulse
/// is half a second, and a servo that takes a second and a half to travel would never reach the
/// open position within one. `Config::log_warnings` complains at boot if that combination is ever
/// configured by hand.
pub const NODE8_REG: NodeSettings = NodeSettings::new(
    8,
    Config::new()
        .with_valve(Valve0, ValveConfig::solenoid_on(HcoId::Hco0))
        .with_sensor(Slot0, pressure(Bus0, Amp0, sensors::OX_TANK_UPPER_P))
        .with_relief(
            ReliefConfig::new(Valve0, Slot0, RELIEF_THRESHOLD_60_BAR).with_pulse_ms(500).with_cooldown_ms(500),
        ),
);
