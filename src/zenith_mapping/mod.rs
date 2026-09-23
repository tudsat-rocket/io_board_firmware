//! Per-node factory defaults for the Zenith vehicle.
//!
//! One [`NodeSettings`] per physical board, selected by the matching `src/bin/nodeN.rs`. These
//! are only what a board comes up with when its NOR flash holds no valid configuration — see
//! [`crate::config`]. Reconfiguring a board in the field is an SDO write plus a save, not a
//! rebuild; this file is the fallback, and the place to record a configuration once it has been
//! proven.

use crate::config::{Config, FallbackAction, NodeSettings, PROMILLE_MAX, ReliefConfig, SensorSlotConfig, ValveConfig};
use crate::heater::{AnalogPin, HeaterConfig, NtcWiring};
#[allow(
    unused_imports,
    reason = "SensorSlot's later variants are only used by node configs that do not exist yet"
)]
use crate::index::{AmplifierId::*, HcoId, HcoPair, I2cBus::*, SensorSlot::*, StepperId, ValveId::*};
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

/// An MCP9700 on an amplifier channel. Cheaper and less fussy than a Pt1000 bridge, and good
/// enough wherever a couple of degrees does not change a decision.
#[allow(dead_code, reason = "a factory default waiting on the harness that uses it")]
const fn mcp9700(bus: crate::index::I2cBus, amplifier: crate::index::AmplifierId) -> SensorSlotConfig {
    SensorSlotConfig::mcp9700(bus, amplifier)
}

/// The AS5600 on `bus`, reporting a valve's travel in promille.
///
/// `zero_counts` is the raw angle at the closed stop and `counts` the signed span to the open one
/// — negative for a valve that opens counter-clockwise. Both are read off 0x2006 on the bench with
/// the valve parked on each stop; see [`crate::config::SensorCalib::angle_over`].
#[allow(dead_code, reason = "a factory default waiting on the harness that uses it")]
const fn encoder(bus: crate::index::I2cBus, zero_counts: u16, counts: i16) -> SensorSlotConfig {
    SensorSlotConfig::encoder(bus, zero_counts, counts)
}

/// Node 2 — nosecone / recovery. One temperature probe, no valves.
pub const NODE2: NodeSettings = NodeSettings::new(2, Config::new().with_sensor(Slot0, pt1000(Bus0, Amp0)));

/// Node 3 — payload avionics. Nothing wired yet.
pub const NODE3: NodeSettings = NodeSettings::new(3, Config::new());

/// Factory default for the temperature the node 4 heating pad holds, in centidegrees Celsius.
/// Placeholder, pick the real one before flight. Adjustable at 0x3070 and saved with 0x1010.
pub const NODE4_HEATER_SETPOINT_CENTI_C: i16 = 3_000;
/// Half-width of its dead band: on below 29 C, off above 31 C.
pub const NODE4_HEATER_HYSTERESIS_MILLI_C: i32 = 1_000;
/// Calibration offset for the pad's NTC, added to what the curve reads. Uncalibrated: measure the
/// pad against a reference thermometer and set this to `reference - uncalibrated`, taking the
/// uncalibrated value from the `heater:` line the node logs every second.
pub const NODE4_HEATER_OFFSET_MILLI_C: i32 = 0;

/// Node 4 — upper propulsion. Oxidizer vent solenoid on HCO1, and a heating pad on pair B
/// (HCO3+4) with its NTC on COM4 pin 1 (PA2), which leaves it without a stepper port. The pad
/// boots off; the master switches it on at 0x2018. See [`crate::heater`].
pub const NODE4: NodeSettings = NodeSettings::new(
    4,
    Config::new()
        .with_valve(Valve0, ValveConfig::solenoid_on(HcoId::Hco0))
        .with_heater_setpoint_centi_c(NODE4_HEATER_SETPOINT_CENTI_C),
)
.with_heater(
    HeaterConfig::new(HcoPair::B, AnalogPin::Pa2)
        // The reading rose with temperature on the bench, so the NTC is on the supply side.
        .with_ntc_wiring(NtcWiring::ToSupply)
        .with_hysteresis_milli_c(NODE4_HEATER_HYSTERESIS_MILLI_C)
        .with_offset_milli_c(NODE4_HEATER_OFFSET_MILLI_C),
);

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
        .with_sensor(Slot2, pt1000(Bus0, Amp2))
        // 40bar-F = D
        .with_sensor(Slot1, pressure(Bus1, Amp0, sensors::COMB_CHAMBER_1_P))
        // 40bar-E = C
        .with_sensor(Slot3, pressure(Bus1, Amp1, sensors::COMB_CHAMBER_2_P)),
    // .with_valve(Valve0, valves::main_valve(HcoPair::A))
    // .with_valve(Valve1, valves::ox_fill_and_dump(HcoPair::B))
    // .with_sensor(Slot0, pressure(Bus0, Amp0, sensors::OX_TANK_LOWER_P))
    // .with_sensor(Slot1, pressure(Bus0, Amp1, sensors::COMB_CHAMBER_1_P))
    // .with_sensor(Slot2, pt1000(Bus0, Amp2))
    // .with_sensor(Slot3, pressure(Bus1, Amp1, sensors::COMB_CHAMBER_2_P)),
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

/// Node 9 — the stepper node: a clock/direction actuator on COM4.
///
/// A Nanotec PD2-C411L18-E-65-01 replaces a servo on a valve that needs more travel resolution
/// and more torque than a hobby servo has. It is commanded exactly like every other valve on this
/// bus — promille to 0x2010, promille back from 0x2012 — and none of the master's code has to
/// know it is a stepper. What changes is underneath: no high current output is involved at all,
/// so all four stay free for solenoids, and the reported position is a real step count rather
/// than a travel-time estimate. See [`crate::stepper`] and `board::stepper`.
///
/// # Wiring
///
/// COM4 pin 1 (PA2) is the step clock and pin 2 (PA3) the direction — COM3 on a rev2 board, same
/// two pins — both through a 5 V buffer:
/// the driver's inputs are not guaranteed to read a 3.3 V high. ENABLE (X3 pin 4) is strapped to
/// 5 V for now, which means the motor is energised and holding whenever it has power. Two things
/// follow: the fallback stages cannot release it (their unpower flags are `false` here and
/// [`Config::log_warnings`] complains if anyone sets them), and the step count survives a
/// firmware reset but not a power cycle — re-home with 0x2016 after one.
///
/// # Order matters
///
/// [`Config::with_stepper`] installs [`ValveConfig::stepper`] on the named slot, so the
/// `with_valve` that narrows the fallback comes after it.
///
/// A throttle closes on a lost master rather than opening: both stages drive to 0, unlike the
/// vent-shaped default the other nodes inherit.
pub const NODE9_STEPPER: NodeSettings = NodeSettings::new(
    9,
    Config::new()
        .with_stepper(StepperId::Stepper0, valves::placeholder_stepper(Valve0))
        .with_valve(Valve0, throttle_stepper())
        // Chamber pressure, so the actuator has something local to be judged against.
        .with_sensor(Slot0, pressure(Bus0, Amp0, sensors::COMB_CHAMBER_1_P)),
);

/// Node 10 — two clock/direction actuators on one board.
///
/// The same arrangement as [`NODE9_STEPPER`] with a second actuator on valve 1. Only buildable
/// with the `dual-stepper` feature, which is what puts a step clock on PA3 and moves both
/// direction lines onto plain GPIO — see `board::stepper` for the pinout and for why the two
/// share a step rate while both are moving.
///
/// All four high current outputs stay free and fully PWM-capable: the actuators cost two timer
/// channels and two GPIOs, and no output.
pub const NODE10_DUAL_STEPPER: NodeSettings = NodeSettings::new(
    10,
    Config::new()
        .with_stepper(StepperId::Stepper0, valves::placeholder_stepper(Valve0))
        .with_stepper(StepperId::Stepper1, valves::placeholder_stepper(Valve1))
        .with_valve(Valve0, throttle_stepper())
        .with_valve(Valve1, throttle_stepper())
        .with_sensor(Slot0, pressure(Bus0, Amp0, sensors::COMB_CHAMBER_1_P))
        .with_sensor(Slot1, pressure(Bus1, Amp0, sensors::COMB_CHAMBER_2_P)),
);

/// A stepper valve that closes on a lost master rather than opening, which is what a throttle or
/// metering valve wants — unlike the vent-shaped default the other nodes inherit.
const fn throttle_stepper() -> ValveConfig {
    ValveConfig {
        fallback_b: FallbackAction {
            position: 0,
            unpower: false,
        },
        ..ValveConfig::stepper()
    }
}

/// Node 15 — rev2 board, N2 stepper plus the external N2 servo.
///
/// The same single-actuator arrangement as [`NODE9_STEPPER`] (COM3 on rev2, PA2 step / PA3
/// direction), except that ENABLE is not strapped to 5 V: it is switched from HCO2 and exposed
/// as its own valve slot, see [`stepper_enable`]. The master has to energise valve 1 before
/// valve 0 will move. The external N2 servo is on pair B (HCO3 power, HCO4 signal), whose PWM is
/// a hardware timer channel on rev2; HCO1 is unused.
///
/// Build with `rev2` — `just flash-one node15` does that on its own.
pub const NODE15_N2_STEPPER: NodeSettings = NodeSettings::new(
    15,
    Config::new()
        // N2 stepper on COM3.
        .with_stepper(StepperId::Stepper0, valves::placeholder_stepper(Valve0))
        .with_valve(Valve0, throttle_stepper())
        // Stepper ENABLE on HCO2.
        .with_valve(Valve1, stepper_enable(HcoId::Hco1))
        // External N2 servo: power on HCO3, signal on HCO4.
        .with_valve(Valve2, valves::external_n2(HcoPair::B)),
);

/// Node 16 — rev2 board, OX stepper plus the external oxidizer fill solenoid.
///
/// Stepper wired like [`NODE15_N2_STEPPER`] — COM3, its ENABLE on HCO2 — and a solenoid on
/// HCO1. The fill solenoid closes in both fallback stages rather than taking the vent-shaped
/// default — a lost master must never leave the tank filling.
///
/// Build with `rev2` — `just flash-one node16` does that on its own.
pub const NODE16_OX_STEPPER: NodeSettings = NodeSettings::new(
    16,
    Config::new()
        // OX stepper on COM3.
        .with_stepper(StepperId::Stepper0, valves::placeholder_stepper(Valve0))
        .with_valve(Valve0, throttle_stepper())
        // Stepper ENABLE on HCO2.
        .with_valve(Valve1, stepper_enable(HcoId::Hco1))
        // External OX fill solenoid on HCO1.
        .with_valve(Valve2, closed_on_fallback(ValveConfig::solenoid_on(HcoId::Hco0))),
);

/// A stepper's ENABLE line on a high current output, commanded as a solenoid: 1000 promille is
/// enabled, 0 disabled.
///
/// Both fallback stages keep it energised. The stepper's own fallback drives it closed, which it
/// can only do while enabled, and the step count is only trustworthy while the motor holds —
/// pulses sent to a disabled drive are counted but never happen.
const fn stepper_enable(hco: HcoId) -> ValveConfig {
    ValveConfig {
        fallback_a: FallbackAction {
            position: PROMILLE_MAX,
            unpower: false,
        },
        fallback_b: FallbackAction {
            position: PROMILLE_MAX,
            unpower: false,
        },
        ..ValveConfig::solenoid_on(hco)
    }
}

/// `config` with both fallback stages closing it and dropping the drive.
const fn closed_on_fallback(config: ValveConfig) -> ValveConfig {
    let closed = FallbackAction {
        position: 0,
        unpower: true,
    };
    ValveConfig {
        fallback_a: closed,
        fallback_b: closed,
        ..config
    }
}
