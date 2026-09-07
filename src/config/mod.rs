//! Runtime configuration: everything in the 0x3000 block of the object dictionary.
//!
//! A `Config` is what makes one physical node different from another — which valve hangs off
//! which high current output, how a servo's pulse widths map to open and closed, which amplifier
//! on which I2C bus feeds which sensor slot, and what to do when the master stops talking.
//!
//! It comes from one of two places:
//!
//! 1. The compile-time constants in [`crate::zenith_mapping`], which are the *factory defaults*
//!    for a given node id, or
//! 2. the on-board NOR flash, written by an operator over SDO and committed with 0x1010.
//!
//! The stored config wins whenever it is present and passes its CRC. See [`persist`].
//!
//! Every field here is expressible as one expedited SDO transfer (at most 4 bytes), which is what
//! keeps the config plane to a single request/response frame pair per value.
//!
//! # Layout
//!
//! [`Config`] itself, its cross-cutting invariants ([`Config::sanity_check`]) and the build-time
//! [`NodeSettings`] live here. The three sub-domains it is made of get a module each, and every
//! public name in them is re-exported below — so `crate::config::Unit` keeps working regardless
//! of which file it happens to be defined in:
//!
//! - [`valves`] — what is fitted to a valve slot and how it is driven.
//! - [`sensors`] — what is on a sensor slot, how its raw number is calibrated, and in what unit.
//! - [`relief`] — the local overpressure loop.
//! - [`persist`] — serializing the whole thing to NOR flash.

pub mod persist;
pub mod relief;
pub mod sensors;
pub mod valves;

use crate::index::{
    HcoId, I2cBus, PdoSensorChannel, PerSensorSlot, PerStepper, PerTpdoKind, PerValve, SensorSlot, StepperId, ValveId,
};
pub use crate::stepper::StepperConfig;

pub use relief::ReliefConfig;
pub use sensors::{
    AMPLIFIER_ADDRESSES, ENCODER_ADDRESS, ENCODER_FULL_SCALE, ENCODER_MAGNET_OK_BIT, ENCODER_PRESENT_BIT,
    NUM_ADC_SLOTS, NUM_AMPLIFIERS, NUM_PDO_SENSOR_CHANNELS, NUM_SENSOR_SLOTS, Quantity, SensorCalib, SensorKind,
    SensorSlotConfig, SensorSource, Unit, UnitPrefix,
};
pub use valves::{FallbackAction, NUM_HCO, NUM_VALVES, PROMILLE_MAX, ValveConfig, ValveKind};

use valves::shares_output;

/// Size of the one fixed domain that is not a valve or a sensor.
///
/// The `NUM_*` constants are each the count of the matching id type in [`crate::index`] — kept as
/// plain numbers only for the places that genuinely want one (a CANopen array's entry count, a log
/// line), never as the basis for an index.
pub const NUM_I2C_BUSES: usize = I2cBus::COUNT;

/// Number of fixed TPDO kinds. Defined in `iocan-proto` (the wire protocol crate) so `TpdoKind`
/// and this array size can never drift apart; re-exported here since so much of the object
/// dictionary (0x3040's `array_size` among it) is sized against it.
pub use iocan_proto::ids::NUM_TPDO_KINDS;

/// Default TPDO periods in milliseconds, indexed by `TpdoKind`. 0 disables a kind.
///
/// The defaults broadcast what the master needs to fly (valve state, selected sensors) quickly,
/// the raw ADC channels slowly enough to leave bus headroom, and the assembly-verification
/// channels (i2c scan, sensor units) rarely.
const DEFAULT_TPDO_MS: PerTpdoKind<u16> = PerTpdoKind::new([
    500,  // 0  ValveCommanded
    500,  // 1  ValveTarget
    200,  // 2  ValveMeasured
    500,  // 3  ValveStatus
    500,  // 4  HcoState
    100,  // 5  RawBus0A
    0,    // 6  RawBus0B
    100,  // 7  RawBus1A
    0,    // 8  RawBus1B
    50,   // 9  Sensor0
    50,   // 10 Sensor1
    0,    // 11 Sensor3
    5000, // 12 SensorUnits
    1000, // 13 I2cScan
    1000, // 14 RailVoltage
    1000, // 15 RailCurrent
    1000, // 16 Status
    0,    // 17 ValveCurrent
]);

#[derive(Clone, Debug)]
pub struct Config {
    pub master_node_id: u8,
    /// Time without a master heartbeat before stage A fires.
    pub fallback_a_ms: u32,
    /// Time without a master heartbeat before stage B fires. Must exceed `fallback_a_ms`.
    pub fallback_b_ms: u32,
    pub fallback_enabled: bool,
    /// Period of our own outgoing heartbeat. 0 disables it.
    pub heartbeat_period_ms: u16,
    pub valves: PerValve<ValveConfig>,
    pub sensors: PerSensorSlot<SensorSlotConfig>,
    pub sensor_interval_ms: u16,
    pub scan_interval_ms: u16,
    pub tpdo_interval_ms: PerTpdoKind<u16>,
    pub relief: ReliefConfig,
    /// The board's clock/direction actuators. At most two, because PA2/PA3 carry the only timer
    /// channels left on pins this board can reach.
    pub steppers: PerStepper<StepperConfig>,
}

impl Config {
    pub const fn new() -> Self {
        Self {
            master_node_id: 1,
            fallback_a_ms: 3_000,
            fallback_b_ms: 300_000,
            fallback_enabled: false,
            heartbeat_period_ms: 1000,
            valves: PerValve::splat(ValveConfig::unmapped()),
            sensors: PerSensorSlot::splat(SensorSlotConfig::unused()),
            sensor_interval_ms: 10,
            scan_interval_ms: 500,
            tpdo_interval_ms: DEFAULT_TPDO_MS,
            relief: ReliefConfig::disabled(),
            steppers: PerStepper::splat(StepperConfig::disabled()),
        }
    }

    pub const fn with_relief(mut self, relief: ReliefConfig) -> Self {
        self.relief = relief;
        self
    }

    /// Fit one clock/direction actuator, and give its valve slot the matching kind.
    ///
    /// Both halves in one call on purpose: a `StepperConfig` naming a valve that is not a
    /// [`ValveKind::Stepper`], or a stepper valve with no actuator behind it, is exactly the
    /// mismatch `sanity_check` rejects — so the ordinary way in cannot produce one.
    pub const fn with_stepper(mut self, id: StepperId, stepper: StepperConfig) -> Self {
        if let Some(valve) = stepper.valve {
            self.valves = self.valves.with_at(valve.index(), ValveConfig::stepper());
        }
        self.steppers = self.steppers.with_at(id.index(), stepper);
        self
    }

    /// Which actuator, if any, answers to a given valve slot.
    pub fn stepper_for(&self, valve: ValveId) -> Option<StepperId> {
        self.steppers.iter().find_map(|(id, s)| (s.valve == Some(valve)).then_some(id))
    }

    pub const fn with_valve(mut self, valve: ValveId, config: ValveConfig) -> Self {
        self.valves = self.valves.with_at(valve.index(), config);
        self
    }

    /// Fit a sensor and, unless it already claims one, broadcast it on the TPDO channel matching
    /// its own slot number.
    ///
    /// That default is what keeps the first twelve slots behaving as they always have: slot *n*
    /// lands in channel *n* of the `Sensor0`/`Sensor1`/`Sensor3` frames. Slots past the twelfth
    /// have no matching channel and get none, so putting a sensor there is a deliberate decision
    /// to read it over SDO — or to give it a channel by hand with
    /// [`SensorSlotConfig::on_channel`].
    pub const fn with_sensor(mut self, slot: SensorSlot, config: SensorSlotConfig) -> Self {
        let mut config = config;
        if config.pdo_channel.is_none() {
            config.pdo_channel = PdoSensorChannel::from_index(slot.index());
        }
        self.sensors = self.sensors.with_at(slot.index(), config);
        self
    }

    /// Fit a sensor that is never broadcast: sampled, calibrated and readable at 0x2004, but off
    /// the process data plane. For the slow and the merely diagnostic, which is most of what the
    /// slots past the twelfth are for.
    pub const fn with_quiet_sensor(mut self, slot: SensorSlot, config: SensorSlotConfig) -> Self {
        let mut config = config;
        config.pdo_channel = None;
        self.sensors = self.sensors.with_at(slot.index(), config);
        self
    }

    /// Which valve, if any, drives a given high current output. Ownership is derived from the
    /// valve mapping rather than stored, so it can never disagree with it.
    ///
    /// Note the two id types: an [`HcoId`] goes in and a [`ValveId`] comes out. Both were `u8`
    /// before, which made the two ends of this lookup silently interchangeable.
    pub fn hco_owner(&self, hco: HcoId) -> Option<ValveId> {
        self.valves.iter().find_map(|(id, v)| {
            if !v.is_mapped() {
                return None;
            }
            (v.signal_hco == Some(hco) || v.power_hco == Some(hco)).then_some(id)
        })
    }

    /// Reject configurations that would misbehave rather than silently running with them. Called
    /// after a load from NOR and after every SDO write that could break an invariant.
    ///
    /// This is the one place that gets to look across the sub-domains at once, which is why it
    /// lives here rather than in any of them: every check below is about two things disagreeing —
    /// a valve and the sensor it trusts, two valves and one output, two sensors and one channel.
    pub fn sanity_check(&self) -> Result<(), ConfigError> {
        if self.master_node_id > 0x0F {
            return Err(ConfigError::NodeIdOutOfRange);
        }
        if self.fallback_b_ms <= self.fallback_a_ms {
            return Err(ConfigError::FallbackOrder);
        }
        for (id, v) in self.valves.iter() {
            if !v.is_mapped() {
                continue;
            }
            if v.min_promille > v.max_promille {
                return Err(ConfigError::ClampInverted(id));
            }
            if v.kind == ValveKind::Servo && v.travel_ms == 0 {
                return Err(ConfigError::ZeroTravelTime(id));
            }
            // A stepper valve with no actuator behind it has nothing to drive: it would accept
            // commands over the bus and move nothing at all.
            if v.kind == ValveKind::Stepper && self.stepper_for(id).is_none() {
                return Err(ConfigError::StepperUnmapped(id));
            }
            // A valve told to trust a sensor that reports something other than promille would
            // take a pressure reading as a position. Better to refuse the config than to drive a
            // valve against a number that means nothing to it.
            if let Some(slot) = v.position_sensor {
                let sensor = &self.sensors[slot];
                if sensor.kind == SensorKind::None {
                    return Err(ConfigError::PositionSensorUnmapped(id, slot));
                }
                if sensor.unit != Unit::Promille {
                    return Err(ConfigError::PositionSensorUnit(id, slot));
                }
            }
            // Two valves sharing an output would fight each other every control tick.
            for (other, w) in self.valves.iter().skip(id.index() + 1) {
                if w.is_mapped() && shares_output(v, w) {
                    return Err(ConfigError::OutputShared(id, other));
                }
            }
        }

        for (id, s) in self.sensors.iter() {
            if s.kind == SensorKind::None {
                continue;
            }
            // Every kind is read over I2C, so a configured slot without a bus reads nothing at
            // all while looking configured — the same silent-failure shape as an armed relief
            // loop pointing at an unfitted valve.
            if s.bus.is_none() {
                return Err(ConfigError::SensorBusUnset(id));
            }
            // Two slots on one channel means one of them silently never reaches the bus.
            for (other, t) in self.sensors.iter().skip(id.index() + 1) {
                if s.pdo_channel.is_some() && s.pdo_channel == t.pdo_channel {
                    return Err(ConfigError::PdoChannelShared(id, other));
                }
            }
        }

        for (id, stepper) in self.steppers.iter() {
            let Some(valve) = stepper.valve else {
                continue;
            };
            // The mirror of the check above: an actuator pointing at a slot that is not a stepper
            // would have the control task planning moves for a valve driving an HCO pair.
            if self.valves[valve].kind != ValveKind::Stepper {
                return Err(ConfigError::StepperValveKind(valve));
            }
            if stepper.span() == 0 {
                return Err(ConfigError::StepperZeroTravel(id));
            }
            // Zero here is not "disabled", it is "never start", and the actuator would sit
            // reporting Moving forever without emitting a pulse.
            if stepper.start_step_hz == 0 || stepper.max_step_hz == 0 {
                return Err(ConfigError::StepperZeroSpeed(id));
            }
            // Two actuators on one valve slot would both plan moves for it, from two different
            // step counters, and fight over the same reported position.
            for (other, w) in self.steppers.iter().skip(id.index() + 1) {
                if w.valve == Some(valve) {
                    return Err(ConfigError::StepperValveShared(id, other));
                }
            }
        }

        if self.relief.is_armed() {
            // An armed relief loop pointing at a valve that is not fitted would look configured
            // while doing nothing, which is the worst way for a safety function to fail. Refuse
            // it instead. That the *slot numbers* are in range no longer needs checking — a
            // `ValveId`/`SensorSlot` cannot be out of range — so what is left here is the part
            // that is genuinely about configuration rather than about integers.
            let valve = self.relief.valve.ok_or(ConfigError::ReliefValveUnmapped(ValveId::Valve0))?;
            if !self.valves[valve].is_mapped() {
                return Err(ConfigError::ReliefValveUnmapped(valve));
            }
            if self.sensors[self.relief.sensor].kind == SensorKind::None {
                return Err(ConfigError::ReliefSensorUnmapped(self.relief.sensor));
            }
            if self.relief.pulse_ms == 0 {
                return Err(ConfigError::ReliefPulseZero);
            }
        }
        Ok(())
    }

    /// Configuration that is legal but probably not what was meant. Logged once at boot rather
    /// than rejected, because each of these has a defensible use.
    pub fn log_warnings(&self) {
        for (id, stepper) in self.steppers.iter() {
            let Some(valve) = stepper.valve else {
                continue;
            };
            let cfg = &self.valves[valve];
            // With ENABLE strapped to 5 V there is no drive to drop, so a stepper holds its
            // position through a fallback whatever the stage asked for. Configuring an unpower
            // reads as "this will go limp" and it will not.
            if cfg.fallback_a.unpower || cfg.fallback_b.unpower {
                defmt::warn!(
                    "valve {} is stepper {}: its fallback unpower flags (0x3005/0x3006) do \
                     nothing, the motor holds torque as long as it has power",
                    valve,
                    id
                );
            }
            if stepper.start_step_hz > stepper.max_step_hz {
                defmt::warn!(
                    "stepper {} pull-in rate {} Hz is above the traverse speed {} Hz, so the \
                     traverse speed has no effect",
                    id,
                    stepper.start_step_hz,
                    stepper.max_step_hz
                );
            }
            if stepper.accel_hz_per_s == 0 {
                defmt::warn!("stepper {} acceleration is 0: every move runs at the pull-in rate", id);
            }
        }

        if !self.relief.is_armed() {
            return;
        }
        let Some(valve) = self.relief.valve.map(|v| &self.valves[v]) else {
            return;
        };
        // A servo needs `travel_ms` to reach the relief position at all. Pulsing for less than
        // that opens it partway and closes it again, which still bleeds pressure but is almost
        // never the intent — a relief valve usually wants to be a solenoid.
        if valve.kind == ValveKind::Servo && self.relief.pulse_ms < valve.travel_ms {
            defmt::warn!(
                "relief pulse is {} ms but the valve takes {} ms to travel: it will only open \
                 partway before closing again. Lengthen the pulse (0x3055) or fit a solenoid.",
                self.relief.pulse_ms,
                valve.travel_ms
            );
        }
        if self.relief.threshold == i16::MAX {
            defmt::warn!("relief is enabled but its threshold is i16::MAX, so it can never fire");
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// What distinguishes one physical node from another at build time.
pub struct NodeSettings {
    /// Also the node's address on the bus. Four bits: at most 16 nodes.
    pub node_id: u8,
    /// Factory defaults, used when the NOR flash holds no valid configuration.
    pub config: Config,
}

impl NodeSettings {
    pub const fn new(node_id: u8, config: Config) -> Self {
        Self { node_id, config }
    }
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum ConfigError {
    NodeIdOutOfRange,
    FallbackOrder,
    ClampInverted(ValveId),
    ZeroTravelTime(ValveId),
    OutputShared(ValveId, ValveId),
    /// A valve's position sensor points at a slot with nothing configured on it.
    PositionSensorUnmapped(ValveId, SensorSlot),
    /// A valve's position sensor reports something other than promille.
    PositionSensorUnit(ValveId, SensorSlot),
    /// A configured sensor slot has no I2C bus to read from.
    SensorBusUnset(SensorSlot),
    /// Two sensor slots claim the same TPDO channel.
    PdoChannelShared(SensorSlot, SensorSlot),
    /// A valve is configured as a stepper, but the board's actuator does not answer to it.
    StepperUnmapped(ValveId),
    /// The actuator names a valve slot that is not a [`ValveKind::Stepper`].
    StepperValveKind(ValveId),
    /// Closed and open are the same step count, so the actuator can never move.
    StepperZeroTravel(StepperId),
    /// A pulse rate of zero: the actuator would never take a step.
    StepperZeroSpeed(StepperId),
    /// Both actuators point at the same valve slot.
    StepperValveShared(StepperId, StepperId),
    /// Relief is armed against a valve that is not fitted.
    ReliefValveUnmapped(ValveId),
    /// Relief is armed against a sensor slot that is not configured.
    ReliefSensorUnmapped(SensorSlot),
    ReliefPulseZero,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::index::{AmplifierId, HcoPair};

    fn with_encoder_on(valve: ValveId, slot: SensorSlot) -> Config {
        Config::new()
            .with_valve(valve, ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 1000).with_position_sensor(slot))
            .with_sensor(slot, SensorSlotConfig::encoder(I2cBus::Bus0, 0, 1024))
    }

    #[test]
    fn a_valve_may_take_its_position_from_an_encoder_slot() {
        assert!(with_encoder_on(ValveId::Valve0, SensorSlot::Slot0).sanity_check().is_ok());
    }

    /// The failure this check exists for: a slot reporting centibar would hand a valve a pressure
    /// and the valve would drive against it as though it were a position.
    #[test]
    fn a_position_sensor_reporting_the_wrong_unit_is_refused() {
        let mut cfg = with_encoder_on(ValveId::Valve0, SensorSlot::Slot0);
        cfg.sensors[SensorSlot::Slot0].unit = Unit::CentiBar;
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::PositionSensorUnit(ValveId::Valve0, SensorSlot::Slot0))));
    }

    /// Pointing a valve at an empty slot looks configured while doing nothing, which is the
    /// silent-failure shape the relief checks already refuse.
    #[test]
    fn a_position_sensor_on_an_empty_slot_is_refused() {
        let mut cfg = with_encoder_on(ValveId::Valve0, SensorSlot::Slot0);
        cfg.sensors[SensorSlot::Slot0] = SensorSlotConfig::unused();
        assert!(matches!(
            cfg.sanity_check(),
            Err(ConfigError::PositionSensorUnmapped(ValveId::Valve0, SensorSlot::Slot0))
        ));
    }

    /// Two slots on one channel means one of them silently never reaches the bus, and which one
    /// wins would depend on iteration order.
    #[test]
    fn two_slots_may_not_claim_the_same_pdo_channel() {
        let cfg = Config::new()
            .with_sensor(SensorSlot::Slot0, SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0))
            .with_sensor(
                SensorSlot::Slot5,
                SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp1).on_channel(PdoSensorChannel::Ch0),
            );
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::PdoChannelShared(SensorSlot::Slot0, SensorSlot::Slot5))));
    }

    #[test]
    fn a_configured_slot_without_a_bus_is_refused() {
        let mut cfg =
            Config::new().with_sensor(SensorSlot::Slot0, SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0));
        cfg.sensors[SensorSlot::Slot0].bus = None;
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::SensorBusUnset(SensorSlot::Slot0))));
    }

    /// The default that keeps the first twelve slots behaving as they always have.
    #[test]
    fn with_sensor_defaults_a_slot_to_its_own_channel() {
        let cfg = Config::new()
            .with_sensor(SensorSlot::Slot3, SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0))
            // Past the twelfth there is no matching channel, so this one stays off the bus.
            .with_sensor(SensorSlot::Slot13, SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp1))
            .with_quiet_sensor(SensorSlot::Slot4, SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp2));

        assert_eq!(cfg.sensors[SensorSlot::Slot3].pdo_channel, Some(PdoSensorChannel::Ch3));
        assert_eq!(cfg.sensors[SensorSlot::Slot13].pdo_channel, None);
        assert_eq!(cfg.sensors[SensorSlot::Slot4].pdo_channel, None);
        assert!(cfg.sanity_check().is_ok());
    }
}
