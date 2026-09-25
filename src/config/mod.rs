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

pub mod heating;
pub mod persist;
pub mod relief;
pub mod sensors;
pub mod valves;

use crate::index::{
    HcoId, I2cBus, PdoSensorChannel, PerAnalogInput, PerSensorSlot, PerTpdoKind, PerValve, SensorSlot, ValveId,
};

pub use heating::{HcoSet, ValveHeatingConfig};
pub use relief::ReliefConfig;
pub use sensors::{
    AMPLIFIER_ADDRESSES, ENCODER_ADDRESS, ENCODER_FULL_SCALE, ENCODER_MAGNET_OK_BIT, ENCODER_PRESENT_BIT,
    NUM_ADC_SLOTS, NUM_AMPLIFIERS, NUM_ANALOG_INPUTS, NUM_PDO_SENSOR_CHANNELS, NUM_SENSOR_SLOTS, Quantity, SensorCalib,
    SensorKind, SensorSlotConfig, SensorSource, Unit, UnitPrefix,
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
    0,    // 11 Sensor2
    5000, // 12 SensorUnits
    1000, // 13 I2cScan
    200,  // 14 RailVoltage
    200,  // 15 RailCurrent
    200,  // 16 Status
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
    pub heating: PerValve<ValveHeatingConfig>,
    pub sensors: PerSensorSlot<SensorSlotConfig>,
    pub sensor_interval_ms: u16,
    pub scan_interval_ms: u16,
    pub tpdo_interval_ms: PerTpdoKind<u16>,
    pub relief: ReliefConfig,
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
            heating: PerValve::splat(ValveHeatingConfig::none()),
            sensors: PerSensorSlot::splat(SensorSlotConfig::unused()),
            sensor_interval_ms: 10,
            scan_interval_ms: 500,
            tpdo_interval_ms: DEFAULT_TPDO_MS,
            relief: ReliefConfig::disabled(),
        }
    }

    pub const fn with_relief(mut self, relief: ReliefConfig) -> Self {
        self.relief = relief;
        self
    }

    pub const fn with_valve(mut self, valve: ValveId, config: ValveConfig) -> Self {
        self.valves = self.valves.with_at(valve.index(), config);
        self
    }

    /// Configure a heater for a valve.
    /// The slot has to be one this same config fits — `sanity_check` refuses a heater watching an
    /// empty slot, or one reporting something that is not a temperature — so a node mapping
    /// usually reads as the sensor first and the pad second:
    ///
    /// ```ignore
    /// Config::new()
    ///     .with_valve(Valve0, ValveConfig::solenoid(Hco0))
    ///     .with_quiet_sensor(Slot1, ntc(Com5Pin1))
    ///     .with_valve_heating(Valve0, ValveHeatingConfig::new(Hco2, Slot1, 3_000))
    /// ```
    pub const fn with_valve_heating(mut self, valve: ValveId, config: ValveHeatingConfig) -> Self {
        self.heating = self.heating.with_at(valve.index(), config);
        self
    }

    /// Fit a sensor and, unless it already claims one, broadcast it on the TPDO channel matching
    /// its own slot number.
    ///
    /// That default is what keeps the first twelve slots behaving as they always have: slot *n*
    /// lands in channel *n* of the `Sensor0`/`Sensor1`/`Sensor2` frames. Slots past the twelfth
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

    /// Which COM5/COM6 pins any [`SensorKind::Ntc`] slot is configured onto.
    ///
    /// The control task samples exactly these, which is what keeps a board with no NTC fitted
    /// from spending four ADC conversions a tick on pins nothing reads. Derived from the sensor
    /// mapping rather than stored, so it cannot disagree with it.
    pub fn analog_inputs_used(&self) -> PerAnalogInput<bool> {
        let mut used = PerAnalogInput::splat(false);
        for slot in self.sensors.values() {
            if let Some(input) = slot.analog_input() {
                used[input] = true;
            }
        }
        used
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

        for (id, h) in self.heating.iter() {
            // Switched on but only half wired is the silent-failure shape again: an output with
            // no slot to watch, or a slot with no output to switch. Either looks configured on
            // the bus and does nothing.
            if h.enabled && !h.is_fitted() {
                return Err(ConfigError::HeatingUnfitted(id));
            }
            if !h.is_fitted() {
                continue;
            }
            // A thermostat reading promille or centibar would hold a valve at "30 hundredths of
            // a bar" — it would still switch an output, which is what makes this worth refusing
            // rather than trusting whoever wrote the setpoint.
            if let Some(slot) = h.sensor {
                let sensor = &self.sensors[slot];
                if sensor.kind == SensorKind::None {
                    return Err(ConfigError::HeatingSensorUnmapped(id, slot));
                }
                if sensor.unit.quantity() != Quantity::Temperature {
                    return Err(ConfigError::HeatingSensorUnit(id, slot));
                }
            }
            // A pad and a valve on one output is the worst of the sharing cases: the valve would
            // be driven by a thermostat, and the pad by a valve command.
            for (valve, v) in self.valves.iter() {
                if !v.is_mapped() {
                    continue;
                }
                if [v.power_hco, v.signal_hco].iter().flatten().any(|&hco| h.switches(hco)) {
                    return Err(ConfigError::HeatingOutputShared(id, valve));
                }
            }
            // Two pads on one output would each switch it from its own dead band.
            for (other, g) in self.heating.iter().skip(id.index() + 1) {
                if g.is_fitted() && g.outputs.intersects(h.outputs) {
                    return Err(ConfigError::HeatingOutputSharedWithHeater(id, other));
                }
            }
        }

        for (id, s) in self.sensors.iter() {
            if s.kind == SensorKind::None {
                continue;
            }
            // A slot on an I2C kind without a bus reads nothing at all while looking configured —
            // the same silent-failure shape as an armed relief loop pointing at an unfitted
            // valve. An NTC is on one of the board's own pins and has no bus to set, so the
            // check follows the kind rather than applying to everything.
            if s.kind.is_on_i2c() && s.bus.is_none() {
                return Err(ConfigError::SensorBusUnset(id));
            }
            // Two slots on one channel means one of them silently never reaches the bus.
            for (other, t) in self.sensors.iter().skip(id.index() + 1) {
                if s.pdo_channel.is_some() && s.pdo_channel == t.pdo_channel {
                    return Err(ConfigError::PdoChannelShared(id, other));
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
    /// Relief is armed against a valve that is not fitted.
    ReliefValveUnmapped(ValveId),
    /// Relief is armed against a sensor slot that is not configured.
    ReliefSensorUnmapped(SensorSlot),
    ReliefPulseZero,
    /// A valve's heating is switched on but has no output, or no sensor slot, or neither.
    HeatingUnfitted(ValveId),
    /// A valve's heating watches a slot with nothing configured on it.
    HeatingSensorUnmapped(ValveId, SensorSlot),
    /// A valve's heating watches a slot that reports something other than a temperature.
    HeatingSensorUnit(ValveId, SensorSlot),

    /// A heating pad and a valve share an output.
    HeatingOutputShared(ValveId, ValveId),
    /// Two heating pads share an output.
    HeatingOutputSharedWithHeater(ValveId, ValveId),
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

    /// The heating pad a node mapping actually writes: a valve, a thermistor slot beside it, and
    /// a pad on a free output. All three have to agree before the config is accepted.
    fn heated_valve() -> Config {
        use crate::index::{AnalogInput, HcoId};
        Config::new()
            .with_valve(ValveId::Valve0, ValveConfig::solenoid_on(HcoId::Hco0))
            .with_quiet_sensor(SensorSlot::Slot1, SensorSlotConfig::ntc(AnalogInput::Com5Pin1))
            .with_valve_heating(ValveId::Valve0, ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000))
    }

    #[test]
    fn a_heated_valve_configures_cleanly() {
        let cfg = heated_valve();
        assert!(cfg.sanity_check().is_ok());
        assert!(cfg.heating[ValveId::Valve0].is_armed());
        assert!(!cfg.heating[ValveId::Valve1].is_fitted(), "and the other valves keep no pad");
    }

    /// Switched on with half the wiring is the silent-failure shape: it looks configured on the
    /// bus and does nothing at all.
    #[test]
    fn heating_switched_on_but_unfitted_is_refused() {
        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve0].sensor = None;
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::HeatingUnfitted(ValveId::Valve0))));
    }

    /// A thermostat pointed at a pressure slot would hold a valve at "30 hundredths of a bar" —
    /// and would still switch a real output while doing it.
    #[test]
    fn heating_must_watch_a_temperature() {
        use crate::index::AmplifierId;
        let mut cfg = heated_valve();
        cfg.sensors[SensorSlot::Slot1] =
            SensorSlotConfig::pressure(I2cBus::Bus0, AmplifierId::Amp0, Unit::CentiBar, SensorCalib::UNITY);
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::HeatingSensorUnit(ValveId::Valve0, SensorSlot::Slot1))));

        // ...and at a slot that exists at all.
        let mut cfg = heated_valve();
        cfg.sensors[SensorSlot::Slot1] = SensorSlotConfig::unused();
        assert!(matches!(
            cfg.sanity_check(),
            Err(ConfigError::HeatingSensorUnmapped(ValveId::Valve0, SensorSlot::Slot1))
        ));
    }

    /// The pad owns its outputs, so the config is where the conflict is settled — not the control
    /// tick, where a valve command and a thermostat would take turns winning.
    #[test]
    fn a_pad_and_a_valve_may_not_share_an_output() {
        use crate::index::HcoId;
        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve0] = cfg.heating[ValveId::Valve0].also_on(HcoId::Hco0); // the valve's own output
        assert!(matches!(cfg.sanity_check(), Err(ConfigError::HeatingOutputShared(ValveId::Valve0, ValveId::Valve0))));

        // Two pads on one output is the same mistake between two heaters.
        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve1] = ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000);
        assert!(matches!(
            cfg.sanity_check(),
            Err(ConfigError::HeatingOutputSharedWithHeater(ValveId::Valve0, ValveId::Valve1))
        ));

        // Overlapping in one output of several is still sharing it.
        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve1] = ValveHeatingConfig::new(HcoId::Hco3, SensorSlot::Slot1, 3_000).also_on(HcoId::Hco2);
        assert!(matches!(
            cfg.sanity_check(),
            Err(ConfigError::HeatingOutputSharedWithHeater(ValveId::Valve0, ValveId::Valve1))
        ));
    }

    /// Several pads on one thermistor, either as one thermostat or as independent ones: the slot
    /// is only read, so sharing it is fine where sharing an output is not.
    #[test]
    fn several_pads_may_follow_one_sensor() {
        use crate::index::HcoId;
        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve0] = cfg.heating[ValveId::Valve0].also_on(HcoId::Hco3);
        assert!(cfg.sanity_check().is_ok(), "{:?}", cfg.sanity_check());

        let mut cfg = heated_valve();
        cfg.heating[ValveId::Valve1] = ValveHeatingConfig::new(HcoId::Hco3, SensorSlot::Slot1, 2_000);
        assert!(cfg.sanity_check().is_ok(), "{:?}", cfg.sanity_check());
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

    /// The exception to the check above: an NTC is on one of the board's own pins, so requiring
    /// a bus of it would make a perfectly wired sensor unconfigurable.
    #[test]
    fn an_ntc_slot_is_accepted_without_a_bus() {
        use crate::index::AnalogInput;
        let cfg = Config::new().with_sensor(SensorSlot::Slot0, SensorSlotConfig::ntc(AnalogInput::Com6Pin1));
        assert!(cfg.sanity_check().is_ok());
        let used = cfg.analog_inputs_used();
        assert!(used[AnalogInput::Com6Pin1]);
        assert!(!used[AnalogInput::Com5Pin1], "only the pin a slot actually names is sampled");
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
