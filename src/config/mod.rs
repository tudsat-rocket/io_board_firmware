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

pub mod persist;

use crate::index::{
    AdcSlot, AmplifierId, HcoId, HcoPair, I2cBus, PerAmplifier, PerSensorSlot, PerTpdoKind, PerValve, SensorSlot,
    ValveId,
};

/// Sizes of the fixed domains. Each is the count of the matching id type in [`crate::index`] —
/// kept as plain constants only for the places that genuinely want a number (a CANopen array's
/// entry count, a log line), never as the basis for an index.
pub const NUM_VALVES: usize = ValveId::COUNT;
pub const NUM_HCO: usize = HcoId::COUNT;
pub const NUM_SENSOR_SLOTS: usize = SensorSlot::COUNT;
pub const NUM_I2C_BUSES: usize = I2cBus::COUNT;

/// ADC101C027 amplifier addresses, in scan order. Everything that talks about an "amplifier
/// index" means an [`AmplifierId`], never a raw I2C address — the index is what travels over CAN,
/// so that a 9-entry bitmap fits one u16 per bus.
pub const AMPLIFIER_ADDRESSES: PerAmplifier<u8> = PerAmplifier::new([
    0b101_0000, // floating, floating
    0b101_0001, // floating, gnd
    0b101_0010, // floating, vcc
    0b101_0100, // gnd, floating
    0b101_0101, // gnd, gnd
    0b101_0110, // gnd, vcc
    0b101_1000, // vcc, floating
    0b101_1001, // vcc, gnd
    0b101_1010, // vcc, vcc
]);

pub const NUM_AMPLIFIERS: usize = AmplifierId::COUNT;
/// Every probe-able amplifier slot on the board: both buses, all nine straps.
pub const NUM_ADC_SLOTS: usize = AdcSlot::COUNT;

/// Number of fixed TPDO kinds. Defined in `iocan-proto` (the wire protocol crate) so `TpdoKind`
/// and this array size can never drift apart; re-exported here since so much of the object
/// dictionary (0x3040's `array_size` among it) is sized against it.
pub use iocan_proto::ids::NUM_TPDO_KINDS;

/// A valve position, 0 = fully closed, 1000 = fully open.
pub const PROMILLE_MAX: u16 = 1000;

#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum ValveKind {
    /// No valve fitted on this slot. Commands to it are rejected.
    None = 0,
    /// On/off coil on a single output. Any non-zero promille energises it.
    Solenoid = 1,
    /// Hobby-style servo on a PWM output, optionally with a separate power output that lets us
    /// take it to [`crate::valves::ValveStatus::Unpowered`].
    Servo = 2,
}

impl ValveKind {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::None),
            1 => Some(Self::Solenoid),
            2 => Some(Self::Servo),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum SensorKind {
    None = 0,
    /// Linear pressure transducer through an instrumentation amplifier.
    Pressure = 1,
    /// Pt1000 RTD in a Wheatstone bridge, fixed conversion.
    Pt1000 = 2,
}

impl SensorKind {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::None),
            1 => Some(Self::Pressure),
            2 => Some(Self::Pt1000),
            _ => None,
        }
    }
}

/// How to scale a slot's physical value into the signed 16-bit number that goes on the bus.
///
/// A fixed unit cannot serve every sensor: a 400 bar transducer overflows i16 centibar, while
/// centibar is the natural resolution for a 40 bar one. So each slot declares its own, and the
/// codes are mirrored read-only into 0x2005 so a master can decode 0x2004 without reading config.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum Unit {
    /// 0.01 bar per count. Range +-327.67 bar.
    CentiBar = 0,
    /// 0.1 bar per count. For transducers above 300 bar.
    DeciBar = 1,
    /// 0.01 degrees Celsius per count.
    CentiCelsius = 2,
    /// Uncalibrated ADC counts, passed through. Useful while calibrating.
    RawCounts = 3,
}

impl Unit {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::CentiBar),
            1 => Some(Self::DeciBar),
            2 => Some(Self::CentiCelsius),
            3 => Some(Self::RawCounts),
            _ => None,
        }
    }
}

/// What a valve should do when a fallback stage fires.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct FallbackAction {
    pub position: u16,
    /// Drop the power output once the position is reached and the settle time has elapsed.
    pub unpower: bool,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ValveConfig {
    pub kind: ValveKind,
    /// High current output that powers the valve, if it has a separate one.
    pub power_hco: Option<HcoId>,
    /// High current output carrying the signal: PWM for a servo, the coil for a solenoid. A valve
    /// with no signal output is effectively unmapped.
    pub signal_hco: Option<HcoId>,
    pub closed_us: u16,
    pub open_us: u16,
    /// Time for a full 0 -> 1000 promille sweep, used to estimate measured position and to set
    /// the settle deadline.
    pub travel_ms: u16,
    /// Rail current above which a moving valve counts as stalled. 0 disables stall detection,
    /// which is also the only correct setting on rev2 (no on-board current sensing).
    pub stall_ma: u16,
    pub stall_ms: u16,
    /// How long to keep driving after arriving before an unpower is allowed.
    pub settle_ms: u16,
    pub min_promille: u16,
    pub max_promille: u16,
    pub fallback_a: FallbackAction,
    pub fallback_b: FallbackAction,
}

impl ValveConfig {
    pub const fn unmapped() -> Self {
        Self {
            kind: ValveKind::None,
            power_hco: None,
            signal_hco: None,
            closed_us: 2000,
            open_us: 1000,
            travel_ms: 1000,
            stall_ma: 0,
            stall_ms: 500,
            settle_ms: 500,
            min_promille: 0,
            max_promille: PROMILLE_MAX,
            fallback_a: FallbackAction {
                position: 0,
                unpower: true,
            },
            fallback_b: FallbackAction {
                position: PROMILLE_MAX,
                unpower: true,
            },
        }
    }

    /// A servo on an HCO pair wired the way the vehicle harness does it: the lower output of the
    /// pair carries power, the upper one carries the signal. Which output is which is
    /// [`HcoPair`]'s to say, so the `pair * 2` / `pair * 2 + 1` arithmetic no longer appears here.
    pub const fn servo_on_pair(pair: HcoPair, closed_us: u16, open_us: u16, travel_ms: u16) -> Self {
        Self {
            kind: ValveKind::Servo,
            power_hco: Some(pair.power()),
            signal_hco: Some(pair.signal()),
            closed_us,
            open_us,
            travel_ms,
            ..Self::unmapped()
        }
    }

    pub const fn solenoid_on(hco: HcoId) -> Self {
        Self {
            kind: ValveKind::Solenoid,
            power_hco: None,
            signal_hco: Some(hco),
            ..Self::unmapped()
        }
    }

    /// Linear interpolation from promille open to servo pulse width.
    ///
    /// Correct when `open_us < closed_us`, which is the common case here: several of the vehicle
    /// valves open counter-clockwise.
    pub fn pulse_width_us(&self, promille: u16) -> u16 {
        let promille = promille.min(PROMILLE_MAX) as i32;
        let closed = self.closed_us as i32;
        let delta = self.open_us as i32 - closed;
        (closed + (delta * promille) / PROMILLE_MAX as i32) as u16
    }

    pub fn clamp(&self, promille: u16) -> u16 {
        promille.min(PROMILLE_MAX).clamp(self.min_promille, self.max_promille.min(PROMILLE_MAX))
    }

    pub fn is_mapped(&self) -> bool {
        self.kind != ValveKind::None && self.signal_hco.is_some()
    }
}

/// Fixed-point linear calibration for one pressure transducer.
///
/// The bench calibration for these sensors is defined as
///
/// ```text
///   pressure_bar = (adc_reading - offset) * linear_factor
/// ```
///
/// which is what [`PressureCalib::from_bar_per_count`] expresses. Some transducers are instead
/// characterised against ambient and want a constant added afterwards (1.013 bar, to report
/// absolute rather than gauge pressure); that is [`PressureCalib::with_constant_bar`]. Keeping
/// the constant as a configurable term rather than baking one in means both conventions are
/// expressible, and which one a slot uses is visible in its calibration rather than implied by
/// the firmware version.
///
/// Kept in integers on purpose. The STM32F105 is a Cortex-M3 without an FPU, so a float in the
/// hot sensor path pulls the soft-float runtime into a tight flash budget. The human-readable
/// constants stay floats in [`crate::zenith_mapping::sensors`] and are folded down by the `const
/// fn` constructors at compile time.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct PressureCalib {
    /// Zero offset in milli ADC counts.
    pub offset_milli_counts: i32,
    /// Slope in nanobar per ADC count.
    pub slope_nanobar: i32,
    /// Constant added after the linear part, in millibar. Zero for the plain
    /// `(reading - offset) * factor` form.
    pub constant_millibar: i32,
}

impl PressureCalib {
    /// `pressure_bar = (adc_reading - offset) * linear_factor`.
    ///
    /// `const`, so the float arithmetic happens in the compiler and never reaches the target.
    pub const fn from_bar_per_count(offset_counts: f32, bar_per_count: f32) -> Self {
        Self {
            offset_milli_counts: (offset_counts * 1000.0) as i32,
            slope_nanobar: (bar_per_count * 1_000_000_000.0) as i32,
            constant_millibar: 0,
        }
    }

    /// Add a constant term: `pressure_bar = (reading - offset) * factor + constant`.
    ///
    /// Use `1.013` for a transducer calibrated against ambient that should report absolute
    /// pressure.
    pub const fn with_constant_bar(self, bar: f32) -> Self {
        Self {
            constant_millibar: (bar * 1000.0) as i32,
            ..self
        }
    }

    pub const ZERO: Self = Self {
        offset_milli_counts: 0,
        slope_nanobar: 0,
        constant_millibar: 0,
    };

    /// Apply the calibration to a raw 10-bit conversion result.
    ///
    /// `millibar = (raw * 1000 - offset) * slope / 1e9 + constant`. Widest intermediate is about
    /// 9e14, which is why this is i64.
    pub fn to_millibar(&self, raw: u16) -> i32 {
        let counts = raw as i64 * 1000 - self.offset_milli_counts as i64;
        let millibar = (counts * self.slope_nanobar as i64) / 1_000_000_000 + self.constant_millibar as i64;
        millibar.clamp(i32::MIN as i64, i32::MAX as i64) as i32
    }
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct SensorSlotConfig {
    pub kind: SensorKind,
    /// Which I2C bus, or `None` for an unused slot.
    pub bus: Option<I2cBus>,
    /// Which address strap, i.e. which entry of [`AMPLIFIER_ADDRESSES`].
    pub amplifier: AmplifierId,
    pub unit: Unit,
    pub calib: PressureCalib,
}

impl SensorSlotConfig {
    pub const fn unused() -> Self {
        Self {
            kind: SensorKind::None,
            bus: None,
            amplifier: AmplifierId::Amp0,
            unit: Unit::CentiBar,
            calib: PressureCalib::ZERO,
        }
    }

    pub const fn pressure(bus: I2cBus, amplifier: AmplifierId, unit: Unit, calib: PressureCalib) -> Self {
        Self {
            kind: SensorKind::Pressure,
            bus: Some(bus),
            amplifier,
            unit,
            calib,
        }
    }

    pub const fn pt1000(bus: I2cBus, amplifier: AmplifierId) -> Self {
        Self {
            kind: SensorKind::Pt1000,
            bus: Some(bus),
            amplifier,
            unit: Unit::CentiCelsius,
            calib: PressureCalib::ZERO,
        }
    }

    /// Which probe-able amplifier position this slot reads, or `None` when it is unused.
    ///
    /// Both halves are already known-in-range, so unlike the old flat-index version there is no
    /// bounds check left to get wrong — the only remaining question is whether a bus is set.
    pub const fn adc_slot(&self) -> Option<AdcSlot> {
        match self.bus {
            Some(bus) => Some(AdcSlot::new(bus, self.amplifier)),
            None => None,
        }
    }
}

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

    pub const fn with_sensor(mut self, slot: SensorSlot, config: SensorSlotConfig) -> Self {
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
            // Two valves sharing an output would fight each other every control tick.
            for (other, w) in self.valves.iter().skip(id.index() + 1) {
                if w.is_mapped() && shares_output(v, w) {
                    return Err(ConfigError::OutputShared(id, other));
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

fn shares_output(a: &ValveConfig, b: &ValveConfig) -> bool {
    let a_outs = [a.signal_hco, a.power_hco];
    let b_outs = [b.signal_hco, b.power_hco];
    a_outs.iter().flatten().any(|x| b_outs.iter().flatten().any(|y| x == y))
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Local overpressure relief.
///
/// At most one loop per node. because more is not required at the moment and I'm lazy.
/// See [`crate::relief`] for the state machine.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ReliefConfig {
    pub enabled: bool,
    /// Valve to open. `None` disables the loop regardless of `enabled`.
    pub valve: Option<ValveId>,
    /// The sensor slot to watch.
    pub sensor: SensorSlot,
    /// Open when the reading goes strictly above this, in that slot's own unit (0x2005) — so a
    /// slot reporting centibar takes 6000 for 60 bar.
    pub threshold: i16,
    /// How far to open while relieving, promille.
    pub position: u16,
    pub pulse_ms: u16,
    /// Settling time after a pulse before the threshold is looked at again.
    pub cooldown_ms: u16,
}

impl ReliefConfig {
    /// Off, and with a threshold that cannot be reached — so a node that has never been
    /// configured for relief cannot start venting because some unrelated slot reads high.
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            valve: None,
            sensor: SensorSlot::Slot0,
            threshold: i16::MAX,
            position: PROMILLE_MAX,
            pulse_ms: 500,
            cooldown_ms: 500,
        }
    }

    /// Watch `sensor` and pulse `valve` open when it goes above `threshold`, in the sensor's unit.
    pub const fn new(valve: ValveId, sensor: SensorSlot, threshold: i16) -> Self {
        Self {
            enabled: true,
            valve: Some(valve),
            sensor,
            threshold,
            ..Self::disabled()
        }
    }

    pub const fn with_pulse_ms(mut self, pulse_ms: u16) -> Self {
        self.pulse_ms = pulse_ms;
        self
    }

    pub const fn with_cooldown_ms(mut self, cooldown_ms: u16) -> Self {
        self.cooldown_ms = cooldown_ms;
        self
    }

    pub fn is_armed(&self) -> bool {
        self.enabled && self.valve.is_some()
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
    /// Relief is armed against a valve that is not fitted.
    ReliefValveUnmapped(ValveId),
    /// Relief is armed against a sensor slot that is not configured.
    ReliefSensorUnmapped(SensorSlot),
    ReliefPulseZero,
}
