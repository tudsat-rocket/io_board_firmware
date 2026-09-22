//! The object dictionary: the one piece of shared state in this firmware.
//!
//! Every task reads and writes here and nowhere else. In particular the SDO server never touches
//! hardware — it parks a request in the store and raises a flag, and the control task
//! ([`crate::control`]) is the sole owner of the high current outputs. That is what makes the
//! valve state machine authoritative: nothing can move an output behind its back.
//!
//! The dictionary itself — which index means what, in what unit, with which coded values — is
//! defined in [`iocan_proto::od`], re-exported below as [`od`]. What lives here is the *state*
//! those indices name and the read/write arms that serve them. Adding an object means adding a
//! constant there, a summary line in `device-conf/can-io.toml` so `zencan-build` can generate the
//! EDS, and an arm in each direction here.
//!
//! Locking discipline: hold [`STORE`] for a short, `await`-free critical section. Never call into
//! I2C, SPI or CAN while holding it.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use zencan_common::sdo::AbortCode;

use crate::config::{Config, SensorKind, SensorSlotConfig, Unit, ValveKind};
use crate::index::{
    AmplifierId, HcoId, I2cBus, Id, PdoSensorChannel, PerAdcSlot, PerErrorCounter, PerHco, PerI2cBus, PerPdoSensor,
    PerRail, PerSensorSlot, PerStepper, PerValve, SensorSlot, StepperId, ValveId,
};
use crate::valves::position_of;

pub static STORE: Mutex<CriticalSectionRawMutex, Store> = Mutex::new(Store::new());

/// Raised whenever a write lands that the control task must act on promptly: a valve command, a
/// direct output write, a config change, or a save/restore request. Cheaper and more responsive
/// than making the control task poll at its tick rate.
pub static CONTROL_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Raised for the persistence task on a save (0x1010) or restore (0x1011) request.
///
/// A separate signal from [`CONTROL_WAKE`] because an `embassy_sync` `Signal` only ever wakes one
/// waiter — sharing one between the control and persistence tasks would lose wakeups.
pub static PERSIST_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Object dictionary indices and the wire sentinels that go with them. Defined in
/// [`iocan_proto::od`], which is where an object's meaning is documented; re-exported here
/// because the whole firmware reaches for them through the store.
pub use iocan_proto::od;
pub use iocan_proto::od::{NO_INDEX, RAW_INVALID, SENSOR_INVALID, SIGNATURE_LOAD, SIGNATURE_SAVE, TEMPERATURE_INVALID};

/// 0x2032. How the node currently sees the master.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum LinkState {
    /// No master heartbeat since boot. The fallback timers still run from boot, so a node that
    /// never hears a master ends up in the same safe state as one that lost it.
    NeverSeen = 0,
    Alive = 1,
    FallbackA = 2,
    FallbackB = 3,
    /// Raw debug mode is on, so the fallback machinery is deliberately suspended.
    Suspended = 4,
}

/// Requests parked by the SDO server for the control task.
#[derive(Clone, Copy, Default, Debug)]
pub struct Pending {
    /// Set for each valve whose commanded position changed. Per valve rather than one flag
    /// because the control task has to know *which* valve, so a fresh command takes back exactly
    /// that valve's outputs from a raw debug override.
    pub valves: PerValve<bool>,
    /// A direct HCO write landed and needs arbitrating.
    pub outputs: bool,
    /// 0x2018 was written. Only a wake-up: the control task reads the mode every tick anyway.
    pub heater: bool,
    /// Config changed; mappings and derived state need recomputing.
    pub config: bool,
    /// 0x1010 was written with the save signature.
    pub save: bool,
    /// 0x1011 was written with the load signature.
    pub restore: bool,
    /// 0x2016 was written: declare that actuator's shaft to be at this step count without moving
    /// it. `Option` rather than a flag plus a value because "re-zero to 0" is a real request and
    /// must not be indistinguishable from "no request".
    pub stepper_zero: PerStepper<Option<i32>>,
}

impl Pending {
    pub fn take(&mut self) -> Self {
        core::mem::take(self)
    }

    pub fn any(&self) -> bool {
        self.valves.any()
            || self.outputs
            || self.heater
            || self.config
            || self.save
            || self.restore
            || self.stepper_zero.values().any(Option::is_some)
    }
}

pub struct Store {
    // --- 0x2000 process data ------------------------------------------------
    /// Raw conversion results, one per probe-able amplifier position.
    pub raw_adc: PerAdcSlot<u16>,
    /// Raw AS5600 angle per bus, [`RAW_INVALID`] when no encoder answered.
    pub raw_angle: PerI2cBus<u16>,
    pub i2c_present: PerI2cBus<u16>,
    pub i2c_sweeps: u32,
    pub sensor_value: PerSensorSlot<i16>,
    pub sensor_unit: PerSensorSlot<u8>,
    /// The subset of `sensor_value` that goes out as process data, gathered by channel.
    ///
    /// Derived from `sensor_value` and the config's channel assignment, not written to directly.
    /// It exists so the TPDO builder is a straight window over an array rather than a search
    /// through sixteen slots for the one that claimed a channel — that search would run once per
    /// frame per channel, on a 10 ms tick.
    pub pdo_sensor_value: PerPdoSensor<i16>,
    pub pdo_sensor_unit: PerPdoSensor<u8>,

    pub valve_commanded: PerValve<u16>,
    pub valve_target: PerValve<u16>,
    pub valve_measured: PerValve<u16>,
    pub valve_status: PerValve<u8>,
    pub valve_current_ma: PerValve<u16>,
    /// 0x2015, a [`crate::relief::ReliefState`] discriminant.
    pub relief_state: u8,
    /// 0x2016. Pulses each clock/direction actuator has emitted since boot, direction-signed.
    /// Reported as well as commanded because it is the only observable that says where a shaft
    /// really is — 0x2012 clamps it into the configured travel, this does not.
    pub stepper_position_steps: PerStepper<i32>,
    /// 0x2017: heater NTC temperature (m°C), its raw ADC counts, and a
    /// [`crate::heater::HeaterState`] discriminant.
    pub heater_milli_c: i32,
    pub heater_raw: u16,
    pub heater_state: u8,
    /// 0x2018, a [`crate::heater::HeaterMode`] discriminant. Off at every boot; the control task
    /// writes it back to off when fallback stage B fires.
    pub heater_mode: u8,
    /// Whether this build has a heater at all. Set once at boot, from [`crate::config::NodeSettings`];
    /// 0x2018 is refused without one.
    pub heater_fitted: bool,

    pub hco_digital: PerHco<u8>,
    pub hco_pwm_us: PerHco<u16>,
    /// Owning valve of each output, 1-indexed with 0 for "unowned" — the wire encoding of 0x2022.
    pub hco_owner: PerHco<u8>,
    /// Set for each output with a direct write waiting for the control task. The control task
    /// needs to know *which* output was written, not just that one was, so that a raw debug
    /// override lands on that output alone.
    pub hco_direct_dirty: PerHco<bool>,
    /// Whether the last direct write to output i asked for PWM (rather than a digital level).
    pub hco_direct_pwm: PerHco<bool>,

    pub leds: u8,
    /// 0x2031. Volatile by design: cleared by every reset, never persisted.
    pub raw_debug: bool,
    pub link_state: LinkState,
    pub ms_since_heartbeat: u32,

    /// Zero on rev2, which has no on-board sensing.
    pub rail_current_ma: PerRail<u16>,
    pub rail_voltage_mv: PerRail<u16>,

    /// Mirror of [`crate::errors`]'s atomics, refreshed on the control tick.
    ///
    /// A copy rather than a read-through because the atomics are bumped from places that cannot
    /// take the store's lock — an I2C error path, the CAN receive loop, the panic handler. See
    /// [`Self::refresh_error_counters`].
    pub error_counts: PerErrorCounter<u32>,

    // --- 0x3000 runtime config ----------------------------------------------
    pub config: Config,

    // --- not on the wire ----------------------------------------------------
    pub pending: Pending,
}

impl Store {
    pub const fn new() -> Self {
        Self {
            raw_adc: PerAdcSlot::splat(RAW_INVALID),
            raw_angle: PerI2cBus::splat(RAW_INVALID),
            i2c_present: PerI2cBus::splat(0),
            i2c_sweeps: 0,
            sensor_value: PerSensorSlot::splat(SENSOR_INVALID),
            sensor_unit: PerSensorSlot::splat(0),
            pdo_sensor_value: PerPdoSensor::splat(SENSOR_INVALID),
            pdo_sensor_unit: PerPdoSensor::splat(0),
            valve_commanded: PerValve::splat(0),
            valve_target: PerValve::splat(0),
            valve_measured: PerValve::splat(0),
            valve_status: PerValve::splat(0),
            valve_current_ma: PerValve::splat(0),
            relief_state: crate::relief::ReliefState::Disabled as u8,
            stepper_position_steps: PerStepper::splat(0),
            heater_milli_c: TEMPERATURE_INVALID,
            heater_raw: RAW_INVALID,
            heater_state: crate::heater::HeaterState::Disabled as u8,
            heater_mode: crate::heater::HeaterMode::Off as u8,
            heater_fitted: false,
            hco_digital: PerHco::splat(0),
            hco_pwm_us: PerHco::splat(0),
            hco_owner: PerHco::splat(0),
            hco_direct_dirty: PerHco::splat(false),
            hco_direct_pwm: PerHco::splat(false),
            leds: 0,
            raw_debug: false,
            link_state: LinkState::NeverSeen,
            ms_since_heartbeat: 0,
            rail_current_ma: PerRail::splat(0),
            rail_voltage_mv: PerRail::splat(0),
            error_counts: PerErrorCounter::splat(0),
            config: Config::new(),
            pending: Pending {
                valves: PerValve::splat(false),
                outputs: false,
                heater: false,
                config: false,
                save: false,
                restore: false,
                stepper_zero: PerStepper::splat(None),
            },
        }
    }

    /// Recompute everything derived from `config`. Call after any config change.
    pub fn refresh_derived(&mut self) {
        for (hco, owner) in self.hco_owner.iter_mut() {
            // 1-indexed on the wire, 0 for "no valve owns this output".
            *owner = self.config.hco_owner(hco).map_or(0, |v| v.as_u8() + 1);
        }
        for (slot, unit) in self.sensor_unit.iter_mut() {
            *unit = self.config.sensors[slot].unit as u8;
        }
        self.refresh_pdo_sensors();
    }

    /// Copy [`crate::errors`]'s counters in, if any of them has moved since the last call.
    ///
    /// Called from the control tick, which already holds the lock to write its own observations —
    /// so on a healthy board this costs one atomic load and nothing else, and on an unhealthy one
    /// it costs a whole array copy at the tick rate. Neither is worth a task of its own.
    pub fn refresh_error_counters(&mut self) {
        if let Some(counts) = crate::errors::take_if_changed() {
            self.error_counts = counts;
        }
    }

    /// Gather the slots that claim a TPDO channel into the by-channel arrays the broadcaster
    /// reads. Called after a config change *and* after every sensor sample, since the values
    /// move even when the assignment does not.
    ///
    /// A channel no slot claims keeps [`SENSOR_INVALID`], which is the same thing a listener
    /// already sees for a slot that has no reading — so an unmapped channel needs no new
    /// encoding to say "nothing here".
    pub fn refresh_pdo_sensors(&mut self) {
        self.pdo_sensor_value = PerPdoSensor::splat(SENSOR_INVALID);
        self.pdo_sensor_unit = PerPdoSensor::splat(0);
        for (slot, cfg) in self.config.sensors.iter() {
            if let Some(channel) = cfg.pdo_channel {
                self.pdo_sensor_value[channel] = self.sensor_value[slot];
                self.pdo_sensor_unit[channel] = cfg.unit as u8;
            }
        }
    }
}

impl Default for Store {
    fn default() -> Self {
        Self::new()
    }
}

/// A value read out of the dictionary, sized for an expedited SDO payload.
#[derive(Clone, Copy, Debug)]
pub struct OdValue {
    pub bytes: [u8; 4],
    pub len: u8,
}

impl OdValue {
    pub fn u8(v: u8) -> Self {
        Self {
            bytes: [v, 0, 0, 0],
            len: 1,
        }
    }

    pub fn u16(v: u16) -> Self {
        let b = v.to_le_bytes();
        Self {
            bytes: [b[0], b[1], 0, 0],
            len: 2,
        }
    }

    pub fn i16(v: i16) -> Self {
        Self::u16(v as u16)
    }

    pub fn u32(v: u32) -> Self {
        Self {
            bytes: v.to_le_bytes(),
            len: 4,
        }
    }

    pub fn i32(v: i32) -> Self {
        Self::u32(v as u32)
    }

    pub fn data(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------
//
// CANopen array convention: sub 0 is the entry count as a u8, subs 1..=N are the elements. We
// keep it because the point of borrowing SDO framing is that ordinary CANopen tooling works.

/// Index an array object, mapping sub 0 to the count and rejecting anything past the end.
fn read_array<T: Copy, F: Fn(T) -> OdValue>(arr: &[T], sub: u8, to_value: F) -> Result<OdValue, AbortCode> {
    if sub == 0 {
        return Ok(OdValue::u8(arr.len() as u8));
    }
    arr.get(sub as usize - 1).copied().map(to_value).ok_or(AbortCode::NoSuchSubIndex)
}

/// Read a valve config field across all four valves as if it were an array object.
fn read_valve_array<F: Fn(&crate::config::ValveConfig) -> OdValue>(
    cfg: &Config,
    sub: u8,
    field: F,
) -> Result<OdValue, AbortCode> {
    read_array(cfg.valves.as_slice(), sub, |v| field(&v))
}

fn read_stepper_array<F: Fn(&crate::stepper::StepperConfig) -> OdValue>(
    cfg: &Config,
    sub: u8,
    field: F,
) -> Result<OdValue, AbortCode> {
    read_array(cfg.steppers.as_slice(), sub, |s| field(&s))
}

fn read_sensor_array<F: Fn(&SensorSlotConfig) -> OdValue>(
    cfg: &Config,
    sub: u8,
    field: F,
) -> Result<OdValue, AbortCode> {
    read_array(cfg.sensors.as_slice(), sub, |s| field(&s))
}

/// Read one object. Errors are CANopen abort codes so the SDO server can pass them straight back.
pub fn read(store: &Store, index: u16, sub: u8) -> Result<OdValue, AbortCode> {
    use od::*;
    let cfg = &store.config;

    // Scalar objects take no sub-index other than 0.
    let scalar = |v: OdValue| -> Result<OdValue, AbortCode> {
        if sub <= 1 {
            Ok(v)
        } else {
            Err(AbortCode::NoSuchSubIndex)
        }
    };

    match index {
        // Command objects. Reading them back tells you nothing, but a read must not abort.
        STORE_PARAMETERS | RESTORE_DEFAULTS => scalar(OdValue::u32(1)),
        HEARTBEAT_PERIOD => scalar(OdValue::u16(cfg.heartbeat_period_ms)),

        RAW_ADC_BUS0 => read_array(&store.raw_adc.as_slice()[..crate::config::NUM_AMPLIFIERS], sub, OdValue::u16),
        RAW_ADC_BUS1 => read_array(&store.raw_adc.as_slice()[crate::config::NUM_AMPLIFIERS..], sub, OdValue::u16),
        I2C_PRESENT => read_array(store.i2c_present.as_slice(), sub, OdValue::u16),
        I2C_SWEEPS => scalar(OdValue::u32(store.i2c_sweeps)),
        SENSOR_VALUE => read_array(store.sensor_value.as_slice(), sub, OdValue::i16),
        SENSOR_UNIT => read_array(store.sensor_unit.as_slice(), sub, OdValue::u8),
        RAW_ENCODER => read_array(store.raw_angle.as_slice(), sub, OdValue::u16),

        VALVE_COMMANDED => read_array(store.valve_commanded.as_slice(), sub, OdValue::u16),
        VALVE_TARGET => read_array(store.valve_target.as_slice(), sub, OdValue::u16),
        VALVE_MEASURED => read_array(store.valve_measured.as_slice(), sub, OdValue::u16),
        VALVE_STATUS => read_array(store.valve_status.as_slice(), sub, OdValue::u8),
        VALVE_CURRENT => read_array(store.valve_current_ma.as_slice(), sub, OdValue::u16),
        RELIEF_STATE => scalar(OdValue::u8(store.relief_state)),
        STEPPER_POSITION => read_array(store.stepper_position_steps.as_slice(), sub, OdValue::i32),
        HEATER => {
            read_array(&[store.heater_milli_c, store.heater_raw as i32, store.heater_state as i32], sub, OdValue::i32)
        }
        HEATER_MODE => scalar(OdValue::u8(store.heater_mode)),
        HEATER_SETPOINT => scalar(OdValue::i16(cfg.heater_setpoint_centi_c)),

        RELIEF_ENABLED => scalar(OdValue::u8(cfg.relief.enabled as u8)),
        RELIEF_VALVE => scalar(OdValue::u8(cfg.relief.valve.map_or(0xFF, ValveId::as_u8))),
        RELIEF_SENSOR => scalar(OdValue::u8(cfg.relief.sensor.as_u8())),
        RELIEF_THRESHOLD => scalar(OdValue::i16(cfg.relief.threshold)),
        RELIEF_POSITION => scalar(OdValue::u16(cfg.relief.position)),
        RELIEF_PULSE_MS => scalar(OdValue::u16(cfg.relief.pulse_ms)),
        RELIEF_COOLDOWN_MS => scalar(OdValue::u16(cfg.relief.cooldown_ms)),

        STEPPER_VALVE => read_stepper_array(cfg, sub, |s| OdValue::u8(s.valve.map_or(NO_INDEX, ValveId::as_u8))),
        STEPPER_CLOSED_STEPS => read_stepper_array(cfg, sub, |s| OdValue::i32(s.closed_steps)),
        STEPPER_OPEN_STEPS => read_stepper_array(cfg, sub, |s| OdValue::i32(s.open_steps)),
        STEPPER_MAX_HZ => read_stepper_array(cfg, sub, |s| OdValue::u32(s.max_step_hz)),
        STEPPER_START_HZ => read_stepper_array(cfg, sub, |s| OdValue::u32(s.start_step_hz)),
        STEPPER_ACCEL_HZ_PER_S => read_stepper_array(cfg, sub, |s| OdValue::u32(s.accel_hz_per_s)),

        HCO_DIGITAL => read_array(store.hco_digital.as_slice(), sub, OdValue::u8),
        HCO_PWM_US => read_array(store.hco_pwm_us.as_slice(), sub, OdValue::u16),
        HCO_OWNER => read_array(store.hco_owner.as_slice(), sub, OdValue::u8),

        LEDS => scalar(OdValue::u8(store.leds)),
        RAW_DEBUG_MODE => scalar(OdValue::u8(store.raw_debug as u8)),
        LINK_STATE => scalar(OdValue::u8(store.link_state as u8)),
        MS_SINCE_HEARTBEAT => scalar(OdValue::u32(store.ms_since_heartbeat)),
        RAIL_CURRENT => read_array(store.rail_current_ma.as_slice(), sub, OdValue::u16),
        RAIL_VOLTAGE => read_array(store.rail_voltage_mv.as_slice(), sub, OdValue::u16),
        ERROR_COUNTERS => read_array(store.error_counts.as_slice(), sub, OdValue::u32),

        MASTER_NODE_ID => scalar(OdValue::u8(cfg.master_node_id)),
        FALLBACK_A_MS => scalar(OdValue::u32(cfg.fallback_a_ms)),
        FALLBACK_B_MS => scalar(OdValue::u32(cfg.fallback_b_ms)),
        FALLBACK_ENABLED => scalar(OdValue::u8(cfg.fallback_enabled as u8)),
        FALLBACK_A_POSITION => read_valve_array(cfg, sub, |v| OdValue::u16(v.fallback_a.position)),
        FALLBACK_B_POSITION => read_valve_array(cfg, sub, |v| OdValue::u16(v.fallback_b.position)),
        FALLBACK_A_UNPOWER => read_valve_array(cfg, sub, |v| OdValue::u8(v.fallback_a.unpower as u8)),
        FALLBACK_B_UNPOWER => read_valve_array(cfg, sub, |v| OdValue::u8(v.fallback_b.unpower as u8)),

        VALVE_KIND => read_valve_array(cfg, sub, |v| OdValue::u8(v.kind as u8)),
        VALVE_POWER_HCO => read_valve_array(cfg, sub, |v| OdValue::u8(hco_to_wire(v.power_hco))),
        VALVE_SIGNAL_HCO => read_valve_array(cfg, sub, |v| OdValue::u8(hco_to_wire(v.signal_hco))),
        VALVE_CLOSED_US => read_valve_array(cfg, sub, |v| OdValue::u16(v.closed_us)),
        VALVE_OPEN_US => read_valve_array(cfg, sub, |v| OdValue::u16(v.open_us)),
        VALVE_TRAVEL_MS => read_valve_array(cfg, sub, |v| OdValue::u16(v.travel_ms)),
        VALVE_STALL_MA => read_valve_array(cfg, sub, |v| OdValue::u16(v.stall_ma)),
        VALVE_STALL_MS => read_valve_array(cfg, sub, |v| OdValue::u16(v.stall_ms)),
        VALVE_SETTLE_MS => read_valve_array(cfg, sub, |v| OdValue::u16(v.settle_ms)),
        VALVE_MIN_PROMILLE => read_valve_array(cfg, sub, |v| OdValue::u16(v.min_promille)),
        VALVE_MAX_PROMILLE => read_valve_array(cfg, sub, |v| OdValue::u16(v.max_promille)),
        VALVE_POSITION_SENSOR => {
            read_valve_array(cfg, sub, |v| OdValue::u8(v.position_sensor.map_or(NO_INDEX, SensorSlot::as_u8)))
        }

        SENSOR_BUS => read_sensor_array(cfg, sub, |s| OdValue::u8(s.bus.map_or(0xFF, I2cBus::as_u8))),
        SENSOR_AMPLIFIER => read_sensor_array(cfg, sub, |s| OdValue::u8(s.amplifier.as_u8())),
        SENSOR_KIND => read_sensor_array(cfg, sub, |s| OdValue::u8(s.kind as u8)),
        SENSOR_OFFSET => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.offset_milli)),
        SENSOR_SLOPE => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.slope_nano)),
        SENSOR_CONSTANT => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.constant_milli)),
        SENSOR_UNIT_CFG => read_sensor_array(cfg, sub, |s| OdValue::u8(s.unit as u8)),
        SENSOR_PDO_CHANNEL => {
            read_sensor_array(cfg, sub, |s| OdValue::u8(s.pdo_channel.map_or(NO_INDEX, PdoSensorChannel::as_u8)))
        }

        SENSOR_INTERVAL_MS => scalar(OdValue::u16(cfg.sensor_interval_ms)),
        SCAN_INTERVAL_MS => scalar(OdValue::u16(cfg.scan_interval_ms)),
        TPDO_INTERVAL_MS => read_array(cfg.tpdo_interval_ms.as_slice(), sub, OdValue::u16),

        _ => Err(AbortCode::NoSuchObject),
    }
}

/// [`HcoId`] is 0-indexed internally but 1-indexed on the wire with 0 meaning "none", so that the
/// wire form reads as the HCO number silkscreened on the board. This pair of functions is the only
/// place the two conventions meet.
fn hco_to_wire(hco: Option<HcoId>) -> u8 {
    hco.map_or(0, HcoId::silkscreen)
}

fn hco_from_wire(v: u8) -> Result<Option<HcoId>, AbortCode> {
    match v {
        0 => Ok(None),
        _ => HcoId::from_u8(v - 1).map(Some).ok_or(AbortCode::InvalidValue),
    }
}

/// Read an optional id off the wire, rejecting anything that is neither [`NO_INDEX`] nor a valid
/// index of the domain. The counterpart of `map_or(NO_INDEX, ..)` on the read side.
fn opt_id_from_wire<I: Id>(v: u8) -> Result<Option<I>, AbortCode> {
    match v {
        NO_INDEX => Ok(None),
        _ => I::from_index(v as usize).map(Some).ok_or(AbortCode::InvalidValue),
    }
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

fn as_u8(data: &[u8]) -> Result<u8, AbortCode> {
    match data.len() {
        1 => Ok(data[0]),
        0 => Err(AbortCode::DataTypeMismatchLengthLow),
        _ => Err(AbortCode::DataTypeMismatchLengthHigh),
    }
}

fn as_u16(data: &[u8]) -> Result<u16, AbortCode> {
    match data.len() {
        2 => Ok(u16::from_le_bytes([data[0], data[1]])),
        n if n < 2 => Err(AbortCode::DataTypeMismatchLengthLow),
        _ => Err(AbortCode::DataTypeMismatchLengthHigh),
    }
}

fn as_u32(data: &[u8]) -> Result<u32, AbortCode> {
    match data.len() {
        4 => Ok(u32::from_le_bytes(data.try_into().unwrap())),
        n if n < 4 => Err(AbortCode::DataTypeMismatchLengthLow),
        _ => Err(AbortCode::DataTypeMismatchLengthHigh),
    }
}

fn as_i32(data: &[u8]) -> Result<i32, AbortCode> {
    as_u32(data).map(|v| v as i32)
}

/// Resolve an array sub-index to an id of the addressed domain, rejecting sub 0 (the read-only
/// count) and anything past the end.
///
/// This is the boundary: every write below goes through it, so past this point the index is known
/// to be in range *and* known to belong to the right domain — `slot::<ValveId>` cannot be used to
/// subscript the HCO arrays even though both have four entries.
fn slot<I: Id>(sub: u8) -> Result<I, AbortCode> {
    if sub == 0 {
        return Err(AbortCode::ReadOnly);
    }
    I::from_index(sub as usize - 1).ok_or(AbortCode::NoSuchSubIndex)
}

/// A valve position word: 0..=1000 promille in bits 14..0, bit 15 to release the drive.
///
/// The flag is accepted and preserved; the promille field is still range-checked, so 0xFFFF —
/// which means "invalid" or "not connected" elsewhere on this vehicle — is rejected here rather
/// than being mistaken for a command.
fn position(data: &[u8]) -> Result<u16, AbortCode> {
    let v = as_u16(data)?;
    if position_of(v) > crate::config::PROMILLE_MAX {
        return Err(AbortCode::ValueTooHigh);
    }
    Ok(v)
}

/// A plain promille value, with no unpowered flag permitted.
///
/// Used for the configuration objects: whether a fallback releases a valve is a separate field
/// (0x3005/0x3006), and the input clamp has no business carrying a drive state.
fn promille(data: &[u8]) -> Result<u16, AbortCode> {
    let v = as_u16(data)?;
    if v > crate::config::PROMILLE_MAX {
        return Err(AbortCode::ValueTooHigh);
    }
    Ok(v)
}

fn expect_scalar(sub: u8) -> Result<(), AbortCode> {
    // CANopen scalars live at sub 0; some tools address them as sub 1. Accept both rather than
    // making an operator guess which one this node wants.
    if sub <= 1 {
        Ok(())
    } else {
        Err(AbortCode::NoSuchSubIndex)
    }
}

/// Guard direct writes to an output that a valve owns.
///
/// This is the arbitration rule the whole output model rests on: in normal operation a valve owns
/// its outputs outright, so the master cannot desynchronise the valve state machine by poking the
/// underlying PWM. Raw debug mode drops the guard on purpose, for servo travel testing on a bench
/// where the valve model is exactly what you are trying to bypass.
fn check_direct_access(store: &Store, hco: HcoId) -> Result<(), AbortCode> {
    if store.raw_debug {
        return Ok(());
    }
    match store.hco_owner[hco] {
        0 => Ok(()),
        owner => {
            defmt::warn!(
                "rejected direct write to hco{}: owned by valve {}. Enable raw debug mode (0x2031) to override.",
                hco.silkscreen(),
                owner - 1
            );
            Err(AbortCode::CantStoreLocalControl)
        }
    }
}

/// Write one object.
///
/// This validates and stores; it never drives hardware. Anything requiring action sets a flag in
/// [`Store::pending`], which the caller signals to the control task.
pub fn write(store: &mut Store, index: u16, sub: u8, data: &[u8]) -> Result<(), AbortCode> {
    use od::*;

    match index {
        STORE_PARAMETERS => {
            expect_scalar(sub)?;
            if as_u32(data)? != SIGNATURE_SAVE {
                return Err(AbortCode::InvalidValue);
            }
            store.pending.save = true;
        }
        RESTORE_DEFAULTS => {
            expect_scalar(sub)?;
            if as_u32(data)? != SIGNATURE_LOAD {
                return Err(AbortCode::InvalidValue);
            }
            store.pending.restore = true;
        }
        HEARTBEAT_PERIOD => {
            expect_scalar(sub)?;
            store.config.heartbeat_period_ms = as_u16(data)?;
            store.pending.config = true;
        }

        VALVE_COMMANDED => {
            let i: ValveId = slot(sub)?;
            // Commanding an unfitted valve is a wiring or configuration mistake worth surfacing,
            // not something to silently accept.
            if !store.config.valves[i].is_mapped() {
                return Err(AbortCode::ResourceNotAvailable);
            }
            store.valve_commanded[i] = position(data)?;
            store.pending.valves[i] = true;
        }

        HEATER_MODE => {
            expect_scalar(sub)?;
            if !store.heater_fitted {
                return Err(AbortCode::ResourceNotAvailable);
            }
            let mode = as_u8(data)?;
            crate::heater::HeaterMode::from_u8(mode).ok_or(AbortCode::InvalidValue)?;
            store.heater_mode = mode;
            store.pending.heater = true;
        }

        HCO_DIGITAL => {
            let i: HcoId = slot(sub)?;
            check_direct_access(store, i)?;
            store.hco_digital[i] = (as_u8(data)? != 0) as u8;
            store.hco_direct_pwm[i] = false;
            store.hco_direct_dirty[i] = true;
            store.pending.outputs = true;
        }
        HCO_PWM_US => {
            let i: HcoId = slot(sub)?;
            check_direct_access(store, i)?;
            store.hco_pwm_us[i] = as_u16(data)?;
            store.hco_direct_pwm[i] = true;
            store.hco_direct_dirty[i] = true;
            store.pending.outputs = true;
        }

        // Not a motion command: this says where the shaft already is, so that the step counter
        // and the physical actuator agree again after a hand-turn or a power cycle. See
        // `crate::stepper` on why there is no other homing.
        STEPPER_POSITION => {
            let i: StepperId = slot(sub)?;
            if !store.config.steppers[i].is_mapped() {
                return Err(AbortCode::ResourceNotAvailable);
            }
            let steps = as_i32(data)?;
            store.stepper_position_steps[i] = steps;
            store.pending.stepper_zero[i] = Some(steps);
        }

        LEDS => {
            expect_scalar(sub)?;
            store.leds = as_u8(data)?;
        }
        RAW_DEBUG_MODE => {
            expect_scalar(sub)?;
            let on = as_u8(data)? != 0;
            if on != store.raw_debug {
                defmt::warn!(
                    "raw debug mode {}: hco arbitration is last-writer-wins and the heartbeat fallback is suspended",
                    if on { "ON" } else { "off" }
                );
            }
            store.raw_debug = on;
            store.pending.outputs = true;
        }

        MASTER_NODE_ID => {
            expect_scalar(sub)?;
            let id = as_u8(data)?;
            if id > 0x0F {
                return Err(AbortCode::ValueTooHigh);
            }
            store.config.master_node_id = id;
            store.pending.config = true;
        }
        FALLBACK_A_MS => {
            expect_scalar(sub)?;
            store.config.fallback_a_ms = as_u32(data)?;
            store.pending.config = true;
        }
        FALLBACK_B_MS => {
            expect_scalar(sub)?;
            store.config.fallback_b_ms = as_u32(data)?;
            store.pending.config = true;
        }
        FALLBACK_ENABLED => {
            expect_scalar(sub)?;
            store.config.fallback_enabled = as_u8(data)? != 0;
            store.pending.config = true;
        }
        FALLBACK_A_POSITION => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].fallback_a.position = promille(data)?;
            store.pending.config = true;
        }
        FALLBACK_B_POSITION => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].fallback_b.position = promille(data)?;
            store.pending.config = true;
        }
        FALLBACK_A_UNPOWER => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].fallback_a.unpower = as_u8(data)? != 0;
            store.pending.config = true;
        }
        FALLBACK_B_UNPOWER => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].fallback_b.unpower = as_u8(data)? != 0;
            store.pending.config = true;
        }

        VALVE_KIND => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].kind = ValveKind::from_u8(as_u8(data)?).ok_or(AbortCode::InvalidValue)?;
            store.pending.config = true;
        }
        VALVE_POWER_HCO => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].power_hco = hco_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        VALVE_SIGNAL_HCO => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].signal_hco = hco_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        VALVE_CLOSED_US => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].closed_us = as_u16(data)?;
            store.pending.config = true;
        }
        VALVE_OPEN_US => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].open_us = as_u16(data)?;
            store.pending.config = true;
        }
        VALVE_TRAVEL_MS => {
            let i: ValveId = slot(sub)?;
            let v = as_u16(data)?;
            if v == 0 {
                return Err(AbortCode::ValueTooLow);
            }
            store.config.valves[i].travel_ms = v;
            store.pending.config = true;
        }
        VALVE_STALL_MA => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].stall_ma = as_u16(data)?;
            store.pending.config = true;
        }
        VALVE_STALL_MS => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].stall_ms = as_u16(data)?;
            store.pending.config = true;
        }
        VALVE_SETTLE_MS => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].settle_ms = as_u16(data)?;
            store.pending.config = true;
        }
        VALVE_MIN_PROMILLE => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].min_promille = promille(data)?;
            store.pending.config = true;
        }
        VALVE_MAX_PROMILLE => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].max_promille = promille(data)?;
            store.pending.config = true;
        }
        VALVE_POSITION_SENSOR => {
            let i: ValveId = slot(sub)?;
            store.config.valves[i].position_sensor = opt_id_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }

        SENSOR_BUS => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].bus = opt_id_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        SENSOR_AMPLIFIER => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].amplifier = AmplifierId::from_u8(as_u8(data)?).ok_or(AbortCode::ValueTooHigh)?;
            store.pending.config = true;
        }
        SENSOR_KIND => {
            let i: SensorSlot = slot(sub)?;
            let kind = SensorKind::from_u8(as_u8(data)?).ok_or(AbortCode::InvalidValue)?;
            let cfg = &mut store.config.sensors[i];
            // Changing the kind invalidates everything downstream of it: a slope in nanobar per
            // count means nothing to a Pt1000, and a unit of centibar means nothing to either.
            // So a kind change re-seeds both from the new kind — which for a Pt1000 or an
            // MCP9700 is a complete, working calibration, and for the kinds whose curve is
            // per-installation is the uncalibrated `ZERO` that reports no reading until the
            // coefficients arrive. Either way the slot is never left describing itself with the
            // previous kind's numbers.
            //
            // Write the kind first and the coefficients after; the reverse order loses them.
            if cfg.kind != kind {
                cfg.kind = kind;
                cfg.calib = kind.default_calib();
                cfg.unit = kind.natural_unit();
            }
            store.pending.config = true;
        }
        SENSOR_OFFSET => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.offset_milli = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_SLOPE => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.slope_nano = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_CONSTANT => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.constant_milli = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_UNIT_CFG => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].unit = Unit::from_u8(as_u8(data)?).ok_or(AbortCode::InvalidValue)?;
            store.pending.config = true;
        }
        SENSOR_PDO_CHANNEL => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].pdo_channel = opt_id_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        SENSOR_INTERVAL_MS => {
            expect_scalar(sub)?;
            let v = as_u16(data)?;
            if v == 0 {
                return Err(AbortCode::ValueTooLow);
            }
            store.config.sensor_interval_ms = v;
            store.pending.config = true;
        }
        SCAN_INTERVAL_MS => {
            expect_scalar(sub)?;
            store.config.scan_interval_ms = as_u16(data)?;
            store.pending.config = true;
        }
        TPDO_INTERVAL_MS => {
            let i: iocan_proto::TpdoKind = slot(sub)?;
            store.config.tpdo_interval_ms[i] = as_u16(data)?;
            store.pending.config = true;
        }

        STEPPER_VALVE => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].valve = opt_id_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        STEPPER_CLOSED_STEPS => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].closed_steps = as_i32(data)?;
            store.pending.config = true;
        }
        STEPPER_OPEN_STEPS => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].open_steps = as_i32(data)?;
            store.pending.config = true;
        }
        STEPPER_MAX_HZ => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].max_step_hz = as_u32(data)?;
            store.pending.config = true;
        }
        STEPPER_START_HZ => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].start_step_hz = as_u32(data)?;
            store.pending.config = true;
        }
        STEPPER_ACCEL_HZ_PER_S => {
            let i: StepperId = slot(sub)?;
            store.config.steppers[i].accel_hz_per_s = as_u32(data)?;
            store.pending.config = true;
        }

        RELIEF_ENABLED => {
            expect_scalar(sub)?;
            store.config.relief.enabled = as_u8(data)? != 0;
            store.pending.config = true;
        }
        RELIEF_VALVE => {
            expect_scalar(sub)?;
            store.config.relief.valve = opt_id_from_wire(as_u8(data)?)?;
            store.pending.config = true;
        }
        RELIEF_SENSOR => {
            expect_scalar(sub)?;
            store.config.relief.sensor = SensorSlot::from_u8(as_u8(data)?).ok_or(AbortCode::ValueTooHigh)?;
            store.pending.config = true;
        }
        RELIEF_THRESHOLD => {
            expect_scalar(sub)?;
            store.config.relief.threshold = as_u16(data)? as i16;
            store.pending.config = true;
        }
        RELIEF_POSITION => {
            expect_scalar(sub)?;
            store.config.relief.position = promille(data)?;
            store.pending.config = true;
        }
        RELIEF_PULSE_MS => {
            expect_scalar(sub)?;
            let v = as_u16(data)?;
            if v == 0 {
                return Err(AbortCode::ValueTooLow);
            }
            store.config.relief.pulse_ms = v;
            store.pending.config = true;
        }
        RELIEF_COOLDOWN_MS => {
            expect_scalar(sub)?;
            store.config.relief.cooldown_ms = as_u16(data)?;
            store.pending.config = true;
        }

        HEATER_SETPOINT => {
            expect_scalar(sub)?;
            let v = as_u16(data)? as i16;
            if v > iocan_proto::od::HEATER_SETPOINT_MAX {
                return Err(AbortCode::ValueTooHigh);
            }
            store.config.heater_setpoint_centi_c = v;
            store.pending.config = true;
        }

        // Everything else in the 0x2000 block is process data we produce.
        RAW_ADC_BUS0 | RAW_ADC_BUS1 | RAW_ENCODER | I2C_PRESENT | I2C_SWEEPS | SENSOR_VALUE | SENSOR_UNIT
        | VALVE_TARGET | VALVE_MEASURED | VALVE_STATUS | VALVE_CURRENT | RELIEF_STATE | HEATER | HCO_OWNER
        | LINK_STATE | MS_SINCE_HEARTBEAT | RAIL_CURRENT | RAIL_VOLTAGE => return Err(AbortCode::ReadOnly),

        _ => return Err(AbortCode::NoSuchObject),
    }

    if store.pending.config {
        store.refresh_derived();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{NUM_VALVES, ValveConfig};
    use crate::index::HcoPair;

    fn store_with_servo() -> Store {
        let mut s = Store::new();
        s.config.valves[ValveId::Valve0] = ValveConfig::servo_on_pair(HcoPair::A, 2000, 1000, 1000);
        s.refresh_derived();
        s
    }

    #[test]
    fn the_heater_mode_starts_off_and_needs_a_heater() {
        let mut s = Store::new();
        assert_eq!(read(&s, od::HEATER_MODE, 0).unwrap().data(), &[0], "off at boot");
        assert!(matches!(write(&mut s, od::HEATER_MODE, 0, &[1]), Err(AbortCode::ResourceNotAvailable)));

        s.heater_fitted = true;
        write(&mut s, od::HEATER_MODE, 0, &[2]).unwrap();
        assert_eq!(read(&s, od::HEATER_MODE, 0).unwrap().data(), &[2]);
        assert!(s.pending.heater, "a mode change has to wake the control task");
        assert!(matches!(write(&mut s, od::HEATER_MODE, 0, &[3]), Err(AbortCode::InvalidValue)));
        assert_eq!(s.heater_mode, 2, "a rejected write leaves the mode alone");
    }

    #[test]
    fn the_heater_setpoint_is_capped() {
        let mut s = Store::new();
        write(&mut s, od::HEATER_SETPOINT, 0, &od::HEATER_SETPOINT_MAX.to_le_bytes()).unwrap();
        assert!(matches!(
            write(&mut s, od::HEATER_SETPOINT, 0, &(od::HEATER_SETPOINT_MAX + 1).to_le_bytes()),
            Err(AbortCode::ValueTooHigh)
        ));
        write(&mut s, od::HEATER_SETPOINT, 0, &(-500i16).to_le_bytes()).unwrap();
        assert_eq!(read(&s, od::HEATER_SETPOINT, 0).unwrap().data(), &(-500i16).to_le_bytes());
        assert!(s.pending.config, "the setpoint is configuration, saved with 0x1010");
    }

    #[test]
    fn valve_owns_its_outputs() {
        let s = store_with_servo();
        // HCO1 (power) and HCO2 (signal) both belong to valve 0, reported 1-indexed.
        assert_eq!(s.hco_owner.as_array(), &[1, 1, 0, 0]);
    }

    #[test]
    fn direct_write_to_owned_output_is_rejected() {
        let mut s = store_with_servo();
        assert!(matches!(
            write(&mut s, od::HCO_PWM_US, 1, &1500u16.to_le_bytes()),
            Err(AbortCode::CantStoreLocalControl)
        ));
        // ...and permitted on an output no valve claims.
        assert!(write(&mut s, od::HCO_PWM_US, 3, &1500u16.to_le_bytes()).is_ok());
    }

    #[test]
    fn raw_debug_mode_lifts_the_guard() {
        let mut s = store_with_servo();
        write(&mut s, od::RAW_DEBUG_MODE, 0, &[1]).unwrap();
        assert!(write(&mut s, od::HCO_PWM_US, 1, &1500u16.to_le_bytes()).is_ok());
    }

    #[test]
    fn commanding_an_unmapped_valve_is_rejected() {
        let mut s = store_with_servo();
        assert!(matches!(
            write(&mut s, od::VALVE_COMMANDED, 2, &500u16.to_le_bytes()),
            Err(AbortCode::ResourceNotAvailable)
        ));
        assert!(write(&mut s, od::VALVE_COMMANDED, 1, &500u16.to_le_bytes()).is_ok());
    }

    #[test]
    fn out_of_range_promille_is_rejected() {
        let mut s = store_with_servo();
        assert!(matches!(write(&mut s, od::VALVE_COMMANDED, 1, &1001u16.to_le_bytes()), Err(AbortCode::ValueTooHigh)));
    }

    #[test]
    fn the_unpowered_flag_is_accepted_and_preserved() {
        use crate::valves::unpowered_at;
        let mut s = store_with_servo();
        let word = unpowered_at(250);
        write(&mut s, od::VALVE_COMMANDED, 1, &word.to_le_bytes()).unwrap();
        assert_eq!(s.valve_commanded[ValveId::Valve0], word);
    }

    #[test]
    fn a_flagged_word_still_has_its_promille_checked() {
        use crate::valves::UNPOWERED_FLAG;
        let mut s = store_with_servo();
        let bad = UNPOWERED_FLAG | 1001;
        assert!(matches!(write(&mut s, od::VALVE_COMMANDED, 1, &bad.to_le_bytes()), Err(AbortCode::ValueTooHigh)));
    }

    /// 0xFFFF means "invalid" or "not connected" elsewhere on this vehicle. It must never be
    /// mistaken for a valve command here, which the promille range check guarantees.
    #[test]
    fn all_ones_is_not_a_valid_command() {
        let mut s = store_with_servo();
        assert!(matches!(
            write(&mut s, od::VALVE_COMMANDED, 1, &0xFFFFu16.to_le_bytes()),
            Err(AbortCode::ValueTooHigh)
        ));
    }

    #[test]
    fn config_positions_reject_the_unpowered_flag() {
        use crate::valves::unpowered_at;
        let mut s = store_with_servo();
        // Whether a fallback releases the valve is 0x3005/0x3006, not a bit smuggled into the
        // position, so the flag is not accepted here.
        assert!(matches!(
            write(&mut s, od::FALLBACK_A_POSITION, 1, &unpowered_at(0).to_le_bytes()),
            Err(AbortCode::ValueTooHigh)
        ));
        assert!(write(&mut s, od::FALLBACK_A_POSITION, 1, &500u16.to_le_bytes()).is_ok());
    }

    #[test]
    fn process_data_is_read_only() {
        let mut s = store_with_servo();
        assert!(matches!(write(&mut s, od::VALVE_MEASURED, 1, &0u16.to_le_bytes()), Err(AbortCode::ReadOnly)));
    }

    #[test]
    fn array_sub_zero_reads_the_count() {
        let s = store_with_servo();
        assert_eq!(read(&s, od::VALVE_COMMANDED, 0).unwrap().data(), &[NUM_VALVES as u8]);
    }

    #[test]
    fn reads_past_the_end_abort() {
        let s = store_with_servo();
        assert!(matches!(read(&s, od::VALVE_COMMANDED, 5), Err(AbortCode::NoSuchSubIndex)));
    }

    /// Writing nothing but the kind has to leave a Pt1000 that actually reads a temperature.
    ///
    /// It did not, briefly: the bridge inversion moved behind the per-slot calibration, and a
    /// virgin slot's zero slope turned every reading into 0.00 degC. A master configuring a slot
    /// from scratch over the bus — rather than from `zenith_mapping`, whose constructors set the
    /// calibration for it — would have got a confident, wrong, plausible number.
    #[test]
    fn writing_only_the_kind_gives_a_working_pt1000() {
        use crate::config::SensorKind;
        use crate::sensors::calibrate;

        let mut s = Store::new();
        let sub = SensorSlot::Slot0.as_u8() + 1;
        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Pt1000 as u8]).unwrap();
        write(&mut s, od::SENSOR_BUS, sub, &[I2cBus::Bus0.as_u8()]).unwrap();

        let cfg = &s.config.sensors[SensorSlot::Slot0];
        assert_eq!(cfg.unit, Unit::CentiCelsius, "the kind brings its natural unit with it");
        assert_eq!(calibrate(cfg, Some(600)), 848, "and a working bridge, in centicelsius");
    }

    /// The counterpart: a transducer's curve is not knowable from its kind, so the slot stays
    /// silent rather than inventing one, and starts reporting the moment a slope arrives.
    #[test]
    fn writing_only_the_kind_leaves_a_transducer_uncalibrated() {
        use crate::config::SensorKind;
        use crate::sensors::calibrate;

        let mut s = Store::new();
        let sub = SensorSlot::Slot0.as_u8() + 1;
        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Pressure as u8]).unwrap();
        write(&mut s, od::SENSOR_BUS, sub, &[I2cBus::Bus0.as_u8()]).unwrap();
        assert_eq!(calibrate(&s.config.sensors[SensorSlot::Slot0], Some(600)), SENSOR_INVALID);

        // 0.1 bar per count, no offset and no constant: 600 counts is 60 bar.
        write(&mut s, od::SENSOR_SLOPE, sub, &100_000_000i32.to_le_bytes()).unwrap();
        assert_eq!(calibrate(&s.config.sensors[SensorSlot::Slot0], Some(600)), 6000);
    }

    /// A kind change re-seeds the calibration, because the old one described a different sensor.
    /// Re-writing the *same* kind must not, or a master that resends its whole config would wipe
    /// the calibration it just wrote.
    #[test]
    fn a_kind_change_reseeds_the_calibration_but_a_repeat_does_not() {
        use crate::config::SensorKind;

        let mut s = Store::new();
        let sub = SensorSlot::Slot0.as_u8() + 1;
        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Pressure as u8]).unwrap();
        write(&mut s, od::SENSOR_SLOPE, sub, &123_456_789i32.to_le_bytes()).unwrap();
        write(&mut s, od::SENSOR_OFFSET, sub, &47_000i32.to_le_bytes()).unwrap();

        // Same kind again: a no-op, so the operator's coefficients survive.
        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Pressure as u8]).unwrap();
        assert_eq!(s.config.sensors[SensorSlot::Slot0].calib.slope_nano, 123_456_789);
        assert_eq!(s.config.sensors[SensorSlot::Slot0].calib.offset_milli, 47_000);

        // A different kind: nanobar per count is meaningless to a Pt1000, so it goes.
        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Pt1000 as u8]).unwrap();
        assert_eq!(s.config.sensors[SensorSlot::Slot0].calib, SensorKind::Pt1000.default_calib());
        assert_eq!(s.config.sensors[SensorSlot::Slot0].unit, Unit::CentiCelsius);
    }

    /// The calibration plane an operator actually works through: pick the kind, point it at a
    /// device, write three coefficients, choose whether it goes on the bus.
    #[test]
    fn a_sensor_slot_is_configurable_end_to_end_over_sdo() {
        use crate::config::SensorKind;
        use crate::index::PdoSensorChannel;

        let mut s = Store::new();
        let sub = SensorSlot::Slot12.as_u8() + 1;

        write(&mut s, od::SENSOR_KIND, sub, &[SensorKind::Angle as u8]).unwrap();
        write(&mut s, od::SENSOR_BUS, sub, &[I2cBus::Bus1.as_u8()]).unwrap();
        write(&mut s, od::SENSOR_UNIT_CFG, sub, &[Unit::Promille as u8]).unwrap();
        write(&mut s, od::SENSOR_OFFSET, sub, &1_124_000i32.to_le_bytes()).unwrap();
        write(&mut s, od::SENSOR_SLOPE, sub, &(-976_563i32).to_le_bytes()).unwrap();
        write(&mut s, od::SENSOR_CONSTANT, sub, &0i32.to_le_bytes()).unwrap();
        write(&mut s, od::SENSOR_PDO_CHANNEL, sub, &[PdoSensorChannel::Ch9.as_u8()]).unwrap();

        let cfg = &s.config.sensors[SensorSlot::Slot12];
        assert_eq!(cfg.kind, SensorKind::Angle);
        assert_eq!(cfg.calib.slope_nano, -976_563, "a reversed valve's negative slope survives the wire");
        assert_eq!(cfg.pdo_channel, Some(PdoSensorChannel::Ch9));
        // ...and reads back as what was written.
        assert_eq!(read(&s, od::SENSOR_PDO_CHANNEL, sub).unwrap().data(), &[PdoSensorChannel::Ch9.as_u8()]);
        assert_eq!(read(&s, od::SENSOR_SLOPE, sub).unwrap().data(), &(-976_563i32).to_le_bytes());
    }

    /// Slot 12 exists at all only because there are sixteen now; the wire has to agree.
    #[test]
    fn the_sensor_arrays_are_sixteen_long() {
        let s = store_with_servo();
        assert_eq!(read(&s, od::SENSOR_VALUE, 0).unwrap().data(), &[crate::config::NUM_SENSOR_SLOTS as u8]);
        assert!(read(&s, od::SENSOR_KIND, 16).is_ok());
        assert!(matches!(read(&s, od::SENSOR_KIND, 17), Err(AbortCode::NoSuchSubIndex)));
    }

    /// `NO_INDEX` is the "unset" sentinel for everything but an HCO, whose wire form is 1-indexed
    /// to match the silkscreen. Zero has to stay a real slot on both objects.
    #[test]
    fn an_optional_slot_reference_round_trips_through_its_sentinel() {
        let mut s = store_with_servo();

        write(&mut s, od::VALVE_POSITION_SENSOR, 1, &[NO_INDEX]).unwrap();
        assert_eq!(s.config.valves[ValveId::Valve0].position_sensor, None);
        assert_eq!(read(&s, od::VALVE_POSITION_SENSOR, 1).unwrap().data(), &[NO_INDEX]);

        write(&mut s, od::VALVE_POSITION_SENSOR, 1, &[0]).unwrap();
        assert_eq!(s.config.valves[ValveId::Valve0].position_sensor, Some(SensorSlot::Slot0));

        assert!(matches!(write(&mut s, od::VALVE_POSITION_SENSOR, 1, &[16]), Err(AbortCode::InvalidValue)));
    }

    /// A slot's value goes out on the channel it claimed, not on its own number — and the derived
    /// arrays have to be rebuilt whenever the assignment changes, not just when a sample lands.
    #[test]
    fn changing_a_channel_assignment_regathers_the_pdo_arrays() {
        use crate::index::PdoSensorChannel;

        let mut s = Store::new();
        s.config.sensors[SensorSlot::Slot11] = SensorSlotConfig::pt1000(I2cBus::Bus0, AmplifierId::Amp0);
        s.sensor_value[SensorSlot::Slot11] = 2350;
        s.refresh_derived();
        assert_eq!(s.pdo_sensor_value[PdoSensorChannel::Ch0], SENSOR_INVALID, "nothing claims channel 0 yet");

        write(&mut s, od::SENSOR_PDO_CHANNEL, SensorSlot::Slot11.as_u8() + 1, &[PdoSensorChannel::Ch0.as_u8()])
            .unwrap();

        assert_eq!(s.pdo_sensor_value[PdoSensorChannel::Ch0], 2350);
        assert_eq!(s.pdo_sensor_unit[PdoSensorChannel::Ch0], Unit::CentiCelsius as u8);
    }

    #[test]
    fn the_raw_encoder_angles_are_readable_and_read_only() {
        let mut s = Store::new();
        s.raw_angle[I2cBus::Bus1] = 2048;
        assert_eq!(read(&s, od::RAW_ENCODER, 2).unwrap().data(), &2048u16.to_le_bytes());
        assert!(matches!(write(&mut s, od::RAW_ENCODER, 2, &0u16.to_le_bytes()), Err(AbortCode::ReadOnly)));
    }

    #[test]
    fn wrong_payload_width_aborts() {
        let mut s = store_with_servo();
        assert!(matches!(write(&mut s, od::VALVE_COMMANDED, 1, &[0]), Err(AbortCode::DataTypeMismatchLengthLow)));
    }
}
