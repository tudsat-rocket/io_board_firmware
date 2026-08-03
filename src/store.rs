//! The object dictionary: the one piece of shared state in this firmware.
//!
//! Every task reads and writes here and nowhere else. In particular the SDO server never touches
//! hardware — it parks a request in the store and raises a flag, and the control task
//! ([`crate::control`]) is the sole owner of the high current outputs. That is what makes the
//! valve state machine authoritative: nothing can move an output behind its back.
//!
//! The index layout mirrors `device-conf/can-io.toml` exactly. When you add an object, add it in
//! both places.
//!
//! Locking discipline: hold [`STORE`] for a short, `await`-free critical section. Never call into
//! I2C, SPI or CAN while holding it.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use zencan_common::sdo::AbortCode;

use crate::config::{Config, SensorKind, SensorSlotConfig, Unit, ValveKind};
use crate::index::{
    AmplifierId, HcoId, I2cBus, Id, PerAdcSlot, PerHco, PerI2cBus, PerRail, PerSensorSlot, PerValve, SensorSlot,
    ValveId,
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

/// A raw ADC slot that did not answer during the last sweep.
pub const RAW_INVALID: u16 = u16::MAX;
/// A sensor slot with no usable reading.
pub const SENSOR_INVALID: i16 = i16::MIN;

/// Magic values for 0x1010 / 0x1011, as CANopen defines them: ASCII, little-endian.
pub const SIGNATURE_SAVE: u32 = 0x6576_6173; // "save"
pub const SIGNATURE_LOAD: u32 = 0x6461_6F6C; // "load"

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
    /// Config changed; mappings and derived state need recomputing.
    pub config: bool,
    /// 0x1010 was written with the save signature.
    pub save: bool,
    /// 0x1011 was written with the load signature.
    pub restore: bool,
}

impl Pending {
    pub fn take(&mut self) -> Self {
        core::mem::take(self)
    }

    pub fn any(&self) -> bool {
        self.valves.any() || self.outputs || self.config || self.save || self.restore
    }
}

pub struct Store {
    // --- 0x2000 process data ------------------------------------------------
    /// Raw conversion results, one per probe-able amplifier position.
    pub raw_adc: PerAdcSlot<u16>,
    pub i2c_present: PerI2cBus<u16>,
    pub i2c_sweeps: u32,
    pub sensor_value: PerSensorSlot<i16>,
    pub sensor_unit: PerSensorSlot<u8>,

    pub valve_commanded: PerValve<u16>,
    pub valve_target: PerValve<u16>,
    pub valve_measured: PerValve<u16>,
    pub valve_status: PerValve<u8>,
    pub valve_current_ma: PerValve<u16>,
    /// 0x2015, a [`crate::relief::ReliefState`] discriminant.
    pub relief_state: u8,

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

    // --- 0x3000 runtime config ----------------------------------------------
    pub config: Config,

    // --- not on the wire ----------------------------------------------------
    pub pending: Pending,
}

impl Store {
    pub const fn new() -> Self {
        Self {
            raw_adc: PerAdcSlot::splat(RAW_INVALID),
            i2c_present: PerI2cBus::splat(0),
            i2c_sweeps: 0,
            sensor_value: PerSensorSlot::splat(SENSOR_INVALID),
            sensor_unit: PerSensorSlot::splat(0),
            valve_commanded: PerValve::splat(0),
            valve_target: PerValve::splat(0),
            valve_measured: PerValve::splat(0),
            valve_status: PerValve::splat(0),
            valve_current_ma: PerValve::splat(0),
            relief_state: crate::relief::ReliefState::Disabled as u8,
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
            config: Config::new(),
            pending: Pending {
                valves: PerValve::splat(false),
                outputs: false,
                config: false,
                save: false,
                restore: false,
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
    }
}

impl Default for Store {
    fn default() -> Self {
        Self::new()
    }
}

/// Object dictionary indices. Mirrors `device-conf/can-io.toml`.
pub mod od {
    pub const STORE_PARAMETERS: u16 = 0x1010;
    pub const RESTORE_DEFAULTS: u16 = 0x1011;
    pub const HEARTBEAT_PERIOD: u16 = 0x1017;

    pub const RAW_ADC_BUS0: u16 = 0x2000;
    pub const RAW_ADC_BUS1: u16 = 0x2001;
    pub const I2C_PRESENT: u16 = 0x2002;
    pub const I2C_SWEEPS: u16 = 0x2003;
    pub const SENSOR_VALUE: u16 = 0x2004;
    pub const SENSOR_UNIT: u16 = 0x2005;

    pub const VALVE_COMMANDED: u16 = 0x2010;
    pub const VALVE_TARGET: u16 = 0x2011;
    pub const VALVE_MEASURED: u16 = 0x2012;
    pub const VALVE_STATUS: u16 = 0x2013;
    pub const VALVE_CURRENT: u16 = 0x2014;
    pub const RELIEF_STATE: u16 = 0x2015;

    pub const HCO_DIGITAL: u16 = 0x2020;
    pub const HCO_PWM_US: u16 = 0x2021;
    pub const HCO_OWNER: u16 = 0x2022;

    pub const LEDS: u16 = 0x2030;
    pub const RAW_DEBUG_MODE: u16 = 0x2031;
    pub const LINK_STATE: u16 = 0x2032;
    pub const MS_SINCE_HEARTBEAT: u16 = 0x2033;
    pub const RAIL_CURRENT: u16 = 0x2040;
    pub const RAIL_VOLTAGE: u16 = 0x2041;

    pub const MASTER_NODE_ID: u16 = 0x3000;
    pub const FALLBACK_A_MS: u16 = 0x3001;
    pub const FALLBACK_B_MS: u16 = 0x3002;
    pub const FALLBACK_A_POSITION: u16 = 0x3003;
    pub const FALLBACK_B_POSITION: u16 = 0x3004;
    pub const FALLBACK_A_UNPOWER: u16 = 0x3005;
    pub const FALLBACK_B_UNPOWER: u16 = 0x3006;
    pub const FALLBACK_ENABLED: u16 = 0x3007;

    pub const VALVE_KIND: u16 = 0x3010;
    pub const VALVE_POWER_HCO: u16 = 0x3011;
    pub const VALVE_SIGNAL_HCO: u16 = 0x3012;
    pub const VALVE_CLOSED_US: u16 = 0x3013;
    pub const VALVE_OPEN_US: u16 = 0x3014;
    pub const VALVE_TRAVEL_MS: u16 = 0x3015;
    pub const VALVE_STALL_MA: u16 = 0x3016;
    pub const VALVE_STALL_MS: u16 = 0x3017;
    pub const VALVE_SETTLE_MS: u16 = 0x3018;
    pub const VALVE_MIN_PROMILLE: u16 = 0x3019;
    pub const VALVE_MAX_PROMILLE: u16 = 0x301A;

    pub const SENSOR_BUS: u16 = 0x3020;
    pub const SENSOR_AMPLIFIER: u16 = 0x3021;
    pub const SENSOR_KIND: u16 = 0x3022;
    pub const SENSOR_OFFSET: u16 = 0x3023;
    pub const SENSOR_SLOPE: u16 = 0x3024;
    pub const SENSOR_UNIT_CFG: u16 = 0x3025;
    pub const SENSOR_CONSTANT: u16 = 0x3026;

    pub const SENSOR_INTERVAL_MS: u16 = 0x3030;
    pub const SCAN_INTERVAL_MS: u16 = 0x3031;
    pub const TPDO_INTERVAL_MS: u16 = 0x3040;

    pub const RELIEF_ENABLED: u16 = 0x3050;
    pub const RELIEF_VALVE: u16 = 0x3051;
    pub const RELIEF_SENSOR: u16 = 0x3052;
    pub const RELIEF_THRESHOLD: u16 = 0x3053;
    pub const RELIEF_POSITION: u16 = 0x3054;
    pub const RELIEF_PULSE_MS: u16 = 0x3055;
    pub const RELIEF_COOLDOWN_MS: u16 = 0x3056;
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

        VALVE_COMMANDED => read_array(store.valve_commanded.as_slice(), sub, OdValue::u16),
        VALVE_TARGET => read_array(store.valve_target.as_slice(), sub, OdValue::u16),
        VALVE_MEASURED => read_array(store.valve_measured.as_slice(), sub, OdValue::u16),
        VALVE_STATUS => read_array(store.valve_status.as_slice(), sub, OdValue::u8),
        VALVE_CURRENT => read_array(store.valve_current_ma.as_slice(), sub, OdValue::u16),
        RELIEF_STATE => scalar(OdValue::u8(store.relief_state)),

        RELIEF_ENABLED => scalar(OdValue::u8(cfg.relief.enabled as u8)),
        RELIEF_VALVE => scalar(OdValue::u8(cfg.relief.valve.map_or(0xFF, ValveId::as_u8))),
        RELIEF_SENSOR => scalar(OdValue::u8(cfg.relief.sensor.as_u8())),
        RELIEF_THRESHOLD => scalar(OdValue::i16(cfg.relief.threshold)),
        RELIEF_POSITION => scalar(OdValue::u16(cfg.relief.position)),
        RELIEF_PULSE_MS => scalar(OdValue::u16(cfg.relief.pulse_ms)),
        RELIEF_COOLDOWN_MS => scalar(OdValue::u16(cfg.relief.cooldown_ms)),

        HCO_DIGITAL => read_array(store.hco_digital.as_slice(), sub, OdValue::u8),
        HCO_PWM_US => read_array(store.hco_pwm_us.as_slice(), sub, OdValue::u16),
        HCO_OWNER => read_array(store.hco_owner.as_slice(), sub, OdValue::u8),

        LEDS => scalar(OdValue::u8(store.leds)),
        RAW_DEBUG_MODE => scalar(OdValue::u8(store.raw_debug as u8)),
        LINK_STATE => scalar(OdValue::u8(store.link_state as u8)),
        MS_SINCE_HEARTBEAT => scalar(OdValue::u32(store.ms_since_heartbeat)),
        RAIL_CURRENT => read_array(store.rail_current_ma.as_slice(), sub, OdValue::u16),
        RAIL_VOLTAGE => read_array(store.rail_voltage_mv.as_slice(), sub, OdValue::u16),

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

        SENSOR_BUS => read_sensor_array(cfg, sub, |s| OdValue::u8(s.bus.map_or(0xFF, I2cBus::as_u8))),
        SENSOR_AMPLIFIER => read_sensor_array(cfg, sub, |s| OdValue::u8(s.amplifier.as_u8())),
        SENSOR_KIND => read_sensor_array(cfg, sub, |s| OdValue::u8(s.kind as u8)),
        SENSOR_OFFSET => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.offset_milli_counts)),
        SENSOR_SLOPE => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.slope_nanobar)),
        SENSOR_CONSTANT => read_sensor_array(cfg, sub, |s| OdValue::i32(s.calib.constant_millibar)),
        SENSOR_UNIT_CFG => read_sensor_array(cfg, sub, |s| OdValue::u8(s.unit as u8)),

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

        SENSOR_BUS => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].bus = match as_u8(data)? {
                0xFF => None,
                b => Some(I2cBus::from_u8(b).ok_or(AbortCode::InvalidValue)?),
            };
            store.pending.config = true;
        }
        SENSOR_AMPLIFIER => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].amplifier = AmplifierId::from_u8(as_u8(data)?).ok_or(AbortCode::ValueTooHigh)?;
            store.pending.config = true;
        }
        SENSOR_KIND => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].kind = SensorKind::from_u8(as_u8(data)?).ok_or(AbortCode::InvalidValue)?;
            store.pending.config = true;
        }
        SENSOR_OFFSET => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.offset_milli_counts = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_SLOPE => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.slope_nanobar = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_CONSTANT => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].calib.constant_millibar = as_i32(data)?;
            store.pending.config = true;
        }
        SENSOR_UNIT_CFG => {
            let i: SensorSlot = slot(sub)?;
            store.config.sensors[i].unit = Unit::from_u8(as_u8(data)?).ok_or(AbortCode::InvalidValue)?;
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

        RELIEF_ENABLED => {
            expect_scalar(sub)?;
            store.config.relief.enabled = as_u8(data)? != 0;
            store.pending.config = true;
        }
        RELIEF_VALVE => {
            expect_scalar(sub)?;
            store.config.relief.valve = match as_u8(data)? {
                0xFF => None,
                v => Some(ValveId::from_u8(v).ok_or(AbortCode::InvalidValue)?),
            };
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

        // Everything else in the 0x2000 block is process data we produce.
        RAW_ADC_BUS0 | RAW_ADC_BUS1 | I2C_PRESENT | I2C_SWEEPS | SENSOR_VALUE | SENSOR_UNIT | VALVE_TARGET
        | VALVE_MEASURED | VALVE_STATUS | VALVE_CURRENT | RELIEF_STATE | HCO_OWNER | LINK_STATE
        | MS_SINCE_HEARTBEAT | RAIL_CURRENT | RAIL_VOLTAGE => return Err(AbortCode::ReadOnly),

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

    #[test]
    fn wrong_payload_width_aborts() {
        let mut s = store_with_servo();
        assert!(matches!(write(&mut s, od::VALVE_COMMANDED, 1, &[0]), Err(AbortCode::DataTypeMismatchLengthLow)));
    }
}
