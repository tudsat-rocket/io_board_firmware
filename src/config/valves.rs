//! What is fitted to a valve slot: which outputs drive it, how far and how fast it travels, and
//! what it should do when the master stops talking.
//!
//! This is the *configuration* half of the valve model. The state machine that acts on it — the
//! target/measured/status layering, stall latching, the unpowered flag — is [`crate::valves`].

use crate::index::{HcoId, HcoPair, SensorSlot};

/// Number of valve slots, and of the high current outputs they are wired to. See the note in
/// [`super`] on why these exist as plain numbers at all.
pub const NUM_VALVES: usize = crate::index::ValveId::COUNT;
pub const NUM_HCO: usize = HcoId::COUNT;

/// A valve position, 0 = fully closed, 1000 = fully open. Wire format, so it is defined in
/// [`iocan_proto::valve`].
pub use iocan_proto::valve::PROMILLE_MAX;

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
    /// Sensor slot reporting where this valve actually is, if one is fitted.
    ///
    /// Without it the valve's position is extrapolated from `travel_ms`, which is an open loop:
    /// it says where the valve *should* be by now. A slot named here replaces that estimate with
    /// a reading, so the slot has to report [`super::Unit::Promille`] — the same unit the valve
    /// model already speaks. [`super::Config::sanity_check`] enforces that rather than letting a
    /// slot reporting centibar drive a valve.
    pub position_sensor: Option<SensorSlot>,
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
            position_sensor: None,
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

    /// Close the loop on this valve with a position sensor, replacing the travel-time estimate.
    pub const fn with_position_sensor(mut self, slot: SensorSlot) -> Self {
        self.position_sensor = Some(slot);
        self
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

/// True when two valves would drive any of the same outputs, and so fight each other every
/// control tick. Checked by [`super::Config::sanity_check`].
pub(super) fn shares_output(a: &ValveConfig, b: &ValveConfig) -> bool {
    let a_outs = [a.signal_hco, a.power_hco];
    let b_outs = [b.signal_hco, b.power_hco];
    a_outs.iter().flatten().any(|x| b_outs.iter().flatten().any(|y| x == y))
}
