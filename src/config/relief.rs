//! Configuration for the local overpressure relief loop.
use crate::index::{SensorSlot, ValveId};

use super::valves::PROMILLE_MAX;

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
