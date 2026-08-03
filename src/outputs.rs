//! The single write point for the four high current outputs.
//! Used by:
//! - **Valve control**, which owns the outputs named in the valve mapping, and
//! - **direct digital/PWM control**, which owns everything else.

use crate::hco::{HcoControl, HcoState, Level, PwmMicros, State};
use crate::index::{HcoId, PerHco};

pub struct Outputs {
    ctl: &'static mut dyn HcoControl,
    /// Per-output override installed by a direct write while in raw debug mode. `None` means the
    /// normal owner (valve or direct) decides.
    raw_override: PerHco<Option<State>>,
    /// Last state actually pushed to hardware, so a tick that changes nothing costs nothing.
    last: HcoState,
}

impl Outputs {
    pub fn new(ctl: &'static mut dyn HcoControl) -> Self {
        let last = ctl.get_state();
        Self {
            ctl,
            raw_override: PerHco::splat(None),
            last,
        }
    }

    /// Record a direct write while raw debug mode is on, so it survives the next control tick.
    pub fn install_override(&mut self, hco: HcoId, state: State) {
        self.raw_override[hco] = Some(state);
    }

    /// Hand an output back to its normal owner. Called when the valve that owns it is commanded.
    pub fn release_override(&mut self, hco: HcoId) {
        self.raw_override[hco] = None;
    }

    /// Drop every override, e.g. on leaving raw debug mode.
    pub fn clear_overrides(&mut self) {
        self.raw_override = PerHco::splat(None);
    }

    pub fn has_overrides(&self) -> bool {
        self.raw_override.values().any(Option::is_some)
    }

    /// Push a fully resolved set of output states, applying any raw overrides on top.
    pub fn drive(&mut self, mut desired: HcoState) {
        for (hco, over) in self.raw_override.iter() {
            if let Some(over) = over {
                desired[hco] = *over;
            }
        }
        if desired != self.last {
            self.ctl.set_state(desired);
            self.last = desired;
        }
    }

    /// The state currently on the hardware, for mirroring into the store.
    pub fn current(&self) -> HcoState {
        self.last
    }

    /// De-energize everything. Used when a config change leaves an output with no owner, so a
    /// remapped valve cannot leave its old output stuck high.
    pub fn all_off(&mut self) {
        self.drive(HcoState::splat(State::Digital(Level::Low)));
    }
}

/// Digital level a direct write asks for.
pub fn digital(level: bool) -> State {
    State::Digital(Level::from(level))
}

/// PWM pulse width a direct write or a servo asks for, clamped to the servo-safe range.
pub fn pwm(micros: u16) -> State {
    State::Pwm(PwmMicros::from_u16_clamped(micros))
}
