use crate::index::PerHco;

/// State of all 4 high current outputs.
pub type HcoState = PerHco<State>;

/// Identifies one of 4 high current outputs.
///
/// Note this is 0-indexed, matching [`crate::config::ValveConfig::signal_hco`] and the rest of the
/// firmware; the board silkscreen and the SDO encoding are 1-indexed. See [`HcoId::silkscreen`].
pub use crate::index::HcoId;

#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub enum Level {
    High,
    #[default]
    Low,
}
impl Level {
    pub fn as_u8(&self) -> u8 {
        match self {
            Level::High => 1,
            Level::Low => 0,
        }
    }
}

#[cfg(feature = "hardware")]
impl From<Level> for embassy_stm32::gpio::Level {
    fn from(value: Level) -> Self {
        match value {
            Level::High => embassy_stm32::gpio::Level::High,
            Level::Low => embassy_stm32::gpio::Level::Low,
        }
    }
}
#[cfg(feature = "hardware")]
impl From<embassy_stm32::gpio::Level> for Level {
    fn from(value: embassy_stm32::gpio::Level) -> Self {
        match value {
            embassy_stm32::gpio::Level::High => Level::High,
            embassy_stm32::gpio::Level::Low => Level::Low,
        }
    }
}
impl From<bool> for Level {
    fn from(value: bool) -> Self {
        match value {
            false => Level::Low,
            true => Level::High,
        }
    }
}

/// State of a high current output.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum State {
    Digital(Level),
    /// Set duty cycle for high current output in microseconds
    Pwm(PwmMicros),
}
impl Default for State {
    fn default() -> Self {
        Self::Digital(Level::default())
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PwmMicros(u16);
impl From<PwmMicros> for u16 {
    fn from(value: PwmMicros) -> Self {
        value.as_u16()
    }
}
impl PwmMicros {
    pub fn as_u16(&self) -> u16 {
        self.0
    }
    pub fn from_u16_clamped(value: u16) -> Self {
        Self(value.clamp(500, 2500))
    }
}
impl TryFrom<u16> for PwmMicros {
    type Error = ();
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        if (500..=2500).contains(&value) {
            Ok(Self(value))
        } else {
            Err(())
        }
    }
}

/// The single seam `crate::outputs::Outputs` writes through. Synchronous and dyn-safe so a host
/// test can hand `Outputs` a boxed mock instead of a real HCO driver.
pub trait HcoControl {
    fn get_state(&self) -> HcoState;
    fn set_state(&mut self, target_state: HcoState);

    /// Drive one output, leaving the rest as they are. Provided rather than required: every
    /// implementation pushes whole states to hardware anyway, so read-modify-write is the only
    /// sensible definition and there is no reason for each revision to repeat it.
    fn set_level(&mut self, output: HcoId, level: Level) {
        let mut state = self.get_state();
        state[output] = State::Digital(level);
        self.set_state(state);
    }

    fn set_pwm_micros(&mut self, output: HcoId, micros: u16) {
        let mut state = self.get_state();
        state[output] = State::Pwm(PwmMicros::from_u16_clamped(micros));
        self.set_state(state);
    }
}
