use embassy_stm32::gpio;

pub trait HcoControl {
    fn set_level(&mut self, output: HighCurrentOutput, level: Level);
    fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16);

    fn get_state(&self) -> HcoState;
    fn set_state(&mut self, target_state: HcoState);
}

/// State of all 4 high current outpus.
#[derive(Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
pub struct HcoState {
    pub _1: State,
    pub _2: State,
    pub _3: State,
    pub _4: State,
}

impl HcoState {
    // helper method for avoiding match statements
    pub fn set_high(&mut self, output: HighCurrentOutput) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::High),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::High),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::High),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::High),
        }
    }
    // helper method for avoiding match statements
    pub fn set_low(&mut self, output: HighCurrentOutput) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(Level::Low),
            HighCurrentOutput::_2 => self._2 = State::Digital(Level::Low),
            HighCurrentOutput::_3 => self._3 = State::Digital(Level::Low),
            HighCurrentOutput::_4 => self._4 = State::Digital(Level::Low),
        }
    }
    // helper method for avoiding match statements
    pub fn set_level(&mut self, output: HighCurrentOutput, level: Level) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Digital(level),
            HighCurrentOutput::_2 => self._2 = State::Digital(level),
            HighCurrentOutput::_3 => self._3 = State::Digital(level),
            HighCurrentOutput::_4 => self._4 = State::Digital(level),
        }
    }

    // helper method for avoiding match statements
    pub fn set_pwm_micros(&mut self, output: HighCurrentOutput, micros: u16) {
        match output {
            HighCurrentOutput::_1 => self._1 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_2 => self._2 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_3 => self._3 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
            HighCurrentOutput::_4 => self._4 = State::Pwm(PwmMicros::from_u16_clamped(micros)),
        }
    }
    pub fn get_state_0_indexed(&self, index: usize) -> Option<&State> {
        match index {
            0 => Some(&self._1),
            1 => Some(&self._2),
            2 => Some(&self._3),
            3 => Some(&self._4),
            _ => None,
        }
    }
}

/// Identifies one of 4 high current outputs.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum HighCurrentOutput {
    _1,
    _2,
    _3,
    _4,
}
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

impl From<Level> for gpio::Level {
    fn from(value: Level) -> Self {
        match value {
            Level::High => gpio::Level::High,
            Level::Low => gpio::Level::Low,
        }
    }
}
impl From<gpio::Level> for Level {
    fn from(value: gpio::Level) -> Self {
        match value {
            gpio::Level::High => Level::High,
            gpio::Level::Low => Level::Low,
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
        if 500 <= value && value <= 2500 {
            Ok(Self(value))
        } else {
            Err(())
        }
    }
}
