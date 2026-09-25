//! Valve heating: a bang-bang thermostat per valve, regulating on a sensor slot.
//!
//! The pad is switched by one high current output and the
//! temperature comes from whatever [`crate::index::SensorSlot`] the configuration names.
//! This module makes the decision, [`crate::control`] drives the output.
//!
//! # Failure
//!
//! A slot with no valid reading switches the pad **off**

use crate::config::ValveHeatingConfig;
use crate::store::SENSOR_INVALID;

/// 0x2019.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum HeatingState {
    /// At or above the dead band: pad off, and nothing wrong.
    Idle = 0,
    /// Below the dead band, or still climbing through it: pad on.
    Heating = 1,
    /// Armed, but the watched slot has no valid reading. Pad off — see the module note on why
    /// this does not heat anyway.
    SensorFault = 2,
    /// No pad on this valve, or it is switched off.
    Disabled = 3,
}

pub struct Heating {
    state: HeatingState,
    /// Suppresses repeating the "no valid reading" complaint every tick.
    warned_invalid: bool,
}

impl Heating {
    pub const fn new() -> Self {
        Self {
            state: HeatingState::Disabled,
            warned_invalid: false,
        }
    }

    pub fn state(&self) -> HeatingState {
        self.state
    }

    /// Advance one tick and return whether the pad should be on.
    ///
    /// `reading` is the current value of the configured sensor slot, in that slot's own unit —
    /// the same number that goes out at 0x2004 — so the setpoint and the dead band are written in
    /// the units the sensor already reports and no conversion can go wrong between them.
    ///
    /// temp below `setpoint - hysteresis` -> heating on
    /// temp above `setpoint + hysteresis` -> heating off
    pub fn update(&mut self, cfg: &ValveHeatingConfig, reading: i16) -> bool {
        let next = self.decide(cfg, reading);
        if next != self.state {
            match next {
                HeatingState::SensorFault => defmt::error!("heating: no valid temperature, pad off"),
                _ => defmt::info!("heating: {} at {} (setpoint {})", next, reading, cfg.setpoint),
            }
            self.state = next;
        }
        self.warned_invalid = next == HeatingState::SensorFault;
        next == HeatingState::Heating
    }

    fn decide(&self, cfg: &ValveHeatingConfig, reading: i16) -> HeatingState {
        if !cfg.is_armed() {
            return HeatingState::Disabled;
        }
        if reading == SENSOR_INVALID {
            return HeatingState::SensorFault;
        }

        let (reading, setpoint, band) = (reading as i32, cfg.setpoint as i32, cfg.hysteresis as i32);
        if reading < setpoint - band {
            HeatingState::Heating
        } else if reading > setpoint + band {
            HeatingState::Idle
        } else {
            // Inside the dead band: keep doing what we were doing.
            match self.state {
                HeatingState::Heating => HeatingState::Heating,
                _ => HeatingState::Idle,
            }
        }
    }
}

impl Default for Heating {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::index::{HcoId, SensorSlot};

    /// 30.00 degC, one degree either side, on a slot reporting centicelsius.
    fn cfg() -> ValveHeatingConfig {
        ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000)
    }

    #[test]
    fn it_heats_below_the_band_and_stops_above_it() {
        let mut heating = Heating::new();

        assert!(heating.update(&cfg(), 2_000), "20 C is well below 30 C");
        assert_eq!(heating.state(), HeatingState::Heating);

        assert!(!heating.update(&cfg(), 3_500), "35 C is well above");
        assert_eq!(heating.state(), HeatingState::Idle);
    }

    /// The dead band is the whole point of a bang-bang controller: inside it the pad keeps doing
    /// what it was doing, so it does not switch on and off at the resolution of one ADC count.
    #[test]
    fn inside_the_dead_band_it_holds_its_state() {
        let mut heating = Heating::new();

        // Climbing from cold, still on at the setpoint itself.
        assert!(heating.update(&cfg(), 2_000));
        assert!(heating.update(&cfg(), 2_950));
        assert!(heating.update(&cfg(), 3_000));
        assert!(heating.update(&cfg(), 3_100), "the top of the band, still climbing");
        // Past the top: off, and it stays off back down through the band.
        assert!(!heating.update(&cfg(), 3_101));
        assert!(!heating.update(&cfg(), 3_000));
        assert!(!heating.update(&cfg(), 2_900), "the bottom of the band, still falling");
        // Past the bottom: on again.
        assert!(heating.update(&cfg(), 2_899));
    }

    /// The safety property: no reading means no heat. An unregulated pad heats until something
    /// else gives, so "I do not know the temperature" has exactly one safe answer.
    #[test]
    fn a_slot_with_no_reading_switches_the_pad_off() {
        let mut heating = Heating::new();
        assert!(heating.update(&cfg(), 2_000));

        assert!(!heating.update(&cfg(), SENSOR_INVALID), "a dead thermistor must not keep the pad on");
        assert_eq!(heating.state(), HeatingState::SensorFault);

        // And it recovers on its own when the sensor comes back.
        assert!(heating.update(&cfg(), 2_000));
        assert_eq!(heating.state(), HeatingState::Heating);
    }

    #[test]
    fn an_unconfigured_or_switched_off_heater_never_heats() {
        let mut heating = Heating::new();
        assert!(!heating.update(&ValveHeatingConfig::none(), 0));
        assert_eq!(heating.state(), HeatingState::Disabled);

        // Cold, fitted, but switched off at 0x3080.
        assert!(!heating.update(&cfg().disabled(), -4_000));
        assert_eq!(heating.state(), HeatingState::Disabled);
    }

    /// A setpoint near the end of the i16 range plus a dead band overflows i16; the comparison
    /// has to survive it rather than wrapping into a demand for full heat.
    #[test]
    fn an_extreme_setpoint_does_not_wrap() {
        let mut heating = Heating::new();

        let mut hot = cfg();
        hot.setpoint = i16::MAX;
        assert!(heating.update(&hot, 0), "still below an absurdly high setpoint");

        let mut cold = cfg();
        cold.setpoint = i16::MIN + 1;
        assert!(!heating.update(&cold, 0), "and above an absurdly low one");
    }
}
