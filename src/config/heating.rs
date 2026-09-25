//! What keeps a valve warm: which output switches its heating pad, and which sensor slot the
//! thermostat regulates on.
//!
//! One [`ValveHeatingConfig`] per valve slot, because the thing being kept warm is a valve — a
//! pad bonded to a body that must not freeze shut. The heater is therefore addressed by the valve
//! it serves rather than by an output number, which is also what makes
//! [`super::Config::with_valve_heating`] read like [`super::Config::with_valve`] beside it.
//!
//! # Why the temperature comes from a sensor slot
//!
//! The pad's thermistor is an ordinary [`SensorSlot`] — in practice an NTC on a COM5/COM6 pin
//! ([`crate::config::SensorKind::Ntc`]), but a Pt1000 or an MCP9700 on an amplifier does just as
//! well. Nothing here reads a pin.
//!
//! That is the whole architecture: the sensor plane already linearises, trims, scales and
//! broadcasts a temperature, and it is already recalibratable over the bus. A thermostat with its
//! own private ADC channel would need a second copy of every one of those, and its reading would
//! be invisible at 0x2004 while the number it regulates on could silently disagree with the one
//! the master is watching. Regulating on the published reading means the master sees exactly what
//! the node acts on.
//!
//! It also means the setpoint is written in the unit the slot already reports (0x2005) — the same
//! convention [`super::ReliefConfig::threshold`] uses, and for the same reason: no conversion
//! between the number in the config and the number on the wire, so none can be wrong.
//!
//! All of it is runtime-writable (0x3080..0x3084) and persisted, so a pad can be re-aimed at a
//! different slot, retuned, or switched off without a firmware build.

use crate::index::{HcoId, SensorSlot};

/// The heating pad on one valve. See [`crate::heating`] for the thermostat itself.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ValveHeatingConfig {
    /// Whether the thermostat runs. The switch a master flips to stop heating without forgetting
    /// how the pad is wired; `false` is also what an unconfigured slot has.
    pub enabled: bool,
    /// The output that switches the pad. `None` means no pad on this valve.
    pub hco: Option<HcoId>,
    /// The slot reporting the pad's temperature. `None` means no pad on this valve: a thermostat
    /// with nothing to regulate on is not a thermostat, so this is as load-bearing as the output.
    pub sensor: Option<SensorSlot>,
    /// Temperature to hold, in the sensor slot's own unit (0x2005) — so a slot reporting
    /// centicelsius takes 3000 for 30.00 degC.
    pub setpoint: i16,
    /// Half-width of the dead band, in the same unit. The pad switches on below
    /// `setpoint - hysteresis` and off above `setpoint + hysteresis`, so it does not chatter
    /// around the setpoint at the resolution of one ADC count.
    pub hysteresis: u16,
}

impl ValveHeatingConfig {
    /// One degree either side of the setpoint, for a slot reporting centicelsius.
    pub const DEFAULT_HYSTERESIS: u16 = 100;

    /// No pad on this valve.
    ///
    /// The setpoint is the *lowest* representable rather than zero, so that a slot which is
    /// switched on before it is configured asks for no heat at all rather than for 0.00 degC —
    /// which on a cold vehicle would be a request to heat.
    pub const fn none() -> Self {
        Self {
            enabled: false,
            hco: None,
            sensor: None,
            setpoint: i16::MIN,
            hysteresis: Self::DEFAULT_HYSTERESIS,
        }
    }

    /// A pad on `hco`, held at `setpoint` as reported by `sensor`, in that slot's own unit.
    pub const fn new(hco: HcoId, sensor: SensorSlot, setpoint: i16) -> Self {
        Self {
            enabled: true,
            hco: Some(hco),
            sensor: Some(sensor),
            setpoint,
            ..Self::none()
        }
    }

    /// Widen or narrow the dead band, in the sensor slot's unit.
    pub const fn with_hysteresis(mut self, hysteresis: u16) -> Self {
        self.hysteresis = hysteresis;
        self
    }

    /// Fitted but not switched on: the wiring is configured and the thermostat stays idle.
    pub const fn disabled(mut self) -> Self {
        self.enabled = false;
        self
    }

    /// Whether a pad is wired up at all — an output to switch and a slot to watch.
    ///
    /// Both halves are required together. An output with no sensor would be a heater with no way
    /// to stop, and a sensor with no output is a temperature nobody acts on.
    pub const fn is_fitted(&self) -> bool {
        self.hco.is_some() && self.sensor.is_some()
    }

    /// Whether the thermostat should run this tick.
    pub const fn is_armed(&self) -> bool {
        self.enabled && self.is_fitted()
    }

    /// Whether `hco` is the output this pad switches.
    pub fn switches(&self, hco: HcoId) -> bool {
        self.hco == Some(hco)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Half a heater is not a heater: both the output and the slot have to be there before
    /// anything switches, because each without the other is a distinct way to fail badly.
    #[test]
    fn a_heater_needs_both_an_output_and_a_sensor() {
        let full = ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot1, 3_000);
        assert!(full.is_fitted() && full.is_armed());

        let mut no_sensor = full;
        no_sensor.sensor = None;
        assert!(!no_sensor.is_fitted(), "an output with no sensor is a heater with no way to stop");

        let mut no_output = full;
        no_output.hco = None;
        assert!(!no_output.is_fitted(), "a sensor with no output is a temperature nobody acts on");

        assert!(!full.disabled().is_armed(), "and it still has to be switched on");
        assert!(!ValveHeatingConfig::none().is_fitted());
    }

    /// An unconfigured slot must not read as "please heat to 0 degC" if something switches it on.
    #[test]
    fn an_unconfigured_heater_asks_for_no_heat() {
        assert_eq!(ValveHeatingConfig::none().setpoint, i16::MIN);
    }

    #[test]
    fn a_pad_switches_exactly_one_output() {
        let pad = ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot0, 0);
        assert!(pad.switches(HcoId::Hco2));
        assert!(!pad.switches(HcoId::Hco3), "the other half of the pair is nothing to do with it");
        assert!(!ValveHeatingConfig::none().switches(HcoId::Hco2), "and an unfitted pad switches nothing");
    }
}
