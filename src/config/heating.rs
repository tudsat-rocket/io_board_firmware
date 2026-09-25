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
//! # Several pads, one thermostat
//!
//! A heater may switch more than one output ([`HcoSet`]): two pads bonded to the same valve, say,
//! regulated on the one thermistor between them. They are one thermostat, not two — they share a
//! setpoint, a dead band and a state at 0x2019, and switch on and off together. Two pads that
//! should regulate independently belong in two heater entries, which may still watch one slot.
//!
//! All of it is runtime-writable (0x3080..0x3084) and persisted, so a pad can be re-aimed at a
//! different slot, retuned, or switched off without a firmware build.

use crate::index::{HcoId, SensorSlot};

/// A set of high current outputs, as a bitmask: bit *n* is [`HcoId`] index *n*, so bit 0 is the
/// output silkscreened 1. The same layout goes on the wire at 0x3081 and into flash.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct HcoSet(u8);

impl HcoSet {
    pub const EMPTY: Self = Self(0);
    /// Every bit that names an output on this board.
    const VALID: u8 = (1 << HcoId::COUNT) - 1;

    pub const fn of(hco: HcoId) -> Self {
        Self(1 << hco.index())
    }

    pub const fn with(self, hco: HcoId) -> Self {
        Self(self.0 | Self::of(hco).0)
    }

    /// From a wire or flash byte. `None` for a bit past the last output — a mistake to reject,
    /// not a bit to ignore.
    pub const fn from_bits(bits: u8) -> Option<Self> {
        if bits & !Self::VALID == 0 {
            Some(Self(bits))
        } else {
            None
        }
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub const fn contains(self, hco: HcoId) -> bool {
        self.0 & Self::of(hco).0 != 0
    }

    pub const fn intersects(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    pub fn iter(self) -> impl Iterator<Item = HcoId> {
        HcoId::ALL.into_iter().filter(move |&hco| self.contains(hco))
    }
}

/// The heating pad on one valve. See [`crate::heating`] for the thermostat itself.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ValveHeatingConfig {
    /// Whether the thermostat runs. The switch a master flips to stop heating without forgetting
    /// how the pad is wired; `false` is also what an unconfigured slot has.
    pub enabled: bool,
    /// The outputs that switch the pad, or pads — all together, from the one thermostat. Empty
    /// means no pad on this valve.
    pub outputs: HcoSet,
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
            outputs: HcoSet::EMPTY,
            sensor: None,
            setpoint: i16::MIN,
            hysteresis: Self::DEFAULT_HYSTERESIS,
        }
    }

    /// A pad on `hco`, held at `setpoint` as reported by `sensor`, in that slot's own unit.
    pub const fn new(hco: HcoId, sensor: SensorSlot, setpoint: i16) -> Self {
        Self {
            enabled: true,
            outputs: HcoSet::of(hco),
            sensor: Some(sensor),
            setpoint,
            ..Self::none()
        }
    }

    /// Switch another pad on `hco` from this same thermostat, so both follow the one sensor.
    pub const fn also_on(mut self, hco: HcoId) -> Self {
        self.outputs = self.outputs.with(hco);
        self
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

    /// Whether a pad is wired up at all — at least one output to switch and a slot to watch.
    ///
    /// Both halves are required together. An output with no sensor would be a heater with no way
    /// to stop, and a sensor with no output is a temperature nobody acts on.
    pub const fn is_fitted(&self) -> bool {
        !self.outputs.is_empty() && self.sensor.is_some()
    }

    /// Whether the thermostat should run this tick.
    pub const fn is_armed(&self) -> bool {
        self.enabled && self.is_fitted()
    }

    /// Whether `hco` is one of the outputs this heater switches.
    pub const fn switches(&self, hco: HcoId) -> bool {
        self.outputs.contains(hco)
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
        no_output.outputs = HcoSet::EMPTY;
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

    #[test]
    fn two_pads_can_follow_one_sensor() {
        let pads = ValveHeatingConfig::new(HcoId::Hco2, SensorSlot::Slot0, 0).also_on(HcoId::Hco3);
        assert!(pads.is_fitted());
        assert!(pads.switches(HcoId::Hco2) && pads.switches(HcoId::Hco3));
        assert!(!pads.switches(HcoId::Hco0));
        assert_eq!(pads.outputs.iter().collect::<Vec<_>>(), [HcoId::Hco2, HcoId::Hco3]);
        assert_eq!(pads.outputs.bits(), 0b1100, "bit n is output n+1 as silkscreened");
    }

    /// A bit past the fourth output names nothing on the board.
    #[test]
    fn an_output_set_rejects_bits_past_the_board() {
        assert_eq!(HcoSet::from_bits(0b1111).map(|s| s.iter().count()), Some(4));
        assert_eq!(HcoSet::from_bits(0), Some(HcoSet::EMPTY));
        assert_eq!(HcoSet::from_bits(0b1_0000), None);
    }
}
