//! A bang-bang thermostat for a resistive heating pad on one high current output.
//!
//! The pad carries its own 10k NTC, read through a divider on an analog COM pin. What the heater
//! does is the master's call, through [`iocan_proto::od::HEATER_MODE`]: off (the boot state), a
//! thermostat holding [`iocan_proto::od::HEATER_SETPOINT`], or blind heating with no temperature
//! control, the backup for a broken NTC. Once told, it regulates on its own, the same way
//! [`crate::relief`] acts without a master.
//!
//! The hardware half — which pair, which pin, how the NTC is wired, its calibration — is
//! [`HeaterConfig`], fixed at build time. The setpoint is configuration (0x3070, persisted); the
//! mode is a command (0x2018, not persisted).
//!
//! # Precedence
//! The heater owns its output outright, in every link state. The only exception is raw debug
//! mode, where the output goes back to direct control (0x2020) so it can be exercised on a bench.
//!
//! # Failure
//! In thermostat mode, a thermistor that reads open or shorted switches the pad **off**. An
//! unregulated heater is the worse failure: it keeps heating until something else gives. Blind
//! mode is the master deliberately accepting that failure, so it heats whatever the NTC says.
//!
//! # Fallback
//! Stage A keeps whatever mode was commanded. Stage B switches the heater off and resets the
//! command, so it stays off until the master re-enables it — see [`crate::control`].

use embassy_time::{Duration, Instant};

use crate::index::HcoPair;
use crate::rail_sense::NoRails;
use crate::store::{RAW_INVALID, TEMPERATURE_INVALID};

/// Compile-time heater hardware. Deliberately not part of [`crate::config::Config`]: pin muxing
/// happens at boot, long before a config is read, so none of it can change at runtime. The
/// setpoint, which can, is [`crate::config::Config::heater_setpoint_centi_c`].
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct HeaterConfig {
    /// The pair that switches the pad. Both outputs switch together.
    pub pair: HcoPair,
    /// The analog input the pad's NTC divider is wired to.
    pub ntc: AnalogPin,
    /// Which side of the divider the NTC is on.
    pub ntc_wiring: NtcWiring,
    /// Half-width of the dead band. The pad switches on below `setpoint - hysteresis` and off
    /// above `setpoint + hysteresis`, so it does not chatter around the setpoint.
    pub hysteresis_milli_c: i32,
    /// Calibration offset, in millidegrees Celsius, added to the temperature read off the NTC
    /// curve. Measure it as `reference - uncalibrated` against a thermometer on the pad, reading
    /// the uncalibrated value from the log line [`Heater::update`] prints every second.
    pub offset_milli_c: i32,
}

impl HeaterConfig {
    pub const fn new(pair: HcoPair, ntc: AnalogPin) -> Self {
        Self {
            pair,
            ntc,
            ntc_wiring: NtcWiring::ToGround,
            hysteresis_milli_c: 1_000,
            offset_milli_c: 0,
        }
    }

    pub const fn with_ntc_wiring(mut self, ntc_wiring: NtcWiring) -> Self {
        self.ntc_wiring = ntc_wiring;
        self
    }

    pub const fn with_offset_milli_c(mut self, offset_milli_c: i32) -> Self {
        self.offset_milli_c = offset_milli_c;
        self
    }

    pub const fn with_hysteresis_milli_c(mut self, hysteresis_milli_c: i32) -> Self {
        self.hysteresis_milli_c = hysteresis_milli_c;
        self
    }
}

/// An ADC-capable connector pin free for an external sensor on rev3. Labels are the rev3
/// schematic's.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum AnalogPin {
    /// COM4 pin 1, the stepper step clock. Claiming it leaves the node without a stepper port.
    Pa2,
    /// `A_IN_0`, COM5.
    Pa6,
    /// `A_IN_1`. Taken by the second stepper's direction line in a `dual-stepper` build.
    Pa5,
    /// `A_IN_2`.
    Pc5,
    /// `A_IN_3`.
    Pc4,
}

/// Which side of the 10k divider the NTC sits on.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum NtcWiring {
    /// 10k to +3.3V, NTC to ground. The reading falls as the pad heats.
    ToGround,
    /// NTC to +3.3V, 10k to ground. The reading rises as the pad heats.
    ToSupply,
}

impl NtcWiring {
    /// The reading the [`NtcWiring::ToGround`] divider would give at the same temperature, which
    /// is what [`NTC_CURVE`] is tabulated in. Swapping the two legs of a divider turns `x` into
    /// `full scale - x`, and with a 10k fixed leg that is exact for the curve.
    fn normalise(self, counts: u16) -> u16 {
        match self {
            Self::ToGround => counts,
            Self::ToSupply => ADC_FULL_SCALE.saturating_sub(counts),
        }
    }
}

/// 12-bit ADC.
const ADC_FULL_SCALE: u16 = 4095;

/// 0x2018: what the master has told the heater to do.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum HeaterMode {
    Off = 0,
    /// Hold the setpoint, and refuse to heat without a valid NTC reading.
    Thermostat = 1,
    /// Pad on continuously, whatever the NTC says. The backup for a broken sensor.
    Blind = 2,
}

impl HeaterMode {
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Off),
            1 => Some(Self::Thermostat),
            2 => Some(Self::Blind),
            _ => None,
        }
    }
}

/// 0x2017 sub 3.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
pub enum HeaterState {
    /// Thermostat, in or above the dead band, pad off.
    Idle = 0,
    /// Thermostat, below the dead band or still climbing through it, pad on.
    Heating = 1,
    /// Thermostat, but the NTC has no valid reading. Pad off.
    SensorFault = 2,
    /// No heater on this node.
    Disabled = 3,
    /// Commanded off.
    Off = 4,
    /// Commanded to heat blind. Pad on.
    Blind = 5,
}

#[allow(async_fn_in_trait)]
pub trait HeaterSensing {
    /// Raw 12-bit reading of the heater NTC divider, or `None` if this board cannot read it
    /// (rev2, or no pin was handed over at boot).
    async fn heater_ntc_counts(&mut self) -> Option<u16>;
}

/// rev2 has no ADC wired up at all, so a heater on it never switches on.
impl HeaterSensing for NoRails {
    async fn heater_ntc_counts(&mut self) -> Option<u16> {
        None
    }
}

/// How often [`Heater::update`] logs the reading, for calibration.
const LOG_INTERVAL: Duration = Duration::from_secs(1);

pub struct Heater {
    state: HeaterState,
    milli_c: i32,
    raw: u16,
    last_log: Option<Instant>,
}

impl Heater {
    pub const fn new() -> Self {
        Self {
            state: HeaterState::Disabled,
            milli_c: TEMPERATURE_INVALID,
            raw: RAW_INVALID,
            last_log: None,
        }
    }

    pub fn state(&self) -> HeaterState {
        self.state
    }

    /// Last calibrated temperature, [`TEMPERATURE_INVALID`] if there is none.
    pub fn milli_c(&self) -> i32 {
        self.milli_c
    }

    /// Last raw reading, [`RAW_INVALID`] if there is none.
    pub fn raw(&self) -> u16 {
        self.raw
    }

    /// Advance one tick and return whether the pad should be on.
    ///
    /// The temperature is measured and reported in every mode, so the master can watch the pad
    /// while it heats blind or sits off.
    pub fn update(
        &mut self,
        cfg: Option<&HeaterConfig>,
        mode: HeaterMode,
        setpoint_milli_c: i32,
        counts: Option<u16>,
        now: Instant,
    ) -> bool {
        let Some(cfg) = cfg else {
            self.state = HeaterState::Disabled;
            return false;
        };

        self.raw = counts.unwrap_or(RAW_INVALID);
        let uncalibrated = counts.map_or(TEMPERATURE_INVALID, |c| ntc_counts_to_milli_c(cfg.ntc_wiring.normalise(c)));
        self.milli_c = match uncalibrated {
            TEMPERATURE_INVALID => TEMPERATURE_INVALID,
            t => t.saturating_add(cfg.offset_milli_c),
        };

        let next = if mode == HeaterMode::Off {
            HeaterState::Off
        } else if mode == HeaterMode::Blind {
            HeaterState::Blind
        } else if self.milli_c == TEMPERATURE_INVALID {
            HeaterState::SensorFault
        } else if self.milli_c < setpoint_milli_c - cfg.hysteresis_milli_c {
            HeaterState::Heating
        } else if self.milli_c > setpoint_milli_c + cfg.hysteresis_milli_c {
            HeaterState::Idle
        } else {
            match self.state {
                // Inside the dead band: keep doing what we were doing.
                HeaterState::Heating => HeaterState::Heating,
                _ => HeaterState::Idle,
            }
        };

        if next != self.state {
            match next {
                HeaterState::SensorFault => {
                    defmt::error!("heater: NTC reads {} counts, no valid temperature, pad off", self.raw)
                }
                HeaterState::Blind => {
                    defmt::warn!("heater: heating blind, no temperature control, at {} m°C", self.milli_c)
                }
                _ => defmt::info!("heater: {} at {} m°C", next, self.milli_c),
            }
            self.state = next;
        }

        if self.last_log.is_none_or(|last| now - last >= LOG_INTERVAL) {
            self.last_log = Some(now);
            defmt::info!(
                "heater: raw {} counts, uncalibrated {} m°C, offset {} m°C, calibrated {} m°C, {}",
                self.raw,
                uncalibrated,
                cfg.offset_milli_c,
                self.milli_c,
                self.state
            );
        }

        matches!(self.state, HeaterState::Heating | HeaterState::Blind)
    }
}

impl Default for Heater {
    fn default() -> Self {
        Self::new()
    }
}

/// The pad NTC's curve, sampled every [`NTC_CURVE_STEP_C`] from [`NTC_CURVE_MIN_C`]: the 12-bit
/// ADC reading at each temperature. It falls as the NTC heats.
///
/// Tabulated for a 10k upper leg to +3.3V and the NTC to ground, with R25 = 10k and B = 3950 K.
/// The other wiring is mapped onto it by [`NtcWiring::normalise`]. The pad's beta is
/// **unmeasured**; regenerate if it or the fixed leg differs:
///
/// ```text
///   R_ntc(T) = 10k * exp(3950 * (1/T - 1/298.15))
///   counts(T) = 4095 * R_ntc / (10k + R_ntc)
/// ```
///
/// The reading is ratiometric as long as the divider is fed from the same +3.3V as the ADC
/// reference. Fed from 5V, the table is wrong, and the pin can be driven above its rating when
/// the NTC opens.
const NTC_CURVE: [u16; 34] = [
    3996, // -40 C
    3955, // -35 C
    3900, // -30 C
    3830, // -25 C
    3740, // -20 C
    3629, // -15 C
    3495, // -10 C
    3337, //  -5 C
    3156, //   0 C
    2955, //   5 C
    2738, //  10 C
    2510, //  15 C
    2278, //  20 C
    2048, //  25 C
    1825, //  30 C
    1614, //  35 C
    1419, //  40 C
    1241, //  45 C
    1081, //  50 C
    940,  //  55 C
    815,  //  60 C
    707,  //  65 C
    613,  //  70 C
    532,  //  75 C
    462,  //  80 C
    401,  //  85 C
    350,  //  90 C
    305,  //  95 C
    267,  // 100 C
    234,  // 105 C
    206,  // 110 C
    181,  // 115 C
    160,  // 120 C
    142,  // 125 C
];

/// Temperature of `NTC_CURVE[0]`, in degrees Celsius.
const NTC_CURVE_MIN_C: i32 = -40;
/// Spacing between adjacent `NTC_CURVE` entries, in degrees Celsius.
const NTC_CURVE_STEP_C: i32 = 5;

/// Convert a raw 12-bit reading of the pad NTC divider to millidegrees Celsius.
///
/// [`TEMPERATURE_INVALID`] for anything off the ends of [`NTC_CURVE`]. That covers an open NTC
/// (pin pulled to +3.3V) and a short (pin at ground), which is what switches the pad off.
pub fn ntc_counts_to_milli_c(counts: u16) -> i32 {
    // Descending curve, so the bracket is `curve[i] >= counts >= curve[i + 1]`.
    if counts > NTC_CURVE[0] || counts < NTC_CURVE[NTC_CURVE.len() - 1] {
        return TEMPERATURE_INVALID;
    }

    let mut i = 0;
    while NTC_CURVE[i + 1] > counts {
        i += 1;
    }

    let (high, low) = (NTC_CURVE[i] as i32, NTC_CURVE[i + 1] as i32);
    let base_milli_c = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
    base_milli_c + (NTC_CURVE_STEP_C * 1000 * (high - counts as i32)) / (high - low)
}

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: HeaterConfig = HeaterConfig::new(HcoPair::B, AnalogPin::Pa6);
    const HOLD: HeaterMode = HeaterMode::Thermostat;
    const SETPOINT: i32 = 30_000;

    fn at0() -> Instant {
        Instant::from_millis(0)
    }

    /// 25 C sits at mid-scale with a 10k/10k divider.
    #[test]
    fn the_curve_reads_room_temperature_at_mid_scale() {
        assert_eq!(ntc_counts_to_milli_c(2048), 25_000);
    }

    #[test]
    fn the_curve_descends() {
        for pair in NTC_CURVE.windows(2) {
            assert!(pair[0] > pair[1], "NTC_CURVE must fall monotonically: {pair:?}");
        }
    }

    #[test]
    fn every_curve_point_maps_back_to_its_own_temperature() {
        for (i, &counts) in NTC_CURVE.iter().enumerate() {
            let expected = (NTC_CURVE_MIN_C + i as i32 * NTC_CURVE_STEP_C) * 1000;
            assert_eq!(ntc_counts_to_milli_c(counts), expected, "curve point {i}");
        }
    }

    #[test]
    fn between_points_it_interpolates() {
        // Halfway between 25 C (2048) and 30 C (1825).
        let t = ntc_counts_to_milli_c(1937);
        assert!((27_000..28_000).contains(&t), "{t}");
    }

    #[test]
    fn open_or_shorted_ntc_is_invalid() {
        assert_eq!(ntc_counts_to_milli_c(4095), TEMPERATURE_INVALID);
        assert_eq!(ntc_counts_to_milli_c(0), TEMPERATURE_INVALID);
    }

    #[test]
    fn cold_pad_heats() {
        let mut h = Heater::new();
        assert!(h.update(Some(&CFG), HOLD, SETPOINT, Some(2278), at0())); // 20 C
        assert_eq!(h.state(), HeaterState::Heating);
    }

    #[test]
    fn hot_pad_is_off() {
        let mut h = Heater::new();
        assert!(!h.update(Some(&CFG), HOLD, SETPOINT, Some(1614), at0())); // 35 C
        assert_eq!(h.state(), HeaterState::Idle);
    }

    #[test]
    fn it_keeps_heating_through_the_dead_band_and_stops_above_it() {
        let mut h = Heater::new();
        assert!(h.update(Some(&CFG), HOLD, SETPOINT, Some(2278), at0())); // 20 C
        assert!(h.update(Some(&CFG), HOLD, SETPOINT, Some(1825), at0()), "30 C is inside the band, keep heating"); // 30 C
        assert!(!h.update(Some(&CFG), HOLD, SETPOINT, Some(1760), at0()), "~32 C is above the band"); // ~31.5 C
        assert!(
            !h.update(Some(&CFG), HOLD, SETPOINT, Some(1825), at0()),
            "and it stays off coming back down into the band"
        );
        assert!(h.update(Some(&CFG), HOLD, SETPOINT, Some(1900), at0()), "~28 C is below the band again");
    }

    #[test]
    fn a_broken_ntc_switches_the_pad_off() {
        let mut h = Heater::new();
        assert!(h.update(Some(&CFG), HOLD, SETPOINT, Some(2278), at0()));
        assert!(!h.update(Some(&CFG), HOLD, SETPOINT, Some(4095), at0()));
        assert_eq!(h.state(), HeaterState::SensorFault);
        assert_eq!(h.milli_c(), TEMPERATURE_INVALID);
    }

    #[test]
    fn a_board_that_cannot_read_the_ntc_never_heats() {
        let mut h = Heater::new();
        assert!(!h.update(Some(&CFG), HOLD, SETPOINT, None, at0()));
        assert_eq!(h.state(), HeaterState::SensorFault);
        assert_eq!(h.raw(), RAW_INVALID);
    }

    #[test]
    fn the_offset_shifts_the_reported_temperature_and_the_regulation() {
        // The NTC reads 30 C, but the pad is really 3 C colder.
        let cfg = CFG.with_offset_milli_c(-3_000);
        let mut h = Heater::new();
        assert!(h.update(Some(&cfg), HOLD, SETPOINT, Some(1825), at0()), "27 C calibrated is below the band");
        assert_eq!(h.milli_c(), 27_000);
    }

    #[test]
    fn the_offset_does_not_mask_a_broken_ntc() {
        let cfg = CFG.with_offset_milli_c(5_000);
        let mut h = Heater::new();
        assert!(!h.update(Some(&cfg), HOLD, SETPOINT, Some(4095), at0()));
        assert_eq!(h.milli_c(), TEMPERATURE_INVALID);
    }

    #[test]
    fn an_ntc_to_supply_reads_rising_counts_as_rising_temperature() {
        let cfg = CFG.with_ntc_wiring(NtcWiring::ToSupply);
        let mut h = Heater::new();
        // 4095 - 2278: the mirrored divider at 20 C.
        assert!(h.update(Some(&cfg), HOLD, SETPOINT, Some(4095 - 2278), at0()));
        assert_eq!(h.milli_c(), 20_000);
        // 4095 - 1614: 35 C.
        assert!(!h.update(Some(&cfg), HOLD, SETPOINT, Some(4095 - 1614), at0()));
        assert_eq!(h.milli_c(), 35_000);
    }

    #[test]
    fn an_ntc_to_supply_still_detects_open_and_short() {
        let cfg = CFG.with_ntc_wiring(NtcWiring::ToSupply);
        let mut h = Heater::new();
        assert!(!h.update(Some(&cfg), HOLD, SETPOINT, Some(0), at0()));
        assert_eq!(h.state(), HeaterState::SensorFault);
        assert!(!h.update(Some(&cfg), HOLD, SETPOINT, Some(4095), at0()));
        assert_eq!(h.state(), HeaterState::SensorFault);
    }

    #[test]
    fn off_keeps_a_cold_pad_off_but_still_reports_its_temperature() {
        let mut h = Heater::new();
        assert!(!h.update(Some(&CFG), HeaterMode::Off, SETPOINT, Some(2278), at0())); // 20 C
        assert_eq!(h.state(), HeaterState::Off);
        assert_eq!(h.milli_c(), 20_000);
    }

    #[test]
    fn blind_heats_through_a_broken_ntc() {
        let mut h = Heater::new();
        assert!(h.update(Some(&CFG), HeaterMode::Blind, SETPOINT, Some(4095), at0()));
        assert_eq!(h.state(), HeaterState::Blind);
        assert!(h.update(Some(&CFG), HeaterMode::Blind, SETPOINT, None, at0()), "and with no reading at all");
    }

    #[test]
    fn blind_ignores_the_setpoint() {
        let mut h = Heater::new();
        assert!(h.update(Some(&CFG), HeaterMode::Blind, SETPOINT, Some(1614), at0()), "35 C, above the band");
        assert_eq!(h.milli_c(), 35_000);
    }

    #[test]
    fn the_setpoint_is_an_input_not_a_constant() {
        let mut h = Heater::new();
        // 20 C: below a 30 C setpoint, above a 10 C one.
        assert!(h.update(Some(&CFG), HOLD, 30_000, Some(2278), at0()));
        assert!(!h.update(Some(&CFG), HOLD, 10_000, Some(2278), at0()));
    }

    #[test]
    fn no_config_is_disabled() {
        let mut h = Heater::new();
        assert!(!h.update(None, HeaterMode::Blind, SETPOINT, Some(2278), at0()), "not even blind");
        assert_eq!(h.state(), HeaterState::Disabled);
    }
}
