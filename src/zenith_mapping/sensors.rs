//! Sensor calibration constants for the Zenith vehicle.
//!
//! These are *factory defaults*. A board that has been calibrated over the bus and told to save
//! (0x1010) ignores everything here; what belongs in this file is what should come up on a blank
//! board straight out of a re-flash.
//!
//! The numbers stay in human-readable bar-per-count, exactly as measured on the bench and
//! recorded on the wiki, and [`crate::config::SensorCalib::from_per_count`] folds them to
//! fixed point in the compiler. Nothing here reaches the target as a float.
//!
//! These are calibrated as
//!
//! ```text
//!   pressure_bar = (adc_reading - offset) * linear_factor
//! ```
//!
//! with no constant term. A transducer that instead wants one — `.with_constant(1.013)`, to
//! report absolute rather than gauge pressure — says so at its own definition, so the convention
//! is never left implicit.
//!
//! See <https://wiki.tudsat.space/doc/calibration-5PBw7J4WFq> for how these were taken.

#![allow(
    clippy::excessive_precision,
    reason = "constant source of truth for calibration data"
)]

use crate::config::{SensorCalib as P, Unit};

/// A transducer's calibration together with the unit its readings are reported in.
///
/// The unit is part of a transducer's identity rather than a separate choice: a 400 bar sensor
/// *has* to report decibar, because 400 bar is 40000 centibar and i16 stops at 32767, while a
/// 40 bar sensor would throw away resolution if it did the same.
#[derive(Clone, Copy)]
pub struct Transducer {
    pub calib: P,
    pub unit: Unit,
}

const fn centibar(offset: f32, bar_per_count: f32) -> Transducer {
    Transducer {
        calib: P::from_per_count(offset, bar_per_count),
        unit: Unit::CentiBar,
    }
}

const fn decibar(offset: f32, bar_per_count: f32) -> Transducer {
    Transducer {
        calib: P::from_per_count(offset, bar_per_count),
        unit: Unit::DeciBar,
    }
}

/// A bridge transducer on a NAU7802 ([`crate::config::SensorKind::BridgePressure`]), from its
/// datasheet: `full_scale_bar` at `span_mv_per_v`, reading `zero_uv_per_v` at zero pressure.
///
/// The span is the datasheet's; the zero is the individual part's, read off the bench with the
/// slot on [`Unit::RawCounts`] (which a bridge slot reports in uV/V). Unlike the amplifier
/// calibrations above, nothing here depends on the board: the reading is the transducer's own
/// output ratio, so the same numbers hold on any bus, at any gain.
pub const fn bridge(full_scale_bar: f32, span_mv_per_v: f32, zero_uv_per_v: f32) -> Transducer {
    Transducer {
        calib: P {
            offset_milli: (zero_uv_per_v * 1000.0) as i32,
            ..P::bridge(full_scale_bar, span_mv_per_v)
        },
        unit: Unit::CentiBar,
    }
}

/// [`bridge`], with the zero given as raw NAU7802 counts (0x2008, or the `RawBridge` TPDOs) read at
/// PGA gain `gain` instead of in uV/V.
///
/// **Only right at that gain.** The same zero is twice the counts at twice the gain, so this
/// converts to uV/V once, here — `counts * 1e6 / (gain * 2^24)` — and the slot itself stays
/// gain-independent. Changing the bus's gain later (0x3032) keeps the calibration valid; changing
/// `gain` here without re-reading the counts at the new gain does not. No excitation voltage
/// enters into it: the ADC's reference is the bridge excitation, so a count is already a ratio.
///
/// A gain the PGA does not have is a compile error, since this is only ever evaluated in a
/// `const`.
pub const fn bridge_counts(full_scale_bar: f32, span_mv_per_v: f32, zero_counts: i32, gain: u8) -> Transducer {
    assert!(crate::config::bridge_gain_code(gain).is_some(), "not a NAU7802 gain: 1, 2, 4, .. 128");
    let zero_uv_per_v = zero_counts as f32 * 1_000_000.0 / (gain as f32 * 16_777_216.0);
    bridge(full_scale_bar, span_mv_per_v, zero_uv_per_v)
}

/// [`bridge_counts`], with the slope given as bar per raw NAU7802 count at PGA gain `gain` too,
/// as a two-point bench calibration measures it, rather than as a datasheet range and span.
///
/// The same caveat holds: both numbers are only right at the gain they were read at. A count is
/// `1e9 / (gain * 2^24)` nV/V, so the slope folds to `bar_per_count * gain * 2^24` bar per nV/V.
pub const fn bridge_counts_slope(zero_counts: i32, bar_per_count: f32, gain: u8) -> Transducer {
    assert!(crate::config::bridge_gain_code(gain).is_some(), "not a NAU7802 gain: 1, 2, 4, .. 128");
    let counts_per_unity = gain as f32 * 16_777_216.0;
    Transducer {
        calib: P {
            offset_milli: (zero_counts as f32 * 1_000_000_000.0 / counts_per_unity) as i32,
            // Millibar per nV/V, times 1e9.
            slope_nano: (bar_per_count * counts_per_unity * 1000.0) as i32,
            constant_milli: 0,
        },
        unit: Unit::CentiBar,
    }
}

/// A WIKA TTF-1 thin-film bridge (datasheet PE 81.16) of `range_bar`, at its nominal span and an
/// untrimmed zero — a placeholder until the individual sensor is on the bench.
///
/// The datasheet span is 2.0 mV/V nominal for every range, with a tolerance that narrows as the
/// range grows: 1.4..2.6 mV/V at 10 bar, 1.7..2.3 from 250 bar. So the slope is within 30 % before
/// it is trimmed. The zero is the larger error: up to +-0.5 mV/V, a quarter of the span, so an
/// untrimmed slot can read tens of bar at ambient. Trim it (with the slot on raw counts, in uV/V)
/// before believing an absolute reading; differences are good from the start.
///
/// These are gauge sensors (Relativdruck): zero is ambient, so trim the zero vented.
pub const fn wika_ttf1(range_bar: f32) -> Transducer {
    bridge(range_bar, 2.0, 170.0)
}

// --- NAU bridges ---------------------------------------------------------
pub const NAU_40BAR_B: Transducer = {
    let t = bridge_counts_slope(66_494, 3.37276e-05, 32);
    Transducer {
        calib: t.calib.with_constant(1.013),
        ..t
    }
};

pub const NAU_40BAR_A: Transducer = {
    let t = bridge_counts_slope(-69_092, 3.695e-05, 32);
    Transducer {
        calib: t.calib.with_constant(1.013),
        ..t
    }
};

pub const NAU_100BAR_I: Transducer = {
    let t = bridge_counts_slope(44070, 9.01e-05, 32);
    Transducer {
        calib: t.calib.with_constant(1.013),
        ..t
    }
};

// --- 10 bar ---------------------------------------------------------------
pub const PLCHOLD_10BAR: Transducer = centibar(1.0, 0.0855);

// --- 40 bar ---------------------------------------------------------------
pub const C_40BAR: Transducer = centibar(1.0, 0.0855);
pub const D_40BAR: Transducer = centibar(-385.0, 0.0535);
pub const E_40BAR: Transducer = centibar(341.5, 0.1250);
pub const F_40BAR: Transducer = centibar(341.5, 0.1250);

// --- 100 bar --------------------------------------------------------------
pub const A_100BAR: Transducer = centibar(262.0, 0.15528);
pub const B_100BAR: Transducer = centibar(413.7, 0.334);
pub const C_100BAR: Transducer = centibar(370.3, 0.1815);
pub const D_100BAR: Transducer = centibar(360.55, 0.1822);
pub const E_100BAR: Transducer = centibar(233.0, 0.12787724);
pub const F_100BAR: Transducer = centibar(200.0, 0.12771392);
pub const G_100BAR: Transducer = centibar(496.0, 0.23419204);
pub const H_100BAR: Transducer = centibar(420.5, 0.3368);

// --- 400 bar --------------------------------------------------------------
pub const B_400BAR: Transducer = centibar(439.0, 0.91116173);
pub const C_400BAR: Transducer = centibar(319.4, 1.2934);
pub const X_400BAR: Transducer = centibar(215.0, 1.0);

/// An uncalibrated slot: reports raw ADC counts, which is where a calibration starts.
pub const PLACEHOLDER_P: Transducer = Transducer {
    calib: P::ZERO,
    unit: Unit::RawCounts,
};

// --- which transducer is plumbed where ------------------------------------
//
// See the P&ID: https://wiki.tudsat.space/doc/plumbing-and-valvery-BCyIc3l2TW

pub const PRESSURANT_TANK_P: Transducer = C_400BAR;
pub const REG_1_P: Transducer = C_100BAR;
pub const REG_2_P: Transducer = D_100BAR;
pub const OX_TANK_UPPER_P: Transducer = A_100BAR;
pub const OX_TANK_LOWER_P: Transducer = B_100BAR;
pub const COMB_CHAMBER_1_P: Transducer = D_40BAR;
pub const COMB_CHAMBER_2_P: Transducer = C_40BAR;
pub const OX_FILL_EXT_P: Transducer = E_100BAR;
pub const EXT_N2: Transducer = X_400BAR;
pub const EXT_OX: Transducer = H_100BAR;

#[cfg(test)]
mod tests {
    use super::*;

    /// 50 000 counts at gain 32 is 50 000 / (32 * 2^24) of the excitation: 93.13 uV/V.
    #[test]
    fn a_zero_in_counts_lands_on_the_same_calibration_as_in_microvolts_per_volt() {
        let from_counts = bridge_counts(40.0, 2.0, 50_000, 32);
        let from_ratio = bridge(40.0, 2.0, 93.132_26);
        assert!((from_counts.calib.offset_milli - from_ratio.calib.offset_milli).abs() <= 1);
        assert_eq!(from_counts.calib.slope_nano, from_ratio.calib.slope_nano);

        // The same zero read at four times the gain is four times the counts.
        assert_eq!(bridge_counts(40.0, 2.0, 200_000, 128).calib.offset_milli, from_counts.calib.offset_milli);
    }

    /// 40 bar over 2 mV/V is 40 / (2e-3 * 32 * 2^24) bar per count at gain 32.
    #[test]
    fn a_slope_per_count_lands_on_the_same_calibration_as_a_span() {
        let from_slope = bridge_counts_slope(50_000, 40.0 / (2.0e-3 * 32.0 * 16_777_216.0), 32);
        let from_span = bridge_counts(40.0, 2.0, 50_000, 32);
        assert!((from_slope.calib.offset_milli - from_span.calib.offset_milli).abs() <= 1);
        assert!((from_slope.calib.slope_nano - from_span.calib.slope_nano).abs() <= 1);
    }
}
