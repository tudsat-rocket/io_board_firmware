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
