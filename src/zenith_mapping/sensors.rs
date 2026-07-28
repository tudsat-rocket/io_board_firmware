#![allow(
    clippy::excessive_precision,
    reason = "constant source of truth for calibration data"
)]
use crate::sensors::{PressureSensorCalib, SensorKind as SK};

// --- zenith sensors ---

// See P&ID: https://wiki.tudsat.space/doc/plumbing-and-valvery-BCyIc3l2TW

pub const PRESSURANT_TANK_P: SK = B_400BAR;
pub const REG_1_P: SK = C_100BAR;
pub const REG_2_P: SK = D_100BAR;
pub const OX_TANK_UPPER_P: SK = B_100BAR;

pub const OX_TANK_LOWER_P: SK = A_100BAR;

// combustion chamber
pub const COMB_CHAMBER_1_P: SK = C_40BAR;
pub const COMB_CHAMBER_2_P: SK = D_40BAR;

pub const OX_FILL_EXT_P: SK = E_100BAR;

pub const OX_TANK_UPPER_T: SK = PT_1000;
pub const OX_TANK_LOWER_T: SK = PT_1000;

pub const PT_1000: SK = SK::TempPt1000;

pub const PLACEHOLDER_T: SK = SK::TempPt1000;
pub const PLACEHOLDER_P: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 0.0,
    linear_factor: 0.0,
});

// --- all sensors ---

// see for sensor calibration https://wiki.tudsat.space/doc/calibration-5PBw7J4WFq

// --- 40 bar ---

// NOTE: not optimized, may change
pub const C_40BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 15.0,
    linear_factor: 0.0855,
});

// NOTE: not optimized, may change
pub const D_40BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: -385.0,
    linear_factor: 0.0535,
});

// --- 100 bar ---

// At R_gain = 100Ω, ref = 0V
pub const A_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 47.0,
    linear_factor: 0.10604454,
});

// At R_gain = 220Ω, ref = 0V
pub const B_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 190.0,
    linear_factor: 0.22675737,
});

// At R_gain = 120Ω, ref = 0V
pub const C_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 204.0,
    linear_factor: 0.12578616,
});

// At R_gain = 120Ω, ref = 0V
pub const D_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 166.0,
    linear_factor: 0.12594458,
});

// At R_gain = 120Ω, ref = 0V
pub const E_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 233.0,
    linear_factor: 0.12787724,
});

// At R_gain = 120Ω, ref = 0V
pub const F_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 200.0,
    linear_factor: 0.12771392,
});

// At R_gain = 220Ω, ref = 1.65V
pub const G_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 496.0,
    linear_factor: 0.23419204,
});

// NOTE: not optimized, may change
// At R_gain = 220Ω, ref = 0V
pub const H_100BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 74.0,
    linear_factor: 0.232,
});

// --- 400 bar ---

// At R_gain = 220Ω, ref = 1.65V
pub const B_400BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 439.0,
    linear_factor: 0.91116173,
});

// NOTE: not optimized, may change
// At R_gain = 220Ω, ref = 0V
pub const C_400BAR: SK = SK::SimplePressure(PressureSensorCalib {
    offset: 49.0,
    linear_factor: 0.888,
});
