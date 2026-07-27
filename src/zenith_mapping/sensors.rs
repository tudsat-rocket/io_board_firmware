use crate::sensors::{PressureSensorCalib, SensorKind};

pub const PROV_100BAR_H: SensorKind = SensorKind::SimplePressure(PressureSensorCalib {
    offset: 76.0,
    linear_factor: 0.232,
});
pub const PT_1000: SensorKind = SensorKind::TempPt1000;

pub const PLACEHOLDER_T: SensorKind = SensorKind::TempPt1000;
pub const PLACEHOLDER_P: SensorKind = SensorKind::SimplePressure(PressureSensorCalib {
    offset: 0.0,
    linear_factor: 0.0,
});
