use crate::sensors::{PressureSensorCalib, SensorKind, TempSensorCalib};

pub const PROV_100BAR_H: SensorKind = SensorKind::SimplePressure(PressureSensorCalib {
    offset: 76.0,
    linear_factor: 0.232,
});
pub const PLACEHOLDER_T: SensorKind = SensorKind::SimpleTemp(TempSensorCalib { gain: 0.0, offset: 0.0 });
pub const PLACEHOLDER_P: SensorKind = SensorKind::SimpleTemp(TempSensorCalib { gain: 0.0, offset: 0.0 });
