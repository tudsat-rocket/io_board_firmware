use crate::ext_adc::SensorSettings;
use crate::node::NodeSettings;
use crate::sensors::{PressureSensorCalib, Sensor, SensorKind, SensorMapping, TempSensorCalib};
use crate::tpdo::TpdoIntervals;
use crate::valves::{ServoValve, ServoValveCalib, Valve, ValveEntry, ValveMapping};

pub const PROV_100BAR_H: SensorKind = SensorKind::SimplePressure(PressureSensorCalib {
    offset: 76.0,
    linear_factor: 0.232,
});
