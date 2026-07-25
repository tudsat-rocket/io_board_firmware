use embassy_time::Duration;

use crate::ext_adc::SensorSettings;
use crate::node::NodeSettings;
use crate::sensors::{PressureSensorCalib, Sensor, SensorKind, SensorMapping, TempSensorCalib};
use crate::tpdo::TpdoIntervals;
use crate::valves::{ServoValve, ServoValveCalib, Valve, ValveEntry, ValveMapping};
use crate::zenith_mapping::valves::PRESSURIZATION;

mod sensors;
mod valves;

pub const NODE6REV2TEST: NodeSettings = NodeSettings {
    node_id: 6,
    valve_mapping: ValveMapping([
        None,
        // main valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_2,
                power_con: Some(crate::board::HighCurrentOutput::_1),
                calib: ServoValveCalib {
                    open_us: 1000,
                    closed_us: 2000,
                },
            }),
        }),
        None,
        // fill and dump valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_4,
                power_con: Some(crate::board::HighCurrentOutput::_3),
                calib: ServoValveCalib {
                    open_us: 1000,
                    closed_us: 2000,
                },
            }),
        }),
    ]),
    sensor_mapping: SensorMapping([
        Some(Sensor {
            kind: sensors::PROV_100BAR_H,
            bus_idx: 0,
            sensor_idx: 0,
        }),
        Some(Sensor {
            kind: SensorKind::SimplePressure(
                // TODO: calib
                PressureSensorCalib {
                    linear_factor: 0.0,
                    offset: 0.0,
                },
            ),
            bus_idx: 0,
            sensor_idx: 1,
        }),
        Some(Sensor {
            // TODO: calib
            kind: crate::sensors::SensorKind::SimpleTemp(TempSensorCalib { gain: 0.0, offset: 0.0 }),
            bus_idx: 0,
            sensor_idx: 2,
        }),
        None,
        None,
        None,
        None,
        None,
    ]),

    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// nosecose or recovery
pub const NODE2: NodeSettings = NodeSettings {
    node_id: 2,
    valve_mapping: ValveMapping::new_empty(),
    sensor_mapping: SensorMapping([
        // temperature sensor regulator
        Some(Sensor {
            // TODO: calib
            kind: SensorKind::SimplePressure(PressureSensorCalib {
                linear_factor: 0.0,
                offset: 0.0,
            }),
            bus_idx: 0,
            sensor_idx: 0,
        }),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    ]),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// payvionics
pub const NODE3: NodeSettings = NodeSettings {
    node_id: 3,
    valve_mapping: ValveMapping::new_empty(),
    sensor_mapping: SensorMapping::new_empty(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// upper propulsion
pub const NODE4: NodeSettings = NodeSettings {
    node_id: 4,
    valve_mapping: ValveMapping([
        None,
        // oxidizer vent valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_2,
                power_con: Some(crate::board::HighCurrentOutput::_1),
                calib: valves::VENT,
            }),
        }),
        None,
        None,
    ]),
    sensor_mapping: SensorMapping::new_empty(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};
// upper propulsion
pub const NODE5: NodeSettings = NodeSettings {
    node_id: 5,
    valve_mapping: ValveMapping([
        None,
        // Presurization Valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_2,
                power_con: Some(crate::board::HighCurrentOutput::_1),
                calib: valves::PRESSURIZATION,
            }),
        }),
        None,
        // Vent Valve
        Some(ValveEntry {
            init_state_promille: 500,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_4,
                power_con: Some(crate::board::HighCurrentOutput::_3),
                calib: ServoValveCalib {
                    open_us: 2000,
                    closed_us: 1000,
                },
            }),
        }),
    ]),
    sensor_mapping: SensorMapping([
        // temperature sensor regulator
        Some(Sensor {
            kind: SensorKind::SimpleTemp(TempSensorCalib {
                // TODO: calib
                gain: 0.0,
                offset: 0.0,
            }),
            bus_idx: 0,
            sensor_idx: 0,
        }),
        // pressure sensor regulator upper
        Some(Sensor {
            kind: SensorKind::SimplePressure(
                // TODO: calib
                PressureSensorCalib {
                    linear_factor: 0.0,
                    offset: 0.0,
                },
            ),
            bus_idx: 0,
            sensor_idx: 1,
        }),
        // pressure sensor regulator lower
        Some(Sensor {
            // TODO: calib
            kind: SensorKind::SimplePressure(PressureSensorCalib {
                linear_factor: 0.0,
                offset: 0.0,
            }),
            bus_idx: 0,
            sensor_idx: 2,
        }),
        // pressure sensor upper oxidizer
        Some(Sensor {
            // TODO: calib
            kind: SensorKind::SimplePressure(PressureSensorCalib {
                linear_factor: 0.0,
                offset: 0.0,
            }),
            bus_idx: 1,
            sensor_idx: 0,
        }),
        // presssure sensor pressurant (N2) tank
        Some(Sensor {
            // TODO: calib
            kind: SensorKind::SimplePressure(PressureSensorCalib {
                linear_factor: 0.0,
                offset: 0.0,
            }),
            bus_idx: 1,
            sensor_idx: 1,
        }),
        None,
        None,
        None,
    ]),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// lower propulsion / valve control
pub const NODE6: NodeSettings = NodeSettings {
    node_id: 6,
    valve_mapping: ValveMapping([
        None,
        // main valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_2,
                power_con: Some(crate::board::HighCurrentOutput::_1),
                calib: ServoValveCalib {
                    open_us: 1000 - 450,
                    closed_us: 2000 + 450,
                },
            }),
        }),
        None,
        // fill and dump valve
        Some(ValveEntry {
            init_state_promille: 0,
            kind: Valve::Servo(ServoValve {
                pwm_con: crate::board::HighCurrentOutput::_4,
                power_con: Some(crate::board::HighCurrentOutput::_3),
                calib: valves::FILL_AND_DUMP,
            }),
        }),
    ]),
    sensor_mapping: SensorMapping([
        Some(Sensor {
            kind: sensors::PROV_100BAR_H,
            bus_idx: 0,
            sensor_idx: 0,
        }),
        Some(Sensor {
            kind: SensorKind::SimplePressure(
                // TODO: calib
                PressureSensorCalib {
                    linear_factor: 0.0,
                    offset: 0.0,
                },
            ),
            bus_idx: 0,
            sensor_idx: 1,
        }),
        Some(Sensor {
            // TODO: calib
            kind: crate::sensors::SensorKind::SimpleTemp(TempSensorCalib { gain: 0.0, offset: 0.0 }),
            bus_idx: 0,
            sensor_idx: 2,
        }),
        None,
        None,
        None,
        None,
        None,
    ]),

    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// lower propulsion / igniter control
pub const NODE7: NodeSettings = NodeSettings {
    node_id: 7,
    valve_mapping: ValveMapping::new_empty(),
    sensor_mapping: SensorMapping::new_empty(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};
