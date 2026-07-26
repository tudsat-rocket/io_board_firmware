use embassy_time::Duration;

use crate::ext_adc::SensorSettings;
use crate::node::NodeSettings;
use crate::sensors::SensorMapping;
use crate::tpdo::TpdoIntervals;
use crate::valves::ValveMapping;
use crate::zenith_mapping::sensors::{PLACEHOLDER_P, PLACEHOLDER_T};
use crate::zenith_mapping::valves::PLACEHOLDER_S;

mod sensors;
mod valves;

// nosecose or recovery
pub const NODE2: NodeSettings = NodeSettings {
    node_id: 2,
    valve_mapping: ValveMapping::new_empty(),
    sensor_mapping: SensorMapping::new_empty().add_consecutive(PLACEHOLDER_T, 0, 0).unwrap(),
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
    valve_mapping: ValveMapping::new_empty().add_std_servo_hco12(valves::VENT, 0).unwrap(),
    sensor_mapping: SensorMapping::new_empty(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};
// upper propulsion
pub const NODE5: NodeSettings = NodeSettings {
    node_id: 5,
    valve_mapping: ValveMapping::new_empty()
        .add_std_servo_hco12(valves::PRESSURIZATION, 0)
        .unwrap()
        .add_std_servo_hco34(valves::PLACEHOLDER_S, 500)
        .unwrap(),
    sensor_mapping: SensorMapping::new_empty()
        // temperature sensor regulator
        .add_consecutive(PLACEHOLDER_T, 0, 0)
        .unwrap()
        // pressure sensor regulator upper
        .add_consecutive(PLACEHOLDER_P, 0, 1)
        .unwrap()
        // pressure sensor regulator lower
        .add_consecutive(PLACEHOLDER_P, 0, 2)
        .unwrap()
        // pressure sensor upper oxidizer
        .add_consecutive(PLACEHOLDER_P, 1, 0)
        .unwrap()
        // presssure sensor pressurant (N2) tank
        .add_consecutive(PLACEHOLDER_P, 1, 1)
        .unwrap(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

// lower propulsion / valve control
pub const NODE6: NodeSettings = NodeSettings {
    node_id: 6,
    valve_mapping: ValveMapping::new_empty()
        // main valve
        .add_std_servo_hco12(PLACEHOLDER_S, 0)
        .unwrap()
        .add_std_servo_hco34(valves::FILL_AND_DUMP, 0)
        .unwrap(),
    sensor_mapping: SensorMapping::new_empty()
        .add_consecutive(sensors::PROV_100BAR_H, 0, 0)
        .unwrap()
        .add_consecutive(PLACEHOLDER_P, 0, 1)
        .unwrap()
        .add_consecutive(PLACEHOLDER_T, 0, 2)
        .unwrap(),

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
