#![no_std]
#![no_main]
#![allow(unused_imports, reason = "imports change because this is a testing and debug node")]
use embassy_executor::Spawner;
use embassy_time::Duration;

use io_board::valves::SolenoidVavle;
use io_board::zenith_mapping::sensors::{PLACEHOLDER_P, PLACEHOLDER_T};
use io_board::zenith_mapping::valves::PLACEHOLDER_S;

use io_board::zenith_mapping::valves;
use io_board::{
    ext_adc::SensorSettings, node::NodeSettings, sensors::SensorMapping, tpdo::TpdoIntervals, valves::ValveMapping,
};

use defmt_rtt as _;

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, NODE6_DEBUG).await;
}

pub const EMPTY: NodeSettings = NodeSettings {
    node_id: 2,
    valve_mapping: ValveMapping::new_empty(),
    sensor_mapping: SensorMapping::new_empty(),
    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: no_tpdo(),
};

pub const NODE6_DEBUG: NodeSettings = NodeSettings {
    node_id: 6,
    valve_mapping: ValveMapping::new_empty()
        // main valve
        .add_std_servo_hco12(PLACEHOLDER_S, 0)
        .unwrap()
        .add_std_servo_hco34(valves::OX_FILL_AND_DUMP, 0)
        .unwrap(),
    sensor_mapping: SensorMapping::new_empty().add_consecutive(PLACEHOLDER_T, 0, 0).unwrap(),

    sensor_settings: SensorSettings {
        measure_interval: Duration::from_millis(10),
    },
    tpdo_intervals: TpdoIntervals::default(),
};

const fn no_tpdo() -> TpdoIntervals {
    TpdoIntervals {
        valves: Some(Duration::from_millis(1000)),
        binary_outpus: None,
        pwm_us: None,
        raw_bus0a: None,
        raw_bus0b: None,
        raw_bus1a: None,
        raw_bus1b: None,
        sensor0: None,
        sensor1: None,
        sensor2: None,
    }
}

#[allow(dead_code)]
const fn slow() -> TpdoIntervals {
    TpdoIntervals {
        valves: Some(Duration::from_millis(1000)),
        binary_outpus: Some(Duration::from_millis(1000)),
        pwm_us: Some(Duration::from_millis(1000)),
        raw_bus0a: Some(Duration::from_millis(1000)),
        raw_bus0b: None,
        raw_bus1a: Some(Duration::from_millis(1000)),
        raw_bus1b: None,
        sensor0: Some(Duration::from_millis(1000)),
        sensor1: None,
        sensor2: None,
    }
}
