#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Duration;
use io_board::{
    ext_adc::SensorSettings, node::NodeSettings, sensors::SensorMapping, tpdo::TpdoIntervals, valves::ValveMapping,
};

use {defmt_rtt as _, panic_probe as _};

// Firmware metadata generated using `cancan-build`
include!(concat!(env!("OUT_DIR"), "/cancan_metadata.rs"));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    io_board::node::spawn_node(spawner, EMPTY).await;
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
    }
}

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
    }
}
