#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Duration;
use io_board::{
    ext_adc::SensorSettings, node::NodeSettings, sensors::SensorMapping, tpdo::TpdoIntervals, valves::ValveMapping,
    zenith_mapping,
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
    tpdo_intervals: TpdoIntervals::default(),
};
