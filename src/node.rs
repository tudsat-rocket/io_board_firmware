use embassy_executor::{InterruptExecutor, Spawner};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Duration;

#[cfg(feature = "rev3")]
use crate::board::{CurrentSens, OnboardSensRev3, VoltageSens};

use crate::board::{Board, init_board};
use crate::can::{CAN_IN, CAN_OUT};
use crate::canopen_interface::{CanOpenInterface, run_can_command_listener};
use crate::sensors;

use {defmt_rtt as _, panic_probe as _};

use crate::{
    ext_adc::SensorSettings,
    sensors::SensorMapping,
    tpdo::{TpdoIntervals, spawn_tpdo_task},
    valves::ValveMapping,
};

pub struct NodeSettings {
    pub node_id: u8,
    pub valve_mapping: ValveMapping,
    pub sensor_mapping: SensorMapping,
    pub tpdo_intervals: TpdoIntervals,
    pub sensor_settings: SensorSettings,
}
impl NodeSettings {
    pub const fn default() -> Self {
        Self {
            node_id: 2,
            valve_mapping: ValveMapping::new_empty(),
            sensor_mapping: SensorMapping::new_empty(),
            tpdo_intervals: TpdoIntervals::default(),
            sensor_settings: SensorSettings {
                measure_interval: Duration::from_millis(10),
            },
        }
    }
}

pub async fn spawn_node(spawner: Spawner, settings: NodeSettings) {
    let mut board: Board = init_board(spawner).await;
    // let mut p = hw::setup();
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    crate::can::spawn(
        board.can1,
        &mut board.cancan,
        spawner,
        can_in.publisher().unwrap(),
        can_out.subscriber().unwrap(),
    )
    .await;

    spawner.spawn(
        sensors::run_sensors(
            Some(board.com1_i2c),
            Some(board.com2_i2c),
            settings.sensor_settings,
            settings.sensor_mapping,
        )
        .unwrap(),
    );

    // spawner.spawn(run_ereg(hco_contoler).unwrap());

    let can_open_interface = CanOpenInterface::new(
        (can_out.publisher().unwrap(), can_in.subscriber().unwrap()),
        board.hco_controller,
        settings.node_id,
    );
    spawner.spawn(run_can_command_listener(can_open_interface).unwrap());

    spawner.spawn(
        spawn_tpdo_task(spawner, settings.tpdo_intervals, can_out.publisher().unwrap(), settings.node_id).unwrap(),
    );

    #[cfg(feature = "rev3")]
    spawner.spawn(onboard_sens_debug(board.onboard_sens).unwrap());

    spawner.spawn(crate::run_cancan(board.cancan).unwrap());
}

#[embassy_executor::task]
#[cfg(feature = "rev3")]
pub async fn onboard_sens_debug(mut sens: OnboardSensRev3) {
    let mut ticker = embassy_time::Ticker::every(Duration::from_hz(1));
    loop {
        let v_logic = sens.logic_supply_voltage_milli_v().await;
        let v_hco12 = sens.hco12_supply_voltage_milli_v().await;
        let v_hco34 = sens.hco34_supply_voltage_milli_v().await;

        let i_logic = sens.logic_supply_current_ma().await.unwrap_or(0);
        let i_hco12 = sens.hco12_current_ma().await;
        let i_hco34 = sens.hco34_current_ma().await;

        defmt::info!("logic: {} mV, {} mA", v_logic, i_logic);
        defmt::info!("hco12: {} mV, {} mA", v_hco12, i_hco12);
        defmt::info!("hco34: {} mV, {} mA \n", v_hco34, i_hco34);
        defmt::info!(" ");

        ticker.next().await;
    }
}
