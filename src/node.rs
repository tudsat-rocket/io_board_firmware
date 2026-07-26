use embassy_executor::Spawner;
use embassy_stm32::flash::Flash;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Duration;

use cancan::{CanCan, CanCanConfig};
use static_cell::StaticCell;

#[cfg(feature = "rev3")]
use crate::board::{CurrentSens, OnboardSensRev3, VoltageSens};

#[cfg(feature = "rev2")]
use crate::board::HcoControllerRev2;
#[cfg(feature = "rev3")]
use crate::board::HcoControllerRev3;

use crate::{
    board::{Board, init_board},
    can::{CAN_IN, CAN_OUT},
    canopen_interface::{CanOpenInterface, run_can_command_listener},
    ext_adc::SensorSettings,
    sensors::{self, SensorMapping},
    tpdo::{TpdoIntervals, spawn_tpdo_task},
    valves::ValveMapping,
};

use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "rev2")]
static HCO_CONTROLER: StaticCell<HcoControllerRev2> = StaticCell::new();
#[cfg(feature = "rev3")]
static HCO_CONTROLER: StaticCell<HcoControllerRev3> = StaticCell::new();

#[cfg(feature = "rev2")]
pub const NODE_NAME: &str = "I/O [rev2]";
#[cfg(feature = "rev3")]
pub const NODE_NAME: &str = "I/O [rev3]";

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
    let board: Board = init_board(spawner).await;

    let cancan_config = CanCanConfig {
        // FIXME:
        node_id: settings.node_id,
        name: NODE_NAME,
        chip_id: embassy_stm32::pac::DBGMCU.idcode().read().0,
        chip_uid: embassy_stm32::uid::uid(),
        flash_kib: (embassy_stm32::flash::FLASH_SIZE / 1024) as u16,
        build_id: crate::CANCAN_BUILD_ID,
        build_timestamp: crate::CANCAN_BUILD_TIMESTAMP,
        ..Default::default()
    };

    let mut cancan = CanCan::new(cancan_config, Flash::new_blocking(board.flash_peri));

    // let mut p = hw::setup();
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    crate::can::spawn(board.can1, &mut cancan, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap())
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

    let hco_controler = HCO_CONTROLER.init(board.hco_controller);

    let can_open_interface = CanOpenInterface::new(
        (can_out.publisher().unwrap(), can_in.subscriber().unwrap()),
        hco_controler,
        settings.node_id,
        settings.valve_mapping,
    );
    spawner.spawn(run_can_command_listener(can_open_interface).unwrap());

    spawner.spawn(
        spawn_tpdo_task(spawner, settings.tpdo_intervals, can_out.publisher().unwrap(), settings.node_id).unwrap(),
    );

    // #[cfg(feature = "rev3")]
    // spawner.spawn(onboard_sens_debug(board.onboard_sens).unwrap());

    spawner.spawn(run_cancan(cancan).unwrap());
}

/// Runs cancan, the firmware updater/bootloader task.
#[embassy_executor::task]
pub async fn run_cancan(cancan: CanCan<Flash<'static, embassy_stm32::flash::Blocking>>) {
    cancan.run(&crate::CANCAN).await
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
