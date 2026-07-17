#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};

#[cfg(feature = "rev3")]
use crate::board::{CurrentSens, OnboardSensRev3, TemperatureSens, VoltageSens};

use crate::board::{Board, LedsState};
use crate::canopen_interface::{CanOpenInterface, run_can_command_listener};

use {defmt_rtt as _, panic_probe as _};

mod board;
mod can;
mod canopen_interface;
mod ereg;
mod ext_adc;
mod sensors;
mod store;
mod utils;
mod valves;

// const CANOPEN_NODE_ID: u8 = X;

// #[global_allocator]
// static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

// static COM1_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
// static COM2_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board: Board = board::init_board(spawner).await;
    // let mut p = hw::setup();
    // let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    // iwdg.unleash();

    use can::{CAN_IN, CAN_OUT};
    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    can::spawn(board.can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    spawner.spawn(
        ext_adc::run_ext_adc_to_can(
            Some(board.com1_i2c),
            Some(board.com2_i2c),
            can_out.publisher().unwrap(),
            ext_adc::SensorSettings {
                broadcast_interval: Duration::from_millis(100),
            },
        )
        .unwrap(),
    );

    // spawner.spawn(run_ereg(hco_contoler).unwrap());

    let can_open_interface =
        CanOpenInterface::new((can_out.publisher().unwrap(), can_in.subscriber().unwrap()), board.hco_controller);
    spawner.spawn(run_can_command_listener(can_open_interface).unwrap());

    #[cfg(feature = "rev3")]
    spawner.spawn(onboard_sens_debug(board.onboard_sens).unwrap());
}

#[embassy_executor::task]
#[cfg(feature = "rev3")]
pub async fn onboard_sens_debug(mut sens: OnboardSensRev3) {
    let mut ticker = Ticker::every(Duration::from_hz(1));
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
