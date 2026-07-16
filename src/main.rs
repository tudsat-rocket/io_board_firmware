#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{
    adc::Adc,
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};
use static_cell::StaticCell;

use crate::board::{CurrentBoard, LedsState};
use crate::canopen_interface::{CanOpenInterface, run_can_command_listener};
use board::high_current_outputs::HcoControl;

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

static COM1_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();
static COM2_I2C: StaticCell<I2c<'static, Async, Master>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board: CurrentBoard = board::init_board(spawner).await;
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
}

// #[embassy_executor::task]
// pub async fn run_heartbeat(can_tx: CanTxPub) {
//     let mut ticker = Ticker::every(Duration::from_hz(1));
//     let id: u16 = 0x12;
//     let body: Vec<u8, 8> = Vec::from_array([0, 0, 0, 0, 0, 0, 0, 0]);
//     loop {
//         can_tx.publish_immediate(((id, body.clone())));
//         ticker.next().await
//     }
// }

// #[embassy_executor::task]
// pub async fn pdo_watcher(publisher: board::StateLedPub) {
//     // red_led
//     use board::LedsState;
//     let mut leds_state = LedsState::from([false, false, false]);
//     let mut ticker = Ticker::every(Duration::from_hz(10));
//     loop {
//         publisher.publish(leds_state).await;
//         ticker.next().await;
//     }
// }
